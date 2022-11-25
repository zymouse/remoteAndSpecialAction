#!/usr/bin/env python
#--*-- coding:utf-8 --*--
import rospy
import math
import time


from autoware_vehicle_msgs.msg import RawControlCommandStamped,ShiftStamped
from autoware_control_msgs.msg import  GateMode
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped

from pix_driver_msgs.msg import gear_report_503,vcu_report_505

from remote_control.msg import RemoteControl
·

class PAAPRemote:
    """
    桥接PixAutowareArchitectureProposal的remote_control模块
    """
    def __init__(self):
        # self.gear_act = 3

        # self.speed = 0.0
        # self.brake = 0.0
        # self.steer = 0.0
        self.vehicle_current_speed = 0
        self.vehicle_current_gear = 3
        
        wheelMaxTurnAngular = rospy.get_param("/vehicle/wheelMaxTurnAngular", 24)  # 轮胎转动角度
        steeringMaxTurnAngular = rospy.get_param("/remote/steeringMaxTurnAngular", 24)  # 轮胎转动角度
        self.wheelSteeringrRatio = (wheelMaxTurnAngular*(math.pi/180.0))/steeringMaxTurnAngular
        

        self.sub_gear_report = rospy.Subscriber('/pix/gear_report', gear_report_503, self.gear_report_callback)
        self.sub_remote_command = rospy.Subscriber('/pix/vcu_report', vcu_report_505, self.vcu_report_callback)
        self.sub_remote_command = rospy.Subscriber('/remote_control_cmd', RemoteControl, self.remote_control_callback)

        """
        from autoware_vehicle_msgs import RawControlCommandStamped,ShiftStamped,GateMode
        from std_msgs import Bool
        from geometry_msgs import TwistStamped  
        """
        self.pub_RawControlCommand_msg = RawControlCommandStamped()
        self.pub_ShiftStamped_msg      = ShiftStamped()
        self.pub_StopBool_msg          = Bool()
        self.pub_GateMode_msg          = GateMode()
        self.pub_TwistStamped_msg      = TwistStamped()

        """
        /remote/control_cmd
        /remote/emergency_stop
        /remote/gate_mode_cmd
        /remote/latest_raw_control_cmd
        /remote/raw_control_cmd
        /remote/shift_cmd
        /remote/turn_signal_cmd
        """
        # 转向值，转向角速度，油门踏板量，刹车踏板量
        self.pub_raw_control_cmd   = rospy.Publisher('/remote/raw_control_cmd', RawControlCommandStamped, queue_size=10)
        
        self.pub_shift_cmd         = rospy.Publisher('/remote/shift_cmd', ShiftStamped, queue_size=10)
        self.pub_emergency_stop    = rospy.Publisher('/remote/emergency_stop', Bool, queue_size=10)
        self.pub_current_gate_mode = rospy.Publisher('/remote/gate_mode_cmd', GateMode, queue_size=10)
        # self.pub_twistStamped      = rospy.Publisher('/localization/twist', TwistStamped, queue_size=10)

    def remote_control_callback(self, msg):
        ## @brief 订阅回调函数
        # @param[in] msg 传入的话题
        # @return[out] 无
        # @namespace PAAPRemote.remote_control_callback
        """
        msg_type: RemoteControl
        """
        
        # 网络的刹车值, 油门值, 转向值, 转向速率值 ---> /remote/raw_control_cmd
        self.pub_RawControlCommand_msg.header.stamp = rospy.Time.now()
        self.pub_RawControlCommand_msg.control.steering_angle = -msg.st*self.wheelSteeringrRatio
        self.pub_RawControlCommand_msg.control.steering_angle_velocity = self.wheelSteeringrRatio*100
        # if(msg.g ==  "N" or msg.g ==  "P" or msg.g ==  "-"):
        #     self.pub_RawControlCommand_msg.control.throttle = 0
        # else:
        #     self.pub_RawControlCommand_msg.control.throttle = msg.bk
        # 最大10%的踏板量
        self.pub_RawControlCommand_msg.control.throttle = (msg.tr/100)/10
        self.pub_RawControlCommand_msg.control.brake = (msg.bk/100)/10
        self.pub_raw_control_cmd.publish(self.pub_RawControlCommand_msg)

        # 网络的挡位值 ---> /remote/shift_cmd
        self.pub_ShiftStamped_msg.header.stamp = rospy.Time.now()
        if(msg.g=="D"):
            self.pub_ShiftStamped_msg.shift.data = 4
        if(msg.g=="N"):
            self.pub_ShiftStamped_msg.shift.data = 3
        if(msg.g=="R"):
            self.pub_ShiftStamped_msg.shift.data = 2
        if(msg.g=="P"):
            self.pub_ShiftStamped_msg.shift.data = 1
        self.pub_shift_cmd.publish(self.pub_ShiftStamped_msg)  

        # 需要优化后端--------------------------------------
        # 网络的挡位值 ---> /remote/emergency_stop
        if(msg.g ==  "N" or msg.g ==  "P" or msg.g ==  "-"):
            self.pub_StopBool_msg.data = True
        else:
            self.pub_StopBool_msg.data = False
        self.pub_emergency_stop.publish(self.pub_StopBool_msg)

        # 需要优化后端--------------------------------------
        # 网络的车辆模式值 /control/current_gate_mode 
        if msg.m==3:
            self.pub_GateMode_msg.data = 0
        else:
            self.pub_GateMode_msg.data = 1
        # 
        # rospy.loginfo("操作模式更换")
        self.pub_current_gate_mode.publish(self.pub_GateMode_msg)

        # 底盘反馈的底盘速度值 /localization/twist 
        # self.pub_TwistStamped_msg.header.stamp = rospy.Time.now()
        # self.pub_TwistStamped_msg.twist.linear.x = self.vehicle_current_speed
        # self.pub_twistStamped.publish(self.pub_TwistStamped_msg)
        
        

    def gear_report_callback(self, msg):
        ## @brief 订阅回调函数
        # @param[in] msg 传入的话题
        # @return[out] 无
        # @namespace PAAPRemote.gear_report_callback
        """
            msg_type: gear_report_503
            0x4: DRIVE
            0x3: NEUTRAL
            0x2: REVERSE
            0x1: PARK
            0x0: INVALID
        """
        self.vehicle_current_gear = msg.Gear_Actual

    def vcu_report_callback(self, msg):
        ## @brief 订阅回调函数
        # @param[in] msg 传入的话题
        # @return[out] 无
        # @namespace PAAPRemote.gear_report_callback
        self.vehicle_current_speed = msg.Vehicle_Speed


if __name__=="__main__":
    rospy.init_node('Network_PAAPRemote', anonymous=True)
    converter = PAAPRemote()
    rospy.spin()