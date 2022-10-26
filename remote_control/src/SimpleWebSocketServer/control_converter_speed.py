#!/usr/bin/env python

import rospy
# from pix_driver_msgs.msg import GearCommand, ThrottleCommand, BrakeCommand, SteeringCommand, VehicleModeCommand, GearReport
from pix_driver_msgs.msg import gear_command_103, throttle_command_100, brake_command_101, steering_command_102, vehicle_mode_command_105, gear_report_503, acu_sweepctrlcmd_107
from remote_control.msg import RemoteControl
import math
import time

class ControlConverter:
    def __init__(self):
        self.speed = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.gear = 0
        self.gear_act = 3
        self.driver_mode = 0
        # self.AutoCleaningStartCtrl = 0
        
        self.sub_gear_report = rospy.Subscriber('/pix/gear_report', gear_report_503, self.gear_report_callback)
        self.sub_remote_command = rospy.Subscriber('/remote_control_cmd', RemoteControl, self.remote_control_callback)

        self.pub_throttle = rospy.Publisher('/pix/throttle_command', throttle_command_100, queue_size=10)
        self.pub_brake = rospy.Publisher('/pix/brake_command', brake_command_101, queue_size=10)
        self.pub_steer = rospy.Publisher('/pix/steering_command', steering_command_102, queue_size=10)
        self.pub_gear = rospy.Publisher('/pix/gear_command', gear_command_103, queue_size=10)
        self.pub_vehicle = rospy.Publisher('/pix/vehicle_mode_command', vehicle_mode_command_105, queue_size=10)

        # self.pub_acu_sweepctrlcmd = rospy.Publisher('/pix/acu_sweepctrlcmd', acu_sweepctrlcmd_107, queue_size=10)
        

        self.throttle_msg = throttle_command_100()
        self.brake_msg = brake_command_101()
        self.steer_msg = steering_command_102()
        self.gear_msg = gear_command_103()
        self.vehicle_msg = vehicle_mode_command_105()
        # self.acu_sweepctrlcmd_msg = acu_sweepctrlcmd_107()

    def remote_control_callback(self, msg):
        # print("1111111111111111111")
        self.speed = msg.tr/60
        self.brake = msg.bk
        self.steer = -msg.st*5
        
        if(msg.g == 'N'):
            self.gear = 3
        elif(msg.g == 'P'):
            self.gear = 1
        elif(msg.g == 'R'):
            self.gear = 2
        elif(msg.g == 'D'):
            self.gear = 4
        elif(msg.g == '-'):
            self.gear = 0
        if(self.gear_act != self.gear):
            self.speed = 0
        
        stamp = rospy.Time.now()
        self.throttle_msg.header.stamp = stamp
        self.throttle_msg.Dirve_SpeedTarget = self.speed
        self.throttle_msg.Dirve_EnCtrl = 1
        # print("1111111111111111111")
        self.pub_throttle.publish(self.throttle_msg)

        self.brake_msg.header.stamp = stamp
        self.brake_msg.Brake_Pedal_Target = self.brake
        self.brake_msg.Brake_EnCtrl = 1
        self.pub_brake.publish(self.brake_msg)

        self.gear_msg.header.stamp = stamp
        self.gear_msg.Gear_EnCtrl = 1
        self.gear_msg.Gear_Target = self.gear
        self.pub_gear.publish(self.gear_msg) 

        self.driver_mode = 1
        self.vehicle_msg.header.stamp = stamp
        self.vehicle_msg.Drive_ModeCtrl = self.driver_mode
        self.vehicle_msg.Steer_ModeCtrl = 1
        self.pub_vehicle.publish(self.vehicle_msg)

        self.steer_msg.header.stamp = stamp
        self.steer_msg.Steer_EnCtrl = 1
        self.steer_msg.Steer_AngleTarget = self.steer
        self.steer_msg.Steer_AngleSpeed = 250
        self.pub_steer.publish(self.steer_msg)

        # self.acu_sweepctrlcmd_msg.header.stamp = stamp
        # self.acu_sweepctrlcmd_msg.AutoCleaningStartCtrl = msg.autoClean
        # self.pub_acu_sweepctrlcmd.publish(self.acu_sweepctrlcmd_msg)

    def gear_report_callback(self, msg):
        self.gear_act = msg.Gear_Actual
    




if __name__ == '__main__':
    
    rospy.init_node('autoware_pix_control_converter', anonymous=True)
    converter = ControlConverter()
    rospy.spin()
