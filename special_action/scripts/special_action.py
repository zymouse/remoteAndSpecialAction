#!/usr/bin/env python
#--*-- coding:utf-8 --*--
"""
    1.0 特殊动作开始执行时机 -- 到达指定地点
    2.0 特殊动作结束执行时机
    3.0 特殊动作执行器
"""

from math import hypot

from geometry_msgs.msg import Pose 
from geometry_msgs.msg import PoseStamped
from remote_control.msg import RemoteControl
from tf.transformations import euler_from_quaternion

from special_action.srv import special, specialRequest,  specialResponse
import rospy

class SpecialAction(object):
    def __init__(self):
        self.remote_control_msg = RemoteControl()
        _pub_remote_control_topic = rospy.get_param("/fleet/pub_remote_control", "out/pub_remote_control")
        self.pub_remote_control = rospy.Publisher(_pub_remote_control_topic, RemoteControl, queue_size=10)
        
        self.current_pose = Pose()
        _pub_current_pose_topic = rospy.get_param("/fleet/sub_current_pose", "out/sub_current_pose")
        self.sub_current_pose = rospy.Subscriber(_pub_current_pose_topic, PoseStamped, self.callGetCurrentPose,queue_size=10)

    def callGetCurrentPose(self, msg):
        self.current_pose = msg.pose
    def isNearPose(self, pose_1, pose_2, th_dist):
        """
        俩个3D点之间的距离和X方向
        pose_1 3D点 Pose类型
        pose_2 3D点 Pose类型
        th_dist 距离阀值
        return bool (a, b) - a 是否超过距离阀值 b 是否x轴同向 
        """
        
        distanceXYZ = [abs(pose_1.position.x-pose_2.position.x),\
                       abs(pose_1.position.y-pose_2.position.y)]

        dist_12 = hypot(distanceXYZ[0], distanceXYZ[1])
        q_x1 = euler_from_quaternion([pose_1.orientation.x, pose_1.orientation.y, pose_1.orientation.z, pose_1.orientation.w])[0]
        q_x2 = euler_from_quaternion([pose_2.orientation.x, pose_2.orientation.y, pose_2.orientation.z, pose_2.orientation.w])[0]
        a=True
        if dist_12 < th_dist:
            a=True
        else:
            a=False

        b=True
        if abs(q_x1-q_x2)<0.0001:
            b=True
        else:
            b=False
        
        return a,b


    def circleAction(self, end_pose):
        """画圆圈动作"""
        self.remote_control_msg.m = 5            # 远程驾驶模式
        self.remote_control_msg.stVehicleMod=1   # 异向转向模式

        self.remote_control_msg.g = "D"          # 档位模式
        self.remote_control_msg.st = 500         # 方向盘转向角度
        self.remote_control_msg.tr = 10          # 油门踏板量
        a = 0
        while True:
            self.pub_remote_control.publish(self.remote_control_msg)
            # 两次经过点时
            if(all(self.isNearPose(self.current_pose, end_pose, 0.5))):
                a = a + 1
            # 列表全为ture时 - 距离满足阀值， roll角度平行。且第二次经过
            if(all(self.isNearPose(self.current_pose, end_pose, 0.5)) and a>1):
                break
        
        self.remote_control_msg.m = 3            # 远程驾驶模式
        self.remote_control_msg.stVehicleMod=1   # 异向转向模式

        self.remote_control_msg.g = "D"          # 档位模式
        self.remote_control_msg.st = 0           # 方向盘转向角度
        self.remote_control_msg.tr = 0            # 油门踏板量
        self.pub_remote_control.publish(self.remote_control_msg)
            


    def diagonalAction(self, end_pose):
        """斜线动作"""
        self.remote_control_msg.m = 5            # 远程驾驶模式
        self.remote_control_msg.stVehicleMod=2   # 异向转向模式

        self.remote_control_msg.g = "D"          # 档位模式
        self.remote_control_msg.st = 0           # 方向盘转向角度
        self.remote_control_msg.tr = 10          # 油门踏板量
        while True:
            self.pub_remote_control.publish(self.remote_control_msg)
            # 列表全为ture时 - 距离满足阀值， roll角度平行。且第二次经过
            if(all(self.isNearPose(self.current_pose, end_pose, 0.5))):
                break

            
        self.remote_control_msg.m = 3            # 远程驾驶模式
        self.remote_control_msg.stVehicleMod=1   # 异向转向模式

        self.remote_control_msg.g = "D"          # 档位模式
        self.remote_control_msg.st = 0           # 方向盘转向角度
        self.remote_control_msg.tr = 0            # 油门踏板量
        self.pub_remote_control.publish(self.remote_control_msg)
    
class SpecialSrivice(SpecialAction):
    def __init__(self):
        super(SpecialSrivice, self).__init__()
        _special_srv_name = rospy.get_param("special_srv_name", "fleet/special_srv")
        self.service = rospy.Service(_special_srv_name, special, self.callGetCurrentPose)
    
    def call_special_srv(self, req):
        resp = specialResponse()
        
        # 转圈圈
        if req.action_type == 1:
            self.circleAction(req.end_pose)
            resp.status = 1

        # 斜线直行
        if req.action_type == 2:
            self.diagonalAction(req.end_pose)
            resp.status = 2
        return resp
        


if __name__ == "__main__":

    test = SpecialAction()

    var1 = Pose()
    var1.position.x = 210.304000854
    var1.position.y = 112.869895935
    var1.position.z = 0.0
    var1.orientation.x = 0.0
    var1.orientation.y = 0.0
    var1.orientation.z = -0.104807817609
    var1.orientation.w = 0.994492494375

    var2 = Pose()
    var2.position.x = 210.235351562
    var2.position.y = 112.753219604
    var2.position.z = 0.0 
    var2.orientation.x = 0.0
    var2.orientation.y = 0.0
    var2.orientation.z = -0.104807936162
    var2.orientation.w = 0.994492481881

    print(test.isNearPose(var1, var2, 1))
    