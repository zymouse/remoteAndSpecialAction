#!/usr/bin/env python
#--*-- coding:utf-8 --*--
"""
    1.0 特殊动作开始执行时机 -- 到达指定地点
    2.0 特殊动作结束执行时机
    3.0 特殊动作执行器
"""

from math import hypot

from geometry_msgs.msg import Pose
from remote_control.msg import RemoteControl
from tf.transformations import euler_from_quaternion
import rospy

class SpecialAction:
    def __init__(self):
        self.remote_control_msg = RemoteControl()
        _pub_remote_control_topic = rospy.get_param("/fleet/pub_remote_control", "out/pub_remote_control")
        self.pub_remote_control = rospy.Publisher(_pub_remote_control_topic, RemoteControl, queue_size=10)

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


        euler_from_quaternion([pose_1.orientation.x, pose_1.orientation.y, pose_1.orientaton.z, pose_1.orientation.w])
        euler_from_quaternion([pose_2.orientation.x, pose_2.orientation.y, pose_2.orientation.z, pose_2.orientation.w])

        pass


    def circleAction(self):
        """画圆圈动作"""
        self.remote_control_msg.m = 5            # 远程驾驶模式
        self.remote_control_msg.stVehicleMod=1   # 异向转向模式

        self.remote_control_msg.g = "D"          # 档位模式
        self.remote_control_msg.st = 500         # 方向盘转向角度
        self.remote_control_msg.tr = 10          # 油门踏板量
        pass


    def diagonalAction(self):
        """斜线动作"""
        self.remote_control_msg.m = 5            # 远程驾驶模式
        self.remote_control_msg.stVehicleMod=2   # 异向转向模式

        self.remote_control_msg.g = "D"          # 档位模式
        self.remote_control_msg.st = 0           # 方向盘转向角度
        self.remote_control_msg.tr = 10          # 油门踏板量
        pass
    

    

if __name__ == "__main__":
    pass