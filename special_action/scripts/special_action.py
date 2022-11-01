#!/usr/bin/env python
#--*-- coding:utf-8 --*--
"""
    1.0 特殊动作开始执行时机
    2.0 特殊动作结束执行时机
    3.0 特殊动作执行器
"""
from remote_control.msg import RemoteControl
import rospy

class SpecialAction:
    def __init__(self):
        self.remote_control_msg = RemoteControl()
        _pub_remote_control_topic = rospy.get_param("/fleet/pub_remote_control", "out/pub_remote_control")
        self.pub_remote_control = rospy.Publisher(_pub_remote_control_topic, RemoteControl, queue_size=10)

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