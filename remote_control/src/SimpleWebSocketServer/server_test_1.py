#! /usr/bin/env python
# -*- coding: UTF-8 -*-
'''
copyright PixMoving
editor: Mark Jin
e-mail: mark@pixmoving.net
'''

import signal
import sys
import ssl
from SimpleWebSocketServer import WebSocket, SimpleWebSocketServer, SimpleSSLWebSocketServer
import rospy
from threading import Thread

from remote_control.msg import RemoteControl
#from autoware_msgs.msg import RemoteCmd
# from pix_driver_msgs.msg import BmsReport
# from pix_driver_msgs.msg import BrakeReport
# from pix_driver_msgs.msg import GearReport
# from pix_driver_msgs.msg import SteerReport
# from pix_driver_msgs.msg import VcuReport

from pix_driver_msgs.msg import bms_report_512    # ok 
from pix_driver_msgs.msg import brake_report_501  # ok 
from pix_driver_msgs.msg import gear_report_503   # ok
from pix_driver_msgs.msg import steering_report_502  # ok 
from pix_driver_msgs.msg import vcu_report_505   # ok

import json
import time
from datetime import datetime


clients = []
data = ""
feedback_data = ""

class SimpleChat(WebSocket):

    def handleMessage(self):
        for client in clients:
            #client.sendMessage(self.address[0] + u' - ' + self.data)
            global data
            global feedback_data
            data = self.data
            client.sendMessage(feedback_data)

    def handleConnected(self):
        print (self.address, 'connected')
        for client in clients:
            client.sendMessage(self.address[0] + u' - connected')
        clients.append(self)

    def handleClose(self):
        clients.remove(self)
        print (self.address, 'closed')
        for client in clients:
            client.sendMessage(self.address[0] + u' - disconnected')

def close_sig_handler(signal, frame):
    server.close()
    sys.exit()

class RemoteControlNode():
    def __init__(self):
        self.server = SimpleSSLWebSocketServer("localhost", 5109, SimpleChat, "/home/t/cert.pem", "/home/t/key.pem", version=ssl.PROTOCOL_TLS)

        self.remote_msg = RemoteControl()
        #self.autoware_retmote_msg = RemoteCmd()
        self.socket_pub = rospy.Publisher('/remote_control_cmd', RemoteControl, queue_size=10)
        # self.remote_pub = rospy.Publisher("/remote_cmd", RemoteCmd, queue_size=5)

        self.sub_steer = rospy.Subscriber("/pix/steering_report", steering_report_502, self.steer_callback)  # ok
        self.sub_gear = rospy.Subscriber("/pix/gear_report", gear_report_503, self.shift_callback)     # ok 
        self.sub_vcu = rospy.Subscriber("/pix/vcu_report", vcu_report_505, self.vcu_callback)          # ok
        self.bms_report = rospy.Subscriber("/pix/bms_report", bms_report_512, self.bms_callback)  # ok

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.shift = 0
        self.turn_signal = 0
        self.steer = 0.0
        self.mode = 0
        self.dts = 0
        self.previous_ltc = 0

        self.gear = "N"
        self.velocity = 6
        self.battery = 20
        self.drive_mode = 5
        # 自动驾驶发送车辆状态到服务器
        self.update_data = {
                                "ts":0,
                                "bn":9120,
                                "g":"N",
                                "m":0,
                                "pL":85,
                                "spd":45,
                                "located":True,
                                "lng":116.261115,
                                "lat":39.948574,
                                "alt":396.9,
                                "angle":32.2,
                                "satCnt":8,
                                "ltc": 20
        }
        self.previous_ts = 0
        self.bn = 0

    
    def talker(self):
        # 服务器下发到车的数据
        global data
        global feedback_data
        t_t = 0
        while not rospy.is_shutdown():
            
            self.server.serveonce()
            try:
                # print("dsadsdsads")
                temp_json = json.loads(data)
                # print(temp_json)  #-----------------
                self.remote_msg.ts = int(temp_json['ts'])
                self.remote_msg.bn = int(temp_json['bn'])
                self.remote_msg.tr = float(temp_json['tr'])
                self.remote_msg.bk = float(temp_json['bk'])
                self.remote_msg.st = float(temp_json['st'])
                self.remote_msg.g = str(temp_json['g'])
                self.remote_msg.m = int(temp_json['m'])
                self.socket_pub.publish(self.remote_msg)
                tl = int(round(time.time() * 1000))
                # update remote cmd
                """
                self.autoware_retmote_msg.header.stamp = rospy.Time.now() 
                self.autoware_retmote_msg.vehicle_cmd.steer_cmd.steer = -int(self.remote_msg.st)
                self.autoware_retmote_msg.vehicle_cmd.accel_cmd.accel = int(self.remote_msg.tr)
                self.autoware_retmote_msg.vehicle_cmd.brake_cmd.brake = int(self.remote_msg.bk)
                """
                if(self.remote_msg.g == "N"):
                    gear = 3
                elif(self.remote_msg.g == "D"):
                    gear = 4
                elif(self.remote_msg.g == "R"):
                    gear = 2
                else:
                    gear = 3
                """
                self.autoware_retmote_msg.vehicle_cmd.gear_cmd.gear = gear
                if(self.remote_msg.m == 3):
                    self.autoware_retmote_msg.vehicle_cmd.mode = 1
                    self.autoware_remote_msg.control_mode = 1
                elif(self.remote_msg.m == 5):
                    self.autoware_retmote_msg.vehicle_cmd.mode = 2
                    self.autoware_retmote_msg.control_mode = 2
                self.remote_pub.publish(self.autoware_retmote_msg)
                """
                self.update_data['bn'] = self.bn
                self.update_data['ts'] = int(round(time.time() * 1000))
                self.update_data['g'] = self.gear
                self.update_data['pL'] = self.battery
                self.update_data['spd'] = int(self.velocity)
                self.update_data['m'] = self.drive_mode
                #ltc = int((temp_json['dts']- self.previous_ts)/2)
                #if(ltc != 0):
                #    self.previous_ltc = ltc
                self.update_data['ltc'] = self.previous_ltc 
                #self.previous_ts = int(temp_json['dts'])
                self.bn = self.bn + 1
                feedback_data = json.dumps(self.update_data)
                # print feedback_data
                #print update_data_string
            except:
                try:
                    # {u'action': 1, u'bn': 9120, u'cmd': 5}
                    temp_json = json.loads(data)
                    # print(temp_json["action"])
                    self.remote_msg.autoClean = int(temp_json["action"])
                    # print("-----------------------", int(temp_json["action"]))
                except:
                    pass

    def steer_callback(self, msg):
        self.steer = -msg.Steer_AngleActual / 6 
        # publish autoware steer msg 
    
    def vcu_callback(self, msg):
        speed = msg.Vehicle_Speed
        self.linear_velocity = speed
        self.velocity = speed *3.6

    def shift_callback(self, msg):
        gear_actual = msg.Gear_Actual
        if(gear_actual== 3):
            self.gear = "N"
        elif(gear_actual == 1):
            self.gear = "P"
        elif(gear_actual == 4):
            self.gear = "D"
        elif(gear_actual == 2):
            self.gear = "R"
        else:
            self.gear = "-"
        

    def bms_callback(self, msg):
        self.battery = msg.Battery_Soc



if __name__ == "__main__":
    rospy.init_node('remote_talker', anonymous=True)
    A = RemoteControlNode()
    A.talker()
    rospy.spin()
    #signal.signal(signal.SIGINT, close_sig_handler)
