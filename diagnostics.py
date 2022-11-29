#!/usr/bin/env python
#--*-- coding:utf-8 --*--

import rospy 
from diagnostic_msgs.msg import DiagnosticArray,KeyValue


class SubDiagnostics:
    def __init__(self):
        self.sub_diagnostic = rospy.Subscriber("/diagnostics", DiagnosticArray, self.callback_diagnostic, queue_size=10)

    def callback_diagnostic(self, msg):
        for i in msg.status:
            # if(i.name=="net_monitor: Network Usage"):
            #     # for j in i.values :
            #     #     print(j)
            #     print(i)
            #     print("------------------------------------------")
            if(i.name=="system/system_monitor/net_monitor: Network Usage"):
                # for j in i.values :
                #     print(j)
                #     break
                print(i)
                break
                

if __name__=="__main__":
    rospy.init_node("diagnostics_sub_node")
    SubDiagnostics()
    rospy.spin()