#! /usr/bin/env python
import rospy
from pix_driver_msgs.msg import acu_sweepctrlcmd_107
from remote_control.msg import RemoteControl

class Indefinite_frequency:
    def __init__(self):
        self.acu_sweepctrlcmd_AutoCleaningStartCtrl = 0
        self.acu_sweepctrlcmd_msg = acu_sweepctrlcmd_107()

        self.pub_acu_sweepctrlcmd = rospy.Publisher('/pix/acu_sweepctrlcmd', acu_sweepctrlcmd_107, queue_size=10)
        self.sub_remote_command = rospy.Subscriber('/remote_control_cmd', RemoteControl, self.remote_control_callback)

    def remote_control_callback(self, msg):
        self.acu_sweepctrlcmd_AutoCleaningStartCtrl = msg.autoClean
        # self.pub_acu_sweepctrlcmd.publish(self.acu_sweepctrlcmd_msg)

    def func_acu_sweepctrlcmd(self):
        if self.acu_sweepctrlcmd_AutoCleaningStartCtrl != 0:
            stamp = rospy.Time.now()
            self.acu_sweepctrlcmd_msg.header.stamp = stamp
            self.acu_sweepctrlcmd_msg.AutoCleaningStartCtrl = 1
            self.pub_acu_sweepctrlcmd.publish(self.acu_sweepctrlcmd_msg)
        

if __name__ == '__main__':
    rospy.init_node('pix_Indefinite_frequency', anonymous=True)
    converter = Indefinite_frequency()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        converter.func_acu_sweepctrlcmd()
        rate.sleep()
    rospy.spin()