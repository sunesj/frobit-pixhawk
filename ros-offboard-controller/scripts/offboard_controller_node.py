#!/usr/bin/python

import rospy
from mavros_msgs.msg import RCIn
from offboard_controller import OffBoardController
import time


class WaitingClass:
    def __init__(self):
        self.ready = False
        self.sub = rospy.Subscriber("/mavros/rc/in", RCIn, self.subscriber_callback)

    def subscriber_callback(self, msg):
        if msg.channels[4] > 1800:
            self.ready = True


try:
    node = OffBoardController()
    signal = WaitingClass()

    while not rospy.is_shutdown():

        rospy.loginfo('waiting for remote control ready signal')
        while not signal.ready:
            time.sleep(0.5)
        rospy.loginfo('ready to load and run offboard mission')

        node.load_mission("coordinates_1.csv")
        rospy.loginfo('Mission loaded')

        node.arm()
        rospy.loginfo('Vehicle armed')

        node.set_offboard_mode()
        rospy.loginfo('Offboard mode set')

        node.mission()
        rospy.loginfo('Mission complete')

        node.disarm()
        rospy.loginfo('disarmed')

        signal.ready = False

except rospy.ROSInterruptException:
    pass
