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
        while not signal.ready:
            time.sleep(0.5)

        node.load_mission("coordinates_1.csv")
        node.arm()
        node.set_offboard_mode()
        node.mission()
        node.disarm()

        signal.ready = False

except rospy.ROSInterruptException:
    pass
