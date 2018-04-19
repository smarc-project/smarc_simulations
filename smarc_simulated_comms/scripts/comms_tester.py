#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-04-13

import rospy
import time
from smarc_msgs.msg import CommsMessage

class DummyPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/comms/inbound', CommsMessage, queue_size=1)

    def send(self, source, target, data):
        msg = CommsMessage()
        msg.source_ns = source
        msg.target_ns = target
        msg.data = str(data)
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('comms_tester',anonymous=True)
    rate = rospy.Rate(3)

    pubs = DummyPublisher()

    time.sleep(1)
    pubs.send('lolo_auv_0','lolo_auv_1','0to1')
    time.sleep(1)
    pubs.send('lolo_auv_0','broadcast','0toAll')
    time.sleep(1)

    while True:
        pubs.send('lolo_auv_0','lolo_auv_1','time is now'+str(rospy.Time.now()))
        rate.sleep()
