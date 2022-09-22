#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright 2018 Ozer Ozkahraman (ozkahramanozer@gmail.com)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
