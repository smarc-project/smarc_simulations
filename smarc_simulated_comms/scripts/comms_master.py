#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-04-11

import time
import rospy
import random
import math

from gazebo_msgs.msg import ModelStates
from smarc_msgs.msg import CommsMessage

MAX_DIST = 20
RESCAN_PERIOD = 10


class CommsMaster:
    def __init__(self):
        # know all the positions of the models
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._CB_model_states)
        self.model_states = None

        # listen to incoming packets
        rospy.Subscriber('/comms/inbound', CommsMessage, self._CB_packet_received)


        # a list of known communicators. 
        # there should be 'namespace:publisher' pairs in here
        self.known_agents = {}
        self._scan_topics()

        global MAX_DIST
        self.MAX_DIST = MAX_DIST
        global RESCAN_PERIOD
        self.RESCAN_PERIOD = RESCAN_PERIOD
        self._last_update = 0


    def update(self):
        if rospy.Time.now() - self._last_update > self.RESCAN_PERIOD:
            self._scan_topics()
            self._last_update = rospy.Time.now()

    def _scan_topics(self):
        """
        this looks at all topics and tries to find lolos and sams
        """
        target_names = ['lolo', 'sam']
        all_topics = rospy.get_published_topics()
        for name in target_names:
            for topic, topic_type in all_topics:
                # get_published_topics returns [ ['/topic/of/something', 'type of that topic], ... ]
                # get the structure, ex: 'lolo_auv_0'
                found_ns = topic.split('/')[1]
                # get the name of the namespace without the numbers and stuff
                # ex 'lolo'
                parts = found_ns.split('_')
                if parts[0] == name:
                    # create a publisher for that agent
                    # this is slightly inefficient as it will do this multiple times per auv
                    if self.known_agents.get(name) is None:
                        # no need to spam publishers
                        self.known_agents[found_ns] = rospy.Publisher(found_ns+'/comms_inbound', CommsMessage, queue_size=1)




    def _CB_model_states(self, data):
        self.model_states = {'names':data.name, 'positions':data.pose}


    def _get_dist(self, source_ns, target_ns):
        source_idx = None
        target_idx = None
        for i,name in enumerate(self.model_states['names']):
            if name == source_ns:
                source_idx = i
            if name == target_ns:
                target_idx = i

        if source_idx is None or target_idx is None:
            rospy.logwarn('[COMMS] Attempted to _get_dist but an idx is still None!')
            return math.inf

        source_pose = self.model_states['positions'][source_idx]
        target_pose = self.model_states['positions'][target_idx]
        dx = source_pose.position.x - target_pose.position.x
        dy = source_pose.position.y - target_pose.position.y
        dz = source_pose.position.z - target_pose.position.z

        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        return dist


    def _relay(self, comms_msg, target_ns=None):
        """
        simply get the created publisher from known agents and publish to its inbound topic
        only relays if distance is close enough
        """
        if target_ns is None:
            target_ns = comms_msg.target_ns

        if self._get_dist(comms_msg.source_ns, target_ns) <= self.MAX_DIST:
            try:
                publisher = self.known_agents[target_ns]
            except:
                rospy.logwarn('[COMMS] Could not find target_ns in known_agents, message not delivered!')
            publisher.publish(comms_msg)
            return True
        return False


    def _CB_packet_received(self, comms_msg):
        if comms_msg.target_ns == 'broadcast':
            # we need to check distances to all known agents and relay to the ones in range
            for a_target_ns in self.known_agents.keys():
                self._relay(comms_msg, target_ns=a_target_ns)
        else:
                self._relay(comms_msg)


if __name__=='__main__':
    rospy.init_node('comms_master', anonymous=True)
    rate = rospy.Rate(60)
    comms = CommsMaster()
    while not rospy.is_shutdown():
        comms.update()
        rate.sleep()
