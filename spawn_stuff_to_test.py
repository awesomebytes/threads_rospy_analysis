#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 3/3/15

@author: sampfeiffer

example_class.py contains...
"""

# System imports
import sys
import math
import numpy as np
from copy import deepcopy

# Local imports

# ROS imports
import rospy
from actionlib import SimpleActionClient
from actionlib import SimpleActionServer
import tf.transformations

# ROS messages imports
from std_msgs.msg import Header
from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal, FibonacciResult, FibonacciFeedback
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


PUB_TOPIC = '/test_topic_threads'
CLIENT_AS = '/test_as_threads_client'
SERVER_AS = '/test_as_threads_server'
CLIENT_SRV = '/test_client_threads_service'
SERVER_SRV = '/test_server_threads_service'


class SpawnStuff():
    """This class does stuff"""

    def __init__(self, topic_publishers=0, topic_subscribers=0, action_server_clients=0, action_server_servers=0,
                 service_clients=0, service_servers=0):
        self.topic_pubs = []
        self.topic_subs = []

        self.action_server_clients = []
        self.action_server_servers = []

        self.service_clients = []
        self.service_servers = []

        rospy.loginfo("Setting up " + str(topic_publishers) + " topic publishers.")
        for i in range(topic_publishers):
            self.topic_pubs.append(rospy.Publisher(PUB_TOPIC + str(i), Header))
            self.topic_pubs[i].publish(Header())

        rospy.loginfo("Setting up " + str(topic_subscribers) + " topic subscribers.")
        for i in range(topic_subscribers):
            self.topic_subs.append(rospy.Subscriber(PUB_TOPIC + str(i), Header, self.cb_topic, callback_args=i))

        rospy.loginfo("Setting up " + str(action_server_clients) + " action_server_clients.")
        for i in range(action_server_clients):
            self.action_server_clients.append(
                SimpleActionClient(SERVER_AS + str(i), FibonacciAction)
            )

        rospy.loginfo("Setting up " + str(action_server_servers) + " action_server_servers.")
        for i in range(action_server_servers):
            self.action_server_servers.append(
                SimpleActionServer(CLIENT_AS + str(i), FibonacciAction, self.cb_as)
            )

        rospy.loginfo("Setting up " + str(service_clients) + " service_clients.")
        for i in range(service_clients):
            self.service_clients.append(
                rospy.ServiceProxy(SERVER_SRV + str(i), Empty)
            )

        rospy.loginfo("Setting up " + str(service_servers) + " service_servers.")
        for i in range(service_servers):
            self.service_servers.append(
                rospy.Service(CLIENT_SRV + str(i), Empty, self.cb_service)
            )

        rospy.loginfo("Everything setup! Now you can check how many threads are open with\n"
                      + "ls -1 /proc/`ps axf | grep spawn_stuff_to_test.py | grep -v grep | awk '{print $1}'`/task/ | wc -l")




    def run(self):
        # SPAM!
        while not rospy.is_shutdown():
            # Keep publishing on publishers
            rospy.loginfo("Publishing in topics")
            for publisher in self.topic_pubs:
                publisher.publish(Header())

            rospy.loginfo("Connecting to AS's")
            for idx, as_client in enumerate(self.action_server_clients):
                rospy.loginfo("  AS: " + str(idx))
                as_client.wait_for_server(rospy.Duration(0.5))

            # # Connect service clients,
            rospy.loginfo("Calling SRV's")
            for idx, srv_client in enumerate(self.service_clients):
                rospy.loginfo("  SRV: " + str(idx))
                try:
                    srv_client.call(EmptyRequest())
                except rospy.service.ServiceException:
                    rospy.logwarn("  the SRV was not available")

            #rospy.sleep(1.0)

    def cb_topic(self, data, cb_args):
        """
        @type data: Header
        @type cb_args: int
        """
        rospy.loginfo("Subscriber " + str(cb_args) + " topic cb")

    def cb_as(self, goal):
        """
        @type goal: FibonacciGoal
        """
        rospy.loginfo("cb_as received: " + str(goal))
        self.server_as.publish_feedback(FibonacciFeedback())
        if self.server_as.is_preempt_requested():
            self.server_as.set_aborted(FibonacciResult())
        self.server_as.set_succeeded(FibonacciResult())


    def cb_service(self, request):
        """
        @type request: EmptyRequest
        """
        rospy.loginfo("cb_service received: " + str(request))
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('SpawnStuff_node_to_test' )

    # Set up many publishers and subscribers and clients and servers
    # To really test the number of threads
    # (if there is no connection, no thread is thrown)
    node = SpawnStuff(20, 20, # topics pub, sub
                      20, 20, # action servers clients, servers
                      20, 20) # services clients, servers
    # 65 threads, a lot more when stuff connects

    node.run()



