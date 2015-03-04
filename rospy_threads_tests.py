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
                SimpleActionClient(CLIENT_AS + str(i), FibonacciAction)
            )
            if not self.action_server_clients[i].wait_for_server(rospy.Duration(5.0)):
                rospy.logwarn("Failed to connect to AS: " + CLIENT_AS + str(i))

        rospy.loginfo("Setting up " + str(action_server_servers) + " action_server_servers.")
        for i in range(action_server_servers):
            self.action_server_servers.append(
                SimpleActionServer(SERVER_AS + str(i), FibonacciAction, self.cb_as)
            )

        rospy.loginfo("Setting up " + str(service_clients) + " service_clients.")
        for i in range(service_clients):
            self.service_clients.append(
                rospy.ServiceProxy(CLIENT_SRV + str(i), Empty)
            )
            self.service_clients[i].call(EmptyRequest())

        rospy.loginfo("Setting up " + str(service_servers) + " service_servers.")
        for i in range(service_servers):
            self.service_servers.append(
                rospy.Service(SERVER_SRV + str(i), Empty, self.cb_service)
            )

        rospy.loginfo("Everything setup! Now you can check how many threads are open with\n"
                      + "ls -1 /proc/`ps axf | grep rospy_threads_tests.py | grep -v grep | awk '{print $1}'`/task/ | wc -l")

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
    rospy.init_node('SpawnStuff_node' )
    rospy.sleep(1.0)

    # run spawn_stuff_to_test.py
    # before testing this, so you have stuff to connect with

###### NOTHING


    # The node starts with 5 threads having all 0's
    # just by having rospy.init_node() run
    # node = SpawnStuff(0, 0, # topics pub, sub
    #                   0, 0, # action servers clients, servers
    #                   0, 0) # services clients, servers

####### TOPICS

    # If we add a publisher and subscriber we get 7 threads,
    # one for every publisher,
    # one for every subscriber
    # WHEN THEY ARE CONNECTED
    # node = SpawnStuff(1, 1, # topics pub, sub
    #                   0, 0, # action servers clients, servers
    #                   0, 0) # services clients, servers

    # For every pub we add, we dont get more threads O.o
    # This is 5 threads
    # node = SpawnStuff(10, 0, # topics pub, sub
    #                   0, 0, # action servers clients, servers
    #                   0, 0) # services clients, servers

    # For every sub we add, we get one more thread
    # This is 10 threads
    # node = SpawnStuff(0, 5, # topics pub, sub
    #                   0, 0, # action servers clients, servers
    #                   0, 0) # services clients, servers

    # If we add both we get 1 thread per publisher and 1 per subscriber
    # This is 15 threads
    # node = SpawnStuff(5, 5, # topics pub, sub
    #                   0, 0, # action servers clients, servers
    #                   0, 0) # services clients, servers


###### ACTION SERVERS

    # If we add a action server client, we get 3 extra threads (when connected, none if not)
    # This is 8 threads
    # node = SpawnStuff(0, 0, # topics pub, sub
    #                   1, 0, # action servers clients, servers
    #                   0, 0) # services clients, servers

    # # If we add a action server server, we get 4 extra threads (2 without connection)
    # # This is 9 threads (7 threads if there is no client)
    # node = SpawnStuff(0, 0, # topics pub, sub
    #                   0, 1, # action servers clients, servers
    #                   0, 0) # services clients, servers

    # If we add one of each, we should get 3 + 4 = 7 extra threads
    # This is 12 threads (so we are correct)
    # node = SpawnStuff(0, 0, # topics pub, sub
    #                   1, 1, # action servers clients, servers
    #                   0, 0) # services clients, servers

###### SERVICES

    # For every service client we get 0 new threads
    # This is 5 threads
    # node = SpawnStuff(0, 0, # topics pub, sub
    #                   0, 0, # action servers clients, servers
    #                   10, 0) # services clients, servers

    # For every service server we get 0 new threads
    # This is 5 threads
    # node = SpawnStuff(0, 0, # topics pub, sub
    #                   0, 0, # action servers clients, servers
    #                   0, 10) # services clients, servers

    # Note that the last service with the same name to be launched is the one that
    # Will receive all the requests (the other ones won't crash but won't receive them)


##### WHAT WE LEARNT


    # Topics: 1 thread per connection (being a publisher or a subscriber)
    # Action servers:
    #   Clients: 3 thread per connection (0 not connected)
    #   Servers: 4 thread per connection (2 not connected)
    # Services: 0 threads, not for clients or servers



    rospy.spin()


