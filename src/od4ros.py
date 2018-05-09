#!/usr/bin/env python
# Copyright (C) 2018 Ola Benderius

import roslib
import rospy

import socket
import sys

from std_msgs.msg import String

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
connections = []

def dataFromRosToOd4(data):
    global connections

    liveConnections = []

    for connection in connections:
        try:
            print('Python: data sent to Od4', data)
            connection.sendall(data)
            liveConnections.append(connection)
        except:
            pass
    
    connections = liveConnections


def dataFromOd4ToRos(pub, data):
    print('Python: data sent to Ros', data)
    pub.publish(data)

def start():
    global connections, sock

    tcpPort = 9000

    print('Python: starting TCP server')

    server_address = ('localhost', tcpPort)
    sock.bind(server_address)
    sock.listen(1)

    print('Python: setting up ROS node.')

    pub = rospy.Publisher('fromOpendlv', String, queue_size = 10)
    rospy.init_node('opendlvJson')
    rospy.Subscriber('toOpendlv', String, dataFromRosToOd4)

    while True:
        connection, client_address = sock.accept()
        connections.append(connection)

        try:
            print('Connection from', client_address)
            
            buffer = []
            while True:
                data = connection.recv(1024)
                for b in data:
                    buffer.append(b)

                if str(buffer[-1]) == '}' and str(buffer[-2]) == '}':
                    dataFromOd4ToRos(pub, ''.join(buffer))
                    buffer = []
        finally:
            connection.close()
