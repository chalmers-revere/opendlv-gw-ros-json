#!/usr/bin/python
# Copyright (C) 2018 Ola Benderius

import roslib
import rospy
import threading

import select
import socket
import sys

from std_msgs.msg import String

pub = rospy.Publisher('fromOpendlv', String, queue_size = 10)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clients = []
quitting = False
isVerbose = False

def dataFromRosToOd4(data):
    global clients, isVerbose

    for client in clients:
        try:
            if isVerbose:
                print('Python: data sent from ROS to Od4', data.data)
            dataBuffer = data.data.encode()
            client.send(dataBuffer)
        except Exception as e:
            print(e)
            pass


def dataFromOd4ToRos(pub, data):
    global isVerbose

    if isVerbose:
        print('Python: data sent from Od4 to ROS', data)
    pub.publish(data)

def listenToClient(client, address):
    global pub, quitting, isVerbose

    while not quitting:
        try:
            if isVerbose:
                print('Python: connection from', address)
            buffer = []
            while not quitting:
                data = client.recv(1024)
                if data:
                    for b in data:
                        buffer.append(b)

                    if str(buffer[-1]) == '}' and str(buffer[-2]) == '}':
                        dataFromOd4ToRos(pub, ''.join(buffer))
                        buffer = []
        except Exception as e:
            print(e)
        finally:
            client.close()


def startTcpServer():
    global sock, quitting

    tcpPort = 9000

    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('localhost', tcpPort))
    sock.listen(5)

    while not quitting:
        rr,rw,err = select.select([sock], [], [], 1)
        if rr:
            client, address = sock.accept()
            clients.append(client)
            threading.Thread(target=listenToClient, args=(client,address)).start()

def startRosNode():
    rospy.init_node('opendlvJson')
    rospy.Subscriber('toOpendlv', String, dataFromRosToOd4)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rate.sleep()

def start(verbose = False):
    global sock, quitting, isVerbose
 
    isVerbose = verbose

    threading.Thread(target=startTcpServer).start()
    startRosNode()
    
    quitting = True
    sock.close()

if __name__ == '__main__':
  try:
    start()
  except rospy.ROSInterruptException:
    pass
