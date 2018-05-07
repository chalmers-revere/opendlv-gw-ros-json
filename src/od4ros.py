#!/usr/bin/env python
# Copyright (C) 2018 Ola Benderius

import roslib
import rospy

from std_msgs.msg import String
from ctypes import CDLL

pub = rospy.Publisher('fromOpendlv', String, queue_size=10)

def dataToOd4(data):
    # Return to OD4 (C++) part.
    return data

def dataFromOd4(data):
    global pub
    pub.publish(data)

def startRos(cid):
    print('Python: setting up ROS node.')

    od4ros = CDLL('/usr/lib/libod4ros.so')
    od4ros.start(cid)

    rospy.Subscriber('toOpendlv', String, dataToOd4)
    rospy.init_node('opendlvJson')
    rospy.spin()
