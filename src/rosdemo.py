#!/usr/bin/env python
# Copyright (C) 2018 Ola Benderius

import json

import roslib
import rospy

from std_msgs.msg import String
   
def dataFromOpendlv(data):
  parsed = json.loads(data.data)
  print(json.dumps(parsed, indent=2))

def start():
  pub = rospy.Publisher('toOpendlv', String, queue_size = 10)
  rospy.init_node('rosdemo', anonymous=True)
  rospy.Subscriber('fromOpendlv', String, dataFromOpendlv)
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    messageId = 19;
    message = '''{"opendlv_proxy_GeodeticWgs84Reading": {
    "latitude" : 11.94745,
    "longitude" : 57.70906
  }
}'''

    data = str(messageId) + "," + message;

    pub.publish(data)
    rate.sleep()

if __name__ == '__main__':
  try:
    start()
  except rospy.ROSInterruptException:
    pass
