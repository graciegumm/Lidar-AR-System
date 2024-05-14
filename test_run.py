#!/usr/bin/env python3
from realtime_subscriber.Realtime_subscriber_api import BCTWSConnection
import threading
import sys
import math
import rospy, roslib  # MODIFIED: added rosbag

roslib.load_manifest('amrl_msgs')
import rospkg

from amrl_msgs.msg import Point2D

stream = None

if __name__ == '__main__':
    # Check to see if a file named ".credentials" exists in the current directory,
    # and if so, use it to log in
    try:
        with open(".credentials") as f:
            username = f.readline().strip()
            password = f.readline().strip()
    except IOError:
        # If the file doesn't exist, use the command line arguments
        # Accept username as the first argument and password as the second argument
        if len(sys.argv) != 3:
            print("Usage: python example.py <username> <password>")
            sys.exit(1)
        username = sys.argv[1]
        password = sys.argv[2]

    print("Opening Blucity stream...")
    stream = BCTWSConnection(
            "BCT_3D_4G_0206001",
            username,
            password,
            singleton=False,
            subscriptions = [
                BCTWSConnection.subscriptionOption.LOOP_CHANGE,
                BCTWSConnection.subscriptionOption.PHASE_CHANGE,
                BCTWSConnection.subscriptionOption.FRAME])
    print("Stream opened")

    sensorLoc = Point2D()
    sensorLoc.x = 86
    sensorLoc.y = -120
    sensorAngle = math.radians(5)
    # Create a visualization publisher
    while rospy.is_shutdown() == False:
        # print("Getting frame...")
        data = stream.get_frame()
        # print("Frame: ")
        #print(len(data.objects))
        print(data.timestamp)
        for obj in data.objects:
            '''print(time(), obj)'''
            obj.rotation = obj.rotation + sensorAngle
            obj.centerX = obj.centerX + sensorLoc.x
            obj.centerY = -obj.centerY + sensorLoc.y