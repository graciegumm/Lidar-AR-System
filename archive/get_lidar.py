#!/usr/bin/env python3
import sys
import math
from realtime_subscriber.Realtime_subscriber_api import BCTWSConnection
from amrl_msgs.msg import Point2D
import rospy, roslib  # MODIFIED: added rosbag


def get_lidar_data(username, password):
    """
    Function to connect to the Lidar API, retrieve data, process it,
    and return the relevant information.

    Args:
    - username (str): Username for authentication.
    - password (str): Password for authentication.

    Returns:
    - list: List of Lidar data objects, where each object contains relevant information.
    """
    lidar_data = []

    try:
        # Open Blucity stream
        print("Opening Blucity stream...")
        stream = BCTWSConnection(
            "BCT_3D_4G_0206001",
            username,
            password,
            singleton=False,
            subscriptions=[
                BCTWSConnection.subscriptionOption.LOOP_CHANGE,
                BCTWSConnection.subscriptionOption.PHASE_CHANGE,
                BCTWSConnection.subscriptionOption.FRAME])
        print("Stream opened")

        sensorLoc = Point2D()
        sensorLoc.x = 86
        sensorLoc.y = -120
        sensorAngle = math.radians(5)

        # Retrieve Lidar data
        while not rospy.is_shutdown():
            data = stream.get_frame()
            # Process Lidar data
            for obj in data.objects:
                obj.rotation = obj.rotation + sensorAngle
                obj.centerX = obj.centerX + sensorLoc.x
                obj.centerY = -obj.centerY + sensorLoc.y
                # Write object data to CSV 
                row_data = [
                    data.timestamp, 
                    obj.id,
                    obj.centerX,
                    obj.centerY,
                    obj.width,
                    obj.length,
                    obj.rotation,
                    obj.classType,
                    obj.height
                ]
                return row_data

    except Exception as e:
        print(f"Error: {e}")

    return None

if __name__ == '__main__':
    # Check if username and password are provided as command-line arguments
    if len(sys.argv) != 3:
        print("Usage: python get_lidar.py <username> <password>")
        sys.exit(1)

    username = sys.argv[1]
    password = sys.argv[2]

    # Call function to get Lidar data
    row_data = get_lidar_data(username, password)
    if row_data:
        print("Lidar data:", row_data)
