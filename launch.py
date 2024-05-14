#!/usr/bin/env python3
from realtime_subscriber.Realtime_subscriber_api import BCTWSConnection
import threading
import sys
import math
import rospy, roslib
from time import time
import csv
from datetime import datetime
import socket
import os

roslib.load_manifest('amrl_msgs')
from amrl_msgs.msg import Point2D

def get_lidar_data():
    # Open Blucity stream
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

    try:
        while True:
            # Get LiDAR data
            if rospy.is_shutdown() == False:
                data = stream.get_frame()
                print('Lidar: {}'.format(data.timestamp))
                for obj in data.objects:
                    obj.rotation = obj.rotation + sensorAngle
                    obj.centerX = obj.centerX + sensorLoc.x
                    obj.centerY = -obj.centerY + sensorLoc.y
                    # Write object data to CSV 
                    rowdata = [data.timestamp, 
                                None,  # Meta data placeholders
                                None, 
                                None,
                                obj.id,
                                obj.centerX,
                                obj.centerY,
                                obj.width,
                                obj.length,
                                obj.rotation,
                                obj.classType,
                                obj.height]
                    csv_writer.writerow(rowdata)

    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    try:
        with open(".credentials") as f:
            username = f.readline().strip()
            password = f.readline().strip()
    except IOError:
        if len(sys.argv) != 3:
            print("Usage: python example.py <username> <password>")
            sys.exit(1)
        username = sys.argv[1]
        password = sys.argv[2]

    host = '0.0.0.0'  # Listen on all available interfaces
    port = 8888

    # Get the current directory
    current_directory = os.getcwd()

    # Specify the folder name
    folder_name = 'rawdata'

    # Create the folder if it doesn't exist
    folder_path = os.path.join(current_directory, folder_name)
    os.makedirs(folder_path, exist_ok=True)

    # Open CSV file for writing in the "rawdata" folder
    timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_filename = os.path.join(folder_path, f'run_{timestamp_str}.csv')
    with open(csv_filename, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        # Write header
        csv_writer.writerow(["Timestamp", "MetaX", "MetaY", "MetaZ", 
                             "LidarID", "LidarX", "LidarY", "Width", 
                             "Length", "Rotation", "ClassType", 
                             "Height"])

        # Start Lidar data thread
        lidar_thread = threading.Thread(target=get_lidar_data)
        lidar_thread.start()

        try:
            while True:
                server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server_socket.bind((host, port))
                server_socket.listen(1)

                print(f"Listening for Unity on {host}:{port}")

                client_socket, client_address = server_socket.accept()
                print(f"Accepted connection from {client_address}")

                try:
                    while True:
                        data = client_socket.recv(1024).decode('utf-8')
                        if not data:
                            break

                        # Process data from Unity
                        start = data.find('(')
                        end = data.find(')')
                        if start != -1 and end != -1:
                            coordinates_str = data[start + 1:end]
                            coordinates = [float(coord) for coord in coordinates_str.split(',')]
                            # Process the coordinates as needed
                            meta_x, meta_y, meta_z = coordinates
                            # Write Meta data to CSV 
                            rowdata = [datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f"), 
                                        meta_x, 
                                        meta_y, 
                                        meta_z,
                                        None,  # Lidar data placeholders
                                        None,
                                        None,
                                        None,
                                        None,
                                        None,
                                        None,
                                        None]
                            csv_writer.writerow(rowdata)
                            print('Meta: {}'.format(datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")))

                except Exception as e:
                    print(f"Error: {e}")
                finally:
                    client_socket.close()
                    server_socket.close()

        except Exception as e:
            print(f"Error: {e}")
        finally:
            lidar_thread.join()
