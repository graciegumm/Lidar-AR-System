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
import queue  # Import the queue module

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

                # Pop the most recent metadata item from the queue
                try:
                    meta_x, meta_y, meta_z, timestamp_meta = metadata_queue.get_nowait()
                except queue.Empty:
                    # If the queue is empty, no recent metadata available
                    meta_x, meta_y, meta_z, timestamp_meta = None, None, None, None

                # Retrieve the most recent metadata once outside the loop
                # This metadata will be associated with all objects in data.objects
                row_metadata = [timestamp_meta, meta_x, meta_y, meta_z]
                print('Meta: {}'.format(timestamp_meta))

                for obj in data.objects:
                    obj.rotation = obj.rotation + sensorAngle
                    obj.centerX = obj.centerX + sensorLoc.x
                    obj.centerY = -obj.centerY + sensorLoc.y

                    # Check if closest metadata exists
                    if timestamp_meta is not None:
                        # Write object data to CSV with associated metadata
                        rowdata = [data.timestamp,
                                    *row_metadata,
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

def get_meta_data():
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
                        # Put metadata into the queue
                        metadata_queue.put((meta_x, meta_y, meta_z, datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")))

            except Exception as e:
                print(f"Error: {e}")
            finally:
                client_socket.close()
                server_socket.close()
    except Exception as e:
        print(f"Error: {e}")


if __name__ == '__main__':
    import sys
    try:
        # Load credentials
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
        csv_writer.writerow(["TimestampLidar", "TimestampMeta", "MetaX", "MetaY", "MetaZ", 
                             "LidarID", "LidarX", "LidarY", "Width", 
                             "Length", "Rotation", "ClassType", 
                             "Height"])

        # Initialize the metadata queue
        metadata_queue = queue.Queue()

        # Start Lidar data thread
        lidar_thread = threading.Thread(target=get_lidar_data)
        lidar_thread.start()
        
        # Start Metadata acquisition thread
        metadata_thread = threading.Thread(target=get_meta_data)
        metadata_thread.start()

        # Join threads
        lidar_thread.join()
        metadata_thread.join()