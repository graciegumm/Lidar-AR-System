#!/usr/bin/env python3
from realtime_subscriber.Realtime_subscriber_api import BCTWSConnection
import threading
import sys
import math
import rospy, roslib
import socket
import queue
import numpy as np
import time
from datetime import datetime
from scipy.spatial import procrustes
import csv

roslib.load_manifest('amrl_msgs')
from amrl_msgs.msg import Point2D

data_arrays = {}  # Global variable to store np arrays for each object id

def get_lidar_data():
    global data_arrays  # Declare data_arrays as global
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

                # Pop the most recent metadata item from the queue
                try:
                    meta_x, meta_y, meta_z, timestamp_meta = metadata_queue.get_nowait()
                except queue.Empty:
                    # If the queue is empty, no recent metadata available
                    meta_x, meta_y, meta_z, timestamp_meta = None, None, None, None
                # print('Lidar: {} \n Meta: {}'.format(data.timestamp, timestamp_meta))
                row_metadata = [timestamp_meta, meta_x, meta_y, meta_z]
                for obj in data.objects:
                    obj.rotation = obj.rotation + sensorAngle
                    obj.centerX = obj.centerX + sensorLoc.x
                    obj.centerY = -obj.centerY + sensorLoc.y

                    # Check if closest metadata exists
                    if timestamp_meta is not None:
                        # Generate array name based on obj.id
                        array_name = f"id{obj.id}"
                        # Check if the array for this obj.id exists, if not, create it
                        if array_name not in data_arrays:
                            data_arrays[array_name] = np.empty((0, 13), dtype=np.float64)  # Assuming 13 columns for the data
                        # Append rowdata to the respective array
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
                        data_arrays[array_name] = np.append(data_arrays[array_name], [rowdata], axis=0)

            else:
                # Stop writing to the NumPy array when rospy is shut down
                break

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

def process_arrays(arrays):
    print("Processing arrays...")
    lowest_disparity = float('inf')
    best_array_info = None
    # For demonstration purposes, let's print the shape of each array
    for array_name, array_data in arrays.items():
        if array_data.shape[0] >= 100:
            processed_array = array_data[1:]
            meta_coords = processed_array[:, [2, 4]].astype(float)
            lidar_coords = processed_array[:, [6, 7]].astype(float)
            # Apply Procrustes analysis
            mtx1, mtx2, disparity = procrustes(meta_coords, lidar_coords)
            # Store the processed array in the dictionary
            if disparity < lowest_disparity:
                lowest_disparity = disparity
                best_array_info = (mtx1, mtx2, disparity, array_name, array_data)
            # Print the first 2 rows of the processed array
            #print(processed_array[:2])
        #else:
            #print(f"Ignoring: {array_name} (less than 100 rows)")
    # Placeholder for the logic to determine the correct ID
    print("Determining correct ID...")
    if best_array_info:
        mtx1, mtx2, disparity, array_name, array_data = best_array_info
        print(f"Best Array Info: mtx1={mtx1}, mtx2={mtx2}, disparity={disparity}, id={array_name[2:]}, raw_data={array_data}")
        csv_filename = f"{array_name}.csv"  # Modify the filename as needed
        with open(csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write headers
            writer.writerow(["LidarTimestamp", "MetaTimestamp", "MetaX", "MetaY", "MetaZ", "LidarID", "LidarX", "LidarY", "width", "length", "rotation", "classType", "height"])
            # Write array_data to CSV
            writer.writerows(array_data)
    else:
        print("No array with at least 100 rows found.")
    return array_name[2:]

def monitor_array_size():
    global data_arrays
    try:
        while True:
            for array_name, array_data in data_arrays.items():
                print(f"{array_name}: {array_data.shape[0]}")
                if array_data.shape[0] >= 150:
                    process_arrays(data_arrays)
                    return  # Exit the function if one array reaches 150 rows
            time.sleep(1)  # Sleep for 1 second before checking again
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

    # Initialize the metadata queue
    metadata_queue = queue.LifoQueue()

    # Start Lidar data thread
    lidar_thread = threading.Thread(target=get_lidar_data)
    lidar_thread.start()
    
    # Start Metadata acquisition thread
    metadata_thread = threading.Thread(target=get_meta_data)
    metadata_thread.start()

    # Start a thread to monitor the size of each array
    array_monitor_thread = threading.Thread(target=monitor_array_size)
    array_monitor_thread.start()

    # Join threads
    lidar_thread.join()
    metadata_thread.join()