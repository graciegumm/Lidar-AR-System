import socket
import csv
from datetime import datetime
import sys
import os

def write_synced_data(meta_data, lidar_data, lidar_username, lidar_password):
    """
    Function to synchronize Meta headset and Lidar data based on timestamps
    and write the synchronized data to a CSV file.

    Args:
    - meta_data (tuple): Tuple containing Meta headset data (x, y, z).
    - lidar_data (list): List of Lidar data objects.
    - lidar_username (str): Username for Lidar API authentication.
    - lidar_password (str): Password for Lidar API authentication.

    Returns:
    - None
    """
    # Check if both Meta headset and Lidar data are available
    if meta_data is None or not lidar_data:
        print("Error: Missing Meta headset or Lidar data.")
        return

    # Define CSV filename based on current timestamp
    timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_filename = f'synchronized_data_{timestamp_str}.csv'

    # Write synchronized data to CSV file
    with open(csv_filename, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        # Write header
        csv_writer.writerow(["Timestamp", "MetaX", "MetaY", "MetaZ", 
                             "LidarID", "LidarX", "LidarY", "Width", 
                             "Length", "Rotation", "ClassType", 
                             "Height"])

        # Synchronize Meta headset and Lidar data based on timestamps
        for lidar_obj in lidar_data:
            # For each Lidar object, find the corresponding Meta headset data
            # based on the closest timestamp (within a threshold)
            closest_meta_data = None
            closest_time_diff = float('inf')
            for meta_time, meta_coords in meta_data:
                lidar_time = lidar_obj.timestamp
                time_diff = abs(meta_time - lidar_time)
                if time_diff < closest_time_diff:
                    closest_meta_data = (meta_time, *meta_coords)
                    closest_time_diff = time_diff

            # Write synchronized data to CSV row
            csv_writer.writerow([lidar_obj.timestamp] + list(closest_meta_data) + [
                lidar_obj.id,
                lidar_obj.centerX,
                lidar_obj.centerY,
                lidar_obj.width,
                lidar_obj.length,
                lidar_obj.rotation,
                lidar_obj.classType,
                lidar_obj.height
            ])

    print(f"Synchronized data written to '{csv_filename}'.")

if __name__ == '__main__':
    # Check if username and password are provided as command-line arguments
    if len(sys.argv) != 5:
        print("Usage: python write_file.py <meta_host> <meta_port> <lidar_username> <lidar_password>")
        sys.exit(1)

    # Extract command-line arguments
    meta_host = sys.argv[1]
    meta_port = int(sys.argv[2])
    lidar_username = sys.argv[3]
    lidar_password = sys.argv[4]

    # (You need to implement this part based on your `get_meta.py` script)
    meta_data = get_meta_data(meta_host, meta_port)

    # (You need to implement this part based on your `get_lidar.py` script)
    lidar_data = get_lidar_data(lidar_username, lidar_password)

    # Write synchronized data to CSV file
    write_synced_data(meta_data, lidar_data, lidar_username, lidar_password)
