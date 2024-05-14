import threading
import time
from get_meta import get_meta_data
from get_lidar import get_lidar_data
import sys

def meta_thread_func():
    # Implement Meta data thread function
    pass

def lidar_thread_func(username, password):
    # Implement Lidar data thread function
    pass

def main(username, password):
    # Start Meta data thread
    meta_thread = threading.Thread(target=meta_thread_func)
    meta_thread.start()

    # Start Lidar data thread
    lidar_thread = threading.Thread(target=lidar_thread_func, args=(username, password))
    lidar_thread.start()

    try:
        while True:
            # Print the lengths of received data periodically
            time.sleep(5)  # Adjust as needed
            print("Meta data length:", len(meta_data_list))
            print("Lidar data length:", len(lidar_data_list))
    except KeyboardInterrupt:
        print("\nExiting...")

        # Join threads
        meta_thread.join()
        lidar_thread.join()

        # Print final lengths of received data
        print("Final Meta data length:", len(meta_data_list))
        print("Final Lidar data length:", len(lidar_data_list))

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: ./quickstart.py <username> <password>")
        sys.exit(1)

    username = sys.argv[1]
    password = sys.argv[2]
    
    main(username, password)
