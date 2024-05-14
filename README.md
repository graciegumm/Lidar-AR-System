# Lidar-AR-System
Allows for data mobility between a LiDAR sensor and an Augmented Reality headset. 

# Creating CSV file of raw data
This system accesses a Bluecity LiDAR sensor and your own person Augmented Reality headset. To quickstart, execute 
```bash
./launch.py <username> <password>
```
The username and password refer to the username and password for your access to the Bluecity API. 

The order of operations for launching this is such this script is launched first. It will begin writing the Lidar data to a csv file. After it is launched, the headset application is launched and will connect to the computer running the launch.py script. When you are finished, you may use ctrl+c to end the script, else it will continue writing the LiDAR data. 
