# Lidar-AR-System
Allows for data mobility between a LiDAR sensor and an Augmented Reality headset. For information on the Augmented Reality headset side of things, please reference [AR-Pass-Data](https://github.com/graciegumm/AR-Pass-Data).

# Creating CSV file of raw data
This system accesses a Bluecity LiDAR sensor and your own person Augmented Reality headset. To quickstart, execute the following in your terminal:
```bash
./launch.py <username> <password>
```
The username and password refer to the username and password for your access to the Bluecity API. 

The order of operations for launching this is such this script is launched first. It will begin ingesting the Lidar data. Second, launch the headset application. The headset will then connect to the computer running the launch.py script. When you are finished, you may use ctrl+c to end the script, else it will continue running. 

It will write a csv file only for the Lidar object that has been determined as the headset user. 
