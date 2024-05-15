# Lidar-AR-System
Allows for data mobility between a LiDAR sensor and an Augmented Reality headset. For information on the Augmented Reality headset side of things, please reference [AR-Pass-Data](https://github.com/graciegumm/AR-Pass-Data).

# Creating CSV file of raw data
This system accesses a Bluecity LiDAR sensor and your own person Augmented Reality headset. To quickstart, execute the following in your terminal:
```bash
./launch.py <username> <password>
```
The username and password refer to the username and password for your access to the Bluecity API. 

The order of operations for launching this is such this script is launched first. It will begin writing the Lidar data to a csv file. After it is launched, the headset application is launched and will connect to the computer running the launch.py script. When you are finished, you may use ctrl+c to end the script, else it will continue writing the LiDAR data. 

This will create a csv file saved locally in a directory called rawdata. After launch.py is terminated, it will take a bit for the csv file to write. Give it a minute or so before it pops up.
