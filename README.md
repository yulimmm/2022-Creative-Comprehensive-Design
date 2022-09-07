# 2022-capstone
roscore
sudo chmod 0666 /dev/ttyUSB0
roslaunch rplidar_ros view_rplidar.launch
rosrun rplidar_ros rplidar_scan_trial.py
rostopic echo min_distance
rostopic echo distance_result
