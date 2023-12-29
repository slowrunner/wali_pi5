# Create3 Navigation

This package is "lifted" from v1.0.4 of TurtleBot4 Navigation  
(under Apache License) 

There are three modes of use:  
- mapping (SLAM)  
- localization within a map  
- navigation within a map  

Create3 Navigation combines the Create3 /odom topic with periodic /scan topics to function.  
The /scan topic us most often published by a LIDAR.  

This investigation attempts to use the seven "IR Intensity" sensors as a crude LIDAR.  
The create3_ir2scan node  
- subscribes to the /ir_intensity topic and  
- uses the create3_ir_dist.dist_ir_reading() function to convert the IR Intensities   
  to ranges between 0.150 and 0.400 meters, (-99.999 outside those distances) and  
- publish a /scan topic with the distances for the seven sensors at the corresponding angles.


Typical execution:

Run the create3_ir2scan node:  
- ros2 run create3_navigation ir2scan  
or  
- cmds/start_ir2scan.sh  

Run synchronous SLAM:  
- ros2 launch create3_navigation slam.launch.py ['slam_params_file:=path/to/sync_slam.yaml']  
or  
- start_c3nav_sync_slam.sh  

Run asynchronous SLAM:  
- ros2 launch create3_navigation slam.launch.py sync:=false ['slam_params_file:=path/to/async_slam.yaml']  
or  
- start_c3nav_async_slam.sh  

Running localization with an existing map:  
- ros2 launch create3_navigation localization.launch.py map:=/path/to/map.yaml ['slam_params_file:=path/to/localization.yaml']  

Run navigation (Nav2):  
- ros2 launch create3_navigation nav2.launch.py ['slam_params_file:=path/to/nav2.yaml']  


