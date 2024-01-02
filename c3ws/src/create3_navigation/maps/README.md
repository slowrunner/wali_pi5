# NOTES ON MAPS


** CREATED FLOOR PLAN MAP  
1) Edited house floor plan  
  - open areas white  
  - walls/appliances/obstacles black  
2) Resize to 0.05 resolution  
  - opened in gimp  
  - compute scale to:  
    Plan_outer_X_dimension / 0.05 = X pixels  
      19.248m / 0.05 = 385 pixels  

    Plan_outer_Y_dimension / 0.05  
      12.006m / 0.05 = 240 pixels  
  - Image->Scale  
    - break the aspect lock  
    - Width = X pixels  
    - Height = Y pixels  
    - Click Scale  
3) Export as PGM File  
  - File->Export As:  
    - expand Select File Type  
    - choose PGM image  
    - filename:  floorplan.map.pgm  
    - Click Export, Select Raw, Click Export  
    
4) Create matching floorplan.map.yaml file:  
```
image: floorplan.map.pgm
mode: trinary
resolution: 0.05
origin: [-7.363, -2.538, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

- +X is horizontal to the right, +Y is vertical up  
- The "origin" is the bottom left corner  
- odom {0,0} is the center of the robot on the dock,   
- so origin: [ -Corner_from_dock_X, -Corner_from_dock_Y, 0]  

5) scp the file to Create3-Wali:

- Open terminal at folder with floorplan.map.pgm and .yaml  
- scp files  
```
 scp floorplan.map.* pi@10.0.0.xxx:/home/pi/wali_pi5/c3ws/maps/
```
- copy the files to src/create3_navigation/maps/  



** PUBLISHING THE MAP  

- Usually the localization program will   
  - start the map_server and  
  - publish the transform from the robot base_link frame to the map frame  

- For my "No LIDAR Create3" I cannot use localization to publish these  
  - Create create3_navigation launch file:   
    - map_server.launch.py  
    - add to package.xml  

```
  <exec_depend>ros2launch</exec_depend>
```
  - Launch map_server to publish the map  

```
cmds/launch_map_server.sh

or

ros2 launch create3_navigation map_server.launch.py
```

  - Either launch a static_transform_publisher with the odom to map transform  

```
    cmds/pub_static_tf2_odom_map.sh  

    or

    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom map
```
  - Or tell map server to load the map (again) and it will publish a transform  

```
cmds/load_map.sh

or

ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/pi/wali_pi5/c3ws/install/create3_navigation/share/create3_navigation/maps
/floorplan.map.yaml}"

```   

** ADJUSTING for rviz2 convenience  
- dock robot and cmds/reset_pose.sh  sets odom to {0, 0, 0}  
- adjust grid size to ceramic floor tile size in main areas  (0.455m)  
- adjust grid origin to position the grid relative to the dock and {0,0} of the robot center on dock  

- fine adjust the floorplan.map.yaml origin to make distance from dock to walls correct in x,y  


 
