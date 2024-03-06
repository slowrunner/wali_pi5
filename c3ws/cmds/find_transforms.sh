#!/bin/bash
# ubuntu@WALIDESK:~/wali_desk/c3ws$ cmds/view_frames.sh 

# ros2 run tf2_ros tf2_echo parent child

# oak: 
#  parent: 'oak-d-base-frame'
ros2 run tf2_ros tf2_echo oak oak-d-base-frame

# oak_imu_frame: 
#  parent: 'oak'
ros2 run tf2_ros tf2_echo oak oak_imu_frame


# oak_left_camera_frame: 
#  parent: 'oak'
ros2 run tf2_ros tf2_echo oak oak_camera_frame

# oak_left_camera_optical_frame: 
#  parent: 'oak_left_camera_frame'
ros2 run tf2_ros tf2_echo oak_left_camera_frame  oak_left_camera_optical_frame

# oak_model_origin: 
#  parent: 'oak'
ros2 run tf2_ros tf2_echo oak oak_model_origin


# oak_rgb_camera_frame: 
#  parent: 'oak'

# oak_rgb_camera_optical_frame: 
#  parent: 'oak_rgb_camera_frame'

# oak_right_camera_frame: 
#  parent: 'oak'

# oak_right_camera_optical_frame: 
#  parent: 'oak_right_camera_frame'

# odom: 
#  parent: 'map'

# cliff_side_left: 
#  parent: 'base_link'

# base_link: 
#  parent: 'odom'

# cliff_front_left: 
#  parent: 'base_link'

# cliff_front_right: 
#  parent: 'base_link'

# cliff_side_right: 
# parent: 'base_link'

# bump_left: 
# parent: 'base_link'

# bump_front_left: 
# parent: 'base_link'

# bump_front_center: 
# parent: 'base_link'

# bump_front_right: 
# parent: 'base_link'

# bump_right: 
# parent: 'base_link'

# ir_intensity_side_left: 
# parent: 'base_link'

# ir_intensity_left: 
# parent: 'base_link'

# ir_intensity_front_left: 
# parent: 'base_link'

# ir_intensity_front_center_left: 
# parent: 'base_link'

# ir_intensity_front_center_right: 
# parent: 'base_link'

# ir_intensity_front_right: 
# parent: 'base_link'

# ir_intensity_right: 
# parent: 'base_link'

# ir_omni: 
# parent: 'base_link'

# mouse: 
# parent: 'base_link'

# left_wheel: 
# parent: 'base_link'

# right_wheel: 
# parent: 'base_link'

# button_1: 
# parent: 'base_link'

# button_2: 
# parent: 'base_link'

# button_power: 
# parent: 'base_link'

# imu: 
# parent: 'base_link'

# base_footprint: 
# parent: 'odom'
