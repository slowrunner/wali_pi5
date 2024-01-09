# wali_pi5
Create3 Robot WALI's Onboard Raspberry Pi5 Brain  

WALI: Wall-follower Looking for Intelligence  

![WALI: Wall-follower Looking For Intelligence](/graphics/Wali_View_Front.JPG?raw=true)

WALI is an autonomous ROS 2 robot based upon:  
- iRobot Create3  
- Raspberry Pi5  
- Oak-D-Lite RGB-Depth Camera/Processor  

WALI's Software Architecture:  
- ROS 2 wali_node  
- ROS 2 create3_ir2scan node  

- Docker Container  
  - ROS 2 Humble  

- Raspberry Pi OS 64-bit Bookworm Desktop  

- iRobot Create3 ROS 2 Humble Nodes 
  - /_internal/composite_hazard
  - /_internal/kinematics_engine
  - /_internal/mobility
  - /_internal/stasis
  - /mobility_monitor
  - /motion_control
  - /robot_state
  - /root_ble_interface
  - /static_transform
  - /system_health
  - /ui_mgr

- WalI Nodes:  
  - wali_node: Manages battery by undocking, preparing to dock, docking  
  - say_node:  Provides Text-To-Speech /say service 
  - ir2scan:   Publishes /scan containing estimated obstacle distance   
    (0.15m - 0.4m) from Create3 IR Intensity sensors  
  - odometer:  Tracks distance traveled and heading changed (use cmds/tail_life.sh for total stats)  

- WaLI LOGS:  (wali_pi5/logs/)  
  - life.log: Important events recorded with date and local time  
  - say.log:  TTS request history for the /say service  

- WaLI STARTING AND STOPPING  
  - c3ws/start_docker_r2hdp_service.sh  (kills any running r2hdp containers, then starts r2hdp docker container)  
    - docker.r2hdp service runs /home/pi/wali_pi5/configs/docker/start_docker_detached_r2hdp.sh  
    - start_docker_detached_r2hdp.sh starts docker which runs c3ws/start_wali.sh  
    - start_wali.sh launches  
      - teleop_twist_joy (which launches joy_node) 
      - odometer  
      - wali_node   
      - say_node  
