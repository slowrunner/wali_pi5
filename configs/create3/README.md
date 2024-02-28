# Create3-WaLI Specific Configuration 

With H2.5 released 2/27/24 parameters to reduce CPU and RAM usage added

```
motion_control:
  ros__parameters:
    # safety_override options are 
    # "none" - standard safety profile, robot cannot backup more than an inch because of lack of cliff protection in rear, max speed 0.306m/s
    # "backup_only" - allow backup without cliff safety, but keep cliff safety forward and max speed at 0.306m/s
    # "full" - no cliff safety, robot will ignore cliffs and set max speed to 0.46m/s
    safety_override: "backup_only"

robot_state:
  ros__parameters:
    # publish_hazard_msgs: default true, set false to noticeably reduce CPU and RAM useage
    publish_hazard_msgs: false
    # publish_odom_tfs: default true, if not needed will noticeably reduce CPU and RAM use
    publish_odom_tfs: true 
    # raw_kinematics_min_pub_period_ms: default 25 , -1 disables /imu, /mouse, /wheel_status, /wheel_ticks, /wheel_vels (-1 recommended)
    raw_kinematics_min_pub_period_ms: -1
    # hazards_pub_fixed_period_ms: default -1 disables fixed period hazard publication, pubs only when change noted, 1: publish as soon as possible, 2+ period for constant publishing 

```
With wali_node, odometer, say, joy_twist:

```
Feb 27 23:51:11 Create3-WaLi user.notice create-platform: [INFO] [1709077871.792409044] [system_health]: CPU usage: max 86 [%] mean 53 [%] RAM usage: 32/59 [MB]
Feb 27 23:52:11 Create3-WaLi user.notice create-platform: [INFO] [1709077931.793251942] [system_health]: CPU usage: max 66 [%] mean 47 [%] RAM usage: 32/59 [MB]
Feb 27 23:53:12 Create3-WaLi user.notice create-platform: [INFO] [1709077992.004143144] [system_health]: CPU usage: max 64 [%] mean 48 [%] RAM usage: 32/59 [MB]
Feb 27 23:54:11 Create3-WaLi user.notice create-platform: [INFO] [1709078051.800405317] [system_health]: CPU usage: max 63 [%] mean 49 [%] RAM usage: 32/59 [MB]
Feb 27 23:55:11 Create3-WaLi user.notice create-platform: [INFO] [1709078111.800072144] [system_health]: CPU usage: max 62 [%] mean 52 [%] RAM usage: 32/59 [MB]
Feb 27 23:56:11 Create3-WaLi user.notice create-platform: [INFO] [1709078171.796043551] [system_health]: CPU usage: max 62 [%] mean 46 [%] RAM usage: 32/59 [MB]

```
Formerly Under H2.4:

```
Jan  9 18:45:08 Create3-WaLi user.notice create-platform: [INFO] [1704825908.404907699] [system_health]: CPU usage: max 72 [%] mean 59 [%] RAM usage: 37/59 [MB]
Jan  9 18:46:08 Create3-WaLi user.notice create-platform: [INFO] [1704825968.405106467] [system_health]: CPU usage: max 72 [%] mean 58 [%] RAM usage: 37/59 [MB]
Jan  9 18:47:08 Create3-WaLi user.notice create-platform: [INFO] [1704826028.406004878] [system_health]: CPU usage: max 74 [%] mean 65 [%] RAM usage: 37/59 [MB]
Jan  9 18:48:08 Create3-WaLi user.notice create-platform: [INFO] [1704826088.411578423] [system_health]: CPU usage: max 76 [%] mean 65 [%] RAM usage: 37/59 [MB]
Jan  9 18:49:08 Create3-WaLi user.notice create-platform: [INFO] [1704826148.403166359] [system_health]: CPU usage: max 75 [%] mean 62 [%] RAM usage: 37/59 [MB]
Jan  9 18:50:08 Create3-WaLi user.notice create-platform: [INFO] [1704826208.405032324] [system_health]: CPU usage: max 70 [%] mean 56 [%] RAM usage: 37/59 [MB]

```

