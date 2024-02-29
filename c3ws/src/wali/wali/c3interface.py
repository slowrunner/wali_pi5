#!/usr/bin/env python3

# FILE: c3interface.py

"""
    Purpose: Single Node Interface to a Create3 robot
    Context: RMW middleware message traffic imposes a load on the Create3 which can
             overwhelm the Create3 processors causing delayed message handling.
             (This is especially noticeable in a system where many of
              the nodes have minimal or even no interaction with the Create3)
    Methodology:
    - c3interface subscribes to needed Create3 topics, services, actions
      - republishes the topic msgs with added namespace
    - c3interface subscribes to needed /namespace/c3topics, /namespace/c3service, /namespace/c3action
      - republishes the topic msg without the namespace
    - RMW middleware must be configured to shield all namespaces topics 
      from the Create3 usb0 network connection
    - c3Interface should be running before all other nodes

    Requires:

    Create3 published Topics / Types Handled:  (irobot_create_msgs/msg/ if not specific)
    - /odom                  nav_msgs/msg/Odometry
    - /dock_state            DockStatus
    - /battery_state         BatteryState
    - /ir_intensity          IrIntensity IrIntensityVector
    - /hazard_detection      HazardDetection HazardDetectionVector
    - /ir_opcode             IrOpcode
    - /kidnap_status         KidnapStatus
    - /imu                   sensor_msgs/msg/Imu
    - /stop_status           StopStatus
    - /cmd_vel               Twist

    Create3 published Topics that require special listener/broadcasters
    REF: https://github.com/ros2/geometry2/tree/rolling/tf2_ros/src
    - /tf                    tf2_msgs/msg/TFMessage
    - /tf_static             tf2_msgs/msg/TFMessage


    Create3 Services Handled:
    - /robot_power           RobotPower

    Create3 action servers Handled:
    - /dock                  Dock
    - /undock                Undock
    - /reset_pose            ResetPose
    - /rotate angle          RotateAngle
    - /navigate_to_position  NavigateToPosition
    - /wall_follow           WallFollow
    - /drive_distance        DriveDistance
"""

import rclpy
import math
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.time import Time

# Topics
from sensor_msgs.msg import BatteryState       # percentage: 0.0 - 1.0
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import IrOpcode
from irobot_create_msgs.msg import KidnapStatus
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from irobot_create_msgs.msg import StopStatus
from geometry_msgs.msg import Twist

# Services

# Actions
from irobot_create_msgs.action import Undock      # no parms, result: is_docked: true,false
from irobot_create_msgs.msg import DockStatus  # is_docked, dock_visible: true,false 
#from irobot_create_msgs.action import DockServo  # (galactic)
from irobot_create_msgs.action import Dock
from irobot_create_msgs.action import RotateAngle  # angle: float32, max_rotation_speed: float32 (1.9r/s), result: pose, Feedback: remaining_angle_travel: float32


import sys
import traceback
import logging
import datetime as dt

DEBUG = False
# Uncomment for debug prints to console, run kill_docker-r2hdp.sh, then ./run_docker_r2hdp.sh to see console msgs
# DEBUG = True

NAMESPACE="wali"
C3IF_LOGFILE="/home/pi/wali_pi5/logs/c3interface.log"

class C3Interface(Node):

  def __init__(self):
    super().__init__('c3interface')

    self.c3IfLog = logging.getLogger(__name__)
    self.c3IfLog.setLevel(logging.INFO)

    self.loghandler = logging.FileHandler(C3IF_LOGFILE)
    self.logformatter = logging.Formatter('%(asctime)s|%(filename)s| %(message)s',"%Y-%m-%d %H:%M")
    self.loghandler.setFormatter(self.logformatter)
    self.c3IfLog.addHandler(self.loghandler)

    printMsg = '** c3interface node started **'
    self.c3IfLog.info(printMsg)

    # *** battery_state topic ***
    self.batteryStateSub = self.create_subscription(
      BatteryState,
      'battery_state',
      self.battery_state_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '*** /battery_state subscriber created'
      self.c3IfLog.info(printMsg)

    self.batteryStatePub = self.create_publisher(
      BatteryState,
      NAMESPACE+'/battery_state',
      qos_profile_sensor_data)

    # *** dock_status topic ***
    self.dockStatusSub = self.create_subscription(
      DockStatus,
      'dock_status',
      self.dock_status_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '*** /dock_status subscriber created'
      self.c3IfLog.info(printMsg)

    self.dockStatusPub = self.create_publisher(
      DockStatus,
      NAMESPACE+'/dock_status',
      qos_profile_sensor_data)

    # *** odom topic ***
    self.odomSub = self.create_subscription(
      Odometry,
      'odom',
      self.odom_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '*** /odom subscriber created'
      self.c3IfLog.info(printMsg)

    self.odomPub = self.create_publisher(
      Odometry,
      NAMESPACE+'/odom',
      qos_profile_sensor_data)

    # *** ir_intensity topic ***
    self.irIntensitySub = self.create_subscription(
      IrIntensityVector,
      'ir_intensity',
      self.irIntensity_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '*** /ir_intensity subscriber created'
      self.c3IfLog.info(printMsg)

    self.irIntensityPub = self.create_publisher(
      IrIntensityVector,
      NAMESPACE+'/ir_intensity',
      qos_profile_sensor_data)

    # *** hazard_detection topic ***
    self.hazardDetectionSub = self.create_subscription(
      HazardDetectionVector,
      'hazard_detection',
      self.hazardDetection_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '*** /hazard_detection subscriber created'
      self.c3IfLog.info(printMsg)

    self.hazardDetectionPub = self.create_publisher(
      HazardDetectionVector,
      NAMESPACE+'/hazard_detection',
      qos_profile_sensor_data)

    # *** ir_opcode topic ***
    self.irOpcodeSub = self.create_subscription(
      IrOpcode,
      'ir_opcode',
      self.irOpcode_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '*** /ir_opcode subscriber created'
      self.c3IfLog.info(printMsg)

    self.irOpcodePub = self.create_publisher(
      IrOpcode,
      NAMESPACE+'/ir_opcode',
      qos_profile_sensor_data)

    # *** kidnap_status topic ***
    self.kidnapStatusSub = self.create_subscription(
      KidnapStatus,
      'kidnap_status',
      self.kidnapStatus_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '*** /kidnap_status subscriber created'
      self.c3IfLog.info(printMsg)

    self.kidnapStatusPub = self.create_publisher(
      KidnapStatus,
      NAMESPACE+'/kidnap_status',
      qos_profile_sensor_data)

    # *** imu topic ***
    self.imuSub = self.create_subscription(
      Imu,
      'imu',
      self.imu_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '*** /imu subscriber created'
      self.c3IfLog.info(printMsg)

    self.imuPub = self.create_publisher(
      Imu,
      NAMESPACE+'/imu',
      qos_profile_sensor_data)

    # *** stop_status topic ***
    self.stopStatusSub = self.create_subscription(
      StopStatus,
      'stop_status',
      self.stopStatus_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '*** /stop_status subscriber created'
      self.c3IfLog.info(printMsg)

    self.stopStatusPub = self.create_publisher(
      StopStatus,
      NAMESPACE+'/stop_status',
      qos_profile_sensor_data)

    # *** NameSpace/cmd_vel topic ***
    self.cmdVelSub = self.create_subscription(
      Twist,
      NAMESPACE+'/cmd_vel',
      self.cmdVel_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '*** /cmd_vel subscriber created'
      self.c3IfLog.info(printMsg)

    self.cmdVelPub = self.create_publisher(
      Twist,
      'cmd_vel',
      qos_profile_sensor_data)


    # *** undock action ***
    self._undock_action_client = ActionClient(self, Undock, 'undock')
    self._dock_action_client = ActionClient(self, Dock, 'dock')  # the "dock" action server requires a Dock.action msg
    self._rotate_angle_action_client = ActionClient(self, RotateAngle, 'rotate_angle')
    # *********** END __init__ ******


  # *** TOPIC CALLBACKS ***
  def battery_state_sub_callback(self,battery_state_msg):
    self.batteryStatePub.publish(battery_state_msg)
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "battery_state_sub_callback()"
      self.c3IfLog.info(printMsg)

  def dock_status_sub_callback(self,dock_status_msg):
    self.dockStatusPub.publish(dock_status_msg)
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "dock_status_sub_callback()"
      self.c3IfLog.info(printMsg)

  def odom_sub_callback(self,msg):
    self.odomPub.publish(msg)
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "odom_sub_callback()"
      self.c3IfLog.info(printMsg)

  def irIntensity_sub_callback(self,msg):
    self.irIntensityPub.publish(msg)
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "irIntensity_sub_callback()"
      self.c3IfLog.info(printMsg)

  def hazardDetection_sub_callback(self,msg):
    self.hazardDetectionPub.publish(msg)
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "hazardDetection_sub_callback()"
      self.c3IfLog.info(printMsg)

  def irOpcode_sub_callback(self,msg):
    self.irOpcodePub.publish(msg)
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "irOpcode_sub_callback()"
      self.c3IfLog.info(printMsg)

  def kidnapStatus_sub_callback(self,msg):
    self.kidnapStatusPub.publish(msg)
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "kidnapStatus_sub_callback()"
      self.c3IfLog.info(printMsg)

  def imu_sub_callback(self,msg):
    self.imuPub.publish(msg)
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "imu_sub_callback()"
      self.c3IfLog.info(printMsg)

  def stopStatus_sub_callback(self,msg):
    self.stopStatusPub.publish(msg)
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "stopSttus_sub_callback()"
      self.c3IfLog.info(printMsg)

  def cmdVel_sub_callback(self,msg):
    self.cmdVelPub.publish(msg)
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "cmdVel_sub_callback()"
      self.c3IfLog.info(printMsg)

  # *** SERVICES ***


  # *** ACTIONS ***
  def undock_action_send_goal(self):
    undock_msg = Undock.Goal()
    if DEBUG:
      printMsg = "undock_action_send_goal()"
      self.c3IfLog.info(printMsg)
    self._undock_action_client.wait_for_server()
    self._undock_action_send_goal_future = self._undock_action_client.send_goal_async(undock_msg)
    self._undock_action_send_goal_future.add_done_callback(self.undock_goal_response_callback)

  def undock_goal_response_callback(self, future):
    goal_handle = future.result()
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "undock_goal_response_callback()"
      self.c3IfLog.info(printMsg)

    self._get_undock_result_future = goal_handle.get_result_async()
    self._get_undock_result_future.add_done_callback(self.get_undock_result_callback)

  def get_undock_result_callback(self, future):
    result = future.result().result
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "get_undock_result_callback()"
      self.c3IfLog.info(printMsg)


  def dock_action_send_goal(self):
    dock_msg = Dock.Goal()
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "dock_action_send_goal()"
      c3IfLog.info(printMsg)
    self._dock_action_client.wait_for_server()
    if DEBUG:
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.c3IfLog.info("dock_action_send_goal(): After wait_for_server")
    self._dock_action_send_goal_future = self._dock_action_client.send_goal_async(dock_msg)
    if DEBUG:
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.c3IfLog.info("dock_action_send_goal(): after set future")
    self._dock_action_send_goal_future.add_done_callback(self.dock_goal_response_callback)
    if DEBUG:
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.c3IfLog.info("dock_action_send_goal(): after add_done_callback")

  def dock_goal_response_callback(self, future):
    goal_handle = future.result()
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "dock_goal_response_callback()"
      self.c3IfLog.info(printMsg)

    printMsg = 'dock Goal Accepted'
    if DEBUG:
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.c3IfLog.info(printMsg)

    self._get_dock_result_future = goal_handle.get_result_async()
    self._get_dock_result_future.add_done_callback(self.get_dock_result_callback)

  def get_dock_result_callback(self, future):
    result = future.result().result
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "get_dock_result_callback()"
      self.c3IfLog.info(printMsg)


  def rotate_angle_action_send_goal(self,angle):
    rotate_angle_msg = RotateAngle.Goal()
    rotate_angle_msg.angle = angle

    if DEBUG:
      printMsg = "rotate_angle_action_send_goal"
      self.c3IfLog.info(printMsg)
    self._rotate_angle_action_client.wait_for_server()
    self._rotate_angle_action_send_goal_future = self._rotate_angle_action_client.send_goal_async(rotate_angle_msg)
    self._rotate_angle_action_send_goal_future.add_done_callback(self.rotate_angle_goal_response_callback)

  def rotate_angle_goal_response_callback(self, future):
    goal_handle = future.result()
    if DEBUG:
      printMsg = "rotate_angle_goal_response_callback()"
      self.c3IfLog.info(printMsg)

    if not goal_handle.accepted:
      self.c3IfLog.info('rotate Goal Rejected :(')
      return

    if DEBUG:
        self.c3IfLog.info('rotate Goal Accepted :)')

    self._get_rotate_angle_result_future = goal_handle.get_result_async()
    self._get_rotate_angle_result_future.add_done_callback(self.get_rotate_angle_result_callback)

  def get_rotate_angle_result_callback(self, future):
    result = future.result().result
    status = future.result().status
    if DEBUG:
      printMsg = "get_rotate_angle_result_callback()"
      self.c3IfLog.info(printMsg)


def main():
  rclpy.init(args=None)
  c3interface = C3Interface()
  try:
    rclpy.spin(c3interface)
  except KeyboardInterrupt:
    pass
  except ExternalShutdownException:
    sys.exit(0)
  finally:
    c3interface.destroy_node()
    try:
      rclpy.try_shutdown()
    except:
      pass


if __name__ == '__main__':
    main()
