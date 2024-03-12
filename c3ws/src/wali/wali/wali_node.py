#!/usr/bin/env python3

# FILE: wali_node.py

"""
    Make WaLI A 24/7 Autonomous Robot
    - Logs WaLI_node started to life.log
    - subscribes to Create3 /battery_state and /dock_status
    - publishes /undock action goal when BatteryState.percentage full
    - Logs successful undock to life.log
    - publishes /rotate_angle {angle: 1.57} (180deg) when BatteryState.percentage < 20% 
                                                          and dock not visible
    - publishes /dock action goal when BatteryState.percentage < 0.15 and dock visible
    - Logs successful docking to life.log

    Requires:
    - /home/pi/wali_pi5/logs/life.log with 666 permission

    Message Formats:

    - irobot_create_msgs/msg/DockStatus.msg
      std_msgs/Header header  # Header stamp is when dock info was queried.
      bool dock_visible       # Whether robot sees dock in its sensors used for docking
      bool is_docked          # Whether the robot is docked

    - sensor_msgs/msg/BatteryState
      std_msgs/Header  header
      	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
      float32 voltage          # Voltage in Volts (Mandatory)
      float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
      float32 current          # Negative when discharging (A)  (If unmeasured NaN)
      float32 charge           # Current charge in Ah  (If unmeasured NaN)
      float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
      float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
      float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
      ... and more
    - irobot_create_msgs/action/Undock
      # Request
      ---
      # Result
      bool is_docked
      ---
      # Feedback

    - irobot_create_msgs/action/DockServo  (galactic)
    - irobot_create_msgs/action/Dock
      # Request
      ---
      # Result
      bool is_docked
      ---
      # Feedback
      bool sees_dock

    - irobot_create_msgs/action/RotateAngle
      # Request

      # Rotate for relative angle (radians) from current robot position.  Angles greater than 2 PI will cause the robot to rotate in multiple circles
      float32 angle
      # Max rotation speed (positive rad/s), will cap negative angle to negative speed
      float32 max_rotation_speed 1.9
      ---
      # Result
      # Pose where robot finished
      geometry_msgs/PoseStamped pose
      ---
      # Feedback
      # Remaining radians to rotate
      float32 remaining_angle_travel

"""

import rclpy
import math
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.time import Time
from sensor_msgs.msg import BatteryState       # percentage: 0.0 - 1.0
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

LIFELOGFILE = "/home/pi/wali_pi5/logs/life.log"

# "Sleep Time" Wali needs to stay on dock
START_SLEEP_TIME = dt.datetime.strptime('22:00', '%H:%M').time()
END_SLEEP_TIME = dt.datetime.strptime('08:00', '%H:%M').time()

# print('START_SLEEP_TIME: ', START_SLEEP_TIME.strftime("%H:%M") )
# print('END_SLEEP_TIME: ', END_SLEEP_TIME.strftime("%H:%M") )


# For testing
UNDOCK_AT_PERCENTAGE = 0.45 # 0.995
ROTATE_AT_PERCENTAGE = 0.40 # 0.20
DOCK_AT_PERCENTAGE   = 0.38 # 0.15

# Quote out for testing
UNDOCK_AT_PERCENTAGE = 0.995
ROTATE_AT_PERCENTAGE = 0.18
DOCK_AT_PERCENTAGE   = 0.15


def isNotSleepTime(start,end):
    time_now = dt.datetime.now().time()
    is_not_sleep_time = (time_now > end ) and (time_now < start )
    if DEBUG:
        print('Time Now: ', time_now.strftime('%H:%M:%S') )
        print('start sleep: ', start)
        print('end sleep: ', end)
        print('Sleep Time?: ', is_sleep_time)
    return is_not_sleep_time

class WaLINode(Node):

  def __init__(self):
    super().__init__('wali_node')

    self.lifeLog = logging.getLogger(__name__)
    self.lifeLog.setLevel(logging.INFO)

    self.loghandler = logging.FileHandler(LIFELOGFILE)
    self.logformatter = logging.Formatter('%(asctime)s|%(filename)s| %(message)s',"%Y-%m-%d %H:%M")
    self.loghandler.setFormatter(self.logformatter)
    self.lifeLog.addHandler(self.loghandler)

    printMsg = '** WaLI node started - Undock:{:2d}% Rotate:{:2d}% Dock:{:2d}% **'.format(int(UNDOCK_AT_PERCENTAGE * 100), int(ROTATE_AT_PERCENTAGE * 100), int(DOCK_AT_PERCENTAGE * 100) )
    print(printMsg)
    self.lifeLog.info(printMsg)


    self.sub = self.create_subscription(
      BatteryState,
      'battery_state',
      self.battery_state_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '\n*** /battery_state subscriber created'
      print(dtstr,printMsg)

    self.battery_state = BatteryState()  # None

    self.sub = self.create_subscription(
      DockStatus,
      'dock_status',
      self.dock_status_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile

    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = '\n*** /dock_status subscriber created'
      print(dtstr,printMsg)

    self.dock_status = DockStatus() # None


    self._undock_action_client = ActionClient(self, Undock, 'undock')
    # self._dock_action_client = ActionClient(self, DockServo, 'dock')  # the "dock" action server requires a DockServo.action msg
    self._dock_action_client = ActionClient(self, Dock, 'dock')  # the "dock" action server requires a Dock.action msg
    self._rotate_angle_action_client = ActionClient(self, RotateAngle, 'rotate_angle')

    period_for_timer = 60.0  # Once every 60 seconds
    self.timer = self.create_timer( period_for_timer, self.wali_main_cb)  # call the wali_node's main loop when ROS timer triggers
    if DEBUG:
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        printMsg ='wali_node: created wali_main callback for once every {:.0f} seconds'.format(period_for_timer)
        print(dtstr,printMsg)
    self.battery_percentage = -1.0
    self.state = "init"
    self.last_undock_time = dt.datetime.now()
    self.last_dock_time = dt.datetime.now()
    self.all_req_topic_cb_rx = [False, False]  # /battery_state, /dock_status

  def battery_state_sub_callback(self,battery_state_msg):
    self.battery_state = battery_state_msg
    self.battery_percentage = self.battery_state.percentage
    self.all_req_topic_cb_rx[0] = True  # /battery_state, /dock_status
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "battery_state_sub_callback(): battery_state.percentage {:.0f} %".format(100 * self.battery_percentage)
      print(dtstr,printMsg)
      # self.lifeLog.info(printMsg)

  def dock_status_sub_callback(self,dock_status_msg):
    self.dock_status = dock_status_msg
    self.all_req_topic_cb_rx[1] = True  # /battery_state, /dock_status
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "dock_status_sub_callback(): is_docked:{} dock_visible: {}".format(self.dock_status.is_docked, self.dock_status.dock_visible)
      print(dtstr,printMsg)
      # self.lifeLog.info(printMsg)

  def undock_action_send_goal(self):
    undock_msg = Undock.Goal()
    if DEBUG:
      printMsg = "undock_action_send_goal(): executing"
      print(printMsg)
    self._undock_action_client.wait_for_server()
    self._undock_action_send_goal_future = self._undock_action_client.send_goal_async(undock_msg)
    self._undock_action_send_goal_future.add_done_callback(self.undock_goal_response_callback)

  def undock_goal_response_callback(self, future):
    goal_handle = future.result()
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "undock_goal_response_callback(): Goal Accepted: {}".format(goal_handle.accepted)
      print(dtstr,printMsg)

    if not goal_handle.accepted:
      self.get_logger().info('Undock Goal Rejected :(')
      return

    self.get_logger().info('Undock Goal Accepted :)')

    self._get_undock_result_future = goal_handle.get_result_async()
    self._get_undock_result_future.add_done_callback(self.get_undock_result_callback)

  def get_undock_result_callback(self, future):
    result = future.result().result
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "get_undock_result_callback(): Undock Result is_docked {} %".format(result.is_docked)
      print(dtstr, printMsg)

    if result.is_docked:
      self.state = "docked"
      printMsg = "** WaLI Undocking: failed **"
      if DEBUG:
          dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
          print(dtstr, printMsg)
      self.lifeLog.info(printMsg)

    else:
      self.state = "undocked"
      self.last_undock_time = dt.datetime.now()
      chargeDurationInSeconds = (self.last_undock_time - self.last_dock_time).total_seconds()
      chargeDurationInDays = divmod(chargeDurationInSeconds, 86400)
      chargeDurationInHours = round( (chargeDurationInDays[1] / 3600.0), 1)
      if (chargeDurationInHours > 0.1):
          printMsg = "** WaLI Undocking: success at battery {:.0f}%, docked for {:.1f} hrs **".format(self.battery_percentage*100, chargeDurationInHours)
      else:  # Do not know how long on dock, such as just booted after a nap on the dock
          printMsg = "** WaLI Undocking: success at battery {:.0f}% **".format(self.battery_percentage*100)
      self.lifeLog.info(printMsg)
      if DEBUG:
          dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
          print(dtstr, printMsg)



  def dock_action_send_goal(self):
    # dock_msg = DockServo.Goal()
    dock_msg = Dock.Goal()
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "dock_action_send_goal(): executing"
      print(dtstr, printMsg)
    self._dock_action_client.wait_for_server()
    if DEBUG:
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(dtstr, "dock_action_send_goal(): after wait_for_server()")
    self._dock_action_send_goal_future = self._dock_action_client.send_goal_async(dock_msg)
    if DEBUG:
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(dtstr, "dock_action_send_goal(): after set future")
    self._dock_action_send_goal_future.add_done_callback(self.dock_goal_response_callback)
    if DEBUG:
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(dtstr, "dock_action_send_goal(): after add_done_callback")

  def dock_goal_response_callback(self, future):
    goal_handle = future.result()
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "dock_goal_response_callback(): Goal Accepted: {}".format(goal_handle.accepted)
      print(dtstr, printMsg)

    if not goal_handle.accepted:
      printMsg = 'dock Goal Rejected'
      self.get_logger().info(printMsg)
      if DEBUG:
          dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
          print(dtstr, printMsg)
      return

    printMsg = 'dock Goal Accepted'
    self.get_logger().info(printMsg)
    if DEBUG:
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(dtstr, printMsg)

    self.state = "docking"

    self._get_dock_result_future = goal_handle.get_result_async()
    self._get_dock_result_future.add_done_callback(self.get_dock_result_callback)

  def get_dock_result_callback(self, future):
    result = future.result().result
    if DEBUG:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      printMsg = "get_dock_result_callback(): dock Result is_docked {} %".format(result.is_docked)
      print(dtstr, printMsg)
    if result.is_docked:
      dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      self.state = "docked"
      self.last_dock_time = dt.datetime.now()
      playtimeDurationInSeconds = (self.last_dock_time - self.last_undock_time).total_seconds()
      playtimeDurationInDays = divmod(playtimeDurationInSeconds, 86400)
      playtimeDurationInHours = round( (playtimeDurationInDays[1] / 3600.0), 1)
      printMsg = "** WaLI dock goal result - Docking: success at battery {:.0f}% after {:.1f} hrs playtime **".format(self.battery_percentage*100, playtimeDurationInHours)
      self.lifeLog.info(printMsg)
      if DEBUG:
          dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
          print(dtstr, printMsg)
    else:
      self.state = "undocked"
      printMsg = "** WaLi dock goal result - Docking: failed **"
      self.lifeLog.info(printMsg)
      if DEBUG:
          dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
          print(dtstr, printMsg)




  def rotate_angle_action_send_goal(self,angle):
    rotate_angle_msg = RotateAngle.Goal()
    rotate_angle_msg.angle = angle

    if DEBUG:
      printMsg = "rotate_angle_action_send_goal(rotate_angle_msg.angle={:.3f}): executing".format(rotate_angle_msg.angle)
      print(printMsg)
    self._rotate_angle_action_client.wait_for_server()
    self._rotate_angle_action_send_goal_future = self._rotate_angle_action_client.send_goal_async(rotate_angle_msg)
    self._rotate_angle_action_send_goal_future.add_done_callback(self.rotate_angle_goal_response_callback)

  def rotate_angle_goal_response_callback(self, future):
    goal_handle = future.result()
    if DEBUG:
      printMsg = "rotate_angle_goal_response_callback(): Goal Accepted: {}".format(goal_handle.accepted)
      print(printMsg)

    if not goal_handle.accepted:
      self.get_logger().info('rotate Goal Rejected :(')
      return

    self.get_logger().info('rotate Goal Accepted :)')

    self._get_rotate_angle_result_future = goal_handle.get_result_async()
    self._get_rotate_angle_result_future.add_done_callback(self.get_rotate_angle_result_callback)

  def get_rotate_angle_result_callback(self, future):
    result = future.result().result
    status = future.result().status
    if DEBUG:
      printMsg = "get_rotate_angle_result_callback(): rotate Status {} Result {} ".format(status, result)
      print(printMsg)
    if status == GoalStatus.STATUS_SUCCEEDED:
      printMsg = "** WaLI Ready To Dock at battery {:.0f}% **".format(self.battery_percentage*100)
      self.lifeLog.info(printMsg)
      self.state = "ready2dock"
    else:
      printMsg = "** Rotate Angle Goal failed with status: {}".format(status)
      self.lifeLog.info(printMsg)


  def wali_main_cb(self):
    try:
      if (self.state == "init" and (self.all_req_topic_cb_rx[0] == False or self.all_req_topic_cb_rx[1] == False)):
        if DEBUG:
          dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
          printMsg = "wali_main_cb(): waiting for all_req_topic_cb_rx"
          print(dtstr, printMsg)
          printMsg = "wali_main_cb(): cb: battery_state, dock_status = {}".format(self.all_req_topic_cb_rx)
          print(dtstr, printMsg)


      elif (self.dock_status.is_docked):
        # Sometimes docking result is not sent or was manually docked so log docked noticed
        # if (self.state == "docking"):
        if (self.state != "docked"):
          self.last_dock_time = dt.datetime.now()
          playtimeDurationInSeconds = (self.last_dock_time - self.last_undock_time).total_seconds()
          playtimeDurationInDays = divmod(playtimeDurationInSeconds, 86400)
          playtimeDurationInHours = round( (playtimeDurationInDays[1] / 3600.0), 1)
          if (playtimeDurationInHours > 0.1):
              printMsg = "** WaLI Noticed Docking: success at battery {:.0f}% after {:.1f} hrs playtime **".format(self.battery_percentage*100, playtimeDurationInHours)
              self.lifeLog.info(printMsg)
        self.state = "docked"
      elif (self.dock_status.dock_visible):
        self.state = "ready2dock"
      elif (self.state == "docked"):  # not docked and not ready2dock but state still says docked
          self.state = "undocked"
          self.last_undock_time = dt.datetime.now()
          chargeDurationInSeconds = (self.last_undock_time - self.last_dock_time).total_seconds()
          chargeDurationInDays = divmod(chargeDurationInSeconds, 86400)
          chargeDurationInHours = round( (chargeDurationInDays[1] / 3600.0), 1)
          if (chargeDurationInHours > 0.1):
              printMsg = "** WaLI Noticed Undocking: success at battery {:.0f}%, docked for {:.1f} hrs **".format(self.battery_percentage*100, chargeDurationInHours)
              self.lifeLog.info(printMsg)
      else:
        self.state = "undocked"

      if DEBUG:
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        printMsg = "wali_main_cb(): executing"
        print(dtstr, printMsg)
        printMsg = "wali_main_cb(): wali.state = {}".format(self.state)
        print(dtstr, printMsg)

      # WaLI logic
      # publishes /undock action goal when BatteryState.percentage at or near full
      #           (and state="docked" and current time not in sleep hours)
      # publishes /rotate_angle {angle: 1.57} (180deg) when BatteryState.percentage low
      # publishes /dock action goal when BatteryState.percentage very low

      if (self.battery_percentage > UNDOCK_AT_PERCENTAGE) and (self.state in ["docked"]) and isNotSleepTime(START_SLEEP_TIME, END_SLEEP_TIME) :
        self.state = "undocking"
        if DEBUG:
          dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
          printMsg = "wali_main_cb(): battery_percentage {:.0} % sending undock action goal".format(self.battery_percentage)
          print(dtstr, printMsg)
        self.undock_action_send_goal()

      elif (self.battery_percentage < ROTATE_AT_PERCENTAGE) and (self.state in ["undocked"]) and not(self.dock_status.dock_visible):
        self.state = "turning"
        if DEBUG:
           dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
           printMsg = "wali_main_cb(): battery_percentage {:.0} % sending rotate180 action goal".format(self.battery_percentage)
           print(dtstr, printMsg)

        # TODO: change this to NavigateToPosition facing dock
        self.rotate_angle_action_send_goal(angle=math.pi)    # pi=180 deg


      elif (self.battery_percentage < DOCK_AT_PERCENTAGE) and (self.state in ["ready2dock"]):
        self.state = "docking"
        if DEBUG:
           dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
           printMsg = "wali_main_cb(): battery_percentage {:.0} % sending dock action goal".format(self.battery_percentage)
           print(dtstr, printMsg)
        self.dock_action_send_goal()

    except Exception as e:
        print("wali_main_cb(): Exception:",str(e))
        traceback.print_exc(file=sys.stdout)
        sys.exit(1)


def main():
  rclpy.init(args=None)
  wali_node = WaLINode()
  try:
    rclpy.spin(wali_node)
  except KeyboardInterrupt:
    pass
  except ExternalShutdownException:
    sys.exit(0)
  finally:
    wali_node.destroy_node()
    try:
      rclpy.try_shutdown()
    except:
      pass


if __name__ == '__main__':
    main()
