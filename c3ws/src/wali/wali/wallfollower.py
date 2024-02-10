'''
Wallfollower.py

Follow a wall using /ir_intensity and /ir_opcode sensors for avoidance
and /hazard_detections for obstacle detection exit

- wf_init: Dwell For 10 seconds to let topic subscriptions synchronize, -->wf_detect
- wf_detect: Rotate toward known wall or left if not known till wall is detected ->wf_avoid
- wf_drive: Start driving forward in arc toward left or right wall
  - Check for hazards (bumper, cliff, wheel stall) -> wf_escape
  - Check for something close reported by IR Intensity sensors or IR Opcode -> wf_avoid
- wf_avoid:
  - stop fwd motion and rotate away from object closer than .150m
  - rotate away from objects/walls in .400 to .150m
  - rotate away from wf_side if /ir_opcode.opcode = 162
  - no objects close -> wf_drive
- wf_escape:
  - not implemented -> exit()


TOPIC SUBSCRIPTIONS

- irobot_create_msgs/msg/IrIntensityVector.msg
  # This message provides the ir intensity readings

  std_msgs/Header header
  irobot_create_msgs/IrIntensity[] readings

- irobot_create_msgs/msg/IrIntensity.msg
  # This message provides the ir intensity readings

  std_msgs/Header header
  int16 value

- irobot_create_msgs/msg/HazardDetectionVector.msg
  # This message contains a vector of detected hazards.

  std_msgs/Header header
  irobot_create_msgs/HazardDetection[] detections

- irobot_create_msgs/msg/HazardDetection.msg
  # This message describes a detected hazard.
  # The frame ID allows to retrieve the location of the sensor that made the detection.

  # The robot has reached its backup limit. It will not drive further backward for safety reasons.
  # You can disable this limit through the Create3 "safety_override" parameter by setting it 
  # to "backup_only" or "full".
  # The Create3 webserver can be used to set a default value for the parameter.
  uint8 BACKUP_LIMIT=0
  # The robot has bumped against an obstacle
  uint8 BUMP=1
  # The robot detected a cliff
  uint8 CLIFF=2
  # The wheels of the robot are stalled against an obstacle
  uint8 STALL=3
  # The wheels of the robot are fully dropped
  uint8 WHEEL_DROP=4
  # The robot detects an obstacle in close proximity
  uint8 OBJECT_PROXIMITY=5

  std_msgs/Header header
  uint8 type

- irobot_create_msgs/msg/IrOpcode.msg

  # This message describes a detected ir opcode.

  uint8 CODE_IR_FORCE_FIELD=161
  uint8 CODE_IR_VIRTUAL_WALL=162
  uint8 CODE_IR_BUOY_GREEN=164
  uint8 CODE_IR_BUOY_RED=168
  uint8 CODE_IR_BUOY_BOTH=172
  uint8 CODE_IR_EVAC_GREEN_FIELD=244
  uint8 CODE_IR_EVAC_RED_FIELD=248
  uint8 CODE_IR_EVAC_BOTH_FIELD=252

  uint8 SENSOR_OMNI=0
  uint8 SENSOR_DIRECTIONAL_FRONT=1

  std_msgs/Header header      # time when we saw the opcode
  uint8 opcode                # opcode
  uint8 sensor                # the sensor which saw the opcode

- irobot_create_msgs/msg/KidnapStatus.msg
  # This message provides the robot kidnap status

  std_msgs/Header header
  bool is_kidnapped


- nav_msgs.msg/Odometry message consists of:
       std_msgs/Header                    header
       string                             child_frame_id
       geometry_msgs/PoseWithCovariance   pose  (geometry_msgs/Point, geometry_msgs/Quaternion)
       geometry_msgs/TwistWithCovariance  twist



TOPICS PUBLISHED:

  /cmd_vel  geometry_msgs/msg/Twist.msg

'''

import sys
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time, Duration
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.msg import IrOpcode
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import HazardDetection
from irobot_create_msgs.msg import KidnapStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry    # (header, child_frame_id, pose, twist)
from geometry_msgs.msg import Point  # (position: float64 x,y,z)

import datetime as dt
# from rclpy.action import ActionClient
# from action_msgs.msg import GoalStatus
import numpy as np
import random
import math

# Comment undesired mode
DEBUG = False
DEBUG = True

'''
Allow namespaced robots: (uncomment as appropriate)
'''
# namespace = 'myrobot'       # for /myrobot/create3_topics
namespace = ''                # for no namespace


WF_CB_RATE = 10  # times per second wallfollower wf_cb will be executed
DWELL_TIME = 1 * WF_CB_RATE   # do nothing for awhile
INIT_DWELL_TIME = 15 * WF_CB_RATE
DRIVE_SPEED = 0.1 # m/s
AVOID_ANGULAR_RATE = 0.5 # rad/sec
WF_ANGULAR_RATE = 0.2 # rad/sec
IR_SENSOR_LABELS = ["side_left", "left", "front_left", "front_center_left", "front_center_right", "front_right", "right"]
NEAR_DISTANCE = 0.350  # normal 0.350
SAFE_DISTANCE = 0.175
TWO_PI= math.pi * 2.0



# ************* IR Intensity Sensor To Distance Estimate

# Collected with front surface of white baseboard (60mm floor to top)
# Natural maple cabinet kick panel readings result "closer than they appear"
#   at 0.150 recommended "safe" distance from bumper to center front obstacle
#   Center: 16% high  front_center_left/right: 8,9% high  front_left/right: 27,37% high  side_left/right: 1,200% high

# Approximate distance from wall to bumper at range of 1.5cm to 40cm
DISTANCES = [ 0.400, 0.300, 0.200, 0.110, 0.050, 0.025, 0.015]

# Note Distance base_link to Bumper is 0.171 meter

# Average Reading Values For Distances
READINGS  = [
             [14   , 19   ,  73   , 336  , 1148  , 2469 , 3111],
             [21   , 35   ,  74   , 322  , 1252  , 3090 , 3481],
             [44   , 53   , 112   , 366  , 1162  , 2639 , 3412],
             [23   , 42   , 126   , 451  , 1385  , 2843 , 3514],
             [30   , 44   ,  94   , 353  , 1281  , 3118 , 3769],
             [24   , 38   ,  81   , 276  , 1176  , 2972 , 3745],
             [21   , 35   ,  88   , 410  , 1635  , 3596 , 3784],
            ]

def dist_ir_reading(sensor_idx, reading):
      UNDEF = -99.999
      NOTHING = 99.999
      dist = np.interp(reading,READINGS[sensor_idx],DISTANCES, right=UNDEF, left=NOTHING)
      return dist


def heading_from_quaternion(q):
        """
        Convert a quaternion into euler angle yaw
        yaw is rotation around z in radians (counterclockwise)
        """
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z # in radians


def distance(p2,p1):
    # Python3.8+ hypot is n-dimensional
    dist=math.hypot(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z)
    return dist

# ***************** CLASS Wallfollower ****************


class Wallfollower(Node):
    '''
    Wallfollower class is subclass of a ROS 2 Node,
    Subscribing to the /ir_intensity /ir_opcode and /hazard_detection topics.
    The Wf_cbCallback provides the exploration and obstacle avoidance behavior.
    '''

    def __init__(self):
        '''
        Calls the Node class constructor and declares node name
        '''
        super().__init__('wallfollower')

        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        printMsg ='wali.wallfollower.py started'
        print(dtstr,printMsg)

        '''
        Set up subscriptions
        '''
        if DEBUG: print('Subscribing to /ir_intensity topic of IrIntensityVector type')
        self.subscription = self.create_subscription(
            IrIntensityVector, namespace + '/ir_intensity', self.ir_intensity_cb,
            qos_profile_sensor_data)

        # rolling counter (0-61) of ir_intensity msgs received from 62 Hz topic
        self.ir_intensity_counter = 0


        if DEBUG: print('Subscribing to /hazard_detection topic of irobot_create_msgs/msg/HazardDetectionVector type')
        self.subscription = self.create_subscription(
            HazardDetectionVector, namespace + '/hazard_detection', self.hazard_detection_cb,
            qos_profile_sensor_data)

        # rolling counter (0-61) of ir_intensity msgs received from 62 Hz topic
        self.hazard_detection_counter = 0


        if DEBUG: print('Subscribing to /ir_opcode topic of irobot_create_msgs/msg/IrOpcode.msg type')
        self.subscription = self.create_subscription(
            IrOpcode, namespace + '/ir_opcode', self.ir_opcode_cb,
            qos_profile_sensor_data)

        # rolling counter (0-4) of ir_opcode msgs received from 5 Hz topic
        self.ir_opcode_counter = 0


        if DEBUG: print('Subscribing to /odom topic of nav_msgs/msg/Odometry.msg type')
        self.subscription = self.create_subscription(
            Odometry, namespace + '/odom', self.odom_cb,
            qos_profile_sensor_data)

        # rolling counter (0-19) of odom  msgs received from 20 Hz topic
        self.odom_counter = 0



        # /kidnap_status topic is published at approximately 1 Hz
        if DEBUG: print('Subscribing to /kidnap_status topic of irobot_create_msgs/msg/KidnapStatus.msg type')
        self.subscription = self.create_subscription(
            KidnapStatus, namespace + '/kidnap_status', self.kidnap_status_cb,
            qos_profile_sensor_data)







        # Set up Wallfollower callback  wf_cb on a timer

        period_for_timer = 1.0 / WF_CB_RATE
        self.timer = self.create_timer( period_for_timer, self.wf_cb)  # call the wf node callback when ROS timer triggers
        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='wallfollower: created wf_cb callback {:.0f} times per second'.format(1.0/period_for_timer)
            print(dtstr,printMsg)
        self.wf_state = "wf_init"  # start state machine in dwell state (doing nothing)
        self.prior_state = "none"
        self.dwell_counter = 0
        self.command = Twist()
        self.last_hazard_detection_msg = HazardDetectionVector()
        self.last_ir_intensity_msg = IrIntensityVector()
        self.last_ir_opcode_msg = IrOpcode()
        self.counter_obstacles = 0
        self.last_odom_msg = Odometry()
        self.follow_wall_on = "right"            # Default to left wall follow - more sensors on left
        self.distance_wallfollowed = 0.0
        self.was_kidnapped = False
        self.current_heading = 0.0
        self.wall_heading = 0.0
        self.current_point = Point()
        self.start_point = Point()

        # Set up cmd_vel twist publisher
        self.twist_publisher = self.create_publisher(
            Twist,
            namespace + '/cmd_vel',
            qos_profile_sensor_data)

        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg = 'Wallfollower Node Initialization Complete'
            print("\n"+dtstr,printMsg)




    # ********* IR INTENSITY CALLBACK *********

    def ir_intensity_cb(self, msg:IrIntensityVector):
        '''
        Save last_ir_intensity_msg
        If debug: print vector once a second
        '''
        self.last_ir_intensity_msg = msg

        if DEBUG and (self.ir_intensity_counter == 0):  # Print values each time counter is 0
          self.printIR(self.last_ir_intensity_msg)

        # increment/roll msg counter approximately once each second for 62 Hz topic
        self.ir_intensity_counter = (self.ir_intensity_counter + 1) % 62


    def printIR(self, msg):
        '''
        :type msg: IrIntensity
        :rtype: None
        '''

        label_idx = 0
        print('\nCreate3 IR sensor:')
        for reading in msg.readings: 
        	val = reading.value
        	dist = dist_ir_reading(label_idx,val)
        	label = IR_SENSOR_LABELS[label_idx]
        	print("{:<20} intensity: {:>5} dist: {:>-8.3f}m".format(label,str(val),dist) )
        	label_idx += 1

    def obstacle_near(self):
        # If anything is estimated to be within the NEAR_DISTANcE, return [sensor_label, distance] of closest object
        # Returns [] if nothing close

        return_obj = []
        label_idx = 0
        dist_list = []
        for reading in self.last_ir_intensity_msg.readings: 
            val = reading.value
            dist = dist_ir_reading(label_idx,val)
            dist_list += [dist]
            label_idx += 1
        min_dist = min(dist_list)
        if min_dist < NEAR_DISTANCE:
            min_label = IR_SENSOR_LABELS[dist_list.index(min_dist)]
            if DEBUG:
                print("obstacle_near: returning {:<20} estimated dist: {:>-8.3f}m".format(min_label,min_dist) )
            return_obj = [min_label, min_dist]

        return return_obj


    # ********* HAZARD DETECTION CALLBACK *********

    def hazard_detection_cb(self, msg:HazardDetectionVector):
        '''
        Save last_hazard_detection_msg
        If debug: print vector once a second
        '''
        self.last_hazard_detection_msg = msg

        if DEBUG and (self.hazard_detection_counter == 0):  # Print values each time counter is 0
          self.printHazards(self.last_hazard_detection_msg)

        # increment/roll msg counter approximately once each second for 20 Hz topic
        self.hazard_detection_counter = (self.hazard_detection_counter + 1) % 62


    def printHazards(self, msg):
        '''
        :type msg: HazardDetectionVector
        :rtype: None
        '''

        hazard_label = ["BACKUP_LIMIT", "BUMP", "CLIFF", "STALL", "WHEEL_DROP", "OBJECT_PROXIMITY"]
        label_idx = 0
        print('\nHAZARDS DETECTED:')
        for hazard_detection in msg.detections: 
        	label = hazard_label[hazard_detection.type]
        	print("Hazard: {:>10}".format(label) )


    def hazard_detected(self):
        hazard_present = False
        for hazard in self.last_hazard_detection_msg.detections:
            if (hazard.type != HazardDetection.BACKUP_LIMIT):
                hazard_present = True

        if (DEBUG and hazard_present):
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='hazard_detected: hazards: {}'
            print("\n"+dtstr,printMsg)
            self.printHazards(self.last_hazard_detection_msg)

        return hazard_present


    # ********* IR OPCODE CALLBACK *********

    def ir_opcode_cb(self, msg:IrOpcode):
        '''
        Save last_ir_opcode_msg
        If debug and virtual wall detected: print detection once a second
        '''
        self.last_ir_opcode_msg = msg

        if DEBUG:  # Print values each time received
          self.printIrOpcode(self.last_ir_opcode_msg)


    def printIrOpcode(self, msg):
        '''
        :type msg: IrOpcode
        '''

        if (msg.opcode == IrOpcode.CODE_IR_VIRTUAL_WALL):     # == 162
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg='ir_opcode_cb: VIRTUAL WALL DETECTED at {}{:0.1f}'.format(msg.header.stamp.sec, msg.header.stamp.nanosec/1000000000.0)
            print("\n"+dtstr,printMsg)

    def see_virtual_wall(self):
        see_it = False

        if (self.last_ir_opcode_msg.opcode == IrOpcode.CODE_IR_VIRTUAL_WALL):
          if ( ( self.get_clock().now() - Time.from_msg(self.last_ir_opcode_msg.header.stamp) ) < Duration(seconds=0.3)):
            see_it = True
          else:
            self.last_ir_opcode_msg.opcode = 0
        return see_it

    # ********* KIDNAP STATUS CALLBACK *********

    def kidnap_status_cb(self, msg:KidnapStatus):
        '''
        If at any time /kidnap_status reports is_kidnapped, set self.was_kidnapped
        '''
        if msg.is_kidnapped:
            self.was_kidnapped = True

        if DEBUG:  # Print as detected 1 Hz rate
          self.printKidnapStatus(msg)


    def printKidnapStatus(self, msg):
        '''
        bool: is_kidnapped
        '''

        if msg.is_kidnapped:
            print('\nkidnap_detected_cb: KIDNAP DETECTED !!')


    # ********* ODOMETRY CALLBACK *********

    def odom_cb(self, msg:Odometry):
        '''
        Save last_odom_sg
        '''
        self.last_odom_msg = msg

        if DEBUG and (self.odom_counter == 0):  # Print values each time counter is 0
          self.printOdometry(self.last_odom_msg)

        # increment/roll msg counter to print approximately once each second for 20 Hz topic
        self.odom_counter = (self.odom_counter + 1) % 20

    def printOdometry(self, msg):
        '''
        :type msg: Odometry
        '''
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        heading = heading_from_quaternion(msg.pose.pose.orientation)
        heading_deg = math.degrees(heading)
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        printMsg = 'odom_cb: (x,y): ({:.3f},{:.3f})  heading: {:4.0f} deg'.format(x,y,heading_deg)
        print("\n",dtstr,printMsg)




    # ********** MOTION COMMANDS **********

    def stop_all_motion(self):
        self.command.linear.x = 0.0
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0
        self.twist_publisher.publish(self.command)
        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='stop_all_motion: published /cmd_vel with all zeros'
            print("\n"+dtstr,printMsg)


    def stop_fwd_motion(self):
        self.command.linear.x = 0.0
        # self.command.angular.z = 0.0  Don't touch rotation
        self.twist_publisher.publish(self.command)
        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='stop_fwd_motion: published /cmd_vel with linear.x 0'
            print("\n"+dtstr,printMsg)


    def add_left_rotation_to_cmd_vel(self):
        if DEBUG: 
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='add_left_rotation_to_cmd_vel'
            print("\n"+dtstr,printMsg)
        rotation = AVOID_ANGULAR_RATE
        # self.command.linear.x unchanged
        # self.command.linear.y unchanged
        # self.command.linear.z unchanged
        self.command.angular.z = rotation

        self.twist_publisher.publish(self.command)
        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='add_left_rotation_to_cmd_vel: published /cmd_vel with angular.z: {:.3f} m/s'.format(self.command.angular.z)
            print("\n"+dtstr,printMsg)
        return


    def add_right_rotation_to_cmd_vel(self):
        if DEBUG: 
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='add_right_rotation_to_cmd_vel'
            print("\n"+dtstr,printMsg)

        rotation = AVOID_ANGULAR_RATE

        # self.command.linear.x unchanged
        self.command.angular.z = -rotation

        self.twist_publisher.publish(self.command)
        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='add_right_rotation_to_cmd_vel: published /cmd_vel with angular.z: {:.3f} m/s'.format(self.command.angular.z)
            print("\n"+dtstr,printMsg)
        return

    def continue_current_avoidance(self):
        # Fix situation of waiting for obstacle to clear but not turning
        if self.command.angular.z == 0:
            self.command.angular.z = AVOID_ANGULAR_RATE * (-1.0 if self.follow_wall_on == "left" else 1.0)
        self.twist_publisher.publish(self.command)
        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='continue_current_avoidance: published /cmd_vel with linear.x: {:.3f} m/s, angular.z: {:.3f} r/s'.format(self.command.linear.x,self.command.angular.z)
            print("\n"+dtstr,printMsg)
        return

    def continue_current_rotation(self):
        self.twist_publisher.publish(self.command)
        if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='continue_current_rotation: published /cmd_vel with linear.x: {:.3f} m/s, angular.z: {:.3f} r/s'.format(self.command.linear.x,self.command.angular.z)
            print("\n"+dtstr,printMsg)
        return

    def drive_forward(self):
        if DEBUG: 
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='drive_forward: executing'
            print("\n"+dtstr,printMsg)
        speed = DRIVE_SPEED
        self.command.linear.x = speed
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        # self.command.angular.z = 0.0
        if (self.follow_wall_on == "right"):
            self.command.angular.z = -1 * WF_ANGULAR_RATE
        else:
            self.command.angular.z = WF_ANGULAR_RATE

        self.twist_publisher.publish(self.command)
        if DEBUG:
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='drive_forward: published /cmd_vel with linear.x: {:.3f} m/s angular.z: {:.3f} r/s'.format(self.command.linear.x, self.command.angular.z)
                print("\n"+dtstr,printMsg)
        return

    # ********** WALLFOLLOWER STATES **********

    def wf_detect(self):
        if (self.obstacle_near() or self.see_virtual_wall()):
            self.prior_state = self.wf_state
            self.wf_state = "wf_avoid"
            if DEBUG: 
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wf_detect: something nearby or virtual wall, -> wf_avoid'
                print("\n"+dtstr,printMsg)
        elif (self.wf_state != self.prior_state):
                if self.follow_wall_on == "left":
                    self.add_left_rotation_to_cmd_vel()
                else:
                    self.add_right_rotation_to_cmd_vel()
                if DEBUG: 
                    dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    printMsg ='wf_detect: no wall, rotate toward wall'
                    print("\n"+dtstr,printMsg)
                self.prior_state = self.wf_state
        # else no obstacle and not first time here so no change needed
        self.continue_current_rotation()

    def wf_escape(self):
        if DEBUG: 
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='wf_escape: started'
            print("\n"+dtstr,printMsg)
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        printMsg ='wf_escape: escape not implemented - transition to wf_exit'
        print("\n"+dtstr,printMsg)

        self.prior_state = self.wf_state
        self.wf_state = "wf_exit"


    def wf_avoid(self):

        # stop fwd motion and rotate away from front object closer than .150m
        # rotate away from front objects in .400 to .150m
        # rotate away from side objects within 0.xxx
        # rotate away from virtual wall based on side following
        # no objects close - transition to dwell

        obstacle = self.obstacle_near()


        if DEBUG: 
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='wf_avoid: virtual_wall: {} obstacle: {}'.format(self.see_virtual_wall(), obstacle)
            print("\n"+dtstr,printMsg)


        if self.see_virtual_wall():
            if self.prior_state != self.wf_state:
                self.stop_fwd_motion()
                if (self.follow_wall_on == "right"):
                    self.add_left_rotation_to_cmd_vel()
                else:
                    self.add_right_rotation_to_cmd_vel()
            else:
                self.continue_current_avoidance()
        else:   # Do not see virtual wall
            if obstacle and (obstacle[1] < SAFE_DISTANCE):
                self.stop_fwd_motion()
            # if obstacle and (self.prior_state != self.wf_state):
            # print("IR_SENSOR_LABELS[0:4]:",IR_SENSOR_LABELS[0:4])
            if obstacle:
                if self.prior_state != self.wf_state:
                    self.counter_obstacles += 1
                    if (obstacle[0] in IR_SENSOR_LABELS[0:4]):
                        self.add_right_rotation_to_cmd_vel()
                    else:
                        self.add_left_rotation_to_cmd_vel()
                else:
                    self.continue_current_avoidance()
        # mark been here just before
        self.prior_state = self.wf_state

        # after obtacle is cleared, quit turning
        if not (obstacle or self.see_virtual_wall()):
            self.wf_state = "wf_drive"
            if DEBUG:
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wf_avoid: no obstacles - transition to wf_drive'
                print("\n"+dtstr,printMsg)




    def wf_drive(self):
        if DEBUG: 
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='wf_drive: executing'
            print("\n"+dtstr,printMsg)

        if self.hazard_detected():
            self.stop_all_motion()
            self.prior_state = self.wf_state
            self.wf_state = "wf_escape"
            if DEBUG: 
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wf_drive: hazard detected, stopped all motion and transitioning to wf_escape'
                print("\n"+dtstr,printMsg)

        elif self.obstacle_near():
            self.prior_state = self.wf_state
            self.wf_state = "wf_avoid"
            if DEBUG: 
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wf_drive: nearing obstacle, transitioning to wf_avoid'
                print("\n"+dtstr,printMsg)

        elif self.see_virtual_wall():
            self.prior_state = self.wf_state
            self.wf_state = "wf_avoid"
            if DEBUG: 
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wf_drive: see virtual wall, transitioning to wf_avoid'
                print("\n"+dtstr,printMsg)


        else:
            self.drive_forward()
            if self.prior_state != self.wf_state:
                self.prior_state = self.wf_state
            if DEBUG:
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wf_drive: driving forward'
                print("\n"+dtstr,printMsg)



    def wf_dwell(self):
        if self.prior_state != self.wf_state:
            # set so counter will not reaet next time
            self.prior_state = self.wf_state
            self.dwell_counter = 0

        if (self.dwell_counter == 0):
            if DEBUG: 
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wf_dwell: dwell counter started, stopping all motion'
                print("\n"+dtstr,printMsg)
            self.stop_all_motion()
        self.dwell_counter += 1
        self.dwell_counter = self.dwell_counter % DWELL_TIME
        if (self.dwell_counter == 0):
            if DEBUG: 
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wf_dwell: dwell complete, transition to drive state'
                print("\n"+dtstr,printMsg)
            self.wf_state = "wf_drive"

    def wf_init(self):
        # print("wf_init: dwell_counter: {}".format(self.dwell_counter))
        if self.prior_state != self.wf_state:
            # set so counter will not reaet next time
            self.prior_state = self.wf_state
            self.dwell_counter = 0

        if (self.dwell_counter == 0):
            if DEBUG: 
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wf_init: dwell counter started, stopping all motion'
                print("\n"+dtstr,printMsg)
            self.stop_all_motion()
        # print("wf_init: dwell_counter: {}".format(self.dwell_counter))
        self.dwell_counter += 1
        # print("wf_init: dwell_counter: {}".format(self.dwell_counter))
        self.dwell_counter = self.dwell_counter % INIT_DWELL_TIME
        # print("wf_init: dwell_counter: {}".format(self.dwell_counter))

        # If dwell time is passed and the ir_intensity readings have started
        if (self.dwell_counter == 0) and (len(self.last_ir_intensity_msg.readings) != 0):
            if DEBUG: 
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wf_init: dwell complete, transition to wf_detect state'
                print("\n"+dtstr,printMsg)
            self.wf_state = "wf_detect"


    # ********** WANDER CALLBACK *********
    '''
        The Wander callback implements a state machine on self.wf_state:
    '''
    def wf_cb(self):
        if self.was_kidnapped:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg ='wf_cb: kidnap detected'
            print("\n"+dtstr,printMsg)
            self.wf_state = wf_exit

        match self.wf_state:
            case "wf_init":
                self.wf_init()
            case "wf_dwell":
                self.wf_dwell()
            case "wf_detect":
                self.wf_detect()
            case "wf_drive":
                self.wf_drive()
            case "wf_avoid":
                self.wf_avoid()
            case "wf_escape":
                self.wf_escape()
            case "wf_exit":
                self.stop_all_motion()
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wali.wf.py: wf_exit state requested, distance wallfollowed: {}'.format(self.distance_wallfollowed)
                print("\n"+dtstr,printMsg)
                exit()
            case _:
                print("wf_cb: Unknown state: {}".format(self.wf_state))
                dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                printMsg ='wali.wf.py unknown state exit, distance wallfollowed: {}'.format(self.distance_wallfollowed)
                print("\n"+dtstr,printMsg)
                exit()




def main(args=None):
    '''
    Initialize ROS Client Library For Python
    '''
    rclpy.init(args=args)

    '''
    Creates the node.
    '''
    wf = Wallfollower()

    '''
    "Spin" (execute the node)
    '''
    try:
        rclpy.spin(wf)
    except KeyboardInterrupt:
        if DEBUG: print('\nCaught keyboard interrupt')
        dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        printMsg ='wali.wallfollower.py keyboard interrupt exit, distance wallfollowed: {}'.format(wf.distance_wallfollowed)
        print("\n"+dtstr,printMsg)

        pass
    finally:
    	if DEBUG:
            dtstr = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            printMsg = "Destroying the Wallfollower Node"
            print("\n"+dtstr,printMsg)
    	wf.destroy_node()
    	# rclpy.shutdown()


if __name__ == '__main__':
    main()
