'''
Wander.py

Wander using /ir_intensity and /hazard_detections for obstacle detection and avoidance

Loop:
- Check for obstructed
- Clear obstruction
- Start driving

TOPICS

- msg/HazardDetection.msg
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

'''

import sys
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import HazardDetection


# Uncomment desired mode
# DEBUG = False
DEBUG = True

'''
Allow namespaced robots: (uncomment as appropriate)
'''
# namespace = 'myrobot'       # for /myrobot/create3_topics
namespace = ''                # for no namespace

class Wanderer(Node):
    '''
    Wanderer class is subclass of a ROS 2 Node,
    Subscribing to the /ir_intensity and /hazard_detection topics.
    '''
    
    def __init__(self):
        '''
        Calls the Node class constructor and declares node name
        '''
        super().__init__('wanderer')

        '''
        Set up subscriptions
        '''
        If DEBUG: print('Subscribing to /ir_intensity topic of IrIntensityVector type')
        self.subscription = self.create_subscription(
            IrIntensityVector, namespace + '/ir_intensity', self.ir_intensity_cb,
            qos_profile_sensor_data)

        # rolling counter (0-61) of ir_intensity msgs received from 62 Hz topic
        self.ir_intensity_counter = 0


        If DEBUG: print('Subscribing to /hazard_detection topic of irobot_create_msgs/msg/HazardDetectionVector type')
        self.subscription = self.create_subscription(
            HazardDetectionVector, namespace + '/hazard_detection', self.hazard_detection_cb,
            qos_profile_sensor_data)

        # rolling counter (0-19) of ir_intensity msgs received from 20 Hz topic
        self.hazard_detection_counter = 0
        If DEBUG: print('Wanderer Node Initialization Complete')

    # ********* IR INTENSITY CALLBACK *********

    def ir_intensity_cb(self, msg:IrIntensityVector):
        '''
        Save last_ir_intensity_msg
        If debug: print vector once a second
        '''

        if (self.ir_intensity_counter == 0):  # Print values each time counter is 0
          self.printIR(msg)

        # increment/roll msg counter approximately once each second for 62 Hz topic
        self.ir_intensity_counter = (self.ir_intensity_counter + 1) % 62


    def printIR(self, msg):
        '''
        :type msg: IrIntensity
        :rtype: None
        '''

        labels = ["side_left", "left", "front_left", "front_center_left", "front_center_right", "front_right", "right"]
        label_idx = 0
        print('\nCreate3 IR sensor:')
        for reading in msg.readings: 
        	val = reading.value
        	label = labels[label_idx]
        	print("{:<20} intensity: {:>5}".format(label,str(val)) )
        	label_idx += 1


    # ********* HAZARD DETECTION CALLBACK *********

    def hazard_detection_cb(self, msg:HazardDetectionVector):
        '''
        Save last_hazard_detection_msg
        If debug: print vector once a second
        '''

        if (self.hazard_detection_counter == 0):  # Print values each time counter is 0
          self.printHazards(msg)

        # increment/roll msg counter approximately once each second for 20 Hz topic
        self.hazard_detection_counter = (self.hazard_detection_counter + 1) % 20


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

def main(args=None):
    '''
    Initialize ROS Client Library For Python
    '''
    rclpy.init(args=args)

    '''
    Creates the node.
    '''
    wanderer = Wanderer()

    '''
    "Spin" (execute the node)
    '''
    try:
        rclpy.spin(wanderer)
    except KeyboardInterrupt:
        If DEBUG: print('\nCaught keyboard interrupt')
        continue
    finally:
    	'''
    	Destroying the node acts as a "reset" so we don't run into 
    	any problems when we try and run again.
    	'''
    	If DEBUG: print("Destroying the Wanderer Node")
    	wanderer.destroy_node()
    	# rclpy.shutdown()


if __name__ == '__main__':
    main()
