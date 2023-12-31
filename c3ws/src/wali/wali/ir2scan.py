# FILE: ir2scan.py

"""

  Purpose:  Subscribe to ir_intensity vector of seven IR readings
            Publish as a LaserScan topic with seven valid ranges per msg

  Limits:  The IR intensity varies with surface reflectivity, surface color, 
           and angle of incidence.  
           The intensity reading to distance function is based on 
             - white painted household baseboard - other surfaces invalidate the function
             - sensor normal to obstacle/wall - other incidence angles invalidate the function
           Reported Distances may be off by 50 to 150mm


  Subscribes: /ir_intensity  irobot_create_msgs/IrIntensityVector.msg

              std_msgs/Header header
              irobot_create_msgs/IrIntensity[] readings

              irobot_create_msg/IrIntensity
                  std_msgs/Header header
                  int16 value

  Publishes: /scan  sensor_msgs/msg/LaserScan.msg
                      std_msgs/Header header
                      float32         angle_min  # start angle of scan (rad)
                      float32         angle_max  # end angle of scan (rad)
                      float32         angle_increment  # distance between measurements (rad)
                      float32         time_increment   # time between measurements [seconds]
                      float32         scan_time        # time between scans [seconds]
                      float32         range_min        # minimum valid range [m]
                      float32         range_max        # maximum valid range [m]
                      float32[]       ranges           # range data [m]
                      float32[]       intensities      # (optional)

  Uses:  ir_dist.py

"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from irobot_create_msgs.msg import IrIntensityVector
import ir_dist

'''
Namespace for robot 
'''
namespace = ''

DEBUG = False
# Uncomment for debug prints to console
# DEBUG = True

# pick one of the following qos profile methods - explicit or named profile
# from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.qos import qos_profile_sensor_data



def printIR(msg):
    '''
      print reading value and (estimated) distance of seven Create3 IR sensors from /ir_intensity topic
      :type msg: IrIntensity
    '''

    labels = ["side_left", "left", "front_left", "front_center_left", "front_center_right", "front_right", "right"]
    label_idx = 0
    print('\nCreate3 IR sensor:')
    for reading in msg.readings: 
        	val = reading.value
        	dist = ir_dist.dist_ir_reading(label_idx,val)
        	label = labels[label_idx]
        	print("{:<20} intensity: {:>5} dist: {:>-8.3f}m".format(label,str(val),dist) )
        	label_idx += 1

SCAN_TOPIC_HZ = 40              # Publish Scan Topic NN times per second
SCAN_TOPIC_PERIOD = 1.0/SCAN_TOPIC_HZ

ANGLE_MIN =  -1.1344640138        # -65 degrees   65 deg ccw from front
ANGLE_MAX =   1.1344640138        # +65 degrees   65 deg  cw from front
ANGLE_INCREMENT = 0.0174532925  # 1 degree
RANGE_MIN = 0.015
RANGE_MAX = 0.400
IR_SENSOR_LABELS = ["side_left", "left", "front_left", "front_center_left", "front_center_right", "front_right", "right"]
IR_SENSOR_RANGES_INDEX = [130  ,   99  ,      79     ,        62          ,       45            ,     27       ,     0  ]
BASE_LINK_TO_IR_SENSORS = 0.171  # base_link to front of bumper - ir range estimates are distance from front bumper to obstacle

class IR2Scan_Node(Node):

    def __init__(self):
        super().__init__('ir2scan')

        # SUBSCRIBE to /ir_intensity topics
        self.subscription = self.create_subscription(
            IrIntensityVector,
            namespace + '/ir_intensity',
            self.ir_intensity_sub_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        # rolling counter of ir_intensity msgs received from 62 Hz topic for debugging
        self.ir_intensity_counter = 0
        self.ir_intensity_vector = IrIntensityVector()

        # PUBLISH to /scan topic with LaserScan msg type
        self.scan_pub_timer = self.create_timer(SCAN_TOPIC_PERIOD, self.scan_timer_callback)

        self.publisher = self.create_publisher(
            LaserScan,
            namespace + '/scan',
            qos_profile_sensor_data)
        self.scan_msg = LaserScan( 
            header = Header(frame_id = 'base_link'),
            angle_min=ANGLE_MIN, 
            angle_max=ANGLE_MAX, 
            angle_increment=ANGLE_INCREMENT,
            time_increment=0.0,
            scan_time=SCAN_TOPIC_PERIOD,
            range_min=RANGE_MIN,
            range_max=RANGE_MAX,
            ranges=[0.0]*131
            )


    def ir_intensity_sub_callback(self, msg:IrIntensityVector):
        # self.get_logger().info('ir_intensities: {}'.format(msg))
        self.ir_intensity_vector = msg  # readings[].value

        """
        if DEBUG:
            # increment/roll msg counter to print only once per second
            if (self.ir_intensity_counter == 0):
                printIR(self.ir_intensity_vector)
            self.ir_intensity_counter = (self.ir_intensity_counter + 1) % 62
        """

    def scan_timer_callback(self):
            if DEBUG:  self.get_logger().info('self.ir_intensity_vector: {}'.format(self.ir_intensity_vector.readings))
            self.scan_msg.header.stamp = self.ir_intensity_vector.header.stamp
            for idx in range(len(self.ir_intensity_vector.readings)):
                ir_sensor_dist = ir_dist.dist_ir_reading(idx,self.ir_intensity_vector.readings[idx].value)
                self.scan_msg.ranges[IR_SENSOR_RANGES_INDEX[idx]] = ir_sensor_dist + BASE_LINK_TO_IR_SENSORS
                if DEBUG:  
                     self.get_logger().info('{:>20}: reading: {:>4} dist: {:>8.3f} ranges[{:>3}]:{:>8.3f}'.format(
                           IR_SENSOR_LABELS[idx], self.ir_intensity_vector.readings[idx].value, ir_sensor_dist,
                           IR_SENSOR_RANGES_INDEX[idx], self.scan_msg.ranges[IR_SENSOR_RANGES_INDEX[idx]]))
            if (len(self.ir_intensity_vector.readings)==0):
                self.scan_msg.ranges=[0.0]*131
                if DEBUG:  self.get_logger().info('zero length ir_intensity_vector seen')
            self.publisher.publish(self.scan_msg)
            if DEBUG:  self.get_logger().info('Published /scan: {}\n'.format(self.scan_msg))



def main(args=None):
    rclpy.init(args=args)

    ir2scan_node = IR2Scan_Node()

    rclpy.spin(ir2scan_node)

    ir2scan_node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
