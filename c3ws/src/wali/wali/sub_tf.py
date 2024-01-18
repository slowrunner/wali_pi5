'''
sub_tf.py
'''

import sys
import rclpy

'''
Statements that import messages, actions, interfaces, and variable types.
'''
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
# from irobot_create_msgs.msg import IrIntensityVector
from tf2_msgs.msg import TFMessage
'''
Input your namespace here as a global variable. 
'''
# namespace = '[Namespace]'
namespace = ''

class TFSubscriber(Node):
    '''
    The TFSubscriber class is created which is a subclass of Node.
    The Node is subscribing to the /tf topic.
    '''

    def __init__(self):
        '''
        The following line calls the Node class' constructor and declares a node name,
        which is 'TF_subscriber' in this case. 
        '''
        super().__init__('TF_subscriber')
        '''
        This line indicates that the node is subscribing to the TFMessage
        type over the '/tf' topic. 
        '''
        print('Creating subscription to to the TFMessage type over the /tf topic')
        self.subscription = self.create_subscription(
            TFMessage, namespace + '/tf', self.listener_callback,
            qos_profile_sensor_data)

        # rolling counter (0-19) of tf msgs received from 20 Hz topic
        self.tf_counter = 0


    def listener_callback(self, msg:TFMessage):
        '''
        The subscriber's callback listens and as soon as it receives the message,
        this function runs. 
        This callback function is basically printing what it hears. It runs the data
        it receives in your terminal (msg).  
        '''
        if (self.tf_counter == 0):  # Print values each time counter is 0
          self.printTF(msg)

        # increment/roll msg counter approximately once each second for 62 Hz topic
        self.tf_counter = (self.tf_counter + 1) % 20


    def printTF(self, msg):
        '''
        This function is used in the above function. Its purpose is to determine 
        which parts of the info are worth showing.
        :type msg: TFMessage
        geometry_msgs/TransformStamped[] transforms


        The msg is returned from our topic '/tf.'
        To get components of a message, use the '.' dot operator. 
        '''

        print('\nTFMessage:\n{}'.format(msg.transforms))

def main(args=None):
    '''
    This line initializes the rclpy library. 
    '''
    rclpy.init(args=args)
    '''
    This creates the node.
    '''
    TF_subscriber = TFSubscriber()
    '''
    The node is then "spun" so its callbacks are called.
    '''
    print('Callbacks are called.')
    try:
        rclpy.spin(TF_subscriber)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
    	'''
    	Destroying the node acts as a "reset" so we don't run into 
    	any problems when we try and run again.
    	'''
    	print("Done")
    	TF_subscriber.destroy_node()
    	# rclpy.shutdown()


if __name__ == '__main__':
    main()
