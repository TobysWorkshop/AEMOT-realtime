"""
A ROS2 Wrapper Node for the AEB tracker
"""
# ROS2 imports
import rclpy
from rclpy.node import Node

# ROS2 Message types for broadcasting to/receiving from topics
from std_msgs.msg import String
from event_camera_msg.msg import EventPacket

# Other required packages
import event_camera_py

### PARAMS
EVENT_CAMERA_TOPIC = '/...'

########

class AEBNode(Node):
    def __init__(self):
        # Initialise the node with its name
        super().__init__('AEB_node')

        # Declare and get parameters
        self.declare_parameter('name', 'AEB_node')
        self.name = self.get_parameter('name').get_parameter_vale()

        # SUBSCRIPTIONS
        self.create_subscription(EventPacket, EVENT_CAMERA_TOPIC, self.eventpacket_cb, 10)

        # PUBLISHERS
        self.publisher = self.create_publisher(String, '/...', 10)

        # Log startup message
        self.get_logger().info('AEB ROS2 Node has been started.')
    
    # Runs every time we receive an event packet
    def eventpacket_cb(self):
        pass
        ## TAKE THE PACKET AND INJEST IT INTO AEB (INJEST POINT)


# Entry point
def main(args=None):
    # Establish conenction to ROS2
    rclpy.init(args=args)

    try:
        # Start up the node
        node = AEBNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shut down the process and disconnect from ROS2
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()