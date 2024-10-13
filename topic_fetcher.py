import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # Topic name
            self.joint_state_callback,
            10  # QoS depth
        )
        self.subscription  # To prevent unused variable warning

    def joint_state_callback(self, msg):
        # Log or process the joint angles (positions)
        self.get_logger().info('Joint positions: %s' % str(msg.position))

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
