import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from robot_interfaces.msg import RobotState  # Replace with your actual package name
import math

class InitialPublisher(Node):
    def __init__(self):
        super().__init__('initial_publisher')

        self.declare_parameter('robot_name', 'AGV1')
        self.declare_parameter('initial_x', 166.0)
        self.declare_parameter('initial_y', -44.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.initial_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('initial_yaw').get_parameter_value().double_value

        # Initialize the current position and orientation
        self.current_position = PoseStamped().pose.position
        self.current_position.x = self.initial_x
        self.current_position.y = self.initial_y
        self.current_position.z = 0.0

        self.current_orientation = self.get_quaternion_from_yaw(self.initial_yaw)

        # Publisher for robot state
        self.state_publisher = self.create_publisher(RobotState, 'robot_state', 10)

        # Create a future to manage shutdown
        self.future = rclpy.task.Future()

        # Publish initial state
        self.publish_initial_state()

        # Create a timer to set the future result after a delay
        self.create_timer(2.0, self.set_future_result)

    def get_quaternion_from_yaw(self, yaw):
        # Convert yaw to quaternion
        q = Quaternion()
        q.w = float(math.cos(yaw / 2))
        q.z = float(math.sin(yaw / 2))
        q.x = 0.0
        q.y = 0.0
        return q

    def publish_initial_state(self):
        state_msg = RobotState()
        state_msg.status = 'idle'
        
        # Create and populate PoseStamped message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position = self.current_position
        pose.pose.orientation = self.current_orientation

        state_msg.current_pose = pose
        
        # Log message details
        self.get_logger().info(f'Publishing initial state: status={state_msg.status}')
        self.get_logger().info(f'Pose position: x={pose.pose.position.x}, y={pose.pose.position.y}, z={pose.pose.position.z}')
        self.get_logger().info(f'Pose orientation: x={pose.pose.orientation.x}, y={pose.pose.orientation.y}, z={pose.pose.orientation.z}, w={pose.pose.orientation.w}')

        self.state_publisher.publish(state_msg)
        self.get_logger().info('Initial state published')
        
    def set_future_result(self):
        self.get_logger().info('Setting future result to shut down the node')
        self.future.set_result(True)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPublisher()

    # Spin the node until the future is complete
    while rclpy.ok() and not node.future.done():
        rclpy.spin_once(node)

    # Destroy the node explicitly after publishing
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
