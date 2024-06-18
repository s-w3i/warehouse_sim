import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import math

class SimplePathPlanner(Node):
    def __init__(self):
        super().__init__('simple_path_planner')

        self.declare_parameter('robot_name', 'AGV1')
        self.declare_parameter('initial_x', 166.0)
        self.declare_parameter('initial_y', -44.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.initial_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('initial_yaw').get_parameter_value().double_value

        # Initialize the current position and orientation
        self.current_position = TransformStamped().transform.translation
        self.current_position.x = self.initial_x
        self.current_position.y = self.initial_y
        self.current_position.z = 0.0

        self.current_orientation = self.get_quaternion_from_yaw(self.initial_yaw)

        # Create a tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Broadcast the initial transform from map to base_footprint
        self.broadcast_initial_transform()

        # Create subscribers
        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10
        )

        self.target_position = None

        # PD controller gains
        self.kp = 2.0
        self.kd = 0.1

        # Previous yaw error
        self.prev_yaw_error = 0.0

        # Create a timer to run the control loop at a fixed frequency
        self.timer = self.create_timer(0.1, self.control_loop)

    def broadcast_initial_transform(self):
        initial_transform = TransformStamped()
        initial_transform.header.stamp = self.get_clock().now().to_msg()
        initial_transform.header.frame_id = 'map'
        initial_transform.child_frame_id = f'{self.robot_name}/base_footprint'
        initial_transform.transform.translation.x = self.current_position.x
        initial_transform.transform.translation.y = self.current_position.y
        initial_transform.transform.translation.z = 0.0
        initial_transform.transform.rotation = self.current_orientation
        self.tf_broadcaster.sendTransform(initial_transform)

    def goal_pose_callback(self, msg):
        self.target_position = msg.pose.position
        self.get_logger().info(f'Received new target position: x={self.target_position.x}, y={self.target_position.y}')

    def control_loop(self):
        if self.target_position is None:
            return

        try:
            # Compute the distance and angle to the target
            dx = self.target_position.x - self.current_position.x
            dy = self.target_position.y - self.current_position.y
            distance = math.sqrt(dx**2 + dy**2)

            target_yaw = math.atan2(dy, dx)
            current_yaw = self.get_yaw_from_quaternion(self.current_orientation)

            # Calculate the shortest rotation direction
            yaw_error = self.normalize_angle(target_yaw - current_yaw)
            reverse_yaw_error = self.normalize_angle(target_yaw - (current_yaw + math.pi))

            # Determine if the robot needs to rotate
            if abs(yaw_error) < abs(reverse_yaw_error):
                rotation_direction = yaw_error
            else:
                rotation_direction = reverse_yaw_error

            # PD controller for angular speed
            angular_speed = self.kp * rotation_direction + self.kd * (rotation_direction - self.prev_yaw_error)
            self.prev_yaw_error = rotation_direction

            # Rotate first if needed
            if abs(rotation_direction) > 0.1:
                linear_speed = 0.0  # No linear speed during rotation
            else:
                # If rotation is complete, move in a straight line
                if abs(yaw_error) < abs(reverse_yaw_error):
                    linear_speed = min(1.5, distance)  # Cap the linear speed
                else:
                    linear_speed = -min(1.5, distance)  # Cap the linear speed for reverse

            # Update the robot's position
            self.update_position(linear_speed, angular_speed)

            # Check if we reached the target
            if distance < 0.05:
                self.get_logger().info('Target position reached.')
                self.target_position = None

            # Debug logs for tracing
            self.get_logger().info(f'Distance to target: {distance}')
            self.get_logger().info(f'Current position: x={self.current_position.x}, y={self.current_position.y}')
            self.get_logger().info(f'Target position: x={self.target_position.x}, y={self.target_position.y}')
            self.get_logger().info(f'Linear speed: {linear_speed}, Angular speed: {angular_speed}')

        except Exception as e:
            pass

    def update_position(self, linear_speed, angular_speed):
        # Update the position based on linear and angular speeds
        dt = 0.1  # Time step

        # Update the yaw
        current_yaw = self.get_yaw_from_quaternion(self.current_orientation)
        new_yaw = current_yaw + angular_speed * dt

        # Update the position
        self.current_position.x += linear_speed * math.cos(new_yaw) * dt
        self.current_position.y += linear_speed * math.sin(new_yaw) * dt

        # Create a new transform for the updated position
        new_transform = TransformStamped()
        new_transform.header.stamp = self.get_clock().now().to_msg()
        new_transform.header.frame_id = 'map'
        new_transform.child_frame_id = f'{self.robot_name}/base_footprint'
        new_transform.transform.translation.x = float(self.current_position.x)
        new_transform.transform.translation.y = float(self.current_position.y)
        new_transform.transform.translation.z = float(self.current_position.z)

        # Update the orientation
        new_transform.transform.rotation = self.get_quaternion_from_yaw(new_yaw)

        # Broadcast the new transform
        self.tf_broadcaster.sendTransform(new_transform)

        # Update the current orientation
        self.current_orientation = new_transform.transform.rotation

    def get_yaw_from_quaternion(self, q):
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def get_quaternion_from_yaw(self, yaw):
        # Convert yaw to quaternion
        q = Quaternion()
        q.w = float(math.cos(yaw / 2))
        q.z = float(math.sin(yaw / 2))
        q.x = 0.0
        q.y = 0.0
        return q

    def normalize_angle(self, angle):
        # Normalize the angle to the range [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SimplePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
