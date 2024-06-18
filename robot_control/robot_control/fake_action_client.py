import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile

class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        qos_profile = QoSProfile(depth=10)
        self._subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            qos_profile)
        
        self.get_logger().info('Navigate to Pose Action Client is up and running.')

    def goal_pose_callback(self, msg):
        self.get_logger().info('Received goal pose, sending action request...')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        self.send_goal(goal_msg)

    def send_goal(self, goal_msg):
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

def main(args=None):
    rclpy.init(args=args)
    navigate_to_pose_client = NavigateToPoseClient()

    try:
        rclpy.spin(navigate_to_pose_client)
    except KeyboardInterrupt:
        pass
    finally:
        navigate_to_pose_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
