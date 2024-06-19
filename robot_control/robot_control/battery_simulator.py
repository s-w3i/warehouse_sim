import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatterySimulator(Node):

    def __init__(self):
        super().__init__('battery_simulator')
        self.publisher_ = self.create_publisher(BatteryState, 'battery_state', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every second

        self.battery_percentage = 1.0  # Start with a full battery
        self.charging = False
        self.discharging_rate = 0.8 / 50400  # 80% over 14 hours
        self.charging_rate = 0.8 / 7200  # 80% over 2 hours

        self.get_logger().info('Battery simulator node started with full battery.')

    def timer_callback(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = 48.0  # Example voltage
        msg.current = -1.0 if not self.charging else 1.0  # Negative when discharging
        msg.charge = self.battery_percentage * 10.0  # Example conversion
        msg.capacity = 52.0  # Example full capacity
        msg.design_capacity = 10.0
        msg.percentage = self.battery_percentage
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.charging else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        self.publisher_.publish(msg)
        #self.get_logger().info(f'Published battery state: {msg.percentage * 100:.2f}%')

        # Simulate battery draining and charging
        if self.charging:
            self.battery_percentage += self.charging_rate  # Charging rate
            if self.battery_percentage >= 1.0:
                self.battery_percentage = 1.0
                self.charging = False
        else:
            self.battery_percentage -= self.discharging_rate  # Draining rate
            if self.battery_percentage <= 0.2:
                self.battery_percentage = 0.2
                self.charging = True

def main(args=None):
    rclpy.init(args=args)
    battery_simulator = BatterySimulator()
    rclpy.spin(battery_simulator)
    battery_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
