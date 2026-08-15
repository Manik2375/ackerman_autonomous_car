#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__('gamepad_teleop')

        # --- Axis & Tuning Parameters ---
        self.declare_parameter('axis_rt', 4)        # Right Trigger (Forward)
        self.declare_parameter('axis_lt', 5)        # Left Trigger (Reverse)
        self.declare_parameter('axis_steer', 0)     # Left Stick Horizontal
        self.declare_parameter('deadzone_stick', 0.10)     # Ignore small stick drift
        self.declare_parameter('deadzone_trigger', 0.05)   # Trigger threshold
        self.declare_parameter('scale_linear', 5.0)        # Linear speed scale
        self.declare_parameter('scale_angular', 1.0)       # Steering scale

        self.axis_rt = self.get_parameter('axis_rt').value
        self.axis_lt = self.get_parameter('axis_lt').value
        self.axis_steer = self.get_parameter('axis_steer').value
        self.deadzone_stick = self.get_parameter('deadzone_stick').value
        self.deadzone_trigger = self.get_parameter('deadzone_trigger').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value

        # Flags to prevent uninitialized 0.0 trigger readings from launching the car
        self.rt_initialized = False
        self.lt_initialized = False

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info('Gamepad Teleop Initialized: RT=Forward, LT=Reverse, Left Stick=Steer')

    def _process_trigger(self, raw_val: float, is_rt: bool) -> float:
        """
        Linux joy triggers:
        - Uninitialized state: 0.0
        - Calibrated resting (unpressed): 1.0
        - Fully pressed: -1.0
        """
        # Detect if trigger has been calibrated/pressed
        if not (self.rt_initialized if is_rt else self.lt_initialized):
            if raw_val != 0.0:
                if is_rt:
                    self.rt_initialized = True
                else:
                    self.lt_initialized = True
            else:
                return 0.0  # Ignore until first movement

        # Normalize 1.0 (unpressed) -> -1.0 (pressed) to 0.0 -> 1.0
        normalized = (1.0 - raw_val) / 2.0
        return normalized if normalized > self.deadzone_trigger else 0.0

    def joy_callback(self, msg: Joy):
        twist = Twist()

        if max(self.axis_rt, self.axis_lt, self.axis_steer) >= len(msg.axes):
            self.get_logger().warn('Joy message missing expected axes.', throttle_duration_sec=2.0)
            return

        # 1. Read and normalize triggers (0.0 to 1.0)
        rt_pressed = self._process_trigger(msg.axes[self.axis_rt], is_rt=True)
        lt_pressed = self._process_trigger(msg.axes[self.axis_lt], is_rt=False)

        # 2. Strict Throttle Logic:
        # - Both pressed -> STOP (0.0)
        # - Neither pressed -> STOP (0.0)
        # - Only RT -> Forward (+)
        # - Only LT -> Backward (-)
        if rt_pressed > 0.0 and lt_pressed > 0.0:
            linear_input = 0.0
        elif rt_pressed > 0.0:
            linear_input = rt_pressed
        elif lt_pressed > 0.0:
            linear_input = -lt_pressed
        else:
            linear_input = 0.0

        # 3. Steering with Deadzone
        raw_steer = msg.axes[self.axis_steer]
        if abs(raw_steer) < self.deadzone_stick:
            steering_input = 0.0
        else:
            steering_input = raw_steer

        # 4. Publish /cmd_vel
        twist.linear.x = linear_input * self.scale_linear
        twist.angular.z = steering_input * self.scale_angular

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    teleop = GamepadTeleop()
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()