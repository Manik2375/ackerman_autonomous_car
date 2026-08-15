#!/usr/bin/env python3

from __future__ import annotations

import math
import struct
import time
from typing import Sequence

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node

try:
    from smbus2 import SMBus
except ImportError as exc:
    raise RuntimeError(
        "Missing dependency smbus2. Install it on the Jetson Nano."
    ) from exc

try:
    import Jetson.GPIO as GPIO
except ImportError as exc:
    raise RuntimeError(
        "Missing dependency Jetson.GPIO. Install it on the Jetson Nano."
    ) from exc


I2C_ADDR = 0x34
MOTOR_TYPE_ADDR = 20
MOTOR_ENCODER_POLARITY_ADDR = 21
MOTOR_FIXED_SPEED_ADDR = 51
MOTOR_ENCODER_TOTAL_ADDR = 60  # Address to read 32-bit encoder counts
MOTOR_TYPE_JGB37_520_12V_110RPM = 3


class MotorControlNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_control_node")

        # Hardware parameters
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_addr", I2C_ADDR)
        self.declare_parameter("servo_pin", 33)
        self.declare_parameter("servo_hz", 50)
        self.declare_parameter("max_motor_speed", 20)
        self.declare_parameter("max_linear_input", 5.0)
        self.declare_parameter("motor_speed_limit", 100)
        self.declare_parameter("steer_center_deg", 90.0)
        self.declare_parameter("steer_range_deg", 45.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("cmd_topic", "/cmd_vel")
        
        # Kinematic parameters for Odometry
        self.declare_parameter("ticks_per_rev", 1320.0)  # Default for JGB37-520 (needs calibration)
        self.declare_parameter("wheel_radius", 0.035)    # 7cm diameter = 3.5cm radius
        self.declare_parameter("wheelbase", 0.17)        # 17cm from rear to front axle
        self.declare_parameter("odom_pub_rate_hz", 20.0)

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.i2c_addr = int(self.get_parameter("i2c_addr").value)
        self.servo_pin = int(self.get_parameter("servo_pin").value)
        self.servo_hz = int(self.get_parameter("servo_hz").value)
        self.max_motor_speed = int(self.get_parameter("max_motor_speed").value)
        self.max_linear_input = float(self.get_parameter("max_linear_input").value)
        self.motor_speed_limit = int(self.get_parameter("motor_speed_limit").value)
        self.steer_center_deg = float(self.get_parameter("steer_center_deg").value)
        self.steer_range_deg = float(self.get_parameter("steer_range_deg").value)
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        
        self.ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.odom_pub_rate_hz = float(self.get_parameter("odom_pub_rate_hz").value)

        # State Variables
        self.last_cmd_time = self.get_clock().now()
        self.bus = SMBus(self.i2c_bus)
        
        # Odometry State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.current_steer_angle_rad = 0.0

        # GPIO Setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, self.servo_hz)
        self.pwm.start(0.0)

        self._init_driver()
        
        # Publishers / Subscribers / Broadcasters
        self.subscription = self.create_subscription(Twist, self.cmd_topic, self._on_cmd, 20)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timers
        self.watchdog_timer = self.create_timer(0.1, self._watchdog)
        self.odom_timer = self.create_timer(1.0 / self.odom_pub_rate_hz, self._odom_loop)

        self.get_logger().info(f"motor_control_node started. Publishing /odom to TF tree at {self.odom_pub_rate_hz}Hz")

        self.first_run = True
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

    def _init_driver(self) -> None:
        self._write_data(MOTOR_TYPE_ADDR, [MOTOR_TYPE_JGB37_520_12V_110RPM])
        time.sleep(0.005)
        self._write_data(MOTOR_ENCODER_POLARITY_ADDR, [0])
        self._set_servo_angle(self.steer_center_deg)
        self._set_motor_vector([0, 0, 0, 0])

    def _write_data(self, register: int, values: Sequence[int]) -> None:
        payload = [int(value) & 0xFF for value in values]
        self.bus.write_i2c_block_data(self.i2c_addr, register & 0xFF, payload)

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    @staticmethod
    def _angle_to_duty_cycle(angle_deg: float) -> float:
        angle_deg = max(0.0, min(180.0, angle_deg))
        pulse_us = 500.0 + (angle_deg / 180.0) * 2000.0
        return (pulse_us / 20000.0) * 100.0

    def _set_servo_angle(self, angle_deg: float) -> None:
        self.pwm.ChangeDutyCycle(self._angle_to_duty_cycle(angle_deg))

    def _set_motor_vector(self, values: Sequence[int]) -> None:
        self._write_data(MOTOR_FIXED_SPEED_ADDR, values)

    def _on_cmd(self, msg: Twist) -> None:
        self.last_cmd_time = self.get_clock().now()

        linear = self._clamp(msg.linear.x, -self.max_linear_input, self.max_linear_input)
        angular = self._clamp(msg.angular.z, -1.0, 1.0)
        
        # Save commanded steering angle in radians for kinematic calculation
        self.current_steer_angle_rad = angular * math.radians(self.steer_range_deg)

        speed = int(round(linear * self.max_motor_speed))
        speed = int(self._clamp(speed, -self.motor_speed_limit, self.motor_speed_limit))
        left = -speed
        right = speed
        steering = self.steer_center_deg - angular * self.steer_range_deg

        self._set_servo_angle(steering)
        self._set_motor_vector([left, 0, right, 0])


    def _odom_loop(self) -> None:
        try:
            # Read 16 bytes from register 60 (4 motors x 32-bit int)
            data = self.bus.read_i2c_block_data(self.i2c_addr, MOTOR_ENCODER_TOTAL_ADDR, 16)
            m1, m2, m3, m4 = struct.unpack('<iiii', bytearray(data))
        except Exception as e:
            self.get_logger().debug(f"I2C Encoder read failed: {e}")
            return
            
        # Left motor (M1) receives negative speed commands to go forward, so ticks decrease. 
        # We invert it so forward always yields positive ticks. Right motor (M3) is standard.
        current_left_ticks = -m1 
        current_right_ticks = m3

        # --- FIRST TICK FIX START ---
        # If this is the very first time we read the encoders, set the baseline to whatever 
        # the hardware currently says. This forces delta_left and delta_right to be exactly 0.
        if self.first_run:
            self.last_left_ticks = current_left_ticks
            self.last_right_ticks = current_right_ticks
            self.last_odom_time = self.get_clock().now()
            self.first_run = False
        # --- FIRST TICK FIX END ---

        delta_left = current_left_ticks - self.last_left_ticks
        delta_right = current_right_ticks - self.last_right_ticks

        self.last_left_ticks = current_left_ticks
        self.last_right_ticks = current_right_ticks

        # Calculate distances
        dist_left = (delta_left / self.ticks_per_rev) * (2.0 * math.pi * self.wheel_radius)
        dist_right = (delta_right / self.ticks_per_rev) * (2.0 * math.pi * self.wheel_radius)
        
        # Ackermann rear axle center distance
        delta_dist = (dist_left + dist_right) / 2.0
        
        # Ackermann heading change based on steering angle
        delta_theta = (delta_dist / self.wheelbase) * math.tan(self.current_steer_angle_rad)

        # Update pose
        self.x += delta_dist * math.cos(self.theta)
        self.y += delta_dist * math.sin(self.theta)
        self.theta += delta_theta

        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        vx = delta_dist / dt if dt > 0 else 0.0
        vtheta = delta_theta / dt if dt > 0 else 0.0

        # Construct Quaternion manually (yaw only)
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        # Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vtheta
        
        self.odom_pub.publish(odom)

        # Publish TF Transform (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(t)

    def _watchdog(self) -> None:
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_timeout_sec:
            self._set_motor_vector([0, 0, 0, 0])
            self._set_servo_angle(self.steer_center_deg)

    def destroy_node(self) -> bool:
        try:
            self._set_motor_vector([0, 0, 0, 0])
            self._set_servo_angle(self.steer_center_deg)
        except Exception:
            pass
        try:
            self.pwm.ChangeDutyCycle(0.0)
            self.pwm.stop()
        except Exception:
            pass
        try:
            self.bus.close()
        except Exception:
            pass
        GPIO.cleanup()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()