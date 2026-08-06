#!/usr/bin/env python3

from __future__ import annotations

import time
from typing import Sequence

from geometry_msgs.msg import Twist
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
MOTOR_TYPE_JGB37_520_12V_110RPM = 3


class MotorControlNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_control_node")

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

        self.last_cmd_time = self.get_clock().now()
        self.bus = SMBus(self.i2c_bus)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, self.servo_hz)
        self.pwm.start(0.0)

        self._init_driver()
        self.subscription = self.create_subscription(Twist, self.cmd_topic, self._on_cmd, 20)
        self.timer = self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f"motor_control_node started on {self.cmd_topic} using I2C bus {self.i2c_bus} "
            f"at address 0x{self.i2c_addr:02x} and servo pin {self.servo_pin}"
        )

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

        self.get_logger().info(
            f"Received cmd_vel: linear.x={linear:+.2f}, angular.z={angular:+.2f}, "
            f"max_linear_input={self.max_linear_input:.2f}, motor_speed_limit={self.motor_speed_limit}"
        )

        speed = int(round(linear * self.max_motor_speed))
        speed = int(self._clamp(speed, -self.motor_speed_limit, self.motor_speed_limit))
        left = -speed
        right = speed
        steering = self.steer_center_deg - angular * self.steer_range_deg

        self._set_servo_angle(steering)
        self._set_motor_vector([left, 0, right, 0])

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
