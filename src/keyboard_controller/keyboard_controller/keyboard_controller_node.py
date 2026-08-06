#!/usr/bin/env python3

from __future__ import annotations

import sys
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class KeyboardControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_controller_node")

        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("speed_step_percent", 15.0)
        self.declare_parameter("max_speed_percent", 300.0)
        self.declare_parameter("initial_speed_percent", 30.0)
        self.declare_parameter("steer_step_deg", 15.0)
        self.declare_parameter("steer_range_deg", 45.0)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.speed_step_percent = float(self.get_parameter("speed_step_percent").value)
        self.max_speed_percent = float(self.get_parameter("max_speed_percent").value)
        self.initial_speed_percent = float(self.get_parameter("initial_speed_percent").value)
        self.steer_step_deg = float(self.get_parameter("steer_step_deg").value)
        self.steer_range_deg = float(self.get_parameter("steer_range_deg").value)

        self.publisher = self.create_publisher(Twist, self.cmd_topic, 20)
        self.shutdown_event = threading.Event()
        self.state_lock = threading.Lock()
        self.linear = 0.0
        self.angular = 0.0
        self.speed_percent = self._clamp(self.initial_speed_percent, 0.0, self.max_speed_percent)
        self.steer_angle_deg = 0.0
        self.reverse_pending = False

        self.publisher_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.publisher_thread.start()

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def _set_forward(self) -> None:
        with self.state_lock:
            self.linear = self._clamp(self.speed_percent / 100.0, 0.0, self.max_speed_percent / 100.0)
            self.reverse_pending = False

    def _set_reverse(self) -> None:
        with self.state_lock:
            self.linear = -self._clamp(self.speed_percent / 100.0, 0.0, self.max_speed_percent / 100.0)
            self.reverse_pending = False

    def _stop(self) -> None:
        with self.state_lock:
            self.linear = 0.0
            self.angular = 0.0

    def _turn_left(self) -> None:
        with self.state_lock:
            self.steer_angle_deg = self._clamp(
                self.steer_angle_deg + self.steer_step_deg,
                -self.steer_range_deg,
                self.steer_range_deg,
            )
            self.angular = self.steer_angle_deg / self.steer_range_deg

    def _turn_right(self) -> None:
        with self.state_lock:
            self.steer_angle_deg = self._clamp(
                self.steer_angle_deg - self.steer_step_deg,
                -self.steer_range_deg,
                self.steer_range_deg,
            )
            self.angular = self.steer_angle_deg / self.steer_range_deg

    def _increase_speed(self) -> None:
        with self.state_lock:
            self.speed_percent = self._clamp(
                self.speed_percent + self.speed_step_percent,
                0.0,
                self.max_speed_percent,
            )
            if self.linear > 0.0:
                self.linear = self._clamp(self.speed_percent / 100.0, 0.0, self.max_speed_percent / 100.0)
            elif self.linear < 0.0:
                self.linear = -self._clamp(self.speed_percent / 100.0, 0.0, self.max_speed_percent / 100.0)

    def _decrease_speed(self) -> None:
        with self.state_lock:
            self.speed_percent = self._clamp(
                self.speed_percent - self.speed_step_percent,
                0.0,
                self.max_speed_percent,
            )
            if self.linear > 0.0:
                self.linear = self._clamp(self.speed_percent / 100.0, 0.0, self.max_speed_percent / 100.0)
            elif self.linear < 0.0:
                self.linear = -self._clamp(self.speed_percent / 100.0, 0.0, self.max_speed_percent / 100.0)

    def _handle_command(self, command: str) -> bool:
        if not command:
            return True

        handled = True
        for char in command.lower():
            if char == 'w':
                self._set_forward()
            elif char == 's':
                with self.state_lock:
                    moving = abs(self.linear) > 1e-6
                    reverse_pending = self.reverse_pending
                if moving:
                    self._stop()
                    with self.state_lock:
                        self.reverse_pending = True
                else:
                    if reverse_pending:
                        self._set_reverse()
                    else:
                        self._set_reverse()
            elif char == 'j':
                self._stop()
            elif char == 'a':
                self._turn_left()
            elif char == 'd':
                self._turn_right()
            elif char == 'q':
                self._increase_speed()
            elif char == 'e':
                self._decrease_speed()
            elif char == 'x':
                self.shutdown_event.set()
                handled = False
            elif char in (' ', '\t'):
                continue
            else:
                self.get_logger().info("Use w=forward, s=stop/reverse, a/d=turn, q/e=speed, x=quit")
        return handled

    def _publish_loop(self) -> None:
        period_sec = 1.0 / max(1.0, self.publish_rate_hz)
        while rclpy.ok() and not self.shutdown_event.is_set():
            self.publish_cmd()
            time.sleep(period_sec)

    def publish_cmd(self) -> None:
        msg = Twist()
        with self.state_lock:
            msg.linear.x = float(self.linear)
            msg.angular.z = float(self.angular)
        self.publisher.publish(msg)
def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = KeyboardControllerNode()
    tty_stream = None
    try:
        try:
            tty_stream = open("/dev/tty", "r")
        except OSError:
            tty_stream = sys.stdin

        print("Keyboard Controller")
        print("Type a command and press Enter:")
        print("w = forward")
        print("s = stop if moving, otherwise reverse")
        print("a = turn left 15 deg (capped at 45 deg)")
        print("d = turn right 15 deg (capped at 45 deg)")
        print("q = increase speed by 15")
        print("e = decrease speed by 15")
        print("j = stop")
        print("x = quit")
        while rclpy.ok() and not node.shutdown_event.is_set():
            try:
                print("cmd> ", end="", flush=True)
                command = tty_stream.readline().strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                break

            if not node._handle_command(command):
                break

            with node.state_lock:
                linear = node.linear
                angular = node.angular
                speed_percent = node.speed_percent
                steer_angle_deg = node.steer_angle_deg
            print(
                f"set speed={speed_percent:.0f}%, steer={steer_angle_deg:+.0f}deg, "
                f"linear.x={linear:+.2f}, angular.z={angular:+.2f}"
            )
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_event.set()
        node.linear = 0.0
        node.angular = 0.0
        node.publish_cmd()
        if tty_stream is not None and tty_stream is not sys.stdin:
            tty_stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
