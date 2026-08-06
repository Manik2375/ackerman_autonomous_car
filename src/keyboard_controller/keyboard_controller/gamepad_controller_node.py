#!/usr/bin/env python3

from __future__ import annotations

import glob
import os
import select
import struct
import threading
import time
from typing import Final

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

JS_EVENT_BUTTON: Final[int] = 0x01
JS_EVENT_AXIS: Final[int] = 0x02
JS_EVENT_INIT: Final[int] = 0x80
JS_EVENT_STRUCT: Final[struct.Struct] = struct.Struct("IhBB")


class GamepadControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("gamepad_controller_node")

        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("js_device", "")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("steer_deadzone", 0.12)
        self.declare_parameter("axis_lx", 0)
        self.declare_parameter("axis_ly", 1)
        self.declare_parameter("axis_lt", 2)
        self.declare_parameter("axis_rt", 5)
        self.declare_parameter("initial_speed_percent", 30.0)
        self.declare_parameter("max_speed_percent", 100.0)
        self.declare_parameter("speed_change_rate_percent_per_sec", 45.0)
        self.declare_parameter("max_linear_output", 5.0)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.js_device = str(self.get_parameter("js_device").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.steer_deadzone = float(self.get_parameter("steer_deadzone").value)
        self.axis_lx = int(self.get_parameter("axis_lx").value)
        self.axis_ly = int(self.get_parameter("axis_ly").value)
        self.axis_lt = int(self.get_parameter("axis_lt").value)
        self.axis_rt = int(self.get_parameter("axis_rt").value)
        self.initial_speed_percent = float(self.get_parameter("initial_speed_percent").value)
        self.max_speed_percent = float(self.get_parameter("max_speed_percent").value)
        self.speed_change_rate_percent_per_sec = float(
            self.get_parameter("speed_change_rate_percent_per_sec").value
        )
        self.max_linear_output = float(self.get_parameter("max_linear_output").value)

        self.publisher = self.create_publisher(Twist, self.cmd_topic, 20)
        self.shutdown_event = threading.Event()
        self.state_lock = threading.Lock()
        self.left_x = 0.0
        self.left_y = 0.0
        self.lt = 0.0
        self.rt = 0.0
        self.speed_percent = self._clamp(self.initial_speed_percent, 0.0, self.max_speed_percent)

        self.device_path = self._resolve_device_path(self.js_device)
        self.device_fd = os.open(self.device_path, os.O_RDONLY)
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.publisher_thread = threading.Thread(target=self._publisher_loop, daemon=True)
        self.reader_thread.start()
        self.publisher_thread.start()

        self.get_logger().info(
            f"Gamepad controller active on {self.device_path}; left stick steers/drives, triggers change speed."
        )

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _normalize_stick(value: int) -> float:
        normalized = value / 32767.0
        if abs(normalized) < 0.12:
            return 0.0
        return GamepadControllerNode._clamp(normalized, -1.0, 1.0)

    @staticmethod
    def _normalize_trigger(value: int) -> float:
        if value < 0:
            normalized = (value + 32767) / 65534.0
        else:
            normalized = value / 32767.0
        if normalized < 0.0:
            return 0.0
        if normalized > 1.0:
            return 1.0
        return normalized

    @staticmethod
    def _resolve_device_path(js_device: str) -> str:
        if js_device:
            if os.path.exists(js_device):
                return js_device
            raise RuntimeError(f"Gamepad device not found: {js_device}")

        candidates = sorted(glob.glob("/dev/input/js*"))
        if not candidates:
            raise RuntimeError("No joystick device found under /dev/input/js*.")
        return candidates[0]

    def _handle_axis(self, axis_number: int, value: int) -> None:
        with self.state_lock:
            if axis_number == self.axis_lx:
                self.left_x = self._normalize_stick(value)
            elif axis_number == self.axis_ly:
                self.left_y = self._normalize_stick(value)
            elif axis_number == self.axis_lt:
                self.lt = self._normalize_trigger(value)
            elif axis_number == self.axis_rt:
                self.rt = self._normalize_trigger(value)

    def _reader_loop(self) -> None:
        while rclpy.ok() and not self.shutdown_event.is_set():
            try:
                ready, _, _ = select.select([self.device_fd], [], [], 0.1)
                if not ready:
                    continue
                raw = os.read(self.device_fd, JS_EVENT_STRUCT.size)
                if len(raw) != JS_EVENT_STRUCT.size:
                    continue
                _timestamp_ms, value, event_type, number = JS_EVENT_STRUCT.unpack(raw)
                event_type &= ~JS_EVENT_INIT
                if event_type == JS_EVENT_AXIS:
                    self._handle_axis(number, value)
                elif event_type == JS_EVENT_BUTTON:
                    continue
            except OSError:
                break
            except Exception:
                self.get_logger().exception("Gamepad reader loop failed")
                break

        self.shutdown_event.set()

    def _publisher_loop(self) -> None:
        period_sec = 1.0 / max(1.0, self.publish_rate_hz)
        last_time = time.monotonic()

        while rclpy.ok() and not self.shutdown_event.is_set():
            now = time.monotonic()
            dt = max(0.0, now - last_time)
            last_time = now

            with self.state_lock:
                speed_delta = (
                    (self.rt - self.lt)
                    * self.speed_change_rate_percent_per_sec
                    * dt
                )
                self.speed_percent = self._clamp(
                    self.speed_percent + speed_delta,
                    0.0,
                    self.max_speed_percent,
                )
                linear = -self.left_y * (self.speed_percent / 100.0) * self.max_linear_output
                angular = 0.0 if abs(self.left_x) < self.steer_deadzone else self.left_x

            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.publisher.publish(msg)

            time.sleep(period_sec)

    def destroy_node(self) -> bool:
        self.shutdown_event.set()
        try:
            os.close(self.device_fd)
        except Exception:
            pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GamepadControllerNode()
    try:
        print("Gamepad Controller")
        print("Left stick drives and steers.")
        print("Left trigger decreases speed, right trigger increases speed.")
        print("Press Ctrl+C to quit.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
