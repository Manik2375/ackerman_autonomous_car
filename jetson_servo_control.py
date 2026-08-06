#!/usr/bin/env python3
"""Standalone Jetson Nano servo controller with limited travel.

This script only drives the steering servo and keeps the motion small so it
does not collide with nearby hardware.

Wiring on Jetson Nano:
- Servo signal: physical pin 33 on the J41 header

Required package on Jetson Nano:
- Jetson.GPIO

Run with:
- python3 jetson_servo_control.py
"""

from __future__ import annotations

import time

try:
    import Jetson.GPIO as GPIO
except ImportError as exc:  # pragma: no cover - runtime dependency on Jetson
    raise SystemExit("Missing dependency: Jetson.GPIO. Install it on Jetson Nano first.") from exc


SERVO_PIN = 33  # physical pin 33 on the J41 header, PWM-capable on Jetson Nano
SERVO_FREQUENCY_HZ = 50
SERVO_CENTER_ANGLE = 90
SERVO_TRAVEL_DEGREES = 10  # keep the sweep small to avoid hitting nearby parts
SERVO_STEP_DEGREES = 2
SERVO_PAUSE_S = 0.4


class ServoController:
    def __init__(self) -> None:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQUENCY_HZ)
        self.servo_pwm.start(0)
        self.current_angle = SERVO_CENTER_ANGLE

    @staticmethod
    def clamp_angle(angle: int) -> int:
        return max(0, min(180, angle))

    @staticmethod
    def angle_to_duty_cycle(angle: int) -> float:
        pulse_us = 500 + (ServoController.clamp_angle(angle) * (2500 - 500) / 180.0)
        return (pulse_us / 20000.0) * 100.0

    def set_angle(self, angle: int) -> None:
        clamped_angle = self.clamp_angle(angle)
        self.servo_pwm.ChangeDutyCycle(self.angle_to_duty_cycle(clamped_angle))
        self.current_angle = clamped_angle

    def sweep_once(self) -> None:
        center = SERVO_CENTER_ANGLE
        left_limit = center - SERVO_TRAVEL_DEGREES
        right_limit = center + SERVO_TRAVEL_DEGREES

        for angle in range(center, right_limit + 1, SERVO_STEP_DEGREES):
            self.set_angle(angle)
            time.sleep(0.08)

        time.sleep(SERVO_PAUSE_S)

        for angle in range(right_limit, left_limit - 1, -SERVO_STEP_DEGREES):
            self.set_angle(angle)
            time.sleep(0.08)

        time.sleep(SERVO_PAUSE_S)

        self.set_angle(center)

    def cleanup(self) -> None:
        try:
            self.servo_pwm.ChangeDutyCycle(0)
            self.servo_pwm.stop()
        except Exception:
            pass
        GPIO.cleanup()


def main() -> None:
    controller = ServoController()
    try:
        controller.set_angle(SERVO_CENTER_ANGLE)
        time.sleep(0.5)

        while True:
            controller.sweep_once()
    finally:
        controller.cleanup()


if __name__ == "__main__":
    main()