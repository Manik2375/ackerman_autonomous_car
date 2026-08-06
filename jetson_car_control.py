#!/usr/bin/env python3
"""Jetson Nano controller for the I2C motor driver and steering servo.

This script mirrors the Arduino sketch:
- initializes the motor driver over I2C at address 0x34
- sets motor type and encoder polarity
- drives forward, backward, stop, and steering left/right

Wiring on Jetson Nano:
- I2C SDA: physical pin 3 (J41 header, I2C1_SDA)
- I2C SCL: physical pin 5 (J41 header, I2C1_SCL)
- Servo PWM: physical pin 33 by default in this script

Required packages on Jetson Nano:
- smbus2
- Jetson.GPIO

Run with:
- python3 jetson_car_control.py
"""

from __future__ import annotations

import time
from dataclasses import dataclass

try:
    from smbus2 import SMBus
except ImportError as exc:  # pragma: no cover - runtime dependency on Jetson
    raise SystemExit("Missing dependency: smbus2. Install with: pip3 install smbus2") from exc

try:
    import Jetson.GPIO as GPIO
except ImportError as exc:  # pragma: no cover - runtime dependency on Jetson
    raise SystemExit("Missing dependency: Jetson.GPIO. Install it on Jetson Nano first.") from exc


I2C_BUS = 1
I2C_ADDR = 0x34

ADC_BAT_ADDR = 0
MOTOR_TYPE_ADDR = 20
MOTOR_ENCODER_POLARITY_ADDR = 21
MOTOR_FIXED_PWM_ADDR = 31
MOTOR_FIXED_SPEED_ADDR = 51
MOTOR_ENCODER_TOTAL_ADDR = 60

MOTOR_TYPE_WITHOUT_ENCODER = 0
MOTOR_TYPE_TT = 1
MOTOR_TYPE_N20 = 2
MOTOR_TYPE_JGB37_520_12V_110RPM = 3

# Encoder motor target speed used by the fixed-speed register.
MOTOR_SPEED = 120

SERVO_PIN = 33  # physical pin 33 on the J41 header, PWM-capable on Jetson Nano
SERVO_FREQUENCY_HZ = 50


@dataclass
class MotorCommand:
    values: tuple[int, int, int, int]


CAR_FORWARD = MotorCommand((-MOTOR_SPEED, 0, MOTOR_SPEED, 0))
CAR_RETREAT = MotorCommand((MOTOR_SPEED, 0, -MOTOR_SPEED, 0))
CAR_STOP = MotorCommand((0, 0, 0, 0))


class CarController:
    def __init__(self, bus: int = I2C_BUS, address: int = I2C_ADDR) -> None:
        self.address = address
        self.bus = SMBus(bus)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQUENCY_HZ)
        self.servo_pwm.start(0)
        self.current_servo_angle = 90

    @staticmethod
    def clamp_angle(angle: int) -> int:
        return max(0, min(180, angle))

    @staticmethod
    def angle_to_duty_cycle(angle: int) -> float:
        # Matches the Arduino pulse range of about 500us to 2500us at 50Hz.
        pulse_us = 500 + (CarController.clamp_angle(angle) * (2500 - 500) / 180.0)
        return (pulse_us / 20000.0) * 100.0

    def write_byte(self, value: int) -> None:
        self.bus.write_byte(self.address, value & 0xFF)

    def write_data_array(self, register: int, values: list[int] | tuple[int, ...]) -> None:
        payload = [register & 0xFF] + [value & 0xFF for value in values]
        self.bus.write_i2c_block_data(self.address, payload[0], payload[1:])

    def setup_motor_driver(self) -> None:
        self.write_data_array(MOTOR_TYPE_ADDR, [MOTOR_TYPE_JGB37_520_12V_110RPM])
        time.sleep(0.005)
        self.write_data_array(MOTOR_ENCODER_POLARITY_ADDR, [0])
        self.set_servo_angle(90)
        time.sleep(0.2)

    def set_servo_angle(self, angle: int) -> None:
        duty_cycle = self.angle_to_duty_cycle(angle)
        self.servo_pwm.ChangeDutyCycle(duty_cycle)
        self.current_servo_angle = self.clamp_angle(angle)

    def drive(self, command: MotorCommand, duration_s: float) -> None:
        self.write_data_array(MOTOR_FIXED_SPEED_ADDR, list(command.values))
        time.sleep(duration_s)

    def stop(self, duration_s: float = 1.0) -> None:
        self.drive(CAR_STOP, duration_s)

    def cleanup(self) -> None:
        try:
            self.stop(0.2)
        except Exception:
            pass
        try:
            self.servo_pwm.ChangeDutyCycle(0)
            self.servo_pwm.stop()
        except Exception:
            pass
        try:
            self.bus.close()
        except Exception:
            pass
        GPIO.cleanup()


def main() -> None:
    controller = CarController()
    try:
        controller.setup_motor_driver()

        controller.drive(CAR_FORWARD, 4.0)
        controller.stop(1.0)

        controller.drive(CAR_RETREAT, 4.0)
        controller.stop(1.0)

        controller.set_servo_angle(45)
        controller.drive(CAR_FORWARD, 4.0)
        controller.stop(1.0)
        controller.drive(CAR_RETREAT, 4.0)
        controller.set_servo_angle(90)
        controller.stop(1.0)

        controller.set_servo_angle(145)
        controller.drive(CAR_FORWARD, 4.0)
        controller.stop(1.0)
        controller.drive(CAR_RETREAT, 4.0)
        controller.stop(1.0)
        controller.set_servo_angle(90)

        while True:
            time.sleep(1)
    finally:
        controller.cleanup()


if __name__ == "__main__":
    main()
