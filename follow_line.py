#!/usr/bin/env python3.5

import time
import logging
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C, speed_to_speedvalue, SpeedNativeUnits
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.sound import Sound

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
# file_handler = logging.FileHandler("debug.log")
# file_handler.setLevel(logging.DEBUG)
# logger.addHandler(file_handler)
logger.addHandler(handler)


class Robot:
    def __init__(self):
        self.tank_drive = MoveTank(OUTPUT_C, OUTPUT_B)
        self.sensor_middle = ColorSensor("in1")
        self.sensor_left = ColorSensor("in2")
        self.sensor_right = ColorSensor("in3")
        self.sensor_ultrasonic = UltrasonicSensor("in4")
        self.sensor_ultrasonic.mode = "US-DIST-CM"
        self.sensor_middle.mode = "COL-REFLECT"
        self.sensor_left.mode = "COL-REFLECT"
        self.sensor_right.mode = "COL-REFLECT"
        self.target = 0
        self.run = True

    def pid(self, speed, kp, ki, kd):
        integral = 0.0
        last_error = 0.0
        derivative = 0.0
        dt = 500
        speed = speed_to_speedvalue(speed)
        speed_native_units = speed.to_native_units(self.tank_drive.left_motor)

        error = self.target - self.sensor_middle.value()
        integral += (error * dt)
        derivative = (error - last_error) / dt

        u = (kp * error) + (ki * integral) + (kd * derivative)
        left_speed = SpeedNativeUnits(speed_native_units - u)
        right_speed = SpeedNativeUnits(speed_native_units + u)
        logger.debug("u: {}".format(u))
        logger.debug("left speed: {}".format(left_speed))
        logger.debug("right speed: {}".format(right_speed))
        
        return left_speed, right_speed

    def turn(self):

        # logger.debug("before sensor 1: {}".format(self.sensor_middle.value()))
        # logger.debug("before sensor 2: {}".format(self.sensor_left.value()))
        # logger.debug("before sensor 3: {}".format(self.sensor_left.value()))
        if self.sensor_left.value() < self.target:
            logger.debug("left stop")
            self.tank_drive.stop()
            self.tank_drive.on(0, 10)
            while self.sensor_middle.value() > 20 or self.sensor_left.value() < self.target:
                logger.debug("while LEFT")
                logger.debug("sensor middle: {}".format(self.sensor_middle.value()))
                logger.debug("sensor left: {}".format(self.sensor_left.value()))
                self.tank_drive.on(0, 10)
            logger.debug("sensor middle after while: {}".format(self.sensor_middle.value()))
            logger.debug("sensor left after while: {}".format(self.sensor_left.value()))

        if self.sensor_right.value() < self.target:
            logger.debug("right stop")
            self.tank_drive.stop()
            self.tank_drive.on(10, 0)
            while self.sensor_middle.value() > 20 or self.sensor_right.value() < self.target:
                logger.debug("while RIGHT")
                logger.debug("sensor middle: {}".format(self.sensor_middle.value()))
                logger.debug("sensor right: {}".format(self.sensor_right.value()))
                self.tank_drive.on(10, 0)
            logger.debug("sensor middle after while: {}".format(self.sensor_middle.value()))
            logger.debug("sensor right after while: {}".format(self.sensor_right.value()))

    def cruiser(self, left_speed, right_speed):

        distance = self.sensor_ultrasonic.value() / 10
        if 25 >= distance > 20:
            left_speed = left_speed * 0.75
            right_speed = right_speed * 0.75
        elif 20 >= distance > 15:
            left_speed = left_speed * 0.5
            right_speed = right_speed * 0.5
        elif 15 >= distance > 10:
            left_speed = left_speed * 0.25
            right_speed = right_speed * 0.25
        elif distance <= 10:
            left_speed = 0
            right_speed = 0

        return left_speed, right_speed

    def stop(self):
        if self.sensor_left.value() < 10 and self.sensor_right.value() < 10:
            logger.debug("STOP")
            self.tank_drive.stop()
            self.run = False

    def main(self, speed, kp, ki, kd):
        Sound().play_song((('D4', 'e3'), ('D4', 'e3')))
        self.target = self.sensor_middle.value()
        logger.debug("target: {}".format(self.target))
        while self.run:
            left_speed, right_speed = self.pid(speed, kp, ki, kd)
            # left_speed, right_speed = self.cruiser(left_speed, right_speed)
            self.turn()
            self.tank_drive.on(left_speed, right_speed)
            self.stop()
            # time.sleep(0.5)
