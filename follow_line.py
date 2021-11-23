#!/usr/bin/env python3.5

import time
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C, speed_to_speedvalue, SpeedNativeUnits
from ev3dev2.sensor.lego import ColorSensor


class Robot:
    def __init__(self):
        self.tank_drive = MoveTank(OUTPUT_C, OUTPUT_B)
        self.sensor_1 = ColorSensor("in1")
        # self.sensor_2 = ColorSensor("in2")
        # self.sensor_3 = ColorSensor("in3")
        self.sensor_1.mode = "COL-REFLECT"
        # self.sensor_2.mode = "COL-REFLECT"
        # self.sensor_3.mode = "COL-REFLECT"
        self.stop_count = 0
        self.run = True

    def pid(self, target, speed, kp, ki, kd):
        integral = 0.0
        last_error = 0.0
        derivative = 0.0
        dt = 500
        speed = speed_to_speedvalue(speed)
        speed_native_units = speed.to_native_units(self.tank_drive.left_motor)

        error = target - self.sensor_1.value()
        integral += (error * dt)
        derivative = (error - last_error) / dt

        u = (kp * error) + (ki * integral) + (kd * derivative)
        print("u: ", u)

        left_speed = SpeedNativeUnits(speed_native_units - u)
        right_speed = SpeedNativeUnits(speed_native_units + u)

        print("left_speed: ", left_speed)
        print("right_speed: ", right_speed)

        return left_speed, right_speed

    def stop(self, target, threshold_stop):
        if self.sensor_1.value() > threshold_stop:
            self.stop_count += 1
        else:
            self.stop_count = 0
        if self.stop_count > 60:
            self.tank_drive.stop()
            self.run = False

    def main(self, target, speed, kp, ki, kd, threshold_stop):
        while self.run:
            left_speed, right_speed = self.pid(target, speed, kp, ki, kd)
            self.tank_drive.on(left_speed, right_speed)
            self.stop(target, threshold_stop)
            print("value: ", self.sensor_1.value())
            print("counter: ", self.stop_count)
            print("\n")
            # time.sleep(0.5)
