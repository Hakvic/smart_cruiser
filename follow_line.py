#!/usr/bin/env python3.5
import time
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C, speed_to_speedvalue, SpeedNativeUnits
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.sound import Sound


class Robot:
    def __init__(self):
        self.tank_drive = MoveTank(OUTPUT_C, OUTPUT_B)
        self.sensor_1 = ColorSensor("in2")
        self.sensor_2 = ColorSensor("in1")
        self.sensor_3 = ColorSensor("in3")
        self.sensor_4 = UltrasonicSensor("in4")
        self.sensor_4.mode = "US-DIST-CM"
        self.sensor_1.mode = "COL-REFLECT"
        self.sensor_2.mode = "COL-REFLECT"
        self.sensor_3.mode = "COL-REFLECT"
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
        self.stop()
        if not self.sensor_1.value() < 10 and not self.sensor_2.value() < 10 and not self.sensor_3.value() < 10:
            if self.sensor_2.value() < 30:
                self.tank_drive.on_for_degrees(0, 30, 250)
            #left_speed = -10
                #right_speed = 30
            if self.sensor_3.value() < 30:
                self.tank_drive.on_for_degrees(30, 0, 250)
            #left_speed = 30
                #right_speed = -10
        distance = self.sensor_4.value()/10
        if (distance <= 25 and distance > 20):
            left_speed = SpeedNativeUnits((speed_native_units - u) * 0.75)
            right_speed = SpeedNativeUnits((speed_native_units + u) * 0.75)
        elif (distance <= 20 and distance > 15):
            left_speed = SpeedNativeUnits((speed_native_units - u) * 0.5)
            right_speed = SpeedNativeUnits((speed_native_units + u) * 0.5)
        elif (distance <= 15 and distance > 10):
            left_speed = SpeedNativeUnits((speed_native_units - u) * 0.25)
            right_speed = SpeedNativeUnits((speed_native_units + u) * 0.25)
        elif distance <= 10:
            left_speed = 0
            right_speed = 0
        print("left_speed: ", left_speed)
        print("right_speed: ", right_speed)
        return left_speed, right_speed

    def stop(self):
        if self.sensor_2.value() < 10 and self.sensor_3.value() < 10:
            self.tank_drive.stop()
            self.run = False

    def main(self, speed, kp, ki, kd):
        Sound().play_song((('D4','e3'), ('D4','e3')))
        target = self.sensor_1.value()
        while self.run:
            left_speed, right_speed = self.pid(target, speed, kp, ki, kd)
            self.tank_drive.on(left_speed, right_speed)
            #self.stop()
            print("value: ", self.sensor_1.value())
            print("counter: ", self.stop_count)
            print("\n")
            # time.sleep(0.5)