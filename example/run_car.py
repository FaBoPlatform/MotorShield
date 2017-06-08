#!/usr/bin/python
# coding: utf-8 

from __future__ import division
import time
import MotorShield

print("start")


CALIBRATION = 0 # フロントタイヤがまっすぐになりそうな角度に調整
HANDLE_NEUTRAL = 310 + CALIBRATION # フロントタイヤがまっすぐの時のサーボ位置

HANDLE_MAX_ANGLE = 45 # フロントタイヤの左右最大角のサーボ位置
HANDLE_RIGHT = HANDLE_NEUTRAL + HANDLE_MAX_ANGLE
HANDLE_LEFT = HANDLE_NEUTRAL - HANDLE_MAX_ANGLE

car = MotorShield.RobotCar()
car.handle_forward(HANDLE_NEUTRAL) # タイヤのニュートラル位置を記憶し、設定する

car.motor_forward(100)

def main():
    try:
        while True:
            # 人力確認
            #value = car.get_handle_angle()
            #print(value)
            #ANGLE = float(raw_input('Enter angle: '))
            #HANDLE_ANGLE = ANGLE
            #car.handle_angle(HANDLE_ANGLE)

            car.handle_right()
            time.sleep(1)
            car.handle_forward()
            time.sleep(1)
            car.handle_left()
            time.sleep(1)
            car.handle_forward()
            time.sleep(1)
            car.handle_angle(HANDLE_RIGHT)
            time.sleep(1)
            car.handle_angle(HANDLE_NEUTRAL)
            time.sleep(1)
            car.handle_angle(HANDLE_LEFT)
            time.sleep(1)
            car.handle_angle(HANDLE_NEUTRAL)
            time.sleep(2)

    except KeyboardInterrupt:
        pass

main()

car.motor_stop()
car.handle_forward()
print("END")
