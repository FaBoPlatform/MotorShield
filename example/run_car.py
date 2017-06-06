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

r = MotorShield.RobotCar()
r.handle_zero(HANDLE_NEUTRAL)

r.car_forward(100)

def main():
    try:
        while True:
            # 人力確認
            #value = r.handle_value()
            #print(value)
            #ANGLE = float(raw_input('Enter angle: '))
            #HANDLE_ANGLE = ANGLE
            #r.handle_move(HANDLE_ANGLE)

            r.handle_move(HANDLE_RIGHT)
            time.sleep(1)
            r.handle_move(HANDLE_NEUTRAL)
            time.sleep(1)
            r.handle_move(HANDLE_LEFT)
            time.sleep(1)
            r.handle_move(HANDLE_NEUTRAL)
            time.sleep(2)


    except KeyboardInterrupt:
        pass

main()

r.car_stop()
r.handle_zero()
print("END")
