# coding: utf-8
import smbus
import time
import threading
import sys

## DRV8830 Default I2C slave address
SLAVE_ADDRESS_LEFT  = 0x65
SLAVE_ADDRESS_RIGHT  = 0x64
## PCA9685 Default I2C slave address
PCA9685_ADDRESS = 0x40

''' DRV8830 Register Addresses '''
## sample rate driver
CONTROL = 0x00

## Value of Lidar
ACQ_COMMAND = 0x00
STATUS = 0x01
ACQ_CONFIG_REG = 0x04
FULL_DELAY_HIGH = 0x0f
FULL_DELAY_LOW = 0x10

## Value motor.
FORWARD = 0x01
BACK = 0x02
STOP = 0x00

## Value of servlo
CONTROL_REG = 0x00
OSC_CLOCK = 25000000

PWM0_ON_L = 0x06
PWM0_ON_H = 0x07
PWM0_OFF_L = 0x08
PWM0_OFF_H = 0x09

ALL_PWM_ON_L = 0xFA
ALL_PWM_ON_H = 0xFB
ALL_PWM_OFF_L = 0xFC
ALL_PWM_OFF_H = 0xFD
PRE_SCALE = 0xFE

SLEEP_BIT = 0x10

#PWMを50Hzに設定
PWM_HZ = 50


class RobotMouse(threading.Thread):

    def __init__(self, address_right=SLAVE_ADDRESS_RIGHT, address_left=SLAVE_ADDRESS_LEFT): 
        self.address_right = address_right
        self.address_left = address_left
        ## smbus
        self.bus = smbus.SMBus(1)
        threading.Thread.__init__(self)
        
    
    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
    
    def right_forward(self, speed):
        if speed < 0:
            print "value is under 0,  must define 1-100 as speed."
            return
        elif speed > 100:
            print "value is over 100,  must define 1-100 as speed."
            return
        self.direction = FORWARD
        s = self.map(speed, 1, 100, 1, 58)
        sval = FORWARD | ((s+5)<<2) #スピードを設定して送信するデータを1Byte作成
        self.bus.write_i2c_block_data(self.address_right,CONTROL,[sval]) #生成したデータを送信
    
    # speedは0-100で指定
    def right_back(self, speed):
        if speed < 0:
            print "value is under 0,  must define 1-100 as speed."
            return
        elif speed > 100:
            print "value is over 100,  must define 1-100 as speed."
            return
        self.direction = BACK
        s= self.map(speed, 1, 100, 1, 58)
        sval = BACK| ((s+5)<<2) #スピードを設定して送信するデータを1Byte作成
        self.bus.write_i2c_block_data(self.address_right,CONTROL,[sval]) #生成したデータを送信
        
    def right_stop(self):
        self.bus.write_i2c_block_data(self.address_right,CONTROL,[STOP]) #モータへの電力の供給を停止(惰性で動き続ける)
        
    def left_forward(self, speed):
        if speed < 0:
            print "value is under 0,  must define 1-100 as speed."
            return
        elif speed > 100:
            print "value is over 100,  must define 1-100 as speed."
            return
        self.direction = FORWARD
        s = self.map(speed, 1, 100, 1, 58)
        sval = FORWARD | ((s+5)<<2) #スピードを設定して送信するデータを1Byte作成
        self.bus.write_i2c_block_data(self.address_left,CONTROL,[sval]) #生成したデータを送信
    
    # speedは0-100で指定
    def left_back(self, speed):
        if speed < 0:
            print "value is under 0,  must define 1-100 as speed."
            return
        elif speed > 100:
            print "value is over 100,  must define 1-100 as speed."
            return
        self.direction = BACK
        s= self.map(speed, 1, 100, 1, 58)
        sval = BACK| ((s+5)<<2) #スピードを設定して送信するデータを1Byte作成
        self.bus.write_i2c_block_data(self.address_left,CONTROL,[sval]) #生成したデータを送信
        
    def left_stop(self):
        self.bus.write_i2c_block_data(self.address_left,CONTROL,[STOP]) #モータへの電力の供給を停止(惰性で動き続ける)
    
    def right_brake(self):
        self.bus.write_i2c_block_data(self.address_right,CONTROL,[0x03]) #モータをブレーキさせる
    
    def left_brake(self):
        self.bus.write_i2c_block_data(self.address_left,CONTROL,[0x03]) #モータをブレーキさせる
    
