# -*- coding: utf-8 -*- 
import smbus
import time
import threading
import RPi.GPIO as GPIO
import sys

## DRV8830 Default I2C slave address
SLAVE_ADDRESS_LEFT  = 0x64
SLAVE_ADDRESS_RIGHT  = 0x65
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

## smbus
bus = smbus.SMBus(1)
    
class RobotCar(threading.Thread):   
    flg = True
    myServo = ""
    
    def __init__(self, address=SLAVE_ADDRESS_LEFT): 
        self.address = address
        self.c = 0
        threading.Thread.__init__(self)
        print "init"
       
        self.flg = True
    
    def run(self):
        print "run"
        self.count()
    
    def refresh(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(HALL_SENSOR, GPIO.IN) #GPIOを入力に設定
        GPIO.setup(SERVO_PIN , GPIO.OUT) #GPIOを入力に設定
        self.myServo = GPIO.PWM(SERVO_PIN ,PWM_HZ) 
        self.myServo.start(9)
    
    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
                      
    def car_forward(self, speed):
        if speed < 0:
            print "value is under 0,  must define 1-100 as speed."
            return
        elif speed > 100:
            print "value is over 100,  must define 1-100 as speed."
            return
        self.direction = FORWARD
        s = self.map(speed, 1, 100, 1, 58)
        sval = FORWARD | ((s+5)<<2) #スピードを設定して送信するデータを1Byte作成
        bus.write_i2c_block_data(self.address,CONTROL,[sval]) #生成したデータを送信

    def car_stop(self):
        bus.write_i2c_block_data(self.address,CONTROL,[STOP]) #モータへの電力の供給を停止(惰性で動き続ける)
        
    def car_back(self, speed):
        if speed < 0:
            print "value is under 0,  must define 1-100 as speed."
            return
        elif speed > 100:
            print "value is over 100,  must define 1-100 as speed."
            return
        self.direction = BACK
        s= self.map(speed, 1, 100, 1, 58)
        sval = BACK| ((s+5)<<2) #スピードを設定して送信するデータを1Byte作成
        bus.write_i2c_block_data(self.address,CONTROL,[sval]) #生成したデータを送信

    def car_brake(self):
        bus.write_i2c_block_data(self.address,CONTROL,[0x03]) #モータをブレーキさせる
     
    def handle_init(self):
        self.set_freq(50)
    
    def handle_zero(self):
        self.set_PWM(0, 9)
        
    def set_freq(self, hz):
        setval=int(round(OSC_CLOCK/(4096*hz))-1)
        ctrl_dat = bus.read_word_data(PCA9685_ADDRESS,CONTROL_REG)

        #スリープにする
        bus.write_i2c_block_data(PCA9685_ADDRESS,CONTROL_REG,[ctrl_dat | SLEEP_BIT])
        time.sleep(0.01)
        #周波数を設定
        bus.write_i2c_block_data(PCA9685_ADDRESS,PRE_SCALE,[setval])
        time.sleep(0.01)
        #スリープを解除
        bus.write_i2c_block_data(PCA9685_ADDRESS,CONTROL_REG,[ctrl_dat & (~SLEEP_BIT)])
      
    def handle_move(self, direction):
        if direction < 7.5:
            return
        elif direction > 10.5:
            return
        self.set_PWM(0, direction)
        
    def set_PWM(self, pwmpin,value):
        if (value > 100):
            print "Error"
            return
        # 0~100を0~4096に変換
        setval=int(value*4096/100)
        # 最初からオン
        bus.write_i2c_block_data(PCA9685_ADDRESS,PWM0_ON_L+pwmpin*4,[0x00])
        bus.write_i2c_block_data(PCA9685_ADDRESS,PWM0_ON_H+pwmpin*4,[0x00])
        # Value％経過後にオフ
        bus.write_i2c_block_data(PCA9685_ADDRESS,PWM0_OFF_L+pwmpin*4,[setval & 0xff])
        bus.write_i2c_block_data(PCA9685_ADDRESS,PWM0_OFF_H+pwmpin*4,[setval>>8])