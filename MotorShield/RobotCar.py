# -*- coding: utf-8 -*-
import smbus
import time
import sys
import threading

class Motor():
    ## DRV8830 Default I2C address
    MOTOR_ADDR_L = 0x64
    MOTOR_ADDR_R = 0x65

    # DRV8830 Register Addresses
    COMMAND0 = 0x00

    ## Value motor.
    FORWARD = 0x01
    BACK = 0x02
    STOP = 0x00

    def __init__(self, bus, motor_address=MOTOR_ADDR_L):
        self.bus = bus
        self.MOTOR_ADDRESS = motor_address


    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    def forward(self, speed):
        if speed < 0:
            print("value is under 0,  must define 1-100 as speed.")
            return
        elif speed > 100:
            print("value is over 100,  must define 1-100 as speed.")
            return
        self.direction = self.FORWARD
        s = self.map(speed, 1, 100, 1, 58)
        sval = self.FORWARD | ((s+5)<<2) #スピードを設定して送信するデータを1Byte作成
        self.bus.write_i2c_block_data(self.MOTOR_ADDRESS,self.COMMAND0,[sval]) #生成したデータを送信

    def stop(self):
        self.bus.write_i2c_block_data(self.MOTOR_ADDRESS,self.COMMAND0,[self.STOP]) #モータへの電力の供給を停止(惰性で動き続ける)

    def back(self, speed):
        if speed < 0:
            print("value is under 0,  must define 1-100 as speed.")
            return
        elif speed > 100:
            print("value is over 100,  must define 1-100 as speed.")
            return
        self.direction = self.BACK
        s= self.map(speed, 1, 100, 1, 58)
        sval = self.BACK | ((s+5)<<2) #スピードを設定して送信するデータを1Byte作成
        self.bus.write_i2c_block_data(self.MOTOR_ADDRESS,self.COMMAND0,[sval]) #生成したデータを送信

    def brake(self):
        self.bus.write_i2c_block_data(self.MOTOR_ADDRESS,self.COMMAND0,[0x03]) #モータをブレーキさせる

class PCA9685(object):
    # PCA9685 Default I2C address
    PCA9685_ADDRESS = 0x40

    # Value of servlo
    MODE1 = 0x00
    OSC_CLOCK = 25000000

    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09

    PRE_SCALE = 0xFE

    ''' PCA9685 Registers
    # Register definitions
    MODE1 = 0x00 # Mode register 1
    MODE2 = 0x01 # Mode register 2
    SUBADR1 = 0x02 # I2C-bus subaddress 1
    SUBADR2 = 0x03 # I2C-bus subaddress 2
    SUBADR3 = 0x04 # I2C-bus subaddress 3
    ALLCALLADR = 0x05 # LED All Call I2C-bus address
    LED0_ON_L = 0x06 # LED0 output and brightness control byte 0
    LED0_ON_H = 0x07 # LED0 output and brightness control byte 1
    LED0_OFF_L = 0x08 # LED0 output and brightness control byte 2
    LED0_OFF_H = 0x09 # LED0 output and brightness control byte 3
    LED1_ON_L = 0A # LED1 output and brightness control byte 0
    LED1_ON_H = 0B # LED1 output and brightness control byte 1
    LED1_OFF_L = 0C # LED1 output and brightness control byte 2
    LED1_OFF_H = 0D # LED1 output and brightness control byte 3
    LED2_ON_L = 0E # LED2 output and brightness control byte 0
    LED2_ON_H = 0F # LED2 output and brightness control byte 1
    LED2_OFF_L = 10 # LED2 output and brightness control byte 2
    LED2_OFF_H = 11 # LED2 output and brightness control byte 3
    LED3_ON_L = 12 # LED3 output and brightness control byte 0
    LED3_ON_H = 13 # LED3 output and brightness control byte 1
    LED3_OFF_L = 14 # LED3 output and brightness control byte 2
    LED3_OFF_H = 15 # LED3 output and brightness control byte 3
    LED4_ON_L = 16 # LED4 output and brightness control byte 0
    LED4_ON_H = 17 # LED4 output and brightness control byte 1
    LED4_OFF_L = 18 # LED4 output and brightness control byte 2
    LED4_OFF_H = 19 # LED4 output and brightness control byte 3
    LED5_ON_L = 1A # LED5 output and brightness control byte 0
    LED5_ON_H = 1B # LED5 output and brightness control byte 1
    LED5_OFF_L = 1C # LED5 output and brightness control byte 2
    LED5_OFF_H = 1D # LED5 output and brightness control byte 3
    LED6_ON_L = 1E # LED6 output and brightness control byte 0
    LED6_ON_H = 1F # LED6 output and brightness control byte 1
    LED6_OFF_L = 20 # LED6 output and brightness control byte 2
    LED6_OFF_H = 21 # LED6 output and brightness control byte 3
    LED7_ON_L = 22 # LED7 output and brightness control byte 0
    LED7_ON_H = 23 # LED7 output and brightness control byte 1
    LED7_OFF_L = 24 # LED7 output and brightness control byte 2
    LED7_OFF_H = 25 # LED7 output and brightness control byte 3
    LED8_ON_L = 26 # LED8 output and brightness control byte 0
    LED8_ON_H = 27 # LED8 output and brightness control byte 1
    LED8_OFF_L = 28 # LED8 output and brightness control byte 2
    LED8_OFF_H = 29 # LED8 output and brightness control byte 3
    LED9_ON_L = 2A # LED9 output and brightness control byte 0
    LED9_ON_H = 2B # LED9 output and brightness control byte 1
    LED9_OFF_L = 2C # LED9 output and brightness control byte 2
    LED9_OFF_H = 2D # LED9 output and brightness control byte 3
    LED10_ON_L = 2E # LED10 output and brightness control byte 0
    LED10_ON_H = 2F # LED10 output and brightness control byte 1
    LED10_OFF_L = 30 # LED10 output and brightness control byte 2
    LED10_OFF_H = 31 # LED10 output and brightness control byte 3
    LED11_ON_L = 32 # LED11 output and brightness control byte 0
    LED11_ON_H = 33 # LED11 output and brightness control byte 1
    LED11_OFF_L = 34 # LED11 output and brightness control byte 2
    LED11_OFF_H = 35 # LED11 output and brightness control byte 3
    LED12_ON_L = 36 # LED12 output and brightness control byte 0
    LED12_ON_H = 37 # LED12 output and brightness control byte 1
    LED12_OFF_L = 38 # LED12 output and brightness control byte 2
    LED12_OFF_H = 39 # LED12 output and brightness control byte 3
    LED13_ON_L = 3A # LED13 output and brightness control byte 0
    LED13_ON_H = 3B # LED13 output and brightness control byte 1
    LED13_OFF_L = 3C # LED13 output and brightness control byte 2
    LED13_OFF_H = 3D # LED13 output and brightness control byte 3
    LED14_ON_L = 3E # LED14 output and brightness control byte 0
    LED14_ON_H = 3F # LED14 output and brightness control byte 1
    LED14_OFF_L = 40 # LED14 output and brightness control byte 2
    LED14_OFF_H = 41 # LED14 output and brightness control byte 3
    LED15_ON_L = 42 # LED15 output and brightness control byte 0
    LED15_ON_H = 43 # LED15 output and brightness control byte 1
    LED15_OFF_L = 44 # LED15 output and brightness control byte 2
    LED15_OFF_H = 45 # LED15 output and brightness control byte 3

    ALL_LED_ON_L = FA # load all the LEDn_ON registers, byte 0
    ALL_LED_ON_H = FB # load all the LEDn_ON registers, byte 1
    ALL_LED_OFF_L = FC # load all the LEDn_OFF registers, byte 0
    ALL_LED_OFF_H = FD # load all the LEDn_OFF registers, byte 1
    PRE_SCALE = FE # prescaler for PWM output frequency
    TestMode = FF # defines the test mode to be entered
    '''

    # COMMAND
    SLEEP_BIT = 0x10

    # PWMを50Hzに設定
    PWM_HZ = 50

    # シングルトンクラス
    __instance = None
    # ロックオブジェクト
    __lock = threading.Lock()


    def __init__(self, bus):
        self.bus = bus
        self.set_freq(self.PWM_HZ)
        print("init")

    def __new__(cls, *args, **kwargs):
        with cls.__lock:
            if cls.__instance is None:
                cls.__instance = object.__new__(cls)
            return cls.__instance

    def set_freq(self, hz):
        with self.__lock:
            if not hz == self.PWM_HZ:
                self.PWM_HZ = hz

            setval=int(round(self.OSC_CLOCK/(4096*hz))-1)
            ctrl_dat = self.bus.read_word_data(self.PCA9685_ADDRESS,self.MODE1)

            #スリープにする
            self.bus.write_i2c_block_data(self.PCA9685_ADDRESS,self.MODE1,[ctrl_dat | self.SLEEP_BIT])
            time.sleep(0.01)
            #周波数を設定
            self.bus.write_i2c_block_data(self.PCA9685_ADDRESS,self.PRE_SCALE,[setval])
            time.sleep(0.01)
            #スリープを解除
            self.bus.write_i2c_block_data(self.PCA9685_ADDRESS,self.MODE1,[ctrl_dat & (~self.SLEEP_BIT)])

    def get_channel_value(self, channel):
        with self.__lock:
            list_of_bytes = 1 # 16にしても同じ値が16個並ぶだけ
            block0 = self.bus.read_i2c_block_data(self.PCA9685_ADDRESS,self.LED0_OFF_L+channel*4,list_of_bytes) # 0-255
            block256 = self.bus.read_i2c_block_data(self.PCA9685_ADDRESS,self.LED0_OFF_H+channel*4,list_of_bytes) # 桁上がり
            value = (block256[0]<<8) + block0[0]
            return value

    def set_channel_value(self, channel, value):
        with self.__lock:
            print(channel, value)
            setval=int(value)
            # 最初からオン
            self.bus.write_i2c_block_data(self.PCA9685_ADDRESS,self.LED0_ON_L+channel*4,[0x00])
            self.bus.write_i2c_block_data(self.PCA9685_ADDRESS,self.LED0_ON_H+channel*4,[0x00])
            # Value％経過後にオフ
            self.bus.write_i2c_block_data(self.PCA9685_ADDRESS,self.LED0_OFF_L+channel*4,[setval & 0xff]) # 0-255
            self.bus.write_i2c_block_data(self.PCA9685_ADDRESS,self.LED0_OFF_H+channel*4,[setval>>8]) # 桁上がり

class Handle():
    CHANNEL = 0 # PCA9685 サーボ接続チャネル

    # サーボの限界回転位置
    SERVO_MIN_VALUE = 150
    SERVO_MAX_VALUE = 450
    # サーボの中央位置
    SERVO_NEUTRAL_VALUE = 300
    # サーボのステアリングとしての稼働可能角
    HANDLE_MAX_VALUE = 145

    def __init__(self, PCA9685, channel=0):
        self.CHANNEL = channel
        self.PCA9685 = PCA9685

    def set_handle(self, value):
        self.PCA9685.set_channel_value(self.CHANNEL, value)

    def set_neutral(self, value=SERVO_NEUTRAL_VALUE):
        '''
        ハンドルをニュートラル位置に戻す
        引数valueはニュートラル位置を更新する
        '''
        if not self.handle_validation(value):
            return

        # 引数valueをニュートラル位置に更新する
        if not value == self.SERVO_NEUTRAL_VALUE:
            self.SERVO_NEUTRAL_VALUE = value

        self.SERVO_NEUTRAL_VALUE = value
        self.PCA9685.set_channel_value(self.CHANNEL, self.SERVO_NEUTRAL_VALUE)

    def get_handle(self):
        return self.PCA9685.get_channel_value(self.CHANNEL)

    def set_handle(self, value):
        if not self.handle_validation(value):
            return
        self.PCA9685.set_channel_value(self.CHANNEL, value)

    def servo_validation(self,value):
        '''
        引数valueがサーボの可動範囲内かどうかを確認する
        '''
        # バリデーション: SERVO_MIN_VALUE <= value <= SERVO_MAX_VALUE
        if not (self.SERVO_MIN_VALUE <= value):
            return False
        if not (value <= self.SERVO_MAX_VALUE):
            return False
        return True

    def handle_validation(self,value):
        '''
        引数valueがハンドルの可動範囲内かどうかを確認する
        '''
        if not self.servo_validation(value):
            return False

        if not (self.SERVO_NEUTRAL_VALUE - self.HANDLE_MAX_VALUE <= value):
            return False
        if not (value <= self.SERVO_NEUTRAL_VALUE + self.HANDLE_MAX_VALUE):
            return False

        return True


class RobotCar():

    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.motor = Motor(self.bus)
        self.PCA9685 = PCA9685(self.bus)
        # self.PCA9685.set_freq(50) default: 50 Hz
        channel = 0
        self.handle = Handle(self.PCA9685,channel)

    def car_forward(self, speed):
        self.motor.forward(speed)

    def car_stop(self):
        self.motor.stop()

    def car_back(self, speed):
        self.motor.back(speed)

    def car_brake(self):
        self.motor.brake()

    def handle_init(self):
        return

    def handle_zero(self,value=300):
        self.handle.set_neutral(value)

    def handle_move(self, value):
        self.handle.set_handle(value)

    def handle_value(self):
        return self.handle.get_handle()
