# coding: utf-8
import smbus
import Fabo_PCA9685

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


class Handle():
    CHANNEL = 0 # PCA9685 サーボ接続チャネル

    # サーボの限界回転位置
    SERVO_MIN_VALUE = 150
    SERVO_MAX_VALUE = 450
    # サーボの中央位置
    SERVO_NEUTRAL_VALUE = 300
    # サーボのステアリングとしての稼働可能角
    HANDLE_MAX_VALUE = 45

    def __init__(self, PCA9685, channel=0):
        self.CHANNEL = channel
        self.PCA9685 = PCA9685

    def right(self):
        self.angle(self.SERVO_NEUTRAL_VALUE + self.HANDLE_MAX_VALUE)

    def left(self):
        self.angle(self.SERVO_NEUTRAL_VALUE - self.HANDLE_MAX_VALUE)

    def forward(self, value=None):
        if value is None:
            value = self.SERVO_NEUTRAL_VALUE
        '''
        ハンドルをニュートラル位置に戻す
        引数valueはニュートラル位置を更新する
        '''
        if not self.handle_angle_validation(value):
            return

        # 引数valueをニュートラル位置に更新する
        self.SERVO_NEUTRAL_VALUE = value
        self.PCA9685.set_channel_value(self.CHANNEL, self.SERVO_NEUTRAL_VALUE)

    def angle(self, value):
        if not self.handle_angle_validation(value):
            return
        self.PCA9685.set_channel_value(self.CHANNEL, value)

    def get_angle(self):
        return self.PCA9685.get_channel_value(self.CHANNEL)

    def servo_angle_validation(self,value):
        '''
        引数valueがサーボの可動範囲内かどうかを確認する
        '''
        # バリデーション: SERVO_MIN_VALUE <= value <= SERVO_MAX_VALUE
        if not (self.SERVO_MIN_VALUE <= value):
            return False
        if not (value <= self.SERVO_MAX_VALUE):
            return False
        return True

    def handle_angle_validation(self,value):
        '''
        引数valueがハンドルの可動範囲内かどうかを確認する
        '''
        if not self.servo_angle_validation(value):
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
        self.PCA9685 = Fabo_PCA9685.PCA9685(self.bus)
        # self.PCA9685.set_freq(50) default: 50 Hz
        channel = 0
        self.handle = Handle(self.PCA9685,channel)

    def motor_forward(self, speed):
        self.motor.forward(speed)

    def motor_stop(self):
        self.motor.stop()

    def motor_back(self, speed):
        self.motor.back(speed)

    def motor_brake(self):
        self.motor.brake()

    def handle_right(self):
        self.handle.right()

    def handle_left(self):
        self.handle.left()

    def handle_forward(self,value=None):
        self.handle.forward(value)

    def handle_angle(self, value):
        self.handle.angle(value)

    def get_handle_angle(self):
        return self.handle.get_angle()
