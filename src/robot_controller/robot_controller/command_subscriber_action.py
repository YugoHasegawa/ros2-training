import rclpy
from rclpy.node import Node
import pigpio
import threading
from geometry_msgs.msg import Twist

class CommandSubscriberAction(Node):
    def __init__(self, pidController_R, pidController_L):
        super().__init__('command_subscriber_action')
        self.subscription = self.create_subscription(Twist,'cmd_vel', self.listener_callback, 10)
        self.subscription # prevent unused variable warning

        self.pidController_R = pidController_R
        self.pidController_L = pidController_L

    def listener_callback(self, Twist):
        self.get_logger().info(f'並進速度={Twist.linear.x} 角速度={Twist.angular.z}')

        target_speed_R = Twist.linear.x + Twist.angular.z
        self.pidController_R.setTargetSpeed(3*target_speed_R)

        target_speed_L = Twist.linear.x - Twist.angular.z
        self.pidController_L.setTargetSpeed(3*target_speed_L)

class Encoder:
    def __init__(self, pi, pin_number):
        self.pi = pi
        self.pin_number = pin_number
        self.count = 0
        self.positive = 0

        self.initPin()

    def initPin(self):
        self.pi.set_mode(self.pin_number, pigpio.INPUT)
        self.pi.set_pull_up_down(self.pin_number, pigpio.PUD_UP)
        self.pi.callback(self.pin_number, pigpio.EITHER_EDGE, self.onEdge)

    def setSpinDirection(self, positive):
        self.positive = positive

    def onEdge(self, gpio, level, tick):
        if self.positive:
            self.count += 1
        else:
            self.count -= 1

    def reset(self):
        self.count = 0

class Motor:
    def __init__(self, pi, pin_1, pin_2):
        self.pi = pi
        self.pin_1 = pin_1
        self.pin_2 = pin_2

        self.initPin(pin_1)
        self.initPin(pin_2)

    def initPin(self, number):
        self.pi.set_mode(number, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(number, 60)
        self.pi.set_PWM_range(number, 100)

    def turn(self, duty):
        MAX_DUTY = 100.0
        if duty > 0:  
            if duty > MAX_DUTY:
                duty = MAX_DUTY
            self.pi.set_PWM_dutycycle(self.pin_1, duty)
            self.pi.set_PWM_dutycycle(self.pin_2, 0)
        else:
            if duty < -MAX_DUTY:
                duty = -MAX_DUTY
            self.pi.set_PWM_dutycycle(self.pin_1, 0)
            self.pi.set_PWM_dutycycle(self.pin_2, -duty)

class PidController:
    def __init__(self, encoder, motor):
        self.encoder = encoder
        self.motor = motor
        self.target_speed = 0
        self.init_variables()

    def init_variables(self):
        self.prev_count = 0
        self.err_prev = 0
        self.err_I = 0

    def drive(self):
        DURATION = 0.1

        if self.target_speed > -0.01 and self.target_speed < 0.01:
            self.encoder.reset()
            self.motor.turn(0)
            self.init_variables()
        else:
            Kp = 20
            Ki = 100
            Kd = 0.1
            speed = (self.encoder.count - self.prev_count)/40/DURATION
            err_P = self.target_speed - speed
            self.err_I += err_P * DURATION
            err_D = (err_P - self.err_prev)/DURATION
            duty = Kp * err_P + Ki * self.err_I + Kd * err_D
            self.motor.turn(duty)
            self.prev_count = self.encoder.count
            self.err_prev = err_P 

        t = threading.Timer(DURATION, self.drive)
        t.start()

    def setTargetSpeed(self, target_speed):
        self.target_speed = target_speed
        self.encoder.setSpinDirection(target_speed > 0)

def main(args=None):
    pi = pigpio.pi()

    encoder_R = Encoder(pi, 10)
    encoder_L = Encoder(pi, 2)

    motor_R = Motor(pi, 18, 17)
    motor_L = Motor(pi, 23, 22)

    pidController_R = PidController(encoder_R, motor_R)
    pidController_L = PidController(encoder_L, motor_L)

    motor_R.turn(0)
    motor_L.turn(0)

    rclpy.init(args=args)
    command_subscriber_action = CommandSubscriberAction(pidController_R, pidController_L)

    pidController_R.drive()
    pidController_L.drive()
    rclpy.spin(command_subscriber_action)

    command_subscriber_action.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


