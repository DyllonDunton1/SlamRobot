import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pigpio

class MotorDriver(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.pi = pigpio.pi()
        if self.pi.connected:
            print("CONNECTED")
        else:
            print("NOT CONNECTED")

        # Motor A: Left Motor
        self.LEFT_PWM1 = 12
        self.LEFT_PWM2 = 13

        # Motor B: Right Motor
        self.RIGHT_PWM1 = 18
        self.RIGHT_PWM2 = 19

        self.pwm_range = 255
        self.max_speed = 1.0

        for pin in [self.LEFT_PWM1, self.LEFT_PWM2, self.RIGHT_PWM1, self.RIGHT_PWM2]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(pin, 10000)
            self.pi.set_PWM_range(pin, self.pwm_range)
	

    def cmd_callback(self, msg):
        print("GOT MESSAGE: ", msg)
        v = msg.linear.x
        omega = msg.angular.z
        wheel_base = 0.2  # Distance between wheels

        left_speed = v - omega * wheel_base / 2
        right_speed = v + omega * wheel_base / 2
        
        print(left_speed, right_speed)

        self.set_motor(self.LEFT_PWM1, self.LEFT_PWM2, left_speed)
        self.set_motor(self.RIGHT_PWM1, self.RIGHT_PWM2, right_speed)

    def set_motor(self, pin1, pin2, speed):
        
        duty = min(int(abs(speed / self.max_speed) * self.pwm_range), self.pwm_range)
        print(duty)

        if speed > 0:
            self.pi.set_PWM_dutycycle(pin1, duty)
            self.pi.set_PWM_dutycycle(pin2, 0)
        elif speed < 0:
            self.pi.set_PWM_dutycycle(pin1, 0)
            self.pi.set_PWM_dutycycle(pin2, duty)
        else:
            self.pi.set_PWM_dutycycle(pin1, 0)
            self.pi.set_PWM_dutycycle(pin2, 0)

    def destroy_node(self):
        # Stop all motors on exit
        for pin in [self.LEFT_PWM1, self.LEFT_PWM2, self.RIGHT_PWM1, self.RIGHT_PWM2]:
            self.pi.set_PWM_dutycycle(pin, 0)
        self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

