import rclpy
from geometry_msgs.msg import Twist
import numpy as np

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025



# System parameters
M = 0.5  # Mass of the pendulum
m = 0.2  # Mass of the cart
l = 0.3  # Length to the pendulum center of mass
g = 9.81  # Gravity
dt = 0.01  # Time step

# State-space matrices
A = np.array([[0, 1],
              [(M+m)*g*l/(M*l**2), 0]])

B = np.array([[0],
              [1/(M*l)]])

C = np.array([[1, 0]])

D = np.array([[0]])

# LQR controller gains
Q = np.array([[100, 0],
              [0, 10]])
R = np.array([[1]])
K, _, _ = np.linalg.lqr(A, B, Q, R)

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # camera
        # accelerometer
        # gyro
        # left_wheel_joint
        # left_wheel_joint_sensor
        # right_wheel_joint
        # right_wheel_joint_sensor

        robot = self.__robot
        num_devices = robot.getNumberOfDevices()
        for i in range(num_devices):
          device = robot.getDeviceByIndex(i)
          print("- {}".format(device.getName()))
        
        # setup gyroscopic sensor
        self.__gyro = self.__robot.getDevice('gyro')
        self.__gyro.enable(1)

        # setup accelerometer sensor
        self.__accelerometer = self.__robot.getDevice('accelerometer')
        self.__accelerometer.enable(1)

        # Initialize state variables
        self.x = np.array([ [0],  # Initial angle
                            [0]])  # Initial angular velocity

          
        # self.__left_motor = self.__robot.getDevice('left wheel motor')
        # self.__right_motor = self.__robot.getDevice('right wheel motor')

        # self.__left_motor.setPosition(float('inf'))
        # self.__left_motor.setVelocity(0)

        # self.__right_motor.setPosition(float('inf'))
        # self.__right_motor.setVelocity(0)

        # self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        gyro_reading = self.__gyro.getValues()
        print("Gyro reading: {}".format(gyro_reading))

        accelerometer_reading = self.__accelerometer.getValues()
        print("Accelerometer reading: {}".format(accelerometer_reading))



        # forward_speed = self.__target_twist.linear.x
        # angular_speed = self.__target_twist.angular.z

        # command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        # command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        # self.__left_motor.setVelocity(command_motor_left)
        # self.__right_motor.setVelocity(command_motor_right)
        pass
    
if __name__ == '__main__':
    print("Hello World-------------")
