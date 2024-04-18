import rclpy
from geometry_msgs.msg import Twist
import numpy as np
import control
import datetime

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025



# System parameters
M = 3  # Mass of the pendulum
m = 10  # Mass of the cart
l = 0.4  # Length to the pendulum center of mass
g = 9.81  # Gravity
dt = 0.01  # Time step

# State-space matrices
A = np.array([[0, 1],
              [g/l, 0]])

B = np.array([[0],
              [1/(M*l*l)]])

C = np.array([[0, 1]])

D = np.array([[0]])

# LQR controller gains
Q = np.array([[1., 0],
              [0, 1.]])
R = np.array([[1]])
K, _, _ = control.lqr(A, B, Q, R)

K = np.array([[3, 5]])

print("K: {}".format(K))

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__node = None

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
        self.__gyro.enable(int(dt * 1000))

        # setup accelerometer sensor
        self.__accelerometer = self.__robot.getDevice('accelerometer')
        self.__accelerometer.enable(int(dt * 1000))

        # Initialize state variables
        self.x = np.array([ [0.0],  # Initial angle
                            [0.0]])  # Initial angular velocity
        
        self.u_prev = 0

          
        self.__left_motor = self.__robot.getDevice('left_wheel_joint')
        self.__right_motor = self.__robot.getDevice('right_wheel_joint')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        # self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__t = datetime.datetime.now()
        # self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, dt * 1000)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        # if not self.__node:
        #     return
        _dt = (datetime.datetime.now() - self.__t).total_seconds()

        print("dt: {}".format(_dt))

        self.__t = datetime.datetime.now()
        
        rclpy.spin_once(self.__node, timeout_sec= int(dt * 0))

        gyro_reading = self.__gyro.getValues()
        print("Gyro reading: {}".format(gyro_reading))

        accelerometer_reading = self.__accelerometer.getValues()
        print("Accelerometer reading: {}".format(accelerometer_reading), np.arcsin(accelerometer_reading[0]/g) )

        self.x[0,0] += gyro_reading[1] * _dt
        a = np.linalg.norm(accelerometer_reading)
        # self.x[0,0] = -np.arcsin(accelerometer_reading[0]/a)

        self.x[1,0] = gyro_reading[1]

        # self.x = self.x + _dt * (A @ self.x + B * (self.u_prev))


        u = -K @ self.x
        print("u: {}".format(u), "K: {}".format(K), "x: {}".format(self.x))

        u_ = u[0,0]

        self.__left_motor.setTorque(-u_)
        self.__right_motor.setTorque(u_)

        self.u_prev = u_
        pass
    
if __name__ == '__main__':
    print("Hello World-------------")
