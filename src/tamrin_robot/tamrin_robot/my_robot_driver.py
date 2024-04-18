import rclpy
from geometry_msgs.msg import Twist
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
import numpy as np
import control
import datetime



# System parameters
M = 1.2  # Mass of the pendulum
m = 10  # Mass of the cart
l = 0.4  # Length to the pendulum center of mass
r = 0.1
g = 9.81  # Gravity
dt = 0.01  # Time step

# State-space matrices
A = np.array([[0.0, 1.0, 0.],
              [g/l, 0., 0.],
              [0., 0., 0.]])

B = np.array([[0],
              [1/(M*l*l)],
              [-r/(m+M)]])

C = np.array([[0, 1]])

D = np.array([[0]])

# LQR controller gains
Q = np.array([[1., 0., 0.],
              [0., 1., 0.],
              [0., 0., 1.]])
R = np.array([[1]])
K, _, _ = control.lqr(A, B, Q, R)

K = np.array([[6., 5., 0.8]])

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
                            [0.0],  # Initial angular velocity
                            [0.0]]) # Velocity
        
        # Initialize camera
        self.camera = self.__robot.getDevice('camera')
        self.camera.enable(int(dt * 1000))

        self.u_prev = 0

          
        self.__left_motor = self.__robot.getDevice('left_wheel_joint')
        self.__right_motor = self.__robot.getDevice('right_wheel_joint')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__t = datetime.datetime.now()
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 0)


    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        # if not self.__node:
        #     return
        _dt = (datetime.datetime.now() - self.__t).total_seconds()

        self.__t = datetime.datetime.now()
        
        rclpy.spin_once(self.__node, timeout_sec= int(dt * 0))

        gyro_reading = self.__gyro.getValues()

        accelerometer_reading = self.__accelerometer.getValues()

        self.x[0,0] += gyro_reading[1] * _dt
        self.x[1,0] = gyro_reading[1]

        a = -(accelerometer_reading[0] - g * np.sin(self.x[0,0])) / np.cos(self.x[0,0])
        self.x[2,0] += a * _dt

        # print("dt: {}".format(_dt))

        # ar = self.get_aruco_pose_by_id(0)

        # print("ARUCO: {}".format(ar)    )


        # self.x = self.x + _dt * (A @ self.x + B * (self.u_prev))
        

        target = np.array([[0.0], [0.0], [self.__target_twist.linear.x]])

        u = -K @ (self.x - target)


        u_ = u[0,0]

        rotate = -(self.__target_twist.angular.z - gyro_reading[2]) * 2.0


        print("u: {}".format(u), "rotate: {}".format(rotate), "x: {}".format(self.x), "Twist: {}".format(self.__target_twist))

        self.__left_motor.setTorque(-u_ + rotate)
        self.__right_motor.setTorque(u_ + rotate)

        self.u_prev = u_
        pass
    
if __name__ == '__main__':
    print("Hello World-------------")
