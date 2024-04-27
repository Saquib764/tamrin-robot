import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf_transformations
from tf_transformations import quaternion_from_euler, quaternion_from_matrix, translation_from_matrix
from tf_transformations import translation_matrix, quaternion_matrix, concatenate_matrices



class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('detect_aruco')
        self.subscription = self.create_subscription(
            Image,
            '/my_robot/camera/image_color',  # Replace with your camera topic
            self.image_callback, 10)
        self.subscription  # prevent unused variable warning

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/my_robot/camera/camera_info',  # Replace with your camera info topic
            self.camera_info_callback, 10)
        self.cv_bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_length = 0.25  # Marker length in meters

        # Transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)


    def camera_info_callback(self, msg):
        # Assuming no distortion
        self.camera_matrix = np.array(msg.k).reshape([3, 3])
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().error("Camera matrix not set")
            return
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error("Error converting image: %s" % str(e))
            return

        # Detect ArUco markers
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)



        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

        # print(corners, ids)
        # im = Image.fromarray(cv_image)
        # im.save("aruco_input.png")
        # cv2.imwrite("aruco_input.png", cv_image)
        # Draw detected markers
        if ids is not None:
            cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            # Draw the axes of the detected markers
            cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs, tvecs, 0.1)

            for rvec, tvec in zip(rvecs, tvecs):
                now = rclpy.time.Time()

                R, _ = cv2.Rodrigues(rvec[0])
                quaternion = quaternion_from_matrix(np.r_[np.c_[R, [0, 0, 0]], [[0, 0, 0, 1]]])

                # Create a 4x4 transformation matrix
                transform_camera_marker_matrix = concatenate_matrices(
                    translation_matrix(tvec[0]),
                    quaternion_matrix(quaternion)
                )

                try:

                    transform_base_camera = self.tf_buffer.lookup_transform('base_link', 'camera', now)
                    translation = [
                        transform_base_camera.transform.translation.x,
                        transform_base_camera.transform.translation.y,
                        transform_base_camera.transform.translation.z
                    ]
                    rotation = [
                        transform_base_camera.transform.rotation.x,
                        transform_base_camera.transform.rotation.y,
                        transform_base_camera.transform.rotation.z,
                        transform_base_camera.transform.rotation.w
                    ]

                    # Create a 4x4 transformation matrix
                    transform_base_camera_matrix = concatenate_matrices(
                        translation_matrix(translation),
                        quaternion_matrix(rotation)
                    )

                    transform_base_marker_matrix = transform_base_camera_matrix.dot(transform_camera_marker_matrix)


                    quaternion = quaternion_from_matrix(transform_base_marker_matrix)
                    translation = translation_from_matrix(transform_base_marker_matrix)
                    print("Transform Base to Marker: ", translation, quaternion)


                    # transformed_pose = self.tf_buffer.transform(marker_pose, 'shaft_link', rclpy.duration.Duration(seconds=1))
                    # self.get_logger().info("Transformed Marker Pose: %s" % transformed_pose)
                    
                except Exception as e:
                    self.get_logger().info(f'Failed to transform marker pose: {str(e)}')


        # Display the image with markers
        cv2.imshow("ArUco Marker Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArUcoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
