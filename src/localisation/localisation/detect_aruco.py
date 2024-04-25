import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
# from PIL import Image

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('detect_aruco')
        self.subscription = self.create_subscription(
            Image,
            '/my_robot/camera/image_color',  # Replace with your camera topic
            self.image_callback, 10)
        self.subscription  # prevent unused variable warning
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
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

        print(corners, ids)
        # im = Image.fromarray(cv_image)
        # im.save("aruco_input.png")
        # cv2.imwrite("aruco_input.png", cv_image)
        # Draw detected markers
        if ids is not None:
            cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)

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
