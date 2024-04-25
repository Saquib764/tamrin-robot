import cv2
from cv2 import aruco


cv_image = cv2.imread("aruco_input.png") 

gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()


# create aruco markers
SIZE = 512

for i in range(11):
    marker = aruco.generateImageMarker(aruco_dict, i, SIZE)
    cv2.imwrite(f"src/tamrin_robot/arucos/aruco-marker-ID={i}.png", marker)

# detector = aruco.ArucoDetector(aruco_dict, parameters)

# corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

# print(corners, ids)
