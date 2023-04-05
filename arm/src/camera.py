#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Pose

class ObjectDetector:

    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/robot/camera1/image_raw', Image, self.image_callback)
        self.pose_pub = rospy.Publisher('/myrobot', Pose, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Define the color range to detect red box
        lower_color_range = np.array([0, 100, 100])
        upper_color_range = np.array([10, 255, 255])
        # Threshold the image to get the binary mask of the object
        mask = cv2.inRange(hsv_image, lower_color_range, upper_color_range)
        # Find contours in the binary mask
        contours, _ = cv2.findContours(mask , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)[-2:]

        if len(contours) > 0:
            object_contour = max(contours, key=cv2.contourArea)

            # Get the bounding box of the object contour
            x, y, w, h = cv2.boundingRect(object_contour)
            rect = cv2.minAreaRect(object_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Compute the pose of the object based on the bounding box
            pose = Pose()
            center , size, angal = rect
            x1, y1 =center
            if x1>=210:
             x = 0.00416*(x1-210)+0.8
            else:
              x = 0.00416*(210-x1)+0.8
            if y1 >=210:
             y= (-y1+210)*0.0041353  
            else:
               y=y1*0.004135
            pose.position.x = x 
            pose.position.y = y 
            pose.position.z = 0.495
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = angal
           
            print("center:", center)
            print("size:",size)
            print("rotation:", angal)
            
            print(x)
            print(y)
            print(rect)
            print(box)
            print(pose)

            self.pose_pub.publish(pose)

        
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)
        cv2.imshow('Object Detection', cv_image)
        cv2.waitKey(1)
if __name__ == '__main__':
    detector = ObjectDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
