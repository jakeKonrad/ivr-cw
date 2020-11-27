#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import math

GREEN = [0,255,0]
BLUE = [255,0,0]
RED = [0,0,255]
ORANGE = [0, 150, 255]

def get_blob_center(img, color):
    
    color_ = np.uint8([[color]])
    hsv_color = cv2.cvtColor(color_,cv2.COLOR_BGR2HSV)

    h = hsv_color[0][0][0]

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lo = np.array([h-10,100,100])
    hi = np.array([h+10,255,255])

    mask = cv2.inRange(hsv,lo,hi)

    kernel = np.ones((5,5), np.uint8)

    mask = cv2.dilate(mask, kernel, iterations=3)

    M = cv2.moments(mask)

    if int(M['m00']) == 0:
        return np.array([0,0])

    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    return np.array([cx, cy])


def pixel2Meter(img):
    x = get_blob_center(img, BLUE)
    y = get_blob_center(img, GREEN)
    dist = np.sum((x - y) ** 2)
    return 3.5 / np.sqrt(dist)

def joint_angles(img):
    a = pixel2Meter(img)
    blue_center = get_blob_center(img, BLUE)
    green_center = get_blob_center(img, GREEN)
    red_center = get_blob_center(img, RED) 
    ja2 = np.arctan2(blue_center[0] - green_center[0], blue_center[1] - blue_center[1])
    ja3 = np.arctan2(blue_center[0] - green_center[0], blue_center[1] - blue_center[1]) - ja2
    ja4 = np.arctan2(green_center[0] - red_center[0], green_center[1] - red_center[1]) - ja2 - ja3
    return np.array([ja2, ja3, ja4])

def goal_angles(t):
    ja2 = (math.pi/2) * math.sin((math.pi / 15) * t)
    ja3 = (math.pi/2) * math.sin((math.pi / 18) * t)
    ja4 = (math.pi/2) * math.sin((math.pi / 20) * t)
    return np.array([ja2, ja3, ja4])

def get_target(img):
    color = ORANGE
    color_ = np.uint8([[color]])
    hsv_color = cv2.cvtColor(color_,cv2.COLOR_BGR2HSV)

    h = hsv_color[0][0][0]

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lo = np.array([h-10,100,100])
    hi = np.array([h+10,255,255])

    mask = cv2.inRange(hsv,lo,hi)

    kernel = np.ones((5,5), np.uint8)

    mask = cv2.dilate(mask, kernel, iterations=3)

    contours,hierarchy = cv2.findContours(mask,1,2)

    contours.sort(key=lambda cnt: (2 * math.sqrt(cv2.contourArea(cnt) * math.pi)) / cv2.arcLength(cnt, True))

    cnt = contours[0]

    M = cv2.moments(cnt)

    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    orange_center = np.array([cx,cy])
    blue_center = get_blob_center(img, BLUE)

    a = pixel2Meter(img)

    vect = a * (blue_center - orange_center)

    return vect

def get_joint_state_black(img):
    color = [0,0,0]
    color_ = np.uint8([[color]])
    hsv_color = cv2.cvtColor(color_,cv2.COLOR_BGR2HSV)

    h = hsv_color[0][0][0]

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lo = np.array([h-10,100,100])
    hi = np.array([h+10,255,255])

    mask = cv2.inRange(hsv,lo,hi)

    kernel = np.ones((5,5), np.uint8)

    mask = cv2.dilate(mask, kernel, iterations=3)

    contours,hierarchy = cv2.findContours(mask,1,2)

    contours = reversed(contours)

    def centers(x):
        M = cv2.moments(x)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return np.array([cx,cy])

    contour_centers = list(map(centers, contours))

    a = pixel2Meter(img)

    blue_center = contour_centers[0]
    green_center = contour_centers[1]
    red_center = contour_centers[2]

    ja2 = np.arctan2(blue_center[0] - green_center[0], blue_center[1] - blue_center[1])
    ja3 = np.arctan2(blue_center[0] - green_center[0], blue_center[1] - blue_center[1]) - ja2
    ja4 = np.arctan2(green_center[0] - red_center[0], green_center[1] - red_center[1]) - ja2 - ja3
    
    return np.array([ja2, ja3, ja4])

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()


  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    ja = joint_angles(self.cv_image1)
    t = rospy.get_time()
    ja_ = goal_angles(t)
    pub_ja2 = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10)
    pub_ja3 = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10)
    pub_ja4 = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=10)

    est_ja2 = rospy.Publisher('/robot/joint2_position_estimate', Float64, queue_size=10)
    est_ja3 = rospy.Publisher('/robot/joint3_position_estimate', Float64, queue_size=10)
    est_ja4 = rospy.Publisher('/robot/joint4_position_estimate', Float64, queue_size=10)

    target_y = rospy.Publisher('/robot/target_y_estimate', Float64, queue_size=10)
    target_z = rospy.Publisher('/robot/target_z_estimate', Float64, queue_size=10)

    target = get_target(self.cv_image1)

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      pub_ja2.publish(ja_[0])
      pub_ja3.publish(ja_[1])
      pub_ja4.publish(ja_[2])
      
      est_ja2.publish(ja[0])
      est_ja3.publish(ja[1])
      est_ja4.publish(ja[2])

      target_y.publish(target[0])
      target_z.publish(target[1])

    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


