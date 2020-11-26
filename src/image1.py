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

GREEN = [0,255,0]
BLUE = [255,0,0]
RED = [0,0,255]

def get_blob_center(img, color):
    
    color_ = np.uint8([[color]])
    hsv_color = cv2.cvtColor(color_,cv2.COLOR_BGR2HSV)

    h = hsv_color[0][0][0]

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lo = np.array([h-10,100,100])
    hi = np.array([h+10,255,255])

    mask = cv2.inRange(hsv,lo,hi)

    res = cv2.bitwise_and(img,img,mask=mask)

    res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(res,(5,5),0)

    ret, th = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    contours,hierarchy = cv2.findContours(th, 1, 2)

    cnt = contours[0]
    M = cv2.moments(cnt)

    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    return np.array([cx, cy])

def pixel2Meter(img):
    x = get_joint(img, BLUE)
    y = get_joint(img, GREEN)
    dist = np.sum((x - y) ** 2)
    return 3.5 / np.sqrt(dist)

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

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
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


