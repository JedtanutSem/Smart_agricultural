#!/usr/bin/env python
from __future__ import print_function
#test

#test3
import roslib
roslib.load_manifest('binary_cam')
import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import json
from binary_cam.msg import BinData

thresh_cv = 0
number_of_white_pix = 0
number_of_black_pix = 0

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      global number_of_white_pix
      global number_of_black_pix

      #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
      #ret,thresh = cv2.threshold(cv_image, thresh_cv, 255, cv2.THRESH_BINARY)
      #number_of_white_pix = np.sum(thresh == 255)
      #number_of_black_pix = np.sum(thresh == 0)
      #rets,corners = cv2.findChessboardCorners(cv_image,(7,7),None)
      #if rets == True:
          #corners2 = cv2.cornerSubPix(cv_image,corners,(11,11),(-1,-1),criteria)
      #cv_image = cv2.drawChessboardCorners(cv_image, (7,7), corners2, rets)
      #rospy.loginfo("corn0:%s",corners2)

      #rospy.loginfo(number_of_white_pix)
      #rospy.loginfo(thresh_cv)
      #cv_image = cv2.cvtColor(im_pillow, cv2.COLOR_GRAY2BGR)


    dimension = cv_image.shape
    rospy.loginfo(dimension)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))#mono8 for gray
      #pub = rospy.Publisher('/pixel_value',BinData,queue_size=10)
      #rate = rospy.Rate(100)

      #rate.sleep()
    except CvBridgeError as e:
      print(e)

def cira_data_clbk(data):
    global json_data
    global thresh_cv
    get = data.data
    json_acceptable_string = get.replace("'", "\"")
    json_data = json.loads(json_acceptable_string)
    thresh_cv = json_data['thresh_cv']
    #rospy.loginfo(thresh_cv)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  rospy.Subscriber('/from_cira', String, cira_data_clbk)


  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
