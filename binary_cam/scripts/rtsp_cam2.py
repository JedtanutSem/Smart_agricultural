#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os



rospy.init_node('image_converter', anonymous=True)
image_pub = rospy.Publisher("image_topic_2",Image)
bridge = CvBridge()

RTSP_URL = 'rtsp://192.168.1.3/live/ch00_0'

os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;udp'

cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)

if not cap.isOpened():
    print('Cannot open RTSP stream')
    exit(-1)

while True:
    _, frame = cap.read()
    cv2.imshow('RTSP stream', frame)
    image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
