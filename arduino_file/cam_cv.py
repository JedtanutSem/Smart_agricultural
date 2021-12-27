import cv2
import numpy as np
import time
cam = cv2.VideoCapture(1)
if not cam.isOpened():
    print('Cannot open camera')

    while True:
     ret,frame = cam.read()
     cv2.imshow('webcam', frame)
     if cv2.waitKey(1)&0xFF == ord('q'):
      break


cv2.destroyAllWindows()