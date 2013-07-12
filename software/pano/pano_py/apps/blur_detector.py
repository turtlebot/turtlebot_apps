#! /usr/bin/env python
import roslib; roslib.load_manifest('pano_py')
import rospy
import cv
import time
import pano_py as pano
cv.NamedWindow("camera", 1)

capture = cv.CaptureFromCAM(0)

blur_d = pano.BlurDetector()
while True:
    img = cv.QueryFrame(capture)
    #print type(img)
    pof_blur = blur_d.checkBlur( img )
    print "blurred:",pof_blur
    cv.ShowImage("camera", img)
    key = 0xff & cv.WaitKey(10)
    if key == ord('q'):
        break