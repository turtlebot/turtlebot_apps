#! /usr/bin/env python
#sample run command for using the kinect as a capture device
#rosrun pano_ros capture_pano.py image:=/kinect/rgb/image_raw camera_info:=/kinect/rgb/camera_info

import roslib; roslib.load_manifest('pano_py')
import rospy
import sys
import ros
import cv
import tempfile
import rosbag
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import os
import pano_py as pano
import pano_cv as pcv

class StitchJob(object):
    bag_name = "default.bag"
    stitch_file_name = "default.jpg"
    camera_info = CameraInfo()
    def __init__(self,bag_name,stitch_file_name):
        self.bag_name = bag_name
        self.stitch_file_name = stitch_file_name
        self.bridge = CvBridge()
    def stitch(self):
        bag = rosbag.Bag(self.bag_name)
        saved_camera_info = False
        img_n = 0
        pano_dir = tempfile.mkdtemp("pano")
        image_names = []
        
        camera = pano.Camera()
        blur_detector = pano.BlurDetector()
        
        for topic, msg, t in bag.read_messages(topics=['camera_info']):
            if ('camera_info' in topic):
                print "reading camera info"
                K = pcv.Mat(3,3,pcv.CV_32FC1)
                K.fromarray(msg.K)
                img_size = pcv.Size( msg.width, msg.height)                
                camera.setCameraIntrinsics(K,pcv.Mat(),img_size)             
                break
        
        options = pano.Options()
        options.camera = camera
        options.stitch_size = pcv.Size(4000,2000)
        options.directory = pano_dir
        #options.image_names.assign(image_names)
        params = options.fitter_params
        params.error_thresh = 6;
        params.inliers_thresh = 15;
        params.maxiters = 100;
        params.nNeeded = 2;
        options.stitch_output = self.stitch_file_name
        
       
        stitcher = pano.StitchEngine(options)
        
        for topic, msg, t in bag.read_messages(topics=['image']):
            if ('image' in topic):
                try:
                    cv_image = self.bridge.imgmsg_to_cv(msg, "rgb8")
                    b_prob =   blur_detector.checkBlur(cv_image)
                    print "image blur prob = %f" %b_prob         
                    pano_image = pcv.convertCvMat2Mat(cv_image)
                    stitcher.addNewImage(pano_image)
                    img_n += 1                    
                except CvBridgeError, e:
                    print e
                               
        bag.close()
        
        blended = cv.CreateMat(2000,4000,cv.CV_8UC3)
        
        stitcher.stitch(blended,self.stitch_cb)
        
        cv.NamedWindow("blended",pcv.CV_WINDOW_KEEPRATIO)
        cv.ShowImage("blended",blended)
        
        cv.SaveImage(self.stitch_file_name,blended)
        cv.waitKey(0)
        
       
        return True
        
    def stitch_cb(self,progress):
        print "Python progress %d" % progress
        return 0
    
if __name__ == '__main__':
    if len(sys.argv) != 3:
        print "usage:\n%s pano.bag stitch.jpg" % sys.argv[0]
    else:    
        stitch_job = StitchJob(sys.argv[1],sys.argv[2])
        success = stitch_job.stitch()
      
