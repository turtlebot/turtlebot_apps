#! /usr/bin/env python
#sample run command for using the kinect as a capture device
#rosrun pano_ros capture_pano.py image:=/kinect/rgb/image_raw camera_info:=/kinect/rgb/camera_info

import roslib; roslib.load_manifest('pano_ros')
import rospy

import actionlib
import numpy
import pano_ros.msg
import ros
import cv
import tempfile
import rosbag
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import os


class StitchJob(object):
    bag_name = "default.bag"
    stitch_file_name = "default.jpg"
    camera_info = CameraInfo()
    def __init__(self,bag_name,stitch_file_name):
        self.bag_name = bag_name
        self.stitch_file_name = stitch_file_name
        self.bridge = CvBridge()
    def unstuffBag(self):
        bag = rosbag.Bag(self.bag_name)
        saved_camera_info = False
        img_n = 0
        pano_dir = tempfile.mkdtemp("pano")
        image_names = []
        
        for topic, msg, t in bag.read_messages(topics=['image', 'camera_info']):
            if ('image' in topic):
                try:
                    cv_image = self.bridge.imgmsg_to_cv(msg, "rgb8")                 
                    image_name = "%s/image_%05d.png" % (pano_dir , img_n)
                    image_names.append("image_%05d.png" % img_n)
                    cv.SaveImage(image_name,cv_image)
                    rospy.loginfo("saving image to %s",image_name)
                    img_n += 1                    
                except CvBridgeError, e:
                    print e
            if ('camera_info' in topic and not saved_camera_info):
                mK = numpy.array(msg.K)
      
                mK = numpy.reshape(mK, (3,3), 'C' )
                K = cv.fromarray(mK)

                mD = numpy.array(msg.D)    
                mD = numpy.reshape(mD, (5,1), 'C' )           
                D = cv.fromarray(mD)

                cv.Save(pano_dir +"/K.yml",K)
                cv.Save(pano_dir +"/D.yml",D)
               
                saved_camera_info = True
                
                
        bag.close()
        pano_name = self.stitch_file_name
        images = ""
        for x in image_names:
            images += x + " "
        cmd = 'rosrun pano_core stitcher -d %s -K %s -o %s %s' %(pano_dir,"%s/K.yml"% pano_dir,pano_name,images)
        os.system(cmd)
        
        
class BagStitcher(object):
  # create messages that are used to publish feedback/result
  _panos = {}
 
  def __init__(self, name):
    self._action_name = name
    self._stitch_ac = actionlib.SimpleActionServer(self._action_name + "/stitch", pano_ros.msg.StitchAction, execute_cb=self.stitch)


  def stitch(self, goal):
    # helper variables
    stitch_job = StitchJob(goal.bag_file_name,goal.stitch_file_name)
    stitch_job.unstuffBag()
    self._stitch_ac.set_succeeded(pano_ros.msg.StitchResult(result_flags=pano_ros.msg.StitchResult.PANO_SUCCESS))
    
    
if __name__ == '__main__':
  try:    
    
      rospy.init_node('bag_stitcher')
      BagStitcher(rospy.get_name())
      rospy.spin()
  except rospy.ROSInterruptException:
        print "program interrupted before completion"
