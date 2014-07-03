#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: capture_server.py 47238 2010-12-17 22:23:01Z kwc $
"""
Panorama capture action server.

Authors: Ethan Rublee, Ken Conley

Sample run command for using the kinect as a capture device:

rosrun pano_ros capture_server.py camera:=/kinect/rgb
"""

#import roslib; roslib.load_manifest('pano_ros')
import os
import tempfile
import threading
import rospy

from pano_ros.msg import PanoCaptureAction, PanoCaptureResult, PanoCaptureFeedback
from std_msgs.msg import Empty
from sensor_msgs.msg import Image, CameraInfo

import actionlib
import rosbag
import message_filters

import pano_py as pano
import pano_cv as pcv
import cv2
from cv2 import cv
import numpy
from cv_bridge import CvBridge, CvBridgeError

class ImageGrabber:
    """
    Helper class for subscribing to synchronized camera and camera
    info data.
    """

    def __init__(self, image_topic, camera_info_topic, capture_fn):
        
        self._image_sub = message_filters.Subscriber(image_topic, Image)
        self._info_sub = message_filters.Subscriber(camera_info_topic, CameraInfo)
        self._ts = message_filters.TimeSynchronizer([self._image_sub, self._info_sub], 3)
        self._ts.registerCallback(capture_fn) 

    def stop(self):
        # stop subscribing
        self._image_sub.sub.unregister()
        self._info_sub.sub.unregister()

        del self._image_sub
        del self._info_sub        
        del self._ts
    
class PanoCaptureJob(object):

    def __init__(self, pano_id, capture_fn):
        self.pano_id = pano_id
        self.capture_fn = capture_fn
        self.result = None

    def __call__(self, image, info):
        """
        Call job with latest captured imaged
        """
        # passthrough
        self.capture_fn(image, info)
        
    def _get_n_captures(self):
        return self.capture_fn.n_captures if self.capture_fn else self.result.n_captures

    n_captures = property(_get_n_captures)
    
    def cancel(self):
        if self.capture_fn is None:
            return
        capturer = self.capture_fn
        self.capture_fn = None
        capturer.cancel()
        
    def stop(self):
        if self.capture_fn is None:
            raise Exception("already stopped")
        capturer = self.capture_fn
        capturer.stop()
        #todo get rid of capture_fn.bag_filename
        self.result = PanoCaptureResult(pano_id=self.pano_id,
                                        n_captures=capturer.n_captures,
                                        bag_filename=capturer.bag_filename)
        self.capture_fn = None

class CaptureInterface(object):
    """
    Common capture interface, like capture to bag, capture and republish, etc...
    """
    def start(self,pano_id, goal):
        """
        set up a new pano.
        """
        raise NotImplemented
    def __call__(self, image, camera_info):
        """
        capture your synced image and camera_info here
        """
        raise NotImplemented

    def stop(self):
        """
        done capturing, do clean up, close bags, etc.
        """
        raise NotImplemented
    
    def cancel(self):
        """
        no matter what stop capturing, you've been prempted
        """
        raise NotImplemented
class PanoCaptureServer(object):
    def __init__(self, name, capture_interface = CaptureInterface):
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name, 
                                                    PanoCaptureAction, execute_cb=self.pano_capture)
        
        # triggers during action execution
        self._snap_sub = rospy.Subscriber(rospy.names.ns_join(self._action_name, 'snap'), Empty, self.capture_snap)
        self._stop_sub = rospy.Subscriber(rospy.names.ns_join(self._action_name, 'stop'), Empty, self.capture_stop)

        # initialized during active job
        self._capture_job = None
        self._snap_requested = False
        self._capture_interface = capture_interface

        
    def pano_capture(self, goal):
        if self._capture_job is not None:
            raise Exception("cannot run multile capture jobs. TODO: pre-eempt existing job")
        
        rospy.loginfo('%s: Pano Capture Goal: \n\tCamera [%s]'%(self._action_name, goal.camera_topic))
                             
        pano_id = int(rospy.get_time())
        #TODO make this a parameter, bag, publisher, etc...
        capturer = self._capture_interface()#= BagCapture(pano_id,goal.bag_filename or None)
        capturer.start(pano_id,goal)
                
        self._snap_requested = False #reset
        capture_job = self._capture_job = PanoCaptureJob(pano_id, capturer )
        camera_topic = goal.camera_topic or rospy.resolve_name('camera')
        #TODO: FIX ONCE OPENNI API IS FIXED
        image_topic = rospy.names.ns_join(camera_topic, 'image_color')
        camera_info_topic = rospy.names.ns_join(camera_topic, 'camera_info')

        rospy.loginfo('%s: Starting capture of pano_id %d.\n\tImage [%s]\n\tCamera Info[%s]'
                      %(self._action_name, pano_id, image_topic, camera_info_topic) )
        grabber = ImageGrabber(image_topic, camera_info_topic, self.capture_fn)

        # local vars
        server = self._server
        preempted = False        

        rospy.loginfo('%s: Executing capture of pano_id %d' % (self._action_name, pano_id))  
        # this will become true 
        while capture_job.result is None and not preempted and not rospy.is_shutdown():
            if server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                server.set_preempted()
                capture_job.cancel()
                preempted = True
            else:
                rospy.sleep(0.001) #let the node rest a bit
                
        result = capture_job.result
        grabber.stop()
        
        if result:
            rospy.loginfo('%s: Succeeded, [%s] images.\nResult: %s' % (self._action_name, result.n_captures, result))
            server.set_succeeded(result)
  
        self._capture_job = None
        
    def capture_fn(self, image, info):
        if self._capture_job is None:
            rospy.logerr("cannot capture photo while capture job is not active")
            return
        # not really concerned by the possible race condition here
        if self._snap_requested:
            rospy.loginfo("snap!")
            self._snap_requested = False
            self._capture_job.capture_fn(image, info)
            #only send feedback on snap
            feedback = PanoCaptureFeedback()
            feedback.n_captures = self._capture_job.n_captures
            rospy.loginfo("captured %d images so far" % feedback.n_captures)
            self._server.publish_feedback(feedback)
            
    def capture_snap(self, empty):
        if self._capture_job is None:
            rospy.logerr("cannot snap photo while capture job is not active")
            return
        self._snap_requested = True
        
    def capture_stop(self, empty):
        if self._capture_job is None:
            rospy.logerr("no active capture job to stop")
            return

        self._snap_requested = False
        self._capture_job.stop()
        


class BagCapture(CaptureInterface):
    """
    Bag capture is not thread-safe.
    """     
    def start(self, pano_id, goal):
        """
        set up a new pano.
        """
        self.pano_id = pano_id
        if goal.bag_filename is None:
            self.bag_filename = os.path.abspath(os.path.join(tempfile.gettempdir(), "pano_%d.bag" % pano_id))
        else:
            self.bag_filename = goal.bag_filename
        self.bag = rosbag.Bag(self.bag_filename, 'w')
        self.n_captures = 0
        self.lock = threading.Lock()

    def __call__(self, image, camera_info):
        with self.lock:
            if self.bag is not None:
                self.bag.write("image", image)
                self.bag.write("camera_info", camera_info)
                self.n_captures += 1
                          
    def stop(self):
        with self.lock:
            if self.bag is not None:
                self.bag.close()
                self.bag = None
            else:
                raise IOError("already closed")
        
    def cancel(self):
        self.bag.close()
        self.bag = None

class StitchCapture(CaptureInterface):
    """
    Bag capture is not thread-safe.
    """     
    def start(self, pano_id, goal):
        """
        set up a new pano.
        """
        self.bridge = CvBridge()
        self._bag_capture = BagCapture()
        self._bag_capture.start(pano_id, goal)
        self.img_pub = rospy.Publisher('~stitch',Image, queue_size=5)
        

    def __call__(self, image, camera_info):
        if self._bag_capture.n_captures < 1:
            self.setupCamera(camera_info)
            self.setupStitch()
        try:
            self.img_pub.publish(image)	    	
            cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
	    pano_image_t = cv.fromarray(cv_image)
	    pano_image = pcv.convertCvMat2Mat(pano_image_t)	
            self.stitcher.addNewImage(pano_image)
        except CvBridgeError, e:
            print e
        #capture to bag for back up and later stitching        
        self._bag_capture(image,camera_info)
        
    def setupStitch(self):
        options = pano.Options()
        options.camera = self.camera
        options.stitch_size = pcv.Size(4000,2000)
        #options.image_names.assign(image_names)
        params = options.fitter_params
        params.error_thresh = 6;
        params.inliers_thresh = 15;
        params.maxiters = 100;
        params.nNeeded = 2;
        self.stitcher = pano.StitchEngine(options)
          
    def _get_n_captures(self):
        return self._bag_capture.n_captures
    def _get_bag_filename(self):
        return self._bag_capture.bag_filename
    
    n_captures = property(_get_n_captures)
    bag_filename = property(_get_bag_filename)
    
    def setupCamera(self,camera_info):
        self.camera = pano.Camera()
        K = pcv.Mat(3,3,pcv.CV_32FC1)
        K.fromarray(camera_info.K)
        img_size = pcv.Size( camera_info.width, camera_info.height)                
        self.camera.setCameraIntrinsics(K,pcv.Mat(),img_size) 
        
    def stitch_cb(self, x):
        print "progress ",x
        return 0 
          
    def stop(self):
        #allocate a cv mat, this will be stitched to inplace
        #2000 rows = 2000 height
        #4000 cols = 4000 width (this is backwards from widthxheight
        if(self.stitcher):
            blended_t = cv.CreateMat(1500,3000,cv.CV_8UC3)
	    blended_num = numpy.asarray(blended_t) 
	    blended = pcv.convertCvMat2Mat(blended_t)  
            self.stitcher.stitch(blended, self.stitch_cb)
            img_msg = self.bridge.cv2_to_imgmsg(blended_num, "rgb8")
            self.img_pub.publish(img_msg) 
        self._bag_capture.stop()
        
    def cancel(self):
        self._bag_capture.cancel()
    
if __name__ == '__main__':
    rospy.init_node('pano_capture')
    PanoCaptureServer(rospy.get_name(), StitchCapture)
    rospy.spin()

