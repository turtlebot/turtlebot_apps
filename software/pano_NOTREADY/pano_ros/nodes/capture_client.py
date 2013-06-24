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
# Revision $Id: capture_client.py 47138 2010-12-15 05:22:19Z ethanrublee $

"""
Panorama time based capture client.

Authors: Ethan Rublee, Ken Conley

Sample run command, assuming the capture_server has been started, will 
take 30 images and store them in `pwd`/pano.bag

rosrun pano_ros capture_client.py `pwd`/pano.bag 30
"""

import roslib; roslib.load_manifest('pano_ros')
import rospy
import sys
from pano_ros.msg import PanoCaptureResult
from pano_ros.capture_client_interface import CaptureClientInterface
        
        
class TimedCapture(object):
    """
    Uses the CaptureClientInterface to take photos 
    regularly at a time interval and will snap until 
    the desired number of frames are taken
    """
    def __init__(self):
        #CaptureClientInterface does all of the setup
        #for connecting to the pano_capture server
        self._capture = CaptureClientInterface()
            
    def capture(self,bag_filename, total_captures = 20, time_interval = 1):
        """
        This is a timed interval capture.
        total_captures the number of images to take
        time_interval attempt to take an image at 
                      this time interval, specified in seconds
        """
        capture = self._capture
        #start up the capture client, giving absolute path to a 
        #bag where the data will be stored
        capture.start(bag_filename)
        result = PanoCaptureResult()
        try:     
            #we'll just capture total_captures images, 
            #1 every time_interval seconds
            while ( capture.n_captures < total_captures 
                   and not rospy.is_shutdown() ):
                #request a snap
                capture.snap()   
                #sleep for 1 seconds to allow the camera to be moved
                rospy.sleep(time_interval)
        finally:
            tresult = capture.stop()
            result = tresult if tresult else result
        #this will return the PanoCapture result message
        return result           
      
    
def usage():    
    return """usage:
rosrun pano_ros capture_client.py `pwd`/pano.bag 30 [1]
    where `pwd`/pano.bag is the output bag of the capture session
    30 is the number of images to capture
    1 is the time interval"""
        
if __name__ == '__main__':
    try:        
        bag_filename = ""
        n_images = 20    
        time_step = 1       
        if len(sys.argv) == 3 or len(sys.argv) == 4:
            bag_filename = sys.argv[1]
            n_images = int(sys.argv[2])
            if len(sys.argv) == 4:
                time_step = float(sys.argv[3])
        else:
            print usage()
            sys.exit(1)
        rospy.init_node('capture_client_py')
        capture_client = TimedCapture()
        result = capture_client.capture(bag_filename,n_images,time_step)
        print "Captured:%d photos. Saved to %s" % (result.n_captures,
                                                   result.bag_filename)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
