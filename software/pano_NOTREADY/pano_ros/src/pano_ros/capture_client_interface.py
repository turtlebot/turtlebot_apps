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
Panorama capture client interface.  This takes care of all setup required
to talk to the capture_server. Based on Ken's idea that we should expose
the user to simple interfaces that take care of dirty actionlib setup
and pub sub stuff...

Authors: Ethan Rublee, Ken Conley

See capture_client for a simple use case of the CaptureClientInterface
"""
import roslib; roslib.load_manifest('pano_ros')
import rospy
import actionlib
import sys
import pano_ros.msg
from std_msgs.msg import Empty

class CaptureClientInterface(object):
    """
    An attempt at a common capture client interface
    This does all boot strapping for you to hook up to the pano_capture action server
    this will look for the capture_server at 'pano_capture'
    TODO make this resolve pano_capture instead of hardcoding
    """
    def __init__(self):
        self._setupClient()
        self.n_captures = 0
    def _setupClient(self):
        """
        setup pubs and subs, and feedback callback with the actionlib pano_capture server
        """
        self.client = actionlib.SimpleActionClient(
                  'pano_capture',
                  pano_ros.msg.PanoCaptureAction)
        self.client.wait_for_server()  
       
        self.snap_pub = rospy.Publisher('pano_capture/snap',Empty)
        self.stop_pub = rospy.Publisher('pano_capture/stop',Empty)

    def start(self,bag_filename = None):
        """
        start capturing a pano to a bag file
        bag_filename is the absolute pathname of the bagfile to record to
        """ 
        self.n_captures = 0
        #start capturing, atleast give the name of 
        #the bag file to capture to
        goal = pano_ros.msg.PanoCaptureGoal(bag_filename = bag_filename)
        self.client.send_goal(goal = goal,feedback_cb = self.feedback)
        
    def snap(self):
        """
        send a snap message to the capture server, will request a photo to be captured
        expect a call to feedback_c
        """
        print "requesting to snap"
        self.snap_pub.publish()
    def stop(self):
        """
        send a stop message to the capture server
        todo handle ctrl-c escape properly!
        """        
        self.stop_pub.publish()
        self.client.wait_for_result()            
        #the capture_result contains some meta information about the capture,
        #including the number of frames and bag location    
        return self.client.get_result()
        
    def feedback(self, feedback):
        """
        High jack me if you would like to receive feedback calls.
        Keep in mind that this is called asynchronously
        """                
        self.n_captures = feedback.n_captures
        print "captured a photo: %d" % (self.n_captures)
        