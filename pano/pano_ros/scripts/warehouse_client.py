#!/usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('pano_ros')
import rospy
from warehouse.srv import *
from warehouse.msg import Condition
from warehouse.exceptions import *
from pano_ros.msg import Pano

import warehouse

class PanoWarehouseClient:
  def __init__(self):
    self.db_name = "pano_store"
    self.collection_name = "panos"
    self.wc = None
    self.collection = None
    self._setup_db()
    
  def _setup_db(self):
    self.wc = warehouse.Client()
    self.collection = None
    try:
      self.collection = self.wc.setup_collection(self.db_name, self.collection_name, Pano, ["pano_id"])
      self.collection.subscribe_to_insertion(self.mycallback)
    except ClientException:
      rospy.logerr("Unable to create a connection to the database")
      
  def push_data(self,pano):
    rospy.loginfo("publishing pano")
    self.collection.push_data(pano)
    
  def mycallback(self,msg):
      print "Data was inserted by {0}".format(msg.sender)
      
if __name__ == '__main__':
  rospy.init_node("pano_pusher")
  pwc = PanoWarehouseClient()
  pano = Pano()
  pano.pano_id = "my_pano001"
  pwc.push_data(pano)
  rospy.spin()
