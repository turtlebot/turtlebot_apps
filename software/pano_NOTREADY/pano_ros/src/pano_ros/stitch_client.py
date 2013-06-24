#! /usr/bin/env python
import roslib; roslib.load_manifest('pano_ros')
import rospy

import actionlib
import sys

from pano_ros.msg import *

def stitch_client(bag_file_name,stich_file_name):   
    stich_goal = StitchGoal(bag_file_name=bag_file_name, stitch_file_name=stich_file_name)
    client = actionlib.SimpleActionClient('bag_stitcher/stitch', StitchAction)
    client.wait_for_server()
    client.send_goal(stich_goal)
    client.wait_for_result()
    return client.get_result()
    
def usage():
    return "stitch_client.py pano.bag stitch.jpg"
if __name__ == '__main__':
    try:
        bag_file = ""
        stitch_file = ""
        
        if len(sys.argv) == 3:
            bag_file = sys.argv[1]
            stitch_file = sys.argv[2]
        else:
            print usage()
            sys.exit(1)
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('stitch_client')
        result = stitch_client(bag_file,stitch_file)
        print "stitch result:%d" % (result.result_flags)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

