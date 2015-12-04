#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class BehaviorSwitch(object):
    def __init__(self):
        self.running = False

    def callback(self, joy_msg):
        if joy_msg.buttons[5] == 1:
            self.running = not self.running

        rospy.loginfo(repr(joy_msg))

    def run(self):
        rospy.init_node('behavior_switch', anonymous=True)
        pub = rospy.Publisher('cmd_vel_mux/input/switch', Twist, queue_size=10)
        rospy.Subscriber('joy', Joy, self.callback)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.running:
                empty_msg = Twist()
                pub.publish(empty_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        behavior_switch = BehaviorSwitch()
        behavior_switch.run()
    except rospy.ROSInterruptException:
        pass
