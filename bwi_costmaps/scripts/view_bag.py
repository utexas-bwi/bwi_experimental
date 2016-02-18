#!/usr/bin/evn python
"""
Author: Pato Lankenau (plankenau@gmail.com)
License: BSD
"""
import rospy
from nav_msgs.msg import OccupancyGrid

def view():
    rospy.init_node('bag_viewer', anonymous=True)
    rospy.loginfo("hello world")

if __name__ == '__main__':
    try:
        view()
    except rospy.ROSInterruptException:
        pass
