#!/usr/bin/python

import rospy
import sys
from segbot_gui.srv import *

def scav_hunt():

    rospy.wait_for_service('question_dialog')

    try: 

        handle = rospy.ServiceProxy('question_dialog', QuestionDialog)
        res = handle(0, "ABC", ["Button"], 5)

    except rospy.ServiceException, e:

        print "Service call failed: %s"%e


if __name__ == "__main__":

    scav_hunt()
