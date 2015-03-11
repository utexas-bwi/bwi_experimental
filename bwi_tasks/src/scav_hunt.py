#!/usr/bin/python

import rospy
import sys
import subprocess # for openning text and image files
import os # for listing files in a directory
from std_msgs.msg import String

from segbot_gui.srv import *


blue_shirt = ''
object_detection = ''
whiteboard = ''

shirt_color = ''
object_name = ''

blue_shirt_file = ''
object_detection_file = ''
whiteboard_file = ''


def callback_blue_shirt(data):

    shirt_color = data[:data.find(':')]

    if data == 'running':
        blue_shirt = 'ongoing'
    else:
        blue_shirt = 'finished'
        blue_shirt_file = data[data.find(':') + 1:]

def callback_object_detection(data):

    object_name = data[:data.find(':')]

    if data.find('running') >= 0:
        object_detection = 'ongoing'
    else:
        object_detection = 'finished'
        object_detection_file = data[data.find(':') + 1:]

def callback_whiteboard(data):

    if data == 'running':
        whiteboard = 'ongoing'
    else:
        whiteboard = 'finished'
        whiteboard_file = data[data.find(':') + 1:]

def scav_hunt():

    rospy.init_node('scav_hunt', anonymous=True)

    rospy.Subscriber('segbot_blue_shirt_status', String, callback_blue_shirt)

    rospy.Subscriber('segbot_object_detection_status', String, callback_object_detection)

    rospy.Subscriber('segbot_whiteboard_status', String, callback_whiteboard)

    rospy.wait_for_service('question_dialog')

    rospy.spin()

    task_names = ['blueshirt', 'whiteboard', 'logo']

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
    
        files = os.listdir(path_to_files)

        finished = 'Finished tasks:'
        todo = '\nTodo tasks:'
        ending = '\nClick the buttons to see how I performed in the tasks\n'
        buttons = []

        if blue_shirt == 'ongoing':
            todo.append('\n\t* find a person wearing "' + shirt_color + '" shirt')
        elif blue_shirt == 'finished':
            finished.append('\n\t* find a person wearing "' + shirt_color + '" shirt')
            buttons.append('color t-shirt')


        if whiteboard == 'ongoing':
            todo.append('\n\t* find a person standing in front of a whiteboard')
        elif whiteboard == 'finished':
            finished.append('\n\t* find a person standing in front of a whiteboard')
            buttons.append('whiteboard')


        if object_detection == 'ongoing':
            todo.append('\n\t* take a picture of object: ' + object_name)
        elif whiteboard == 'finished':
            finished.append('\n\t* take a picture of object: ' + object_name)
            buttons.append('object detection')


        # print to screen
        try: 
            handle = rospy.ServiceProxy('question_dialog', QuestionDialog)

            if len(finished) > 20: 
                res = handle(1, finished + '\n' + todo + '\n' + ending, buttons, 5)
            else:
                res = handle(1, finished + '\n' + todo, buttons, 5)
    
        except rospy.ServiceException, e:
            ROS_ERRR("Service call failed: %s"%e)

        # which picture/text to view? 
        if res.index < 0: 
            continue

        elif buttons[res.index].find('shirt') >= 0:
            img = subprocess.Popen(["eog", blue_shirt_file])

        elif buttons[res.index].find('object') >= 0:
            img = subprocess.Popen(["eog", object_detection_file])

        elif buttons[res.index].find('whiteboard') >= 0:
            img = subprocess.Popen(["eog", whiteboard_file)

        rate.sleep()

if __name__ == "__main__":

    scav_hunt()



