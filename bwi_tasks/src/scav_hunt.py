#!/usr/bin/python

import rospy
import sys
import subprocess # for openning text and image files
import os # for listing files in a directory

from segbot_gui.srv import *

def scav_hunt():

    rospy.wait_for_service('question_dialog')

    # tasks:
    # 0, blueshirt
    # 1, interaction
    # 2, whiteboard

    tasks = []
    tasks.append('* find a person wearing blue shirt')
    tasks.append('* interact with a person through natural language (testing)')
    tasks.append('* find a person standing in front of a whiteboard')
    tasks.append('* take a picture of something with a "Starbucks Coffee" logo')

    task_names = ['blueshirt', 'interaction', 'whiteboard', 'logo']

    blueshirt_pic = ''
    interaction_txt = ''
    whiteboard_pic = ''
    path_to_files = '/home/bwi/Desktop/'

    while not rospy.is_shutdown():
    
        files = os.listdir(path_to_files)

        finished = 'Finished tasks:'
        todo = '\nTodo tasks:'
        ending = '\nClick the buttons to see how I performed in the tasks\n'
        buttons = []

        for f in files:
            if f.find('blueshirt') >= 0:
                blueshirt_pic = f
                finished = finished + '\n\t' + tasks[0]
                buttons.append('blueshirt')
                break
        else:
            todo = todo + '\n\t' + tasks[0]

        for f in files:
            if f.find('interaction') >= 0:
                interaction_txt = f
                finished = finished + '\n\t' + tasks[1]
                buttons.append('interaction')
                break
        else:
            todo = todo + '\n\t' + tasks[1]

        for f in files:
            if f.find('whiteboard') >= 0:
                whiteboard_pic = f
                finished = finished + '\n\t' + tasks[2]
                buttons.append('whiteboard')
                break
        else:
            todo = todo + '\n\t' + tasks[2]

        for f in files:
            if f.find('logo') >= 0:
                logo_pic = f
                finished = finished + '\n\t' + tasks[3]
                buttons.append('logo')
                break
        else:
            todo = todo + '\n\t' + tasks[3]

        # print to screen
        try: 
            handle = rospy.ServiceProxy('question_dialog', QuestionDialog)

            # in case some tasks have been finished
            if finished != finished_last: 

                if len(finished) > 20: 
                    res = handle(1, finished + '\n' + todo + '\n' + ending, buttons, 5)
                else:
                    res = handle(1, finished + '\n' + todo, buttons, 5)
    
        except rospy.ServiceException, e:
            ROS_ERRR("Service call failed: %s"%e)

        # used for checking if necessary to update segbot_gui
        finished_last = finished

        # which picture/text to view? 
        if res.index < 0: 
            continue
        elif buttons[res.index] == 'blueshirt':
            img = subprocess.Popen(["eog", path_to_files + blueshirt_pic])
        elif buttons[res.index] == 'interaction':
            img = subprocess.Popen(["gedit", path_to_files + interaction_txt])
        elif buttons[res.index] == 'whiteboard':
            img = subprocess.Popen(["eog", path_to_files + whiteboard_pic])
        elif buttons[res.index] == 'logo':
            img = subprocess.Popen(["eog", path_to_files + logo_pic])

if __name__ == "__main__":

    scav_hunt()

