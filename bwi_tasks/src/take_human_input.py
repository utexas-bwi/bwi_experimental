#!/usr/bin/python

import rospy

import roslib; roslib.load_manifest('bwi_tasks')

import actionlib
from bwi_kr_execution.msg import *
import segbot_gui.srv

from multiprocessing import Process, Lock

human_waiting = False
next_room = None

# option 
# 1: click me if you need help
# 2: please let me know your goal place
def gui_thread(option):
  
    print("gui_thread started")

    rospy.wait_for_service('question_dialog')
    
    if option == 1:
        try:
            handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
            res = handle(1, "Please click the button, if you need help.", ["Button"], 0)
            human_waiting = True
            return 1
        except rospy.ServiceException, e:
            print ("Service call failed")
    
    elif option == 2:
        try:
            handle = rospy.ServiceProxy('question_dialog', QuestionDialog)
            res = handle(2, "Please select the room.", ["418", "420"], 0)
            return 1
        except rospy.ServiceException, e:
            print ("Service call failed: %s"%e)

def platform_thread():

    rospy.init_node('take_human_input_py')
    global human_waiting

    print("platform_thread started123")

    doors = ["d3_414b1", "d3_414b2", "d3_420", "d3_414a1", "d3_414a2", "d3_418"]
    
    client = actionlib.SimpleActionClient('/action_executor/execute_plan',\
            ExecutePlanAction)
    client.wait_for_server()

    current_door = 0

    while not rospy.is_shutdown():

        loc = doors[current_door]
        current_door += 1
        
        if current_door >= len(doors):
            current_door = 0
        
        if human_waiting == True:
            next_room = gui_thread(2)
            human_waiting = False
        else:
            print("human_waiting == True")
        
        goal = ExecutePlanGoal()
        rule = AspRule()
        fluent = AspFluent()
        
        fluent.name = "not facing"
        fluent.variables = [loc]
        rule.body = [fluent]
        goal.aspGoal = [rule]
        
        print("sending goal: " + loc)
        client.send_goal(goal)
    
        client.wait_for_result()

    return 1

if __name__ == '__main__':

  try:

    p1 = Process(target = gui_thread, args = (1,))
    p2 = Process(target = platform_thread)

    p1.start()
    p2.start()

  except:

    rospy.loginfo("Error: unable to start thread")


