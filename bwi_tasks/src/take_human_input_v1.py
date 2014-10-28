#!/usr/bin/python

import time
import rospy

import roslib; roslib.load_manifest('bwi_tasks')

import actionlib
from bwi_kr_execution.msg import *
import segbot_gui.srv
import bwi_rlg.srv

from multiprocessing import Process, Value, Array

# human_waiting = False
# curr_goal = []
# next_room = None


# option 
# 1: click me if you need help
# 2: please let me know your goal place
def gui_thread(human_waiting, curr_goal):
  
    rospy.init_node('human_input_gui_thread')

    print("gui_thread started")

    rospy.wait_for_service('question_dialog')
    
    while not rospy.is_shutdown():

        if human_waiting.value == True:
            rospy.sleep(2)
            continue

        try:
            handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
            res = handle(1, "Please click the button, if you need my help.\n\nI am moving to " + curr_goal.value, ["Button"], 2)

            while (res.index < 0):
                handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
                res = handle(1, "Please click the button, if you need my help.\n\nI am moving to " + curr_goal.value, ["Button"], 2)

            human_waiting.value = True
            res = handle(0, "I have to go to " + curr_goal.value + " first.\n\nFollow me please, I will serve you in a moment.", ["Button"], 0)

        except rospy.ServiceException, e:
            print ("Service call failed")
    
    return True

def platform_thread(human_waiting, curr_goal):

    rospy.init_node('human_input_platform_thread')

    print("platform_thread started")

    rospy.wait_for_service('question_dialog')

    rooms = ["414a", "414b", "416", "418", "420"]
    doors = ["d3_414a1", "d3_414b1", "d3_416", "d3_418", "d3_420"]
    
    client = actionlib.SimpleActionClient('/action_executor/execute_plan',\
            ExecutePlanAction)
    client.wait_for_server()


    while not rospy.is_shutdown():
        print("human_waiting: " + str(human_waiting.value))

        if human_waiting.value == True:
            print("Human is waiting. Let me see where the goal is.")

            handle = rospy.ServiceProxy('semantic_parser', bwi_rlg.srv.SemanticParser)
            res_sp = handle(0, "")

            while len(res_sp.query) == 0:

                try:
                    handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
                    res_qd = handle(2, res_sp.output_text, doors, 0)
                except rospy.ServiceException, e:
                    print ("Service call failed: %s"%e)

                try:
                    handle = rospy.ServiceProxy('semantic_parser', bwi_rlg.srv.SemanticParser)
                    res_sp = handle(0, res_qd.text)
                except rospy.ServiceException, e:
                    print ("Service call failed: %s"%e)

            # now the robot has found the query from semantic parser

            loc = res_sp.query
            print ("loc: " + loc)

            if (loc.find("at") >= 0):

                loc = loc.replace("at(l", "d")
                loc = loc[:loc.find(",")]

            elif (loc.find("query") >= 0):

                loc = loc.replace("query(l", "d")
                loc = loc[:loc.find(":")]

            if loc.find("d3_414") > 0:
                loc += "1"

            try:
                handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
                handle(0, "Follow me please. We are arriving soon. ", doors, 0)
            except rospy.ServiceException, e:
                print ("Service call failed: %s"%e)

            # loc = doors[res.index]

            goal = ExecutePlanGoal()
            rule = AspRule()
            fluent = AspFluent()
            
            fluent.name = "not beside"
            fluent.variables = [loc]
            rule.body = [fluent]
            goal.aspGoal = [rule]
            
            print("sending goal: " + loc)
            client.send_goal(goal)
    
            client.wait_for_result()

            try:
                handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
                res = handle(0, "You have arrived. I am leaving. \n\nThank you!", doors, 0)
                rospy.sleep(10)

            except rospy.ServiceException, e:
                print ("Service call failed: %s"%e)

            human_waiting.value = False

        else:
            print("No one needs my help. Let me take a random walk.")

            loc = doors[int(time.time()) % len(rooms)]
            curr_goal.value = loc
            
            goal = ExecutePlanGoal()
            rule = AspRule()
            fluent = AspFluent()
            
            fluent.name = "not beside"
            fluent.variables = [loc]
            rule.body = [fluent]
            goal.aspGoal = [rule]
            
            print("sending goal: " + loc)
            client.send_goal(goal)
    
            client.wait_for_result()


    return 1

if __name__ == '__main__':

    human_waiting = Value('b', False)
    curr_goal = Array('c', "this is a very long string for nothing")

    p1 = Process(target = gui_thread, args = (human_waiting, curr_goal))
    p2 = Process(target = platform_thread, args = (human_waiting, curr_goal))

    p1.start()
    p2.start()

    p1.join()
    p2.join()

