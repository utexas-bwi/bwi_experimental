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

def process_request(query):

    print ("query: " + query)

    if (query.find("at") >= 0): # this is a guiding task! 

        query = query.replace("at(l", "d")
        query = query[:query.find(",")]

    elif (query.find("query") >= 0): # this is a question-asking task! 

        query = query.replace("query(l", "d")
        query = query[:query.find(":")]

    elif (query.find("served") >= 0): # this is a delivery task! 
        # served(shiqi,coffee,n)

        pass

    # TODO TODO TODO TODO

    if query.find("d3_414") > 0:
        query += "1"

    handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
    handle(0, "Follow me please. We are arriving soon. ", doors, 0)

    # query = doors[res.index]

    goal = ExecutePlanGoal()
    rule = AspRule()
    fluent = AspFluent()
    
    fluent.name = "not beside"
    fluent.variables = [query]
    rule.body = [fluent]
    goal.aspGoal = [rule]
    
    print("sending goal: " + query)
    client.send_goal(goal)

    client.wait_for_result()

    handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
    res = handle(0, "You have arrived. I am leaving. \n\nThank you!", doors, 0)
    rospy.sleep(10)


# option 
# 1: click me if you need help
# 2: please let me know your goal place
def gui_thread(human_waiting, curr_goal):
  
    rospy.init_node('human_input_gui_thread')

    rospy.info("gui_thread started")

    rospy.wait_for_service('question_dialog')
    
    while not rospy.is_shutdown():

        if human_waiting.value == True:
            rospy.sleep(2)
            continue

        handle = rospy.ServiceProxy('question_dialog', \
                                    segbot_gui.srv.QuestionDialog)
        res = handle(1, "Please click the button, if you need my help." + \
                     "\n\nI am moving to room " + curr_goal.value[4:], \
                     ["Button"], 2)

        while (res.index < 0):

            handle = rospy.ServiceProxy('question_dialog', \
                                        segbot_gui.srv.QuestionDialog)
            res = handle(1, "Please click the button, if you need my help." + \
                         "\n\nI am moving to room " + curr_goal.value[4:], \
                         ["Button"], 2)

        human_waiting.value = True
        res = handle(0, "I have to go to " + curr_goal.value[4:] + " first." +\
                        "\n\nFollow me please, I will serve you in a moment.",\
                     ["Button"], 0)

    return True

def platform_thread(human_waiting, curr_goal):

    rospy.init_node('human_input_platform_thread')

    rospy.loginfo("platform_thread started")

    rospy.wait_for_service('question_dialog')

    rooms = ["414a", "414a", "414b", "414b", "416", "418", "420", "432"]
    doors = ["d3_414a1", "d3_414a2", "d3_414b1", "d3_414b2", "d3_416", \
             "d3_418", "d3_420", "d3_432"]
    
    client = actionlib.SimpleActionClient('/action_executor/execute_plan',\
                                          ExecutePlanAction)
    client.wait_for_server()

    while not rospy.is_shutdown():

        if human_waiting.value == True:

            rospy.loginfo("Human is waiting. Let me see if I can help.")

            # robot speaks first
            handle = rospy.ServiceProxy('semantic_parser', 
                                        bwi_rlg.srv.SemanticParser)
            res_sp = handle(0, "")

            while len(res_sp.query) == 0:
                
                # take human feedback
                handle = rospy.ServiceProxy('question_dialog', 
                                            segbot_gui.srv.QuestionDialog)
                res_qd = handle(2, res_sp.output_text, doors, 0)

                # robot speaks back
                handle = rospy.ServiceProxy('semantic_parser', 
                                            bwi_rlg.srv.SemanticParser)
                res_sp = handle(0, res_qd.text)

            # now the robot has found the query from semantic parser

            process_request(res_sp.query)


            human_waiting.value = False

        else:

            rospy.loginfo("No one needs me. I will do a random walk.")

            loc = doors[int(time.time()) % len(rooms)]
            curr_goal.value = loc
            
            goal = ExecutePlanGoal()
            rule = AspRule()
            fluent = AspFluent()
            
            fluent.name = "not beside"
            fluent.variables = [loc]
            rule.body = [fluent]
            goal.aspGoal = [rule]
            
            rospy.loginfo("sending goal: " + loc)
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

