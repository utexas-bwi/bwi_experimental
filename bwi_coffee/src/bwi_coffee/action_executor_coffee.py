#! /usr/bin/env python

from bwi_planning import ActionExecutor
from segbot_gui.srv import QuestionDialogRequest
from map_mux.srv import *

import rospy
import time

from .atom_coffee import AtomCoffee

class ActionExecutorCoffee(ActionExecutor):

    def __init__(self, dry_run=False, initial_file=None):
        super(ActionExecutorCoffee, self).__init__(dry_run, initial_file,
                                                     AtomCoffee)

    def execute_action(self, action, next_state, next_step):
        #ros::ServiceClient client = n.serviceClient<map_mux::ChangeMap>("change_map");
        rospy.wait_for_service('change_map')

        success = False
        if action.name not in ["order", "load", "unloadto", "greet", "choosefloor"]:
            success, observations = \
                    super(ActionExecutorCoffee, self).execute_action(action,
                                                                     next_state,
                                                                     next_step)

            return success, observations

        rospy.loginfo("Executing action: " + str(action))

        observations = []
        if action.name == "order":
            self.gui(QuestionDialogRequest.DISPLAY,
                     "Could I get an order of " + str(action.value) + "?",
                     [], 0.0)
            time.sleep(5.0)
            observations.append(AtomCoffee("waiting",str(action.value),time=next_step))
            success = True

        if action.name == "load":
            response = self.gui(QuestionDialogRequest.CHOICE_QUESTION,
                                "Please let me know once " + str(action.value) + " has been loaded!",
                                ["Done!"], 0.0)
            if response.index == 0: # The Done! button was hit
                observations.append(AtomCoffee("loaded",str(action.value),time=next_step))
                success = True

        if action.name == "unloadto":
            response = self.gui(QuestionDialogRequest.CHOICE_QUESTION,
                                "Here is your " + str(action.value.value[0]) +
                                "! Please let me know once you have removed it.",
                                ["Done!"], 0.0)
            if response.index == 0: # The Done! button was hit
                observations.append(AtomCoffee("served",str(action.value.value[1])+","+str(action.value.value[0]),time=next_step))
                success = True

        if action.name == "greet":
            self.gui(QuestionDialogRequest.DISPLAY,
                     "Hello " + str(action.value) + "!!",
                     [], 0.0)
            time.sleep(5.0)
            observations.append(AtomCoffee("closeto",str(action.value),time=next_step))
            success = True

        if action.name == "choosefloor":
            #call change map service
            try:
                service_change_map = rospy.ServiceProxy('change_map', ChangeMap)
                if action.value.value == "f2":
                    resp = service_change_map(2)
                    print "Ran change_map with 2"
                if action.value.value == "f3":
                    resp = service_change_map(3)
                    print "Ran change_map with 3"

                response = self.gui(QuestionDialogRequest.CHOICE_QUESTION,
                                "Please let me know once floor" + str(action.value.value[1]) + "has been reached",
                                ["Done!"], 0.0)
                if response.index == 0: # The Done! button was hit
                    success = True
                #time.sleep(15.0)
            except rospy.ServiceException:
                rospy.loginfo("serviced failed - change_map")
            #observations.append(AtomCoffee("closeto",str(action.value),time=next_step))

            success = True

        rospy.loginfo("  Observations: " + str(observations))
        self.clear_gui()
        return success, observations
