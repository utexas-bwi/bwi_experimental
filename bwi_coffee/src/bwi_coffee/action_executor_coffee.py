#! /usr/bin/env python

from bwi_planning import ActionExecutor
from segbot_gui.srv import QuestionDialogRequest
import time

from .atom_coffee import AtomCoffee

class ActionExecutorCoffee(ActionExecutor):

    def __init__(self, dry_run=False, initial_file=None):
        super(ActionExecutorCoffee, self).__init__(dry_run, initial_file,
                                                     AtomCoffee)

    def execute_action(self, action, next_state, next_step):

        success = False
        if action.name not in ["order", "load", "unloadto"]:
            success, observations = \
                    super(ActionExecutorCoffee, self).execute_action(action,
                                                                     next_state,
                                                                     next_step)

            if action.name == "greet":
                observations.append(AtomCoffee("closeto",str(action.value)))

            return success, observations

        observations = []
        if action.name == "order":
            self.gui(QuestionDialogRequest.DISPLAY,
                     "Could I get an order of " + str(action.value) + "?",
                     [], 0.0)
            time.sleep(10.0)
            observations.append(AtomCoffee("waiting",str(action.value)))
            success = True

        if action.name == "load":
            response = self.gui(QuestionDialogRequest.DISPLAY,
                                "Please let me know once " + str(action.value) + " has been loaded!",
                                ["Done!"], 0.0)
            if response.index == 0: # The Done! button was hit
                observations.append(AtomCoffee("loaded",str(action.value)))
                success = True

        if action.name == "unloadto":
            response = self.gui(QuestionDialogRequest.DISPLAY,
                                "Here is your " + str(action.value[0]) + 
                                "! Please let me know once you have removed it.", 
                                ["Done!"], 0.0)
            if response.index == 0: # The Done! button was hit
                observations.append(AtomCoffee("served",str(action.value[1])+","+str(action.value[0])))
                success = True

        return success, observations
