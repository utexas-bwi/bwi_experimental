#! /usr/bin/env python

from bwi_planning import Planner
from .atom_coffee import AtomCoffee
from .action_executor_coffee import ActionExecutorCoffee

#from bwi_planning import Planner
#from .atom_coffee import AtomCoffee
#from .action_executor_coffee import ActionExecutorCoffee
import rospy
import time
from bwi_planning_common.msg import PlannerAtom
from segbot_simulation_apps.srv import DoorHandlerInterface

class PlannerCoffee(Planner):

    def __init__(self):
        super(PlannerCoffee, self).__init__(AtomCoffee, ActionExecutorCoffee)
