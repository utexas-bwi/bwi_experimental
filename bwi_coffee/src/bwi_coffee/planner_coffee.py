#! /usr/bin/env python

from bwi_planning import Planner
from .atom_coffee import AtomCoffee
from .action_executor_coffee import ActionExecutorCoffee

class PlannerCoffee(Planner):

    def __init__(self, dry_run=False, initial_file=None):
        super(Planner, self).__init__(AtomCoffee, ActionExecutorCoffee)
