#! /usr/bin/env python

from bwi_planning import Atom

class AtomCoffee(Atom):

    ACTION_NAMES = ["askploc", "greet", "gothrough", "opendoor",
                    "approach", "order", "load", "unloadto", "goto", "getin","choosefloor"]
    FLUENT_NAMES = ["inside", "knowinside", "open", "visiting", "closeto",
                    "facing", "beside", "loc", "waiting", "loaded", "served",
                    "at"]
    TERM_NAMES = ["hasdoor", "acc", "accdoor", "knows"]

    def __init__(self, name, value=None, time=None, negated=False):
        super(AtomCoffee, self).__init__(name, value, time, negated)

        if self.type == None:
            if self.name in AtomCoffee.ACTION_NAMES:
                self.type = Atom.ACTION
                return
            if self.name in AtomCoffee.FLUENT_NAMES:
                self.type = Atom.FLUENT
                return
        else:
            return

    def conflicts_with(self, other):
        """
          Test for hard negation conflict and uniqueness constraints only.
        """

        if super(AtomCoffee, self).conflicts_with(other):
            return True

        # Check for uniqueness constraints
        if (self.name == "at" and other.name == "at" or \
            self.name == "facing" and other.name == "facing" or \
            self.name == "beside" and other.name == "beside") and \
           self.value != other.value and \
           not self.negated and not other.negated:
            return True

        # Check for uniqueness constraint for inside/knowinside
        # if the person is same, but the location is different and both
        # fluents are not negated
        inside_names = ["inside", "knowinside"]
        if self.name in inside_names and other.name in inside_names and \
           self.value.value[0] == other.value.value[0] and \
           self.value.value[1] != other.value.value[1] and \
           not self.negated and not other.negated:
            return True

        return False

