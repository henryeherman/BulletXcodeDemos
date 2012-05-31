'''

File: HexapodEnvironment.py

Description: This class specifies the hexapod's environment. It defines the
actions the hexapod can make and performs actions when asked by the task.

'''

from pybrain.utilities import Named
from pybrain.rl.environments.environment import Environment


class HexapodEnvironment(Environment, Named):

    def __init__(self):
        self.reset = False

    def getSensors(self):
        """ the currently visible state of the world """
        pass

    def performAction(self, action):
        """ Perform an action on the world that changes it's internal state """
        pass

    def reset(self):
        """ Resets/reinitializes everything """
        pass
