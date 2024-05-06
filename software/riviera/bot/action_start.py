"""
Module to start the robot.

Mandatory.
"""

from communication_codes import START
import action


class ActionStart(action.Action):
    def __init__(self, esp):
        super().__init__(esp)
        self.codeStart = START

    def execute(self):
        """
        Start the robot.
        """
        trame = [self.codeStart, 0]
        self.get_communicator().send_trame(trame)
