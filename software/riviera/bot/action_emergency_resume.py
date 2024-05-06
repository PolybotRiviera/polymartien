"""
Module to resume the robot's activity after an emergency stop.
"""

from communication_codes import EMERGENCY_STOP
import action

class ActionEmergencyResume(action.Action):
    def __init__(self, esp):
        super().__init__(esp)
        self.codeResumeStop = EMERGENCY_STOP

    def execute(self):
        """
        Resume the robot's activity after an emergency stop.
        """
        trame = [self.codeResumeStop, 0]
        self.get_communicator().send_trame(trame)
