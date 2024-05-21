"""
Module to stop the robot.

Use action_resume_stop to resume the robot.
"""

from communication_codes import EMERGENCY_STOP
import action


class ActionEmergencyStop(action.Action):
    def __init__(self, esp):
        super().__init__(esp)
        self.codeEmergencyStop = EMERGENCY_STOP

    def execute(self):
        """
        Stop the robot.
        """
        trame = [self.codeEmergencyStop]
        self.get_communicator().send_trame(trame)
