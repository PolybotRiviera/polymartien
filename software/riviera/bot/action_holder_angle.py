"""
Module to control the angle of the plant holder inside the robot.
"""

from communication_codes import HOLDER_ANGLE
import action

class ActionHolderAngle(action.Action):
    def __init__(self, esp):
        super().__init__(esp)
        self.codeAngle = HOLDER_ANGLE
        self.minAngle = 0
        self.maxAngle = 45

    def execute(self, angle: float):
        """
        Set the angle of the plant holder inside the robot.
        """
        trame = [self.codeAngle, int((angle-self.minAngle)/(self.maxAngle-self.minAngle)*200)]
        self.get_communicator().send_trame(trame)

    def hold(self):
        """Close the holder."""
        self.execute(self.minAngle)

    def release(self):
        """Open the holder."""
        self.execute(self.maxAngle)
