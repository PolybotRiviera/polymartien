"""
Module to control the angle of the plant grabber on the back of the robot.
"""

from communication_codes import GRABBER_ANGLE
import action

class ActionGrabberAngle(action.Action):
    def __init__(self, esp):
        super().__init__(esp)
        self.codeAngle = GRABBER_ANGLE
        self.minAngle = 0
        self.maxAngle = 45

    def execute(self, angle: float):
        """
        Set the angle of the grabber on the back of the robot.
        """
        trame = [self.codeAngle, int((angle-self.minAngle)/(self.maxAngle-self.minAngle)*200)]
        self.get_communicator().send_trame(trame)

    def grab(self):
        """Close the grabber."""
        self.execute(self.minAngle)

    def release(self):
        """Open the grabber."""
        self.execute(self.maxAngle)
