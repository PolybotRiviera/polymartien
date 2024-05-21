"""
Module to control the distance of the plant holder inside the robot.
"""

from communication_codes import HOLDER_DISTANCE
import action

class ActionHolderDistance(action.Action):
    def __init__(self, esp):
        super().__init__(esp)
        self.codeDistance = HOLDER_DISTANCE
        self.maxDistance = 20

    def execute(self, dist: float):
        """
        Set the distance of the plant holder inside the robot.
        """
        trame = [self.codeDistance, int(dist/self.maxDistance*200)]
        self.get_communicator().send_trame(trame)

    def hold(self):
        """Close the holder."""
        self.execute(0)

    def release(self):
        """Open the holder."""
        self.execute(self.maxDistance)
