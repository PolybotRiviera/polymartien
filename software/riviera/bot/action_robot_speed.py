"""
Module to control the speed of the main motors of the robot.
"""

from communication_codes import SPEED_LEFT, SPEED_RIGHT
import action

class ActionRobotSpeed(action.Action):
    def __init__(self, esp):
        super().__init__(esp)
        self.codeLeft = SPEED_LEFT
        self.codeRight = SPEED_RIGHT


    def execute(self, speedLeft: float, speedRight: float):
        """
        Set the speed of the main motors of the robot.
        
        Inputs are floats between -1 and 1 in m/s.
        """
        trame = [self.codeLeft, int((speedLeft+1)*100), self.codeRight, int((speedRight+1)*100)]
        self.get_communicator().send_trame(trame)

    def stop(self):
        """Stop the robot."""
        return self.execute(0, 0)
