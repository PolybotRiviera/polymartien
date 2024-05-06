"""
Module to control the height of the plant grabber on the back of the robot.
"""

from communication_codes import GRABBER_HEIGHT
import action

class ActionGrabberHeight(action.Action):
    def __init__(self, esp):
        super().__init__(esp)
        self.codeHeight = GRABBER_HEIGHT
        self.maxHeight = 20

    def execute(self, height: float):
        """
        Set the height of the grabber on the back of the robot.
        """
        trame = [self.codeHeight, int(height/self.maxHeight*200)]
        self.get_communicator().send_trame(trame)

    def up(self):
        """Move the grabber up."""
        self.execute(self.maxHeight)

    def down(self):
        """Moves the grabber down."""
        self.execute(0)

    def to_plant(self):
        """Move the grabber to the plant."""
        self.execute(self.maxHeight/2)          #TODO set the right height

    def lift_plant(self):
        """Lift the plant."""
        self.execute(self.maxHeight/2.1)        #TODO set the right height
