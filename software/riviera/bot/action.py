"""
Module containing the Action class.

Action is the base class for all actions that the robot can perform.

It implements a communication protocol using esp_comm.
"""

import esp_comm


class Action():
    """Base class for all actions that the robot can perform."""
    def __init__(self, esp: esp_comm.ESPCommunicator):
        self.esp = esp

    def get_communicator(self):
        return self.esp
