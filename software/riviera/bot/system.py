#/bin/python3

"""
Module to control the whole robot.
"""

import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import numpy

esp = """
    ATTRS{idProduct}=="ea60"
    ATTRS{idVendor}=="10c4"
    ATTRS{serial}=="0000:00:14.0"
    ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="board/arduino/uno1"
    """


import esp_comm
import camera_comm
import server

import action_emergency_stop
import action_emergency_resume
import action_robot_speed
import action_grabber_height
import action_grabber_angle
import action_holder_distance
import action_holder_angle


TIME_INIT = 0

class RobotController:
    """
    Class to control the robot using the esp32.
    """
    def __init__(self) -> None:
        self.esp = esp_comm.ESPCommunicator("/dev/esprobot")
        self.front_cam = camera_comm.CameraCommunicator("/dev/camera/front_cam")
        self.right_cam = camera_comm.CameraCommunicator("/dev/camera/right_cam")
        self.back_cam = camera_comm.CameraCommunicator("/dev/camera/back_cam")
        self.left_cam = camera_comm.CameraCommunicator("/dev/camera/left_cam")

        self.action_speed = action_robot_speed.ActionRobotSpeed(self.esp)                                   # TODO: set max speed
        self.action_emergency_stop = action_emergency_stop.ActionEmergencyStop(self.esp)
        self.action_emergency_resume = action_emergency_resume.ActionEmergencyResume(self.esp)
        self.action_grabber_height = action_grabber_height.ActionGrabberHeight(self.esp)                    # TODO: set max height
        self.action_grabber_angle = action_grabber_angle.ActionGrabberAngle(self.esp)                       # TODO: set min/max angle
        self.action_holder_distance = action_holder_distance.ActionHolderDistance(self.esp)                 # TODO: set max distance
        self.action_holder_angle = action_holder_angle.ActionHolderAngle(self.esp)                          # TODO: set min/max angle

    def set_speed(self, speed_left: float, speed_right: float) -> None:
        """
        Set the speed of the robot.
        """
        self.action_speed.execute(speed_left, speed_right)

    def stop(self) -> None:
        """
        Stop the robot.
        """
        self.action_emergency_stop.execute()

    def resume(self) -> None:
        """
        Resume the robot.
        """
        self.action_emergency_resume.execute()

    def set_grabber_height(self, height: float) -> None:
        """
        Set the height of the grabber.
        """
        self.action_grabber_height.execute(height)

    def set_grabber_angle(self, angle: float) -> None:
        """
        Set the angle of the grabber.
        """
        self.action_grabber_angle.execute(angle)

    def set_holder_distance(self, distance: float) -> None:
        """
        Set the distance of the holder.
        """
        self.action_holder_distance.execute(distance)

    def set_holder_angle(self, angle: float) -> None:
        """
        Set the angle of the holder.
        """
        self.action_holder_angle.execute(angle)

    def close(self) -> None:
        """
        Close the connection with the esp32.
        """
        self.esp.close()

    def close_camera(self) -> None:
        """
        Close the connection with the camera.
        """
        self.front_cam.close()
        self.right_cam.close()
        self.back_cam.close()
        self.left_cam.close()


class CoccinellesServer(BaseHTTPRequestHandler):
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        print("GET request,\nPath: %s\nHeaders:\n%s\n", str(self.path), str(self.headers))
        self._set_response()
        if self.path.startswith("/get_plant"):
            self.wfile.write(json.dumps(numpy.random.rand(36,2).tolist()).encode('utf-8'))
        if self.path.startswith("/get_time"):
            self.wfile.write(json.dumps(numpy.random.rand(1).tolist()).encode('utf-8'))


def main():
    TIME_INIT = time.time()
    robot = RobotController()
    robot.
