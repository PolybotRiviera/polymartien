import time
import serial


class CameraCommunicator:
    """
    Class to receive data from the the camera using serial communication.
    """
    def __init__(self, port='/dev/camera/front_cam', baudrate=115200, timeout=1):
        self.camera = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.chars = "\r\n"

    def read(self) -> str:
        res = ""
        try:
            data = self.camera.readline()
            while data:
                print(f'->{data.decode().strip(self.chars)}<-')
                res += data.decode()
                data = self.camera.readline()
        except:
            self.camera.close()
        return res

    def clean(self) -> bool:
        try:
            self.camera.reset_input_buffer()
            self.camera.reset_output_buffer()
            return True
        except:
            self.camera.close()
            return False