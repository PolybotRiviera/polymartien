#!/usr/bin/env python


"""Module to communicate with the arduino"""

import time

import serial


class ESPCommunicator:
    """
    Class to communicate with the esp32 using serial communication.
    
    Please use send_trame to send a trame of data.
    Trames should be of the form: [code1, data1, code2, data2, ...]
    where code is the code of the command(>200) and data is the data associated with the command(<=200).
    """
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        self.esp = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.chars = "\r\n"

    def send(self, data):
        try:
            self.esp.write(bytes([data]))
            print(f"sent : {bytes([data])}'")
        except:
            self.esp.close()

    def send_trame(self, trame : list, delay=0.001):
        for data in trame:
            self.send(data)
            time.sleep(delay)
        
        return all(self.send(data) for data in trame if time.sleep(delay)==None) or len(trame)==0
        
        self.clean()

    def read(self) -> str:
        res = ""
        try:
            data = self.esp.readline()
            while data:
                print(f'->{data.decode().strip(self.chars)}<-')
                res += data.decode()
                data = self.esp.readline()
        except:
            self.esp.close()
        return res

    def clean(self) -> bool:
        try:
            self.esp.reset_input_buffer()
            self.esp.reset_output_buffer()
            return True
        except:
            self.esp.close()
            return False
