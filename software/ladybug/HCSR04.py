import machine, time
from machine import Pin

__version__ = '0.2.0'
__author__ = 'Roberto S鐠嬶箯chez'
__license__ = "Apache License 2.0. https://www.apache.org/licenses/LICENSE-2.0"



class HCSR04:
    """
    Driver to use the untrasonic sensor HC-SR04.
    The sensor range is between 2cm and 4m.
    The timeouts received listening to echo pin are converted to OSError('Out of range')
    """
    # echo_timeout_us is based in chip range limit (400cm)
    def __init__(self, trigger_pin, echo_pin, echo_timeout_us=500*2*30, offset=0, name = ""):
        """
        trigger_pin: Output pin to send pulses
        echo_pin: Readonly pin to measure the distance. The pin should be protected with 1k resistor
        echo_timeout_us: Timeout in microseconds to listen to echo pin. 
        By default is based in sensor limit range (4m)
        """
        self.echo_timeout_us = echo_timeout_us
        # Init trigger pin (out)
        self.trigger = Pin(trigger_pin, mode=Pin.OUT, pull=None)
        self.trigger.value(0)

        # Init echo pin (in)
        self.echo = Pin(echo_pin, mode=Pin.IN, pull=Pin.PULL_DOWN)
        self.specialNumber = -0.01718213
        self.lastCms = 0
        self.offset = offset
        self.name = name
    def _send_pulse_and_wait(self):
        """
        Send the pulse to trigger and listen on echo pin.
        We use the method `machine.time_pulse_us()` to get the microseconds until the echo is received.
        """
        #print(f"Begin send pulse and wait function {self.name}")
        self.trigger.value(0) # Stabilize the sensor
        time.sleep_us(5)
        self.trigger.value(1)
        # Send a 10us pulse.
        time.sleep_us(10)
        self.trigger.value(0)
        try:
            pulse_time = machine.time_pulse_us(self.echo, 1, self.echo_timeout_us)
            #print(f"Ending send pulse and wait function {self.name}")
            #print(f"pulse_time : {pulse_time} {self.name}")
            return pulse_time
        except OSError as ex:
            if ex.args[0] == 110: # 110 = ETIMEDOUT
                raise OSError('Out of range')
            print("I AM BLOCKED ?")
            raise ex

    def distance_mm(self):
        """
        Get the distance in milimeters without floating point operations.
        """
        pulse_time = self._send_pulse_and_wait()

        # To calculate the distance we get the pulse_time and divide it by 2 
        # (the pulse walk the distance twice) and by 29.1 becasue
        # the sound speed on air (343.2 m/s), that It's equivalent to
        # 0.34320 mm/us that is 1mm each 2.91us
        # pulse_time // 2 // 2.91 -> pulse_time // 5.82 -> pulse_time * 100 // 582 
        mm = pulse_time * 100 // 582
        return mm

    def distance_cm(self):
        """
        Get the distance in centimeters with floating point operations.
        It returns a float
        """
        #print("Begin distance_cm function")
        pulse_time = self._send_pulse_and_wait()

        # To calculate the distance we get the pulse_time and divide it by 2 
        # (the pulse walk the distance twice) and by 29.1 becasue
        # the sound speed on air (343.2 m/s), that It's equivalent to
        # 0.034320 cm/us that is 1cm each 29.1us
        cms = ((pulse_time / 2) / 29.1) - self.offset
        if cms < 0:
            return 0
        self.lastCms = cms
        #print("Ending distance cm function")
        return cms
        
 
"""
sensorRight = HCSR04(trigger_pin=7, echo_pin=6, echo_timeout_us=10000, offset=4, name = "sensorRight")
sensorFront = HCSR04(trigger_pin=5, echo_pin=4, echo_timeout_us=10000, offset=0, name = "sensortFront")
sensorLeft = HCSR04(trigger_pin=9, echo_pin=8, echo_timeout_us=10000, offset=0, name = "sensorLeft")
cpt = 0
while True:
    print("beginning loop")
    distanceRight = sensorRight.distance_cm()
    print('Distance right:', distanceRight, 'cm')
    distanceFront = sensorFront.distance_cm()
    print('Distance front:', distanceFront, 'cm')
    distanceLeft = sensorLeft.distance_cm()
    print('Distance left:', distanceLeft, 'cm')
    cpt+=1
    print(f"cpt : {cpt}")
    time.sleep(0.1)
"""