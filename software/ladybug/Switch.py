from machine import Pin
import utime
class Switch:
    
    pressed = False
    
    def __init__(self, pinNumber = 16):
        self.button = Pin(pinNumber, Pin.IN, Pin.PULL_DOWN)
        
    def getValue(self):
        return self.button.value()    


"""switch = Switch()
while 1:
    print(switch.getValue())
    utime.sleep(0.1)
  """      