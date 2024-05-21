import Point
import network
import socket
import urequests
import utime

class PlantPositionGetter:
    
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
        self.ssid = "here"
        self.password = "Plik est grand"
        self.ip = "192.168.0.1"
        self.initWLAN()
        #self.openSocket()
    
    def initWLAN(self):
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)
        self.wlan.connect(self.ssid, self.password)
    
    def requestForPlantPosition(self):
        #request = urequests.get(self.connection)
        request = urequests.get("http://192.168.1.168/")
    
    def getY(self):
        pass
    
    def transformToPoint(self):
        return Point(self.x, self.y)
    
    def setX(self, x):
        self.x = x
        
    def setY(self, y):
        self.y = y
    
    def connect(self):
        #Connect to WLAN
        while self.wlan.isconnected() == False:
            print('Waiting for connection...')
            sleep(1)
        print(self.wlan.ifconfig())
        
    def openSocket(self):
        # Open a socket
        self.address = (self.ip, 80)
        self.connection = socket.socket()
        self.connection.bind(self.address)
        self.connection.listen(1)

"""plantPositionGetter = PlantPositionGetter()
#plantPositionGetter.connect()
utime.sleep(2)
plantPositionGetter.requestForPlantPosition()
"""

