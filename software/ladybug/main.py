import DWM1001
import RobotController
import machine
import gyro
import math
import Point
import utime
import random
import Switch
import Zone
import PlantPositionGetter

isBlue = False
isYellow = True

yellowMiddleZone = Zone.Zone(xLow = 0.000,
                             xHigh = 0.450,
                             yLow = 1.250,
                             yHigh = 1.725)

blueMiddleZone = Zone.Zone(xLow=2.550,
                           xHigh=3.000,
                           yLow = 1.250,
                           yHigh = 1.725)

blueBottomZone = Zone.Zone(xLow = 0.000,
                           xHigh = 0.450,
                           yLow = 0.000,
                           yHigh = 0.450)

yellowBottomZone = Zone.Zone(xLow = 2.550,
                             xHigh = 3.000,
                             yLow = 0.000,
                             yHigh = 0.450)

yellowProtectedZone = Zone.Zone(xLow = 2.550,
                                xHigh = 3.000,
                                yLow = 1.550,
                                yHigh = 2.000)

blueProtectedZone = Zone.Zone(xLow = 0.000,
                              xHigh = 0.450,
                              yLow = 1.550,
                              yHigh = 2.000)


switch = Switch.Switch()
dt = 0.1 #s
x0 = 1.500 #m
y0 = 2.000 #m

uart = machine.UART(0,
                    baudrate=115200,
                    tx=machine.Pin(0),
                    rx=machine.Pin(1)
                    )
tag = DWM1001.DWM1001(x0,y0)
middlePointYellowMiddleZone = yellowMiddleZone.getMiddlePoint()

"""zonePoint = Point.Point(x=0.90,
                        y=0.30,
                        radius=0.10)"""
zonePoint = middlePointYellowMiddleZone
"""
xLow = 0
yLow = 0
xHigh = 0.90
yHigh = 0.90
zonePoints = [Point.Point(xLow,yLow),
              Point.Point(xLow, yLow),
              Point.Point(xHigh,yLow),
              Point.Point(xHigh, yHigh)]
"""
robot = RobotController.RobotController(x0, y0)
gyroscope = gyro.Gyroscope()
robot.setCurrentOrientation(-math.pi/2) #rad


positionGetCommand = b'\x02\x00'
resetCommand = b'\x17\x00'

def robotGo():
    if switch.getValue() == 1:
        switch.pressed = not switch.pressed
        while switch.getValue() == 1:
            pass
        
    if switch.pressed :
        robot.forcedStop()
        return
    
    uart.write(positionGetCommand)
    #uart.write(b'\x0c\x00')
    if uart.any():
        data = uart.read()
        tagPosition = tag.getFilteredPosition(data)
        tagX = tagPosition[0]
        tagY = tagPosition[1]
        robot.computeCommand(zonePoint, tagX, tagY)
    else:
        print("no data")

while True:
    #print(f"zonePoint = {zonePoint.getX(), zonePoint.getY()}")
    robotGo()
    #robot.goStraight(0.20)
    utime.sleep(dt)