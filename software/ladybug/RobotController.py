import math
from machine import Pin, PWM, UART
from Point import Point
from HCSR04 import HCSR04
import gyro
import Switch
#sg90_left = PWM(Pin(21, mode = Pin.OUT))
#sg90_right = PWM(Pin(20, mode = Pin.OUT))
"""sg90_left.freq(50)
sg90_right.freq(50)
"""

gyroscope = gyro.Gyroscope()

UART_USED = 0
BAUDRATE = 115200
TX_PIN = Pin(0)
RX_PIN = Pin(1)

uart = UART(UART_USED,
            baudrate=BAUDRATE,
            tx=TX_PIN,
            rx=RX_PIN
            )

class RobotController:
    
    def __init__(self,
                 x0 = 0,
                 y0 = 0):
        
        self.x = x0
        self.y = y0
        self.robotWidth = 0.07 # m
        self.maxLinearVelocityX = 0.20 #m/s
        self.maxBackwardsLinearVelocityX = -0.20 #m/s
        self.maxAngularZDeg = 30
        self.maxAngularZRad = self.maxAngularZDeg * math.pi/180
        self.currentOrientation = 0 #rad
        self.dt = 0.1 #s
        #self.acceleration = self.maxLinearVelocityX/self.dt #m/s^2
        self.acceleration = 0.15 #m/s^2

        self.initMotors()
        self.initUART()
        self.init_PWM_values()
        
        self.leftMotorPWM = self.leftVelocityToPWM(0)
        self.rightMotorPWM = self.rightVelocityToPWM(0)
        self.currentTurnRate = 0
        self.angleError = 0
        
        self.lastAngle = 0
        self.estimatedAngleRad = 0
        
        self.offsetDistance = 0.03 #distance entre l'arrière du robot et le centre des roues.
        self.closeToPoint = False
        self.arrivedToPoint = False
        
        self.xPositionWithVelocity = x0
        self.yPositionWithVelocity = y0
        
        self.sensorRight = HCSR04(trigger_pin=7, echo_pin=6, echo_timeout_us=10000, offset=4, name = "rightSensor")
        self.sensorFront = HCSR04(trigger_pin=5, echo_pin=4, echo_timeout_us=10000, offset=0, name = "frontSensor")
        self.sensorLeft = HCSR04(trigger_pin=9, echo_pin=8, echo_timeout_us=10000, offset=0, name = "leftSensor")
        self.frontSafetyDistance = 0.10
        self.isAvoidingObstacle = False
        self.arrived = False
        
        self.velocityWanted = 0
        self.turnRateWanted = 0
        

        
    def initMotors(self,
                   rightMotorPinNumber = 20,
                   leftMotorPinNumber = 21):
        self.rightMotor = PWM(Pin(rightMotorPinNumber, mode = Pin.OUT))
        self.leftMotor = PWM(Pin(leftMotorPinNumber, mode = Pin.OUT))
        self.rightMotor.freq(50)
        self.leftMotor.freq(50)
    
    def initUART(self,
                 uart = 0,
                 baudrate = 115200,
                 txPinNumber = 0,
                 rxPinNumber = 1):
        
        self.uart = UART(uart,
                        baudrate,
                        tx=Pin(txPinNumber),
                        rx=Pin(rxPinNumber)
                        )
    def init_PWM_values(self,
                vMaxLeftPWM=8430,
                vMinLeftPWM=4990,
                vMaxRightPWM=1360,
                vMinRightPWM=4810
                ):
        
        self.vMaxLeftPWM = vMaxLeftPWM
        self.vMinLeftPWM = vMinLeftPWM
        
        self.vMaxRightPWM = vMaxRightPWM
        self.vMinRightPWM = vMinRightPWM
        
        self.backwardsMaxLeftPWM = 1354
        self.backwardsMinLeftPWM = 4789
        
        self.backwardsMaxRightPWM = 8473
        self.backwardsMinRightPWM = 5020

        
    def leftVelocityToPWM(self, vlx):
        a = (self.vMaxLeftPWM - self.vMinLeftPWM)/self.maxLinearVelocityX
        b = self.vMinLeftPWM
        #print(f"left motor PWM : {math.floor(a*vlx+b)}")
        return math.floor(a*vlx+b)
    
    def backwardsLeftVelocityToPWM(self, vlbx):
        #print("going backwards")
        a = (self.backwardsMaxLeftPWM - self.backwardsMinLeftPWM)/self.maxBackwardsLinearVelocityX
        b = self.backwardsMinLeftPWM
        #print(math.ceil(a*vlbx +b))
        return math.ceil(a*vlbx +b)
    
    def rightVelocityToPWM(self, vrx):
        
        a = (self.vMaxRightPWM - self.vMinRightPWM)/self.maxLinearVelocityX
        b = self.vMinRightPWM
        #print(f"right motor PWM : {math.ceil(a*vrx+b)}")
        return math.ceil(a*vrx+b)

    def goStraight(self, v = None):
        if v==None:
            v = self.velocityWanted
        self.rightMotorPWM = self.rightVelocityToPWM(v)
        self.leftMotorPWM = self.leftVelocityToPWM(v)
        self.leftMotor.duty_u16(self.leftMotorPWM) 
        self.rightMotor.duty_u16(self.rightMotorPWM)
    
    def turn(self,
             turnRate,
             v = 0.0):
        
        if turnRate > self.maxAngularZRad:
            turnRate = self.maxAngularZRad
            
        elif turnRate < -self.maxAngularZRad:
            turnRate = -self.maxAngularZRad
        if abs(turnRate) < 0.01:
            turnRate = 0
        vl = v - (turnRate*self.robotWidth/2)
        if v <= turnRate*self.robotWidth/2:
            self.leftMotorPWM = self.backwardsLeftVelocityToPWM(vl)
        else :
            self.leftMotorPWM = self.leftVelocityToPWM(vl)
        #vr = turnRate*self.robotWidth + vl
        vr = v + turnRate*self.robotWidth/2
        self.rightMotorPWM = self.rightVelocityToPWM(vr)
        self.leftMotor.duty_u16(self.leftMotorPWM) 
        self.rightMotor.duty_u16(self.rightMotorPWM)
        
        #self.currentTurnRate = (vr - vl) /self.robotWidth
        self.currentTurnRate = turnRate
        
    def orientTowardsPoint(self,
                         point: Point,
                         v=0.0):

        """angleStartToGoDeg = 45 #deg
        coeff = self.angleToPointDeg/(angleStartToGoDeg)
        baseTurnRate = 0.1
        turnRate = coeff*baseTurnRate"""
        pointRadius = point.getRadius()
        velocity = math.sqrt(
                            2*self.acceleration*(self.distanceToPoint - pointRadius)
                            )
        
        turnRate = 2*(self.getRightVelocity() - velocity)/self.robotWidth
        if turnRate > self.maxAngularZRad:
            turnRate = self.maxAngularZRad

        elif turnRate < -self.maxAngularZRad:
            turnRate =  -self.maxAngularZRad

        print("angleToPointDeg : ", self.angleToPointDeg)
        """if abs(self.angleToPointDeg) < 45:
            if self.distanceToPoint >= pointRadius:
                velocity = math.sqrt(
                                2*self.acceleration*(self.distanceToPoint - pointRadius)
                            )
            else:
                velocity = 0
        """
        self.turnRateWanted = turnRate
        self.velocityWanted = velocity
        #self.velocityWanted = velocity
    
    def goTowardsPoint(self, point: Point, velocity = 0.2):
        
        self.turnRateWanted = 0
        self.velocityWanted = velocity

    def stop(self):
        """self.rightMotorPWM = self.rightVelocityToPWM(0)
        self.leftMotorPWM = self.leftVelocityToPWM(0)
        self.leftMotor.duty_u16(self.leftMotorPWM) 
        self.rightMotor.duty_u16(self.rightMotorPWM)
        """
        self.velocityWanted  = 0
        self.turnRateWanted = 0
    
    def getPosition(self):
        return (self.x, self.y)
    
    def setX(self, x):
        self.x = x
    
    def setY(self, y):
        self.y = y
    
    def setPosition(self, position):
        self.x = position[0]
        self.y = position[1]
    
    def getX(self):
        return self.x
    
    def getY(self):
        return self.y
    
    def computeDistanceToPoint(self,
                               point: Point,
                               usePointRadius = True):
        pointRadius = 0
        pointX = point.getX()
        pointY = point.getY()
        if usePointRadius :
            pointRadius = point.getRadius()
            
        return math.sqrt(
                        (self.x - pointX)*(self.x - pointX) +
                        (self.y - pointY)*(self.y - pointY) 
                        )
        
    
    def computeOrientationToPoint(self,
                                  point: Point):
        #radians

        x = point.getX() - self.x
        y = point.getY() - self.y
        
        if x == 0:
            return math.pi/2
        
        angle = 0
        acceptableError = 0.1
        if y < -acceptableError:
            if x < 0:
                angle = math.atan(y/x) + math.pi - self.currentOrientation
            else:
                angle = math.atan(y/x) - self.currentOrientation
        elif y > acceptableError:
            if x < 0:
                angle = math.atan(y/x) + math.pi - self.currentOrientation
            else:
                angle = math.atan(y/x) - self.currentOrientation
        if angle > 2*math.pi :
            angle -= 2*math.pi
        if angle < -2*math.pi:
            angle += 2*math.pi
        if angle > math.pi:
            angle = -(2*math.pi - angle)
        return angle
    
    def setCurrentOrientation(self, angle):
        """rad"""
        self.currentOrientation = angle
    
    def getCurrentOrientation(self):
        """rad"""
        return self.currentOrientation
    
    def PWMToRightVelocity(self, PWM):
        a = (self.vMaxRightPWM - self.vMinRightPWM)/self.maxLinearVelocityX
        b = self.vMinRightPWM
        v = (PWM - b)/a
        return v
    
    def PWMToLeftVelocity(self, PWM):
        a = (self.vMaxLeftPWM - self.vMinLeftPWM)/self.maxLinearVelocityX
        b = self.vMinLeftPWM
        v = (PWM - b)/a
        return v
    
    def getCurrentLinearVelocityX(self):
        """rightVelocity = self.PWMToRightVelocity(self.rightMotorPWM)
        leftVelocity = self.PWMToLeftVelocity(self.leftMotorPWM)
        return (rightVelocity + leftVelocity) / 2
        """
        return self.velocityWanted
    def getCurrentTurnRate(self):
        return self.currentTurnRate
    
    def setAngleError(self, angleError):
        self.angleError = angleError
    
    def getLeftVelocity(self):
        if(abs(self.PWMToLeftVelocity(self.leftMotorPWM)) < 0.01):
            return 0
        return self.PWMToLeftVelocity(self.leftMotorPWM)
    
    def getRightVelocity(self):
        if(abs(self.PWMToRightVelocity(self.rightMotorPWM)) < 0.01):
            return 0
        return self.PWMToRightVelocity(self.rightMotorPWM)
    
    def discretizedAngleController(self,
                                   kp = 1,
                                   Ts = 0.1,
                                   Ti = 0.1,
                                   ):
        """Ne marche pas"""
        alpha = Ts/(2*Ti)
        
        uRightPast = self.rightMotorPWM
        uLeftPast = self.leftMotorPWM
        uPast = self.currentTurnRate
        
        angleErrorPast = self.angleError
        return (alpha-kp)*angleErrorPast + (alpha+Kp)*self.angleError + uPast
    
    
    def setVelocityWanted(self, velocity):
        self.velocityWanted = velocity
    
    def distanceCommandLaw(self, point: Point):
        distanceToPoint = self.computeDistanceToPoint(point)
        print(f"distance to point = {distanceToPoint}")
        acceptableDistance = 0.03 #m
        if distanceToPoint <= acceptableDistance:
            self.closeToPoint = True
            return 0
        self.closeToPoint = False

        #a = 0.1
        velocityWanted = math.sqrt(2*self.acceleration*(distanceToPoint - acceptableDistance))
        if velocityWanted > self.maxLinearVelocityX:
            velocityWanted = self.maxLinearVelocityX
        self.velocityWanted = velocityWanted
        return velocityWanted
    
    def maneuverCloseToPoint(self, point):
        
        acceptableManeuverAngleErrorDeg = 4 #deg
        lowVelocity = 0.05 #m/s
        coeff = self.angleToPointDeg/acceptableManeuverAngleErrorDeg
        baseTurnRate = self.maxAngularZRad/2
        turnRate = coeff*baseTurnRate
        if (abs(self.angleToPointDeg) >= acceptableManeuverAngleErrorDeg):
            #print("turning")
            self.turnRateWanted = turnRate
            self.velocityWanted = 0
        else:
            if self.distanceToPoint <= point.getRadius():
                self.velocityWanted = lowVelocity
                    
            
    def isCloseToPoint(self, point):
        return self.distanceToPoint <= point.getRadius()
    
    def odomToBaseLink(self):
        
        return (self.x + self.offsetDistance*math.cos(self.currentOrientation),
                self.y + self.offsetDistance*math.sin(self.currentOrientation)
                )
    def computeOdomToBaseLinkXCoordinate(self):
        return self.x + self.offsetDistance*math.cos(self.currentOrientation)
    
    def computeOdomToBaseLinkYCoordinate(self):
        return self.y + self.offsetDistance*math.sin(self.currentOrientation)
    
    def baseLinkToFront(self, odomPoint: Point):
        odomX = odomPoint.getX()
        odomY = odomPoint.getY()
        baseLinkCoords = self.odomToBaseLink()
        xbl = baseLinkCoords[0]
        ybl = baseLinkCoords[1]
        phi = self.currentOrientation
        return (
                (odomX - xbl)*math.cos(phi) + (odomY - ybl)*math.sin(phi),
                (xbl - odomX)*math.sin(phi) + (ybl - odomY)*math.cos(phi)
                )
    
    def getFrontDistance(self):
        #m
        return self.sensorFront.distance_cm()/100
    
    def avoidObstacle(self):
        
        sideSafetyDistance = 0.20 #m
        rightDistance = self.sensorRight.distance_cm()/100 #m
        leftDistance = self.sensorRight.distance_cm()/100 #m
        leftSideIsSafe = leftDistance > sideSafetyDistance
        rightSideIsSafe = rightDistance > sideSafetyDistance
        pointIsOnTheRight = self.angleToPointRad < 0
        pointIsOnTheLeft = self.angleToPointRad >= 0
        #velocity = math.sqrt(2*self.acceleration*(distanceToPoint - safetyDistance))
        if self.frontDistance > self.frontSafetyDistance:
            velocity = math.sqrt(
                                2*self.acceleration*(self.frontDistance - self.frontSafetyDistance)
                                 )
        else:
            velocity = 0
    
            baseTurnRate = 0.5
            turnRate = 0.0
            if rightSideIsSafe:
                if rightDistance > leftDistance :
                    turnRate = -baseTurnRate
                else:
                    turnRate = baseTurnRate
            elif leftSideIsSafe:
                if rightDistance > leftDistance :
                    turnRate = -baseTurnRate
                else:
                    turnRate = baseTurnRate
            else:
                turnRate = 0
                velocity = 0
        """    
        if pointIsOnTheRight:
            if rightSideIsSafe:
                turnRate = -baseTurnRate
            elif leftSideIsSafe:
                turnRate = baseTurnRate
            else:
                turnRate = 0
                velocity = 0
        else: #ie pointIsOnTheLeft
            if leftSideIsSafe:
                turnRate = baseTurnRate
            elif rightSideIsSafe:
                turnRate = -baseTurnRate
            else:
                turnRate = 0
                velocity = 0
        """
        self.velocityWanted = velocity
        self.turnRateWanted = turnRate
        self.isAvoidingObstacle = True
    
    def updateOrientation(self):
        #robot.setPosition(tagPosition)
        yawRateDeg = gyroscope.getYawRate()
        #print(f"yawRateDEg = {yawRateDeg}")
        gyroscope.setCurrentAngle(gyroscope.computeNewAngle())
        robotTurnRate = self.getCurrentTurnRate()
        yawRateRad = yawRateDeg*math.pi/180
        #estimatedTurnRate = (1-alphaTurnRate)*robotTurnRate + round((alphaTurnRate)*yawRateRad,2)
        estimatedTurnRate = round(yawRateRad, 2)
        deltaAngle = estimatedTurnRate * self.dt
        self.estimatedAngleRad += deltaAngle
        
       
        """print(f"gyroTurnRate : {yawRateRad} rad/s = {yawRateRad*180/math.pi} deg/s")
        print(f"robotCurrentTurnRate : {robot.getCurrentTurnRate()} rad/s")
        print(f"estimatedTurnRate : {estimatedTurnRate} rad/s")
        print(f"estimatedAngle : {estimatedAngleRad}rad = {round(estimatedAngleRad*180/math.pi, 2)}deg")
        """
        
        self.setCurrentOrientation(self.currentOrientation+deltaAngle)
        
        print(f"CurrentOrientation : {self.getCurrentOrientation()*180/math.pi} deg")
        
    
    def computeEstimatedPositionWithVelocity(self):
        linearVelocity = self.getCurrentLinearVelocityX()
        phi = self.currentOrientation
        vOdomFrameX = linearVelocity * math.cos(phi)
        vOdomFrameY = linearVelocity * math.sin(phi)
        self.xPositionWithVelocity += vOdomFrameX * self.dt
        self.yPositionWithVelocity += vOdomFrameY * self.dt
        print(f"Estimated position with velocity = {(self.xPositionWithVelocity, self.yPositionWithVelocity)}")
    
    def updatePosition(self, tagX, tagY):
        trustCoefficient = 0.7
        self.computeEstimatedPositionWithVelocity()
        self.x = (1-trustCoefficient)*tagX + trustCoefficient*self.xPositionWithVelocity
        self.y = (1-trustCoefficient)*tagY + trustCoefficient*self.yPositionWithVelocity
        print(f"fused position : = {(self.x, self.y)}")
        
    
    def computeCommand(self,
                       point: Point,
                       tagX,
                       tagY):
    
        try:
            #Initializing variables

            #print("begin computeCommand")
            self.angleToPointRad = self.computeOrientationToPoint(point)
            print("angleToPointRad : ", self.angleToPointRad)
            self.angleToPointDeg = self.angleToPointRad*180/math.pi
            velocity = 0
            self.distanceToPoint = self.computeDistanceToPoint(point)
            acceptableAngleErrorDeg = 10

            print("distanceToPoint : ", self.distanceToPoint)
            self.frontDistance = self.sensorFront.distance_cm()/100 #m
            frontDistanceValueIsValid = self.frontDistance > 0
            pointIsJardiniere = point.isJardinierePoint()
            pointRadius = point.getRadius()
            isCloseToPoint = self.isCloseToPoint(point)
            #Orient the robot towards the point and go towards the Point point
                        
            if (abs(self.angleToPointDeg) >= acceptableAngleErrorDeg) and not self.isAvoidingObstacle:
                self.orientTowardsPoint(point)
                print("self.turnRateWanted after orientself : ", self.turnRateWanted)
            #else:
                #self.stop()
            else:
                
                if not isCloseToPoint:

                    self.goTowardsPoint(point, velocity)
                    print("velocity : ", velocity)
                    print("Velocity after goTowardsPoint : ", self.velocityWanted)
                    #Avoid obstacles  
                    if self.frontDistance < self.frontSafetyDistance and frontDistanceValueIsValid and not self.arrived:
                        print("AVOIDING OBSTACLES : ", self.frontDistance, " m")
                        self.avoidObstacle()
                    else:
                        self.isAvoidingObstacle = False
                    print("Velocity after avoidObstacle function : ", self.velocityWanted)
                    print("TurnRate after avoidObstacle function : ", self.turnRateWanted)
                    
                else:
                    #self.stop()
                    print("I AM CLOSE TO POINT")
                    if pointIsJardiniere:
                        acceptableJardiniereAngleError = math.pi/12 #rad (15°)
                        distanceToPi = math.pi - abs(self.currentOrientation) #rad
                        print("Jardiniere")
                        self.velocityWanted = 0
                        if distanceToPi > acceptableJardiniereAngleError :
                            print(f"distanceToPi = {distanceToPi}")
                            if(math.pi - self.currentOrientation) > 0:
                                self.turnRateWanted = -0.4
                            else:
                                self.turnRateWanted = 0.4
                        else:
                            print("Oriented")
                            self.stop()
                            self.arrived = True
                    else:
                        self.maneuverCloseToPoint(point)
                        print("Velocity after maneuver : ", self.velocityWanted)
                
            self.publish()
            self.updatePosition(tagX, tagY)
            self.updateOrientation()
            print(f"velocityWanted = {self.velocityWanted}")
            print(f"turnRateWanted = {self.turnRateWanted}")
            #print("end computeCommand")
            print("##################")
        except Exception as e:
            self.stop()
            raise e
    
    def publish(self):
        
        if self.velocityWanted > self.maxLinearVelocityX:
            self.velocityWanted = self.maxLinearVelocityX
        self.rightVelocityToPWM(self.velocityWanted)
        self.leftVelocityToPWM(self.velocityWanted)
        self.turn(self.turnRateWanted, self.velocityWanted)
    
    def forcedStop(self):
        self.rightVelocityToPWM(0)
        self.leftVelocityToPWM(0)
        self.turn(0, 0)
        
        
