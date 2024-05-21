import machine
import time
import MPU6050

# Set up the I2C interface
i2c = machine.I2C(1, sda=machine.Pin(2), scl=machine.Pin(3))

# Set up the MPU6050 class 
mpu = MPU6050.MPU6050(i2c)

# wake up the MPU6050 from sleep
mpu.wake()

# continuously print the data

ROLL_RATE_OFFSET = -1.1
PITCH_RATE_OFFSET = 6.5
YAW_RATE_OFFSET = 0.32
rollRate = 0
pitchRate = 0
yawRate = 0
currentAngle = 0
dt = 0.01
k = 90/77.5


class Gyroscope():
    
    """ROLL_RATE_OFFSET = -1.1
    PITCH_RATE_OFFSET = 6.5
    YAW_RATE_OFFSET = 0.32
    rollRate = 0
    pitchRate = 0
    yawRate = 0
    currentAngle = 0
    dt = 0.01
    k = 90/77.5"""
    
    k = 90/38.6
    
    def __init__(self, inverted = True):
        
        self.rollRateOffset = 0.0
        self.pitchRateOffset = 0.0
        self.yawRateOffset = 0.0
        self.rollRate = 0.0
        self.pitchRate= 0.0
        self.yawRate = 0.0
        self.currentAngle = 0.0
        self.dt = 0.01
        self.inverted = inverted
    
    def lowPassFilter(self, alpha, currentSignalValue, lastSignalValue):
        return alpha*lastSignalValue + (1-alpha)*currentSignalValue

    def setRollRateOffset(self, rollRate):
        self.rollRateOffset = rollRate

    def setPitchRateOffset(self, pitchRate):
        self.pitchRateOffset = pitchRate
        
    def setYawRateOffset(self, yawRate):
        self.yawRateOffset = yawRate
    
    def getYawRate(self):
        self.computeYawRate()
        if (abs(self.yawRate) <= 0.05):
            return 0
        if self.inverted:
            return -self.yawRate
        return self.yawRate

    def setDt(self, dt):
        self.dt = dt
        
    def setCurrentAngle(self, angle):
        self.currentAngle = angle
    
    def getCurrentAngle(self):
        return self.currentAngle
    
    def computeYawRate(self):
        gyro = mpu.read_gyro_data()
        self.yawRate = self.lowPassFilter(0.95, self.yawRate, Gyroscope.k*(gyro[2]- self.yawRateOffset))
        
    def computeNewAngle(self):
        return self.currentAngle + Gyroscope.k*dt*self.yawRate


#def lowPassFilter(alpha, currentSignalValue, lastSignalValue):
#    return alpha*lastSignalValue + (1-alpha)*currentSignalValue

"""while True:
    gyro = mpu.read_gyro_data()
    yawRate = lowPassFilter(0.95, yawRate, gyro[2]- YAW_RATE_OFFSET)
    currentAngle+= k*dt*yawRate
    print(f"currentAngle : {currentAngle}")
    time.sleep(dt)
"""