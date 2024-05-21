import Point

class Zone:
    def __init__(self,
                 xLow = 0,
                 xHigh = 0,
                 yLow = 0,
                 yHigh = 0):
        
        self.xLow = xLow
        self.xHigh = xHigh
        self.yLow = yLow
        self.yHigh = yHigh
    
    def __contains__(self, point: Point):
        
        px = point.getX()
        py = point.getY()
        return (px >= self.xLow and px <= self.xHigh) and (py >= self.yLow and py <= self.yHigh)

    def getMiddlePoint(self):
        
        x = (self.xLow + self.xHigh)/2
        y = (self.yLow + self.yHigh)/2
        return Point.Point(x, y)