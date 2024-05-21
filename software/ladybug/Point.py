class Point:
    
    def __init__(self,
                 x=0,
                 y=0,
                 radius = 0.1,
                 isJardiniere = False):
        
        self.x = x #m
        self.y = y #m
        self.radius = radius #m
        self.isJardiniere = isJardiniere #bool
        
    def getX(self):
        #in meters
        return self.x
        
    def getY(self):
        #in meters
        return self.y
    
    def getRadius(self):
        #in meters
        return self.radius
    
    def isJardinierePoint(self):
        return self.isJardiniere
    