import machine

class DWM1001:
    
    def __init__(self,
                 x0=0,
                 y0=0):
    
        self.rawX = 0 
        self.rawY = 0
        self.filteredX = x0
        self.filteredY = y0
        self.started = False #Pour la toute première valeur de position
        self.prefix = "0x"
    
    def hexstringToInt(self, hexstring):
        try:
            return int(hexstring, 16)
        except Exception as e:
            print("In function hexStringToInt in DWM1001.py : ", e)
            return 0
    
    def computeHexstring(self, data):
        if data == None:
            print("data is None")
            return
        info = [data[i:i+1] for i in range(0, len(data), 1)]
        hexstr = ""
        for byte in info:
            try:
                b = byte
                t = self.prefix+byte.hex()
                #print(t, end = " ")
        
                if t == "0xff":
                    t = "0x00"
                hexstr += t + " "                 
            except UnicodeError:
                print (" byte = ", byte)
                break
        return hexstr
    
    def computeRaw2DPosition(self, hexstring):
        #in centimeters
        if hexstring == None:
            return
        hexstrList = hexstring.split(" ")
        try:
            #print(hexstrList)
            x = hexstrList[5:9]
            y = hexstrList[9:13]
                #print(f"xl : {xl}")
                
                #
            rawXMillimeters = ((self.hexstringToInt(x[0]))
                                + (self.hexstringToInt(x[1]) << 8)
                                + (self.hexstringToInt(x[2]) << 16)
                                + (self.hexstringToInt(x[3]) << 24)) #mm
                    
            rawYMillimeters = ((self.hexstringToInt(y[0]))
                                + (self.hexstringToInt(y[1]) << 8)
                                + (self.hexstringToInt(y[2]) << 16)
                                + (self.hexstringToInt(y[3]) << 24)) #mm
            rawXMeters = rawXMillimeters/1000
            rawYMeters = rawYMillimeters/1000
            self.rawX = rawXMeters
            self.rawY = rawYMeters
        except IndexError:
            print(hexstrList)    
        
    
    def getRawX(self):
        return self.rawX
    
    def getRawY(self):
        return self.rawY
    
    def lowPassFilter(self, alpha = 0.5):
        self.filteredX = (1-alpha)*self.rawX + alpha*self.filteredX
        self.filteredY = (1-alpha)*self.rawY + alpha*self.filteredY
    
    def getFilteredX(self):
        return self.filteredX

    def getFilteredY(self):
        return self.filteredY
    
    def getFilteredPosition(self, data):
        hexstring = self.computeHexstring(data)
        self.computeRaw2DPosition(hexstring)
        self.lowPassFilter()
        bigValue = 3
        if self.started :
            if abs(self.rawX - self.filteredX) >= bigValue:
                self.filteredX = 0.1
            if abs(self.rawY - self.filteredY) >= bigValue:
                self.filteredY = 0.1
        self.started = True
        return (self.filteredX, self.filteredY)
    
    def reset(self, uart):
        uart.write(b'\x17\x00')
    
