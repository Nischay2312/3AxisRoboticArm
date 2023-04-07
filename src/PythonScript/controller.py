import hid

#make a class that contains all the controller button states
class Controller:
    def __init__(self):
        self.LXaxis = 0
        self.LYaxis = 0
        self.RXaxis = 0
        self.RYaxis = 0
        self.Dpad = 0
        self.Triangle = 0
        self.Circle = 0
        self.Cross = 0
        self.Square = 0
        self.L1 = 0
        self.L2 = 0
        self.R1 = 0
        self.R2 = 0
        self.gamepad = hid.device()
        self.gamepad.open(0x054c, 0x0ce6)
        self.gamepad.set_nonblocking(True)
    
    def ReadController(self):
        report = self.gamepad.read(64)
        if report:
            #call a function to fill the variables
            self.UpdateVariables(report)


    def UpdateVariables(self,report):
        self.LXaxis = report[1]
        self.LYaxis = report[2]
        self.RXaxis = report[3]
        self.RYaxis = report[4]
        #For D pad, report[5]'s first 4 bits are used
        self.Dpad =  report[5] & 0x0F
        self.Triangle = (report[5] & 0x80) >> 7
        self.Circle = (report[5] & 0x40) >> 6 
        self.Cross = (report[5] & 0x20) >> 5
        self.Square = (report[5] & 0x10) >> 4
        self.L1 = report[6] & 0x01
        self.R1 = (report[6] & 0x02) >> 1 
        self.L2 = report[8]
        self.R2 = report[9]

    def GetControllerState(self):
        #read the controller
        self.ReadController()
        #output all variables as a dictionary
        return {k:v for k, v in self.__dict__.items() if not (k.startswith('__') and k.endswith('__'))}
    
    def PrintData(self):
        print("LXaxis: ",self.LXaxis)
        print("LYaxis: ",self.LYaxis)
        print("RXaxis: ",self.RXaxis)
        print("RYaxis: ",self.RYaxis)
        print("Dpad: ",self.Dpad)
        print("Triangle: ",self.Triangle)
        print("Circle: ",self.Circle)
        print("Cross: ",self.Cross)
        print("Square: ",self.Square)
        print("L1: ",self.L1)
        print("L2: ",self.L2)
        print("R1: ",self.R1)
        print("R2: ",self.R2)
    
    