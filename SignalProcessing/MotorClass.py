

class MotorClass():
    ListOfCLasses = []

    def __init__(self, type=None):
        self.type = type
        self.MotorDictionary = {}

        self.CreateNoneTypeMotor()
        self.CreateMaxonAccessPointMotor()
        self.CreateFaulhaberAccessPointMotor()
        self.CreateZeusD_Drive()
        self.CreateMaxonMainDriver()
        self.CreateRecapperGripper()

        #must be last!
        self.CreateListOfCLasses()

    def CreateListOfCLasses(self):
        for keys in self.MotorDictionary.keys():
            self.ListOfCLasses.append(keys)
    def AddMotorClassToDictionary(self,
                                  tag="None",               #Name for motor
                                  SpeedConstant=0,          #rpm/v (if known)
                                  PPR=0,                    #encoder PulsesPerRev (if known)
                                  CommutatorSegments=0,     #number of bars in commuatator (if known)
                                  CommutatorSideBands=2,    #
                                  StatorSlots=2,            #slots in stator's magnetic field
                                  gearRatio=0):

        #allocate memory
        self.MotorDictionary[tag] = {}

        #add encoder resolution
        self.MotorDictionary[tag]["PPR"] = PPR * 2      #double the rate PPR, due to XOR gate

        #Motor Parameters
        self.MotorDictionary[tag]["sConst"] = SpeedConstant

        self.MotorDictionary[tag]["CommSeg"] = CommutatorSegments
        self.MotorDictionary[tag]["CommBands"] = CommutatorSideBands
        self.MotorDictionary[tag]["StatSlots"] = StatorSlots
        self.MotorDictionary[tag]["gearRatio"] = gearRatio

    def CreateRecapperGripper(self):
        self.AddMotorClassToDictionary(tag="RecapperGripper",
                                       SpeedConstant=8600/24,
                                       PPR=512)
    def CreateMaxonMainDriver(self):
        self.AddMotorClassToDictionary(tag="X/Y/Z/U Drive",
                                       SpeedConstant=182,
                                       PPR=500,
                                       CommutatorSegments=13,
                                       CommutatorSideBands=3)
    def CreateMaxonAccessPointMotor(self):
        self.AddMotorClassToDictionary(tag="Maxon AP/Gripper",
                                       SpeedConstant=156,
                                       PPR=256,
                                       CommutatorSegments=9,
                                       gearRatio=19)

    def CreateNoneTypeMotor(self):
        self.AddMotorClassToDictionary(tag="None",
                                       SpeedConstant=0,
                                       PPR=0)

    def CreateFaulhaberAccessPointMotor(self):
        self.AddMotorClassToDictionary(tag="Faulhaber AP/Recapper Magazine",
                                       SpeedConstant=301,
                                       PPR=1024)

    def CreateZeusD_Drive(self):
        self.AddMotorClassToDictionary(tag="Zeus D-Drive",
                                       SpeedConstant=397,
                                       PPR=128)