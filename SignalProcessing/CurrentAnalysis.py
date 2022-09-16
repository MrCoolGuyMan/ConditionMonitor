from MainHeader import *    #* means import everything

def processDCCurrent(CurrentData,currentSensorOffsetVoltage,MotorType:str = 'DEFAULT'):
    DC_Current = []
    for i in range(0, len(CurrentData)):
        Amplitude = CurrentData[i]
        # convert from ADC to milliAmps
        Amplitude /= 1024   # 10 bit ADC
        Amplitude *= 4.096  # ADC reference Value
        Amplitude -= currentSensorOffsetVoltage
        Amplitude /= 0.4    # sensor Resolution V/A
        Amplitude *= 1000   # convert to mA

        DC_Current.append(Amplitude)
    return  DC_Current

def ProcessRawSensorData(filenames, currentSensorOffsetVoltage:float,MotorType:MotorClass):
    defaultReturnvalue = ('Error','Error', 'Error')

    #check input arguments
    if len(filenames) == 0:
        return defaultReturnvalue

    if MotorType.type == 'None':
        return defaultReturnvalue

    CurrentData = None
    EncoderData = None

    #one will contain a filename with "Current" and one with "Encoder"
    for x in filenames:

        newFile = copyCSVfileToMem(x)

        if x.find('Current') != -1:
            CurrentData = newFile

        if x.find('Encoder') != -1:
            EncoderData = newFile

    if CurrentData is not None:
        DC_Current = processDCCurrent(CurrentData,currentSensorOffsetVoltage)

    #process speed information
    if EncoderData is not None:
        Theta = math.pi / 4
        AC_Current = []
        Speed = []

        for i in range(0, len(EncoderData)):
            pulses = EncoderData[i]
            # use encoder pulses to calculate the current angular position of the motor
            pulses = pulses / MotorType.MotorDictionary[MotorType.type]["PPR"]  # now a percentage of a revolution
            Theta += pulses * 2 * math.pi  # convert to radian; 2Pi = 360 Degrees
            Speed.append(math.sin(Theta))  # append to sine wave

            if CurrentData is not None:
                try:
                    AC_Current.append(DC_Current[i]*math.sin(Theta))
                except:
                    pass

    # final slash, i.e. file directory C:\Program Files\blah blah
    offset = findFinalSlashPosition(filenames[0])


    DirectoryName_AC    = filenames[0][0:offset] + "/AC_Current.csv"
    DirectoryName_VEL   = filenames[0][0:offset] + "/Velocity.csv"
    DirectoryName_DC    = filenames[0][0:offset] + "/DC_Current.csv"


    DirectoryName_AC    = SaveTo_np_SaveText(DirectoryName_AC,AC_Current)
    DirectoryName_VEL   = SaveTo_np_SaveText(DirectoryName_VEL,Speed)
    DirectoryName_DC    = SaveTo_np_SaveText(DirectoryName_DC,DC_Current)

    return DirectoryName_AC,DirectoryName_VEL,DirectoryName_DC

def SaveTo_np_SaveText(fileDir,Data):
    print("Attempting to save to: " + fileDir)
    try:
        np.savetxt(fileDir,Data,delimiter=',')
        print("success")
    except:
        print("failed")
        fileDir = 'Error'

    return fileDir