from MainHeader import *

def createSpectrogram(ax,
                      timeDomainSignal,
                      SamplingFrequency,            #sample rate
                      Title="Spectrogram",
                      scale_type='density'):        #power 'density' or power ‘spectrum’

    #mode [‘psd’, ‘complex’, ‘magnitude’, ‘angle’, ‘phase’]

    f, t, Sxx = signal.spectrogram(x=timeDomainSignal,
                                   fs=SamplingFrequency,
                                   scaling=scale_type,mode ='magnitude')

    try:
        ax.pcolormesh(t, f, Sxx, shading='gouraud')
    except:
        pass
    addLabels(ax, "Time(s)", "Frequency(Hz)", Title)
    return

#Power Spectral Density
def createPSD(ax,timeDomainSignal,SamplingFrequency,Title='PSD',scale_type='linear',MotorType:MotorClass=None):
    f, Pxx_den = signal.periodogram(timeDomainSignal, SamplingFrequency, scaling='density')
    mask = f >= 0   #CutoffFrequency
    createPlot(ax,"Freq(Hz)","Power Density (V^2/Hz)",f[mask], Pxx_den[mask],Title,scale_type,MotorType)
    return

#draw a vertical line on "ax" using all values of "x_data" that are integer multiples of "peak"
def printHarmonics(ax,x_data,peak):
    for Harmonic in range(2, 100):
        try:
            ax.axvline(x=x_data[Harmonic * peak], c='g', linestyle='--', linewidth=0.5)
        except:
            break;
    return
def findPeaks(y_data,height=0):
    peaks = []

    peaks, _ = signal.find_peaks(y_data, height=height)

    max = peaks

    if len(peaks) > 0:
        max = peaks[0]
        for x in peaks:
            if (y_data[x] > y_data[max]):
                max = x

    peaks = max

    return peaks
def displayCalculatedMotorFrequency(ax,x_data,y_data,MotorType:MotorClass=None):

    # add vertical line and legend showing the rated speed
    RPS = MotorType.MotorDictionary[MotorType.type]["sConst"] * MotorType.Voltage / 60
    ax.axvline(x=RPS, c='r', label='Rated Speed: {:.2f}'.format(RPS), linewidth=0.5)

    #count how far into the dataset the rated speed is
    counter = 0
    for x in  x_data:
        if x >= RPS:
            break
        counter += 1

    #find the rotational velocity; using data +=10% of rated speed
    lower = counter *0.5
    upper = counter *1.05
    temp_y_data = y_data[int(lower):int(upper)]

    try:
        peaks = findPeaks(temp_y_data) + int(lower)

        RPS = x_data[peaks]

        printHarmonics(ax, x_data, peaks);
    except:
        pass

    # add vertical line and legend showing the measured speed
    ax.axvline(x=RPS, c='orange', label='Measured Speed: {:.2f}'.format(RPS), linewidth=0.5)

    # slot frequency is fr* (Number Of Stator Slots)
    ax.axvline(x=RPS * MotorType.MotorDictionary[MotorType.type]["StatSlots"], c='y', linestyle='--', linewidth=0.5)

    #output of gearRatio
    if MotorType.MotorDictionary[MotorType.type]["gearRatio"] > 0:
        ax.axvline(x=RPS / MotorType.MotorDictionary[MotorType.type]["gearRatio"], c='y', linestyle='--', linewidth=0.5)

    # show commutator and side bands
    CommSegs = MotorType.MotorDictionary[MotorType.type]["CommSeg"]

    commutatorLinesThickness = 1
    if CommSegs != 0:
        CommSegs *= RPS
        sideBandColor = 'black'
        centreBandColor = "purple"
        numSideBands = MotorType.MotorDictionary[MotorType.type]["CommBands"]

        while numSideBands != 0:
            ax.axvline(x=CommSegs + numSideBands * RPS,    c=sideBandColor,    linestyle='--', linewidth=commutatorLinesThickness)
            ax.axvline(x=CommSegs - numSideBands * RPS,    c=sideBandColor,    linestyle='--', linewidth=commutatorLinesThickness)
            numSideBands-=1

        ax.axvline(x=CommSegs,              c=centreBandColor,  linestyle='--', linewidth=commutatorLinesThickness)

    ax.legend()
    return

def findPeakFrequency(ax,y_data,x_data,Title):
    if Title.find("FFT") != -1:
        try:
            peaks = [0,0]
            counter = 1

            dc_offset = 10
            temp_y_data = y_data[dc_offset:]    #ignore DC component

            peaks = findPeaks(temp_y_data,height=counter)

            peaks += dc_offset
            max_value = float(x_data[peaks])
            ax.plot(x_data[peaks],y_data[peaks],"x",label="Peak: {:.2f}".format(max_value))


            ax.legend()

            if peaks >1:
                printHarmonics(ax,x_data,peak)
        except:
            pass

    return
def createPlot(ax,xLabel,yLabel,x_data,y_data,Title,scale_type='linear',MotorType:MotorClass=None):
    if scale_type == 'none':
        return
    #draw a vertical line for the motor's rate velocity in Hz

    if MotorType is not None:
        if MotorType.type != 'None':    #none is selected by default
            displayCalculatedMotorFrequency(ax,x_data,y_data,MotorType)
    else:
        # indicate the peak rotational frequency
        findPeakFrequency(ax, y_data, x_data, Title)

    if scale_type == 'linear':
        ax.plot(x_data, y_data, '-')

    elif scale_type == 'semilogy':
        ax.semilogy(x_data, y_data, '-')

    elif scale_type == 'semilogx':
        ax.semilogx(x_data, y_data, '-')

    else:
        ax.loglog(x_data, y_data, '-')

    addLabels(ax,xLabel,yLabel,Title)
    return

def addLabels(ax,xLabel,yLabel,Title):
    ax.set_xlabel(xLabel)
    ax.set_ylabel(yLabel)
    ax.set_title(Title)
    return
"""
create an evenly distributed time/frequency axis
"""
def create_time_axis(timeDomainSignal,SamplingFrequency):
    N = len(timeDomainSignal)
    time_axis = np.linspace(0, (N - 1) / SamplingFrequency, N)

    return time_axis
