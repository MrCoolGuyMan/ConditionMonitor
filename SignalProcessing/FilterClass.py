from MainHeader import *

class FilterClass():
    #class constructor
    """
    Filter Class

    order -
        1 for most signals
        higher values for noisy signals

    type
        Use highpass filter to remove DC signal
        Use lowpass filter to remove AC/Noise signal

    "CutoffFrequency"
        Hertz
        -3dB point

    Sampling Frequency -
        for Single ADC sensor, default is 100Hz
        for Accelerometer, default is 100Hz
    """
    def __init__(self, order=1,
                 filterType='lowpass',
                 CutoffFrequency= [1],
                 SamplingFrequency=100,
                 OutputType='sos',
                 windowing = [0, 100000],
                 PSDAxis='linear',
                 FFTAxis='linear',
                 TimeAxis='linear',
                 SpectrogramAxis = 'density'
    ):

        self.Order = order
        self.Type = filterType                      #{‘lowpass’, ‘highpass’, ‘bandpass’, ‘bandstop’}
        self.CutoffFrequency = CutoffFrequency      #use list for bandpass/stop
        self.SamplingFrequency = SamplingFrequency  #only used for normalising frequency axis, not critical
        self.OutputType = OutputType
        self.Windowing = windowing
        self.PSDAxis = PSDAxis
        self.FFTAxis = FFTAxis
        self.TimeAxis = TimeAxis
        self.SpectrogramAxis = SpectrogramAxis

    """
    Basic filter
    creates an array containing the average of the last
    "filter_order" number of samples
    """
    def MovingAverageFilter(self,timeDomainSignal):
        #moving average filter
        newList = []
        if self.Order > len(timeDomainSignal):  #not enough data to fill buffer
            return newList                      #empty list handled in parent

        averageValueBuffer = 0

        for i in range (0,len(timeDomainSignal)):

            averageValueBuffer+=timeDomainSignal[i]

            if i >= self.Order:
                newList.append(averageValueBuffer / self.Order)
                # remove the oldest value from the filter
                averageValueBuffer -= timeDomainSignal[i - self.Order]

        #filteredArray = np.array(newList)

        return newList

    def CheckParameters(self):

        for f in self.CutoffFrequency:
            freq=f*2

            if freq >= self.SamplingFrequency:
                print("Cut off frequencies must be less than Fs/2")
                return False
            #there are two values in the list, we don't care about the second unless this is a bandpass/stop filter
            if self.Type.find('band')==-1:
                break;

        return True
    """
    apply a butterworth filter
    this can be low/high/band pass or band stop
    """
    def butterworthFilter(self,timeDomainSignal):

        filtered_data = []
        if self.CheckParameters() == False:
            return filtered_data

        if self.OutputType == 'sos':
            #bandpass/stop filters pass two critical frequencies
            if self.Type.find('band') != -1:
                CriticalFrequency = self.CutoffFrequency

            else:
                CriticalFrequency = self.CutoffFrequency[0]

            sos = signal.butter(self.Order, Wn=CriticalFrequency, btype=self.Type, fs=self.SamplingFrequency,output=self.OutputType)
            filtered_data = signal.sosfilt(sos, timeDomainSignal)

        elif self.OutputType == 'ba':
            b, a = signal.butter(self.Order, Wn=self.CutoffFrequency, btype=self.Type,fs=self.SamplingFrequency)  # too many values to unpack is full of shit
            zi = signal.lfilter_zi(b, a)
            filtered_data = signal.filtfilt(b, a, timeDomainSignal)

        else:
            print("Invalid Filter Type" + self.OutputType)  #empty list handled in parent

        return filtered_data
    """
    decides which filter to use then returns an array with filtered data
    """
    def applyFilter(self,timeDomainSignal):

        filtered_data = []

        filtered_data = self.butterworthFilter(timeDomainSignal)

        return filtered_data


