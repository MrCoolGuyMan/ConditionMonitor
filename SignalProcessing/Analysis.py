from MainHeader import *

"""
reads in a csv file "filename" and creates some graphs
Time Domain signal
Spectrogram
Fast Fourier Transform (FFT)
Power Spectral Density (PSD)

thens applies a filter defined by "Parameters"
and repeates above
"""
def mainAnalysis(filename,FilterParameters:FilterClass=None,MotorType:MotorClass=None):

    #create default filter if none is provied
    if FilterParameters is None:
        FilterParameters=FilterClass()




    #setup subplots
    DefaultFFTDiagramTitles = ['Time Domain', 'FFT', 'Spectrogram', "PSD"]

    ax1 = [[]]              #holds filtered and unfiltered axes
                            #ax1[0][(un)filtered_index] = time Domain
    filterd_index = 1       #ax1[1][(un)filtered_index] = FFT
    unfiltered_index = 0    #ax1[2][(un)filtered_index] = spectrogram
                            #ax1[3][(un)filtered_index] = PSD

    numRows = countHowManyPlotsToMake(FilterParameters)
    numCols = 1
    if FilterParameters.Type.find('none') == -1:
        numCols = 2

    fig, ax1 = plt.subplots(nrows=numRows,ncols=numCols, constrained_layout=True)

    #find the file name; include some directory information for clarity
    index = findFinalSlashPosition(filename)
    index = findFinalSlashPosition(filename[0:index-1]) #parent folder
    index = findFinalSlashPosition(filename[0:index-1]) #parent's parent folder

    fig.suptitle(filename[index:], fontsize=16)

    #Read data from csv file
    timeDomainSignal = copyCSVfileToMem(filename)

    #FFT of unfiltered data
    FastFourierTransform(timeDomainSignal,FilterParameters,unfiltered_index,ax1,DefaultFFTDiagramTitles,MotorType)

    if FilterParameters.Type.find('none') == -1:
        #Filter and Graph
        filteredArray = FilterParameters.applyFilter(timeDomainSignal)

        #its possible for filteredArray to be empty if an error occurred
        if len(filteredArray) != 0:

            #FFT of filtered data
            Titles = []
            for x in DefaultFFTDiagramTitles:
                Titles.append("Filtered " + x)

            FastFourierTransform(filteredArray, FilterParameters,filterd_index,ax1,Titles,MotorType);

    return 0
def plotOutputs(diagrams):

    return
"""
computes real-FFT
then plots results + input data
"""
def FastFourierTransform(timeDomainSignal, Filter, index,
                         diagrams:list=None,
                         diagramTitles:list=None,
                         MotorType:MotorClass=None):

    if diagrams is None:
        return
    # time axis is evenly distributed based on length of timeDomainSignal and sampling frequency
    time_axis = create_time_axis(timeDomainSignal, Filter.SamplingFrequency)

    #sets a limit on start/end times for analysis
    window = ma.masked_inside(time_axis, Filter.Windowing[0], Filter.Windowing[1])

    # RealFFT does not compute negative frequencies and removes data points past aliasing limit
    FFT = scipyFFT.rfft(timeDomainSignal[window.mask])
    freq = scipyFFT.rfftfreq(int(len(timeDomainSignal[window.mask])))

    # find magnitude and normalise
    FFT = abs(FFT) / len(timeDomainSignal[window.mask])

    # Find Frequency values and multiply by SamplingFrequency to normalise result
    freq = freq * Filter.SamplingFrequency

    # create diagrams if required
    # awkward code section

    counter = 0
    numGraphs = countHowManyPlotsToMake(Filter)
    for i in range (0,4):
        # diagrams type is changed depending on which graphs are enabled
        # Python being a pain in the arse
        try:
            diagram = diagrams[counter][index]      # filtered and unfiltered data with multiple graphs
        except:
            try:
                if(numGraphs > 1):
                    diagram = diagrams[counter]         # filtered or unfiltered with multiple graphs
                else:
                    diagram = digrams[index]  # filter and unfiltered with single graph
            except:
                diagram = diagrams              # filtered or unfiltered with single graphs


        #time domain signal
        if i == 0:
            if Filter.TimeAxis.find('none') == -1:
                createPlot(diagram, "Time(s)","Amplitude", time_axis[window.mask], timeDomainSignal[window.mask], diagramTitles[0],scale_type=Filter.TimeAxis)
                counter+=1
        #Fourier Transform
        if i == 1:
            if Filter.FFTAxis.find('none') == -1:
                createPlot(diagram, "Frequency(Hz)","Amplitude", freq, FFT.real, diagramTitles[1],scale_type=Filter.FFTAxis,MotorType=MotorType)
                counter += 1
        # spectrogram
        if i == 2:
            if Filter.SpectrogramAxis.find('none') == -1:
                createSpectrogram(diagram, timeDomainSignal[window.mask], Filter.SamplingFrequency,diagramTitles[2],Filter.SpectrogramAxis)
                counter += 1
        # Power Spectral density
        if i == 3:
            if Filter.PSDAxis.find('none') == -1:
                createPSD(diagram, timeDomainSignal[window.mask], Filter.SamplingFrequency, diagramTitles[3],scale_type=Filter.PSDAxis,MotorType=MotorType)
                counter += 1

    return FFT, freq, time_axis

def countHowManyPlotsToMake(FilterParameters:FilterClass):
    #count how many plots are to be made

    numAxis = 0
    if FilterParameters.PSDAxis.find('none') == -1:
        numAxis += 1
    if FilterParameters.FFTAxis.find('none') == -1:
        numAxis += 1
    if FilterParameters.TimeAxis.find('none') == -1:
        numAxis += 1
    if FilterParameters.SpectrogramAxis.find('none') == -1:
        numAxis += 1

    return numAxis