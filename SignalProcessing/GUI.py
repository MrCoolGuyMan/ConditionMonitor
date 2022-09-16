from MainHeader import *

class MAIN_PAGE():
    def __init__(self):
        self.toplevel = tk.Tk()
        self.toplevel.title("Frequency analysis")
        self.toplevel.geometry('600x500')

        self._xlabel_position = 0
        self._xwidget_position = 175
        self._y_spacing = 30
        self.yOffset = 0
        self.yOffsetButtons = 0
        self.SpinDial_width = 20
        self.Button_width = 30
        self.Font = Font(family='Helvetica',size=10)

        self.addSpinDials()
        self.addButtons()
        self.addRadioButtons()
        self.addTextBoxes()

    def addTextBoxes(self):
        #text box to display files to be analysed
        self.ListofFilesTextBox = tk.Text(self.toplevel)

        self.ListofFilesTextBox.place(x=self._xlabel_position, y=self._y_spacing * self.yOffset)

        return
    def addRadioButtons(self):
        # motor selection
        self.radio_var = tk.IntVar()
        value=0


        motorlist = MotorClass()
        for x in motorlist.ListOfCLasses:
            tk.Radiobutton(self.toplevel, text=x,value=value,variable = self.radio_var)\
                .place(x=self._xwidget_position*2, y=self._y_spacing * self.yOffsetButtons)

            value+=1
            self.yOffsetButtons += 1

    def addSpinDials(self):
        # spin button ranges
        FilterTypes = ['none','highpass', 'lowpass', 'bandpass', 'bandstop']
        FilerOrder = [1, 10]
        LowCuttoff = [1, 10000]
        UpperCuttoff = [1, 10000]
        SamplFreq = [4000, 10000]
        StartTime = [-1, 10000]
        EndTime = [1, 10000]
        ScaleTypes = ['linear','logarithmic','semilogx','semilogy','none']
        SpectrogramTypes = ['density','spectrum','none']
        MotorVoltage = [24, 48]
        CurrentSensorOffsetVoltage = [2.3, 2.7]

        # sets the filter order
        self.spinbox_Order = self.createSpinDial(self.toplevel, FilerOrder, "Filter Order")

        # sets the filter type
        self.spinbox_FilterType = self.createSpinDial(self.toplevel, FilterTypes, "Filter Type")

        # lower end cuttoff frequency
        self.spinbox_CutoffLow = self.createSpinDial(self.toplevel, LowCuttoff, "Cuttoff Frequency (Lower)")

        # upper end cutoff frequency
        self.spinbox_Cutoffhigh = self.createSpinDial(self.toplevel, UpperCuttoff, "Cuttoff Frequency (Upper)")

        # sets Sampling Frequency
        self.spinbox_SampleFreq = self.createSpinDial(self.toplevel, SamplFreq, "Sampling Frequency")

        # analysis start time
        self.spinbox_AnalysisStart = self.createSpinDial(self.toplevel, StartTime, "Window Start Time(s)")

        # analysis end time
        self.spinbox_AnalysisEnd = self.createSpinDial(self.toplevel, EndTime, "Window End Time(s)")

        # diagram Axis types
        self.spinbox_TimeScale = self.createSpinDial(self.toplevel, ScaleTypes, "Time Axis")
        self.spinbox_FFTScale = self.createSpinDial(self.toplevel, ScaleTypes, "FFT Axis")
        self.spinbox_PSDScale = self.createSpinDial(self.toplevel, ScaleTypes, "PSD Axis")
        self.spinbox_Spectrogram = self.createSpinDial(self.toplevel, SpectrogramTypes, "Spectrogram")

        #motor test parameters
        self.spinbox_MotorVoltage = self.createSpinDial(self.toplevel, MotorVoltage, "Motor Voltage",format='%.2f',increment=0.01)
        self.spinbox_CurrentSensorVoltage = self.createSpinDial(self.toplevel, CurrentSensorOffsetVoltage, "CS Offset Voltage",format='%.2f',increment=0.01)

        # set the default for analysis end time to the max value
        self.spinbox_AnalysisEnd.delete(0, "end")
        self.spinbox_AnalysisEnd.insert(0, EndTime[1])

        self.spinbox_Cutoffhigh.delete(0, "end")
        self.spinbox_Cutoffhigh.insert(0, UpperCuttoff[1])

        # set default current sensor offset votlage
        self.spinbox_CurrentSensorVoltage.delete(0, "end")
        self.spinbox_CurrentSensorVoltage.insert(0, 2.5)
    def createSpinDial(self,parent, RangeList: list,DialLabel='Unallocated',format:str=None,increment=1):

        #create a label
        tk.Label(parent, text=DialLabel).place(x=self._xlabel_position, y=self._y_spacing * self.yOffset)

        if len(RangeList) == 2: # specify a start and end point
            newSpinbox = tk.Spinbox(parent, from_=RangeList[0], to=RangeList[1], wrap=True,width=self.SpinDial_width,font=self.Font,format=format,increment=increment)
        else:   # specify an absolute list
            newSpinbox = tk.Spinbox(parent, values=RangeList, wrap=True,width=self.SpinDial_width, font=self.Font)

        newSpinbox.place(x=self._xwidget_position, y=self._y_spacing * self.yOffset)

        self.yOffset += 1

        return newSpinbox

    def addButtons(self):
        # button to call main analysis
        self.StartButton = self.createButton(self.toplevel, "Begin Analysis", self.AnalyseFiles)

        # select file to analyse
        self.FileSelectButton = self.createButton(self.toplevel, "Select File(s)", self.GetFileName)

        #Process encoder and current data
        self.EncoderSelectButton = self.createButton(self.toplevel, "Process Encoder/Current Data", self.ProcessEncoderCurrentData)

    def createButton(self,parent, Label, Callback):

        NewButton = tk.Button(parent, text=Label, command=Callback, width=self.Button_width, font=self.Font)

        NewButton.place(x=self._xwidget_position*2, y=self._y_spacing * self.yOffsetButtons)

        self.yOffsetButtons += 1

        return NewButton
    """
    Callback Functions
    """
    def GetFileName(self):

        filename = fd.askopenfilenames(filetypes=[("CSV Files",".csv")])

        #filename is a tuple
        for x in filename:
            self.ListofFilesTextBox.insert(tk.END, x+'\n')
        return
    def ConvertRadioButtonIntoMotorClass(self):
        which_button_is_selected = self.radio_var.get()
        Motor = MotorClass(MotorClass.ListOfCLasses[which_button_is_selected])
        Motor.Voltage = float(self.spinbox_MotorVoltage.get())
        return Motor
    def ProcessEncoderCurrentData(self):
        filenames = fd.askopenfilenames(filetypes=[("CSV Files", ".csv")])

        Motor = self.ConvertRadioButtonIntoMotorClass()

        currentSensorOffsetVoltage = float(self.spinbox_CurrentSensorVoltage.get())
        ACfile,SpeedFile,DCfile = ProcessRawSensorData(filenames=filenames,
                                                       currentSensorOffsetVoltage=currentSensorOffsetVoltage,
                                                       MotorType=Motor)

        if ACfile.find('Error') == -1:
            self.ListofFilesTextBox.insert(index=tk.END,chars=ACfile + '\n')

        if SpeedFile.find('Error') == -1:
            self.ListofFilesTextBox.insert(index=tk.END,chars=SpeedFile + '\n')

        if DCfile.find('Error') == -1:
            self.ListofFilesTextBox.insert(index=tk.END, chars=DCfile + '\n')
        return

    def AnalyseFiles(self):
        StartTime = timeit.default_timer()
                             # filter order
        Filter = FilterClass(order=int(self.spinbox_Order.get()),
                             # 'lowpass', 'highpass', 'bandpass', 'bandstop'
                             filterType=self.spinbox_FilterType.get() ,
                             # -3dB point(s) in Hz
                             CutoffFrequency=[float(self.spinbox_CutoffLow.get()),float(self.spinbox_Cutoffhigh.get())],
                             # Samping frequency in Hz
                             SamplingFrequency=float(self.spinbox_SampleFreq.get()),
                             # Restrict analysis to this time window (seconds) x < t < y
                             windowing=[float(self.spinbox_AnalysisStart.get()), float(self.spinbox_AnalysisEnd.get())],
                             # linear/logarithmic etc
                             PSDAxis=self.spinbox_PSDScale.get(),
                             # linear/logarithmic etc
                             FFTAxis=self.spinbox_FFTScale.get(),
                             # linear/logarithmic etc
                             TimeAxis=self.spinbox_TimeScale.get(),
                             #density or spectrum
                             SpectrogramAxis=self.spinbox_Spectrogram.get()
                             )

        #get the list of file names from the Text Box
        Text = self.ListofFilesTextBox.get('1.0', tk.END)

        Motor = self.ConvertRadioButtonIntoMotorClass()

        main(Text,Filter,Motor)

        plt.interactive(True)   #stop gui locking up
        plt.show()

        #benchmarking
        print(timeit.default_timer()- StartTime )

