import graph_wizard
from PyQt5.QtWidgets import QButtonGroup
from PyQt5 import QtCore


class Debug_RawSignals_Wizard():
    def __init__(self, debug_raw_signals_subwindow, ect_controller, debug_mode, filter_output):
        # Get ECT controller instances
        self.subwindow = debug_raw_signals_subwindow
        self.ect_controller = ect_controller
        # Set graph
        self.setGraph()
        # Set Radiobutton Group
        self.radioButtonGroup()
        # Currently selected electrode button flags
        self.excElecRbtnSelFlag = 0
        self.detElecRbtnSelFlag = 0
        # Set signal connection
        self.setSignalConnect()
        # Disable all electrode selection radio buttons at first
        self.enable_All_Electrode_Buttons(False)
        # Set initial channel selection
        self.currentChannelFlag = None
        self.RbtnChannels[0].toggle()
        # FFT flag
        self.fftOnFlag = False
        # Debug mode
        self.debug_mode = debug_mode
        self.filter_output = filter_output

    def setSignalConnect(self):
        self.subwindow.btnMeasure.clicked.connect(self.btnMeasureClicked)
        self.subwindow.btnFFT.clicked.connect(self.btnFFTClicked)
        self.subwindow.btnResetElectrodes.clicked.connect(self.btnResetElectrodesClicked)
        # RbtnExcElec1 - RbtnExcElec8
        self.RbtnExcElecs[0].toggled.connect(lambda: self.RbtnExcElecToggled(1))
        self.RbtnExcElecs[1].toggled.connect(lambda: self.RbtnExcElecToggled(2))
        self.RbtnExcElecs[2].toggled.connect(lambda: self.RbtnExcElecToggled(3))
        self.RbtnExcElecs[3].toggled.connect(lambda: self.RbtnExcElecToggled(4))
        self.RbtnExcElecs[4].toggled.connect(lambda: self.RbtnExcElecToggled(5))
        self.RbtnExcElecs[5].toggled.connect(lambda: self.RbtnExcElecToggled(6))
        self.RbtnExcElecs[6].toggled.connect(lambda: self.RbtnExcElecToggled(7))
        self.RbtnExcElecs[7].toggled.connect(lambda: self.RbtnExcElecToggled(8))
        # RbtnDetElec1 - RbtnDetElec8
        self.RbtnDetElecs[0].toggled.connect(lambda: self.RbtnDetElecToggled(1))
        self.RbtnDetElecs[1].toggled.connect(lambda: self.RbtnDetElecToggled(2))
        self.RbtnDetElecs[2].toggled.connect(lambda: self.RbtnDetElecToggled(3))
        self.RbtnDetElecs[3].toggled.connect(lambda: self.RbtnDetElecToggled(4))
        self.RbtnDetElecs[4].toggled.connect(lambda: self.RbtnDetElecToggled(5))
        self.RbtnDetElecs[5].toggled.connect(lambda: self.RbtnDetElecToggled(6))
        self.RbtnDetElecs[6].toggled.connect(lambda: self.RbtnDetElecToggled(7))
        self.RbtnDetElecs[7].toggled.connect(lambda: self.RbtnDetElecToggled(8))
        # RbtnChannels
        self.RbtnChannels[0].toggled.connect(lambda: self.RbtnChannelsToggled('A'))
        self.RbtnChannels[1].toggled.connect(lambda: self.RbtnChannelsToggled('B'))

    def radioButtonGroup(self):
        self.RbtnExcElecs = [self.subwindow.RbtnExcElec1, self.subwindow.RbtnExcElec2, self.subwindow.RbtnExcElec3,
                             self.subwindow.RbtnExcElec4,
                             self.subwindow.RbtnExcElec5, self.subwindow.RbtnExcElec6, self.subwindow.RbtnExcElec7,
                             self.subwindow.RbtnExcElec8]
        self.RbtnDetElecs = [self.subwindow.RbtnDetElec1, self.subwindow.RbtnDetElec2, self.subwindow.RbtnDetElec3,
                             self.subwindow.RbtnDetElec4,
                             self.subwindow.RbtnDetElec5, self.subwindow.RbtnDetElec6, self.subwindow.RbtnDetElec7,
                             self.subwindow.RbtnDetElec8]
        self.RbtnChannels = [self.subwindow.RbtnChannelA, self.subwindow.RbtnChannelB]
        self.excElecButtonGroup = QButtonGroup()
        self.detElecButtonGroup = QButtonGroup()
        self.channelButtonGroup = QButtonGroup()
        self.setButtonGroup(self.excElecButtonGroup, self.RbtnExcElecs)
        self.setButtonGroup(self.detElecButtonGroup, self.RbtnDetElecs)
        self.setButtonGroup(self.channelButtonGroup, self.RbtnChannels)

    def setButtonGroup(self, groupName, buttonList):
        for button in buttonList:
            groupName.addButton(button)

    def setGraph(self):
        # Create graphWizard instance
        self.graphWizard = graph_wizard.GraphWizard()
        # Connect plot widget to graphLayout
        self.subwindow.graphLayout.addWidget(self.graphWizard.plot, 1, 0, 1, 2)
        labelStyle = {'color': 'k', 'font-size': '14pt', 'font-weight': 'bold', 'font-family': 'Arial'}
        self.graphWizard.plotAxisBottom.setLabel(text='Time', units=None, **labelStyle)
        self.graphWizard.curve.setSymbol(None)
        # Create graph update timer
        self.timerGraphUpdate = QtCore.QTimer()
        self.timerGraphUpdate.timeout.connect(self.graphUpdate)

    def graphUpdate(self):
        if self.fftOnFlag:
            self.ect_controller.dsp.fft()
            self.graphWizard.curve.setData(self.ect_controller.dsp.fft_rfreq, self.ect_controller.dsp.fft_ry)
        else:
            self.graphWizard.curve.setData(self.ect_controller.dsp.discreteTimeRef, self.ect_controller.dsp.rawSigBuffer)

    def graphUpdate_start(self):
        # Set image frame per sec (fps)
        fps = 60
        self.timerGraphUpdate.start(int(1000 / fps))

    def graphUpdate_stop(self):
        self.timerGraphUpdate.stop()

    def btnMeasureClicked(self):
        if self.subwindow.btnMeasure.isChecked():
            # ECT hardware - Start measurement
            self.ect_controller.measureStart(debug_mode=self.debug_mode, debug_filter_output=self.filter_output)
            # Toggle btnMeasure content
            self.subwindow.btnMeasure.setText('Stop')
            # Connect data-containing instance & start graph update timer
            self.graphUpdate_start()
            # Enable electrode selection radio buttons
            self.enable_All_Electrode_Buttons(True)
        else:
            # Disable electrode selection radio buttons
            self.enable_All_Electrode_Buttons(False)
            # ECT hardware - Stop measurement
            self.ect_controller.measureStop()
            # Pyqt5 - Toggle btnMeasure content
            self.subwindow.btnMeasure.setText('Start')
            # GraphWizard - Graph update timer stop
            self.graphUpdate_stop()

    def btnFFTClicked(self):
        if self.subwindow.btnFFT.isChecked():
            self.fftOnFlag = True
            self.subwindow.btnFFT.setText('FFT OFF')
            self.graphWizard.predefineRange(enable=True)
            self.graphWizard.plotAxisBottom.setLabel(text='Frequency', units='Hz')
            self.graphWizard.plotAxisBottom.enableAutoSIPrefix(enable=True)
            plotAxisRangeLimit = {'yMin': 0}
            self.graphWizard.plot.setLimits(**plotAxisRangeLimit)
        else:
            self.fftOnFlag = False
            self.subwindow.btnFFT.setText('FFT ON')
            self.graphWizard.predefineRange(enable=False)
            self.graphWizard.plotAxisBottom.setLabel(text='Time', units='')
            self.graphWizard.plotAxisBottom.enableAutoSIPrefix(enable=False)
            plotAxisRangeLimit = {'yMin': None}
            self.graphWizard.plot.setLimits(**plotAxisRangeLimit)

    def reset_Electrodes(self):
        # Disable exclusive selection of the electrode radio button groups
        self.excElecButtonGroup.setExclusive(False)
        self.detElecButtonGroup.setExclusive(False)
        # Uncheck all RbtnExcElecs
        for RbtnExcElec in self.RbtnExcElecs:
            RbtnExcElec.setChecked(False)
        # Uncheck all RbtnDetElecs
        for RbtnDetElec in self.RbtnDetElecs:
            RbtnDetElec.setChecked(False)
        # Enable exclusive selection of the electrode radio button groups
        self.excElecButtonGroup.setExclusive(True)
        self.detElecButtonGroup.setExclusive(True)
        # Reset electrode radio button selection flags
        self.excElecRbtnSelFlag = 0
        self.detElecRbtnSelFlag = 0

    def set_Selectable_Electrode_Button(self, Exc_or_Det, target_elec, enable):
        if Exc_or_Det == ('Exc' or 'exc' or 'EXC'):
            self.RbtnExcElecs[target_elec - 1].setEnabled(enable)
        elif Exc_or_Det == ('Det' or 'det' or 'DET'):
            self.RbtnDetElecs[target_elec - 1].setEnabled(enable)
        else:
            pass

    def enable_All_Electrode_Buttons(self, enable):
        if not enable:
            self.reset_Electrodes()
        # Enable/disable all RbtnExcElec
        for RbtnExcElec in self.RbtnExcElecs:
            RbtnExcElec.setEnabled(enable)
        # Enable/disable all RbtnDetElec
        for RbtnDetElec in self.RbtnDetElecs:
            RbtnDetElec.setEnabled(enable)

    def btnResetElectrodesClicked(self):
        self.ect_controller.tsw.tsw_reset()
        self.reset_Electrodes()

    def RbtnExcElecToggled(self, exc_electrode):
        # Ignore the toggle signal came from the old electrode radio button
        if exc_electrode == self.excElecRbtnSelFlag:
            # Enable the currently disabled detection part radio button
            self.set_Selectable_Electrode_Button('Det', self.excElecRbtnSelFlag, True)
        else:
            # Change the excitation electrode
            self.ect_controller.tsw.exc_tsw_control(exc_electrode)
            self.excElecRbtnSelFlag = self.ect_controller.tsw.tsw_status[0]
            # And disable the corresponding detection part radio button
            self.set_Selectable_Electrode_Button('Det', exc_electrode, False)

    def RbtnDetElecToggled(self, det_electrode):
        # Ignore the toggle signal came from the old radio button
        if det_electrode == self.detElecRbtnSelFlag:
            # Enable the currently disabled detection part radio button
            self.set_Selectable_Electrode_Button('Exc', self.detElecRbtnSelFlag, True)
        else:
            # Change the detection electrode
            self.ect_controller.tsw.det_tsw_control(det_electrode)
            self.detElecRbtnSelFlag = self.ect_controller.tsw.tsw_status[1]
            # And disable the corresponding detection part radio button
            self.set_Selectable_Electrode_Button('Exc', det_electrode, False)

    def RbtnChannelsToggled(self, channel):
        # Ignore the toggle signal came from the old radio button
        if not channel == self.currentChannelFlag:
            # Change ADC output channel
            self.ect_controller.adc.set_data_capture_channel(channel)
            self.currentChannelFlag = channel
