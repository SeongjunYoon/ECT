import usb_controller
import ect_controller
import graph_wizard
from fem import fem_solver
from inverse_solver import inverse_solver
import viewer
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import QInputDialog
from debugger.debug_raw_signals_widget import Subwindow_Debug_Raw_Signals
from debugger.debug_fem_widget import Subwindow_Debug_FEM



class MainWizard():
    def __init__(self, parent):
        # Get parent instance
        self.mainWindow = parent
        # Graph setting
        self.setGraph()
        # Set signal connection
        self.setSignalConnect()
        # Disable ECT measure button at first
        self.mainWindow.btnECTStart.setEnabled(False)
        self.mainWindow.menuDebugMode.setEnabled(False)
        # Import fem_data solver
        self.import_fem_solver()
        #
        self.measure_status = 'Init'
        self.mainWindow.btnECTStart.setText('Measure')
        self.mainWindow.labelMeasStatus.setText('Connect to Device')

    def setSignalConnect(self):
        self.mainWindow.btnECTStart.clicked.connect(self.btnECTStartClicked)
        self.mainWindow.actUSBConnect.triggered.connect(self.actUSBConnectTriggered)
        self.mainWindow.actVdet.triggered.connect(self.actVdetTriggered)
        self.mainWindow.actDemod.triggered.connect(self.actDemodTriggered)
        self.mainWindow.actLPF1.triggered.connect(self.actLPF1Triggered)
        self.mainWindow.actDeci1.triggered.connect(self.actDeci1Triggered)
        self.mainWindow.actLPF2.triggered.connect(self.actLPF2Triggered)
        self.mainWindow.actDeci2.triggered.connect(self.actDeci2Triggered)
        self.mainWindow.actLPF3.triggered.connect(self.actLPF3Triggered)
        self.mainWindow.actAvgAmp.triggered.connect(self.actAvgAmpTriggered)
        self.mainWindow.actDebugFEM.triggered.connect(self.actDebugFEMTriggered)

    def setGraph(self):
        # Create graphWizard instance
        self.mainGraphWizard = graph_wizard.GraphWizard()
        # Connect plot widget to cMapGraphLayout
        self.mainWindow.cMapGraphLayout.addWidget(self.mainGraphWizard.plot, 1, 0, 1, 2)
        labelStyle = {'color': 'k', 'font-size': '12pt', 'font-weight': 'bold', 'font-family': 'Arial'}
        self.mainGraphWizard.plotAxisBottom.setLabel(text='Electrode Pair #', units=None, **labelStyle)
        self.mainGraphWizard.plotAxisLeft.setLabel(text='Capacitance', units='F', **labelStyle)
        # Create graph update timer
        self.setdataUpdateTimer()

    def setdataUpdateTimer(self):
        # Create graph update timer
        self.dataUpdateTimer = QtCore.QTimer()
        self.dataUpdateTimer.timeout.connect(self.dataUpdate)

    def dataUpdate(self):
        if self.ect_controller.dsp.halfArrayFlag:
            if self.mainWindow.cMapTabWidget.currentIndex() == 0:
                x_data = np.arange(self.ect_controller.dsp.dspHalfArray.size, dtype=np.uint16)
                y_data = self.ect_controller.dsp.dspHalfArray.reshape(self.ect_controller.dsp.dspHalfArray.size, )
                self.mainGraphWizard.curve.setData(x_data, y_data)
            elif self.mainWindow.cMapTabWidget.currentIndex() == 1:
                self.display_c_map_table()
        else:
            if self.mainWindow.cMapTabWidget.currentIndex() == 0:
                x_data = np.arange(self.ect_controller.dsp.dspFullArray.size, dtype=np.uint16)
                y_data = self.ect_controller.dsp.dspFullArray.reshape(self.ect_controller.dsp.dspFullArray.size, )
                self.mainGraphWizard.curve.setData(x_data, y_data)

    def display_c_map_table(self):
        c_string = '{:09.5f} fF'
        shift = 1000000000000000.0
        shift_array = shift * self.ect_controller.dsp.dspHalfArray
        self.mainWindow.cmap12.setText(c_string.format(shift_array[0]))
        self.mainWindow.cmap13.setText(c_string.format(shift_array[1]))
        self.mainWindow.cmap14.setText(c_string.format(shift_array[2]))
        self.mainWindow.cmap15.setText(c_string.format(shift_array[3]))
        self.mainWindow.cmap16.setText(c_string.format(shift_array[4]))
        self.mainWindow.cmap17.setText(c_string.format(shift_array[5]))
        self.mainWindow.cmap18.setText(c_string.format(shift_array[6]))
        #
        self.mainWindow.cmap23.setText(c_string.format(shift_array[7]))
        self.mainWindow.cmap24.setText(c_string.format(shift_array[8]))
        self.mainWindow.cmap25.setText(c_string.format(shift_array[9]))
        self.mainWindow.cmap26.setText(c_string.format(shift_array[10]))
        self.mainWindow.cmap27.setText(c_string.format(shift_array[11]))
        self.mainWindow.cmap28.setText(c_string.format(shift_array[12]))
        #
        self.mainWindow.cmap34.setText(c_string.format(shift_array[13]))
        self.mainWindow.cmap35.setText(c_string.format(shift_array[14]))
        self.mainWindow.cmap36.setText(c_string.format(shift_array[15]))
        self.mainWindow.cmap37.setText(c_string.format(shift_array[16]))
        self.mainWindow.cmap38.setText(c_string.format(shift_array[17]))
        #
        self.mainWindow.cmap45.setText(c_string.format(shift_array[18]))
        self.mainWindow.cmap46.setText(c_string.format(shift_array[19]))
        self.mainWindow.cmap47.setText(c_string.format(shift_array[20]))
        self.mainWindow.cmap48.setText(c_string.format(shift_array[21]))
        #
        self.mainWindow.cmap56.setText(c_string.format(shift_array[22]))
        self.mainWindow.cmap57.setText(c_string.format(shift_array[23]))
        self.mainWindow.cmap58.setText(c_string.format(shift_array[24]))
        #
        self.mainWindow.cmap67.setText(c_string.format(shift_array[25]))
        self.mainWindow.cmap68.setText(c_string.format(shift_array[26]))
        #
        self.mainWindow.cmap78.setText(c_string.format(shift_array[27]))

    def start_data_update(self):
        # Set image frame per sec (fps)
        fps = 30
        self.dataUpdateTimer.start(int(1000 / fps))

    def stop_data_update(self):
        self.dataUpdateTimer.stop()

    def btnECTStartClicked(self):
        if self.mainWindow.btnECTStart.isChecked():
            if self.measure_status == 'Init':
                text, calibrate = QInputDialog.getText(self.mainWindow, 'Calibration', 'New Calibration?')
                if calibrate:
                    self.measure_status = 'Cmin'
                    self.mainWindow.btnECTStart.toggle()
                    self.mainWindow.labelMeasStatus.setText('[Prepare to measure Cmin]')
                    self.mainWindow.btnECTStart.setText('Start Measure Cmin')
                else:
                    self.measure_status = 'Imaging'
                    self.mainWindow.btnECTStart.toggle()
                    self.ect_controller.dsp.dspArrayAvgBufferEmpty = True
                    self.viewer = viewer.Viewer(self, self.mainWindow, self.ect_controller, self.fem)
                    self.viewer.show()
            elif self.measure_status == 'Cmin':
                self.mainWindow.labelMeasStatus.setText('[Measuring Cmin...]')
                self.mainWindow.btnECTStart.setText('Stop Measure Cmin')
                self.ect_controller.dsp.dspArrayAvgBufferEmpty = True
                self.ect_controller.measureStart()
                self.start_data_update()
            elif self.measure_status == 'Cmax':
                self.mainWindow.labelMeasStatus.setText('[Measuring Cmax...]')
                self.mainWindow.btnECTStart.setText('Stop Measure Cmax')
                self.ect_controller.dsp.dspArrayAvgBufferEmpty = True
                self.ect_controller.measureStart()
                self.start_data_update()
            elif self.measure_status == 'Ready':
                self.measure_status = 'Imaging'
                self.mainWindow.btnECTStart.toggle()
                self.ect_controller.dsp.dspArrayAvgBufferEmpty = True
                self.viewer = viewer.Viewer(self, self.mainWindow, self.ect_controller, self.fem)
                self.viewer.show()
            else:
                pass
        else:
            if self.measure_status == 'Cmin':
                self.measure_status = 'Cmax'
                self.ect_controller.measureStop()
                self.stop_data_update()
                np.savetxt(self.fem.measured_dir + '/Online_C_min.txt', self.ect_controller.dsp.dspArrayAvgBuffer)
                self.mainWindow.labelMeasStatus.setText('[Prepare to measure Cmax]')
                self.mainWindow.btnECTStart.setText('Start Measure Cmax')
            elif self.measure_status == 'Cmax':
                self.measure_status = 'Ready'
                self.ect_controller.measureStop()
                self.stop_data_update()
                np.savetxt(self.fem.measured_dir + '/Online_C_max.txt', self.ect_controller.dsp.dspArrayAvgBuffer)
                self.mainWindow.labelMeasStatus.setText('[Press <Start Imaging> button to get ECT image]')
                self.mainWindow.btnECTStart.setText('Start Imaging')
            else:
                pass

    def actUSBConnectTriggered(self):
        print("Connecting to ECT hardware...")
        # Create USB controller instance
        self.usb = usb_controller.USB_Controller()
        # Create ECT controller instance
        self.ect_controller = ect_controller.Controller(self.usb)
        # SysTimer - Set system variables update timer
        self.timerSysVar = QtCore.QTimer()
        self.timerSysVar.timeout.connect(self.sysVarRegularUpdate)
        self.timerSysVar.start(1000)  # Update interval for sysVarRegularUpdate = 1000 msec
        # Update statusLayout Information
        contentADCSpeedGrade = '* ADC Speed Grade = {msps} MSPS'.format(msps=self.ect_controller.adc.adc_speed_grade)
        self.mainWindow.labelADCSpeedGrade.setText(contentADCSpeedGrade)
        contentADCSamplingFreq = '* Sampling Frequency = {freq:.4f} MHz'.format(freq=self.ect_controller.adc.samplingFreq/1000000)
        self.mainWindow.labelADCSamplingFreq.setText(contentADCSamplingFreq)
        # Enable ECT measure button
        self.mainWindow.btnECTStart.setEnabled(True)
        self.mainWindow.menuDebugMode.setEnabled(True)
        print("USB connection success")
        #
        self.mainWindow.labelMeasStatus.setText('Press <ECT Start> Button to start measurement')

    def sysVarRegularUpdate(self):
        if not self.usb.usb_lock:
            self.ect_controller.battery.update()
            self.ect_controller.usb_c.update()
            # Update information in MainWindow
            contentBatterySoC = '* Battery = {percent} % ({voltage} V)     * Charger = {mode}'.format(
                percent=self.ect_controller.battery.battery_soc,
                voltage=self.ect_controller.battery.battery_voltage,
                mode=self.ect_controller.battery.CHG_MODE)
            contentUSBStatus = '* USB Connection Status = {state}     * USB Power Mode = {mode}'.format(
                state=self.ect_controller.usb_c.attached_state,
                mode=self.ect_controller.usb_c.current_mode)
            self.mainWindow.labelBatterySoC.setText(contentBatterySoC)
            self.mainWindow.labelUSBStatus.setText(contentUSBStatus)
        else:
            pass

    def show_debug_raw_signals(self, debug_mode, filter_output=None):
        # Hide main-window
        self.mainWindow.hide()
        # ECT hardware - Stop measurement & graph update for safety
        self.ect_controller.measureStop()
        self.stop_data_update()
        # Enter to debug mode - Raw signals
        print("Entering to debug mode - Raw signals")
        self.debugRawSignalsSubWindow = Subwindow_Debug_Raw_Signals(self.mainWindow, self.ect_controller, debug_mode, filter_output)
        self.debugRawSignalsSubWindow.show()

    def actVdetTriggered(self):
        self.show_debug_raw_signals(debug_mode='Vdet')

    """
    # Digital filter design instruction
    # Demodulation signal = 0x00, LPF1 = 0x01
    # Decimation1 = 0x02, LPF2 = 0x03
    # Decimation2 = 0x04, LPF3 = 0x05
    # Average amplitude = 0x06
    """
    def actDemodTriggered(self):
        self.show_debug_raw_signals(debug_mode='Filter', filter_output=0x00)

    def actLPF1Triggered(self):
        self.show_debug_raw_signals(debug_mode='Filter', filter_output=0x01)

    def actDeci1Triggered(self):
        self.show_debug_raw_signals(debug_mode='Filter', filter_output=0x02)

    def actLPF2Triggered(self):
        self.show_debug_raw_signals(debug_mode='Filter', filter_output=0x03)

    def actDeci2Triggered(self):
        self.show_debug_raw_signals(debug_mode='Filter', filter_output=0x04)

    def actLPF3Triggered(self):
        self.show_debug_raw_signals(debug_mode='Filter', filter_output=0x05)

    def actAvgAmpTriggered(self):
        self.show_debug_raw_signals(debug_mode='Filter', filter_output=0x06)

    def actDebugFEMTriggered(self):
        # Hide main-window
        self.mainWindow.hide()
        # Enter to debug mode - FEM
        print("Entering to debug mode - FEM")
        self.debugFEMSubWindow = Subwindow_Debug_FEM(self.mainWindow, self.fem)
        self.debugFEMSubWindow.show()

    def import_fem_solver(self):
        self.fem = fem_solver.FEM_Solver(mesh_filename='ECT_mesh', subdomain=True, boundary=True)
        self.fem.set_number_of_electrodes(8)
        self.fem.set_exc_voltage(6.7)
        # Import image reconstructor
        self.inv_solver = inverse_solver.InverseSolver()
        # Get reference capacitances
        self.fem.get_Cmin('C_min.txt')
        self.fem.get_Cmax('C_max.txt')
        # Get sensitivity map
        self.fem.calculate_sensitivity_map()
