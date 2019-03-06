from ect_hardware import tsw, pmic_adp5350, adc_ad9269, dds_ad9834, \
    usb_c_hd3ss3220
import afe_manager
import dsp_controller
from PyQt5 import QtCore



class Controller():
    def __init__(self, usb):
        self.usb = usb
        # Create instances
        self.battery = pmic_adp5350.Battery(self)
        self.usb_c = usb_c_hd3ss3220.USB_TypeC_IC(self)
        self.dds = dds_ad9834.DDS(self)
        self.adc = adc_ad9269.ADC(self)
        self.tsw = tsw.T_SW(self)
        self.afe = afe_manager.AFE(self)
        self.dsp = dsp_controller.DSP(self)
        # Create ADC data update timer
        self.timerDataUpdate = QtCore.QTimer()
        self.timerDataUpdate.timeout.connect(self.dataUpdate)
        # Data update interval variable
        self.update_interval = None
        # Debug mode
        self.debug_mode = None

    def measureStart(self, debug_mode=None, debug_filter_output=None):
        self.debug_mode = debug_mode
        if self.debug_mode == None:
            pass
        elif self.debug_mode == 'Vdet':
            self.dsp.raw_signal_monitor_init()
        elif self.debug_mode == 'Filter':
            self.dsp.digital_filter_design_init(debug_filter_output)
        else:
            print('Invalid measure mode')
        # ECT Hardware - AFE part initialization
        self.afe.afe_init()
        self.dds.dds_init()
        self.tsw.exc_tsw_control(self.tsw.tsw_status[0])
        self.tsw.det_tsw_control(self.tsw.tsw_status[1])
        self.adc.adc_init()
        self.dsp.mcu_dsp_init()
        # Start ADC data update timer
        self.update_interval = 30 # msec
        self.timerDataUpdate.start(self.update_interval)

    def measureStop(self):
        # Stop ADC data update timer
        self.timerDataUpdate.stop()
        # ECT Hardware - AFE part sleep
        self.afe.afe_deinit()

    def dataUpdate(self):
        if not self.usb.usb_lock:
            if self.debug_mode == None:
                self.dsp.get_ECT_array()
            elif self.debug_mode == 'Vdet':
                self.dsp.get_adc_output()
            elif self.debug_mode == 'Filter':
                self.dsp.lock_in_amplifier()
            else:
                print('Debug mode error')

