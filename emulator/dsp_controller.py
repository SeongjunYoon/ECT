"""
[ECT Hardware Design]

=> C/V converter output (Vo) = (Cx / Cf) * V_exc
where, Cx = measured capacitance, Cf = feedback capacitance (27 pF), V_exc = excitation signal amplitude
Cx = Cf * Vo / V_exc

=> V_detection signal path:
V_detection -> C/V converter -> Amplifier 1 (g1 = x9.879 V/V) -> Amplifier 2 (g2 = x9.879 V/V) -> ADC driver (g3 = x0.9 V/V)
Therefore, C/V converter output (Vo) = V_ADC_amplitude / (g1 * g2 * g3)

=> Cx = Cf * (V_ADC_amplitude / (g1 * g2 * g3)) / V_exc
"""

import dev_instruction_set
import numpy as np
import struct


C_feedback = 100.0          # Feedback capacitance of CV converter, Unit: pF
G1 = 9.879                  # Gain of amplifier 1
G2 = 1                      # Gain of amplifier 2
G3 = 0.9                    # Gain of ADC driver
V_excitation = 6.7          # Excitation voltage gain, Unit: V


Cf = np.float64(C_feedback * 0.000000000001)
G_t = np.float64(G1 * G2 * G3)
Vexc = np.float64(V_excitation)
Vadc_coeff = np.float64(0.993 * 2 / 65536)


class DSP:
    def __init__(self, ect_controller):
        self.ect_controller = ect_controller
        self.adc = self.ect_controller.adc
        self.usb = self.ect_controller.usb
        self.packet = [dev_instruction_set.DSP_INST, 0x00, 0x00, 0x00]
        self.float64_packet = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.dwt_cnt_packet = [0x00, 0x00, 0x00, 0x00]
        #
        self.electrodeNum = 8
        self.dspFullArraySize = (self.electrodeNum)*(self.electrodeNum - 1)
        self.dspFullArray = np.zeros(self.dspFullArraySize, dtype=np.float64).reshape((self.electrodeNum, self.electrodeNum - 1))
        self.dspHalfArraySize = int(self.dspFullArraySize / 2)
        self.dspHalfArray = np.zeros(self.dspHalfArraySize, dtype=np.float64)
        self.halfArrayFlag = True
        #
        self.dspArrayAvgBuffer = None
        self.dspArrayAvgBufferEmpty = True
        #
        self.dspBlockSize = int(4096/2)
        # Send DSP initialization request to MCU
        self.mcu_dsp_init()

    def send_request(self, dsp_instruction, data1=0x00, data2=0x00):
        # Packet Structure
        # packet[0] = Global instruction indicator
        # packet[1] = DSP instruction indicator
        # packet[2] = DSP send data
        self.packet[1] = dsp_instruction
        self.packet[2] = data1
        self.packet[3] = data2
        self.usb.write(data=self.packet)

    def mcu_dsp_init(self):
        # Set digital filter instances in MCU
        self.send_request(dev_instruction_set.DSP_INST_INIT)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('Request fail !! -> init()')
        # Update Reference Signal Data of MCU
        self.update_Vref_data()

    def convert2Analog(self, f64_data):
        return (Cf * ((Vadc_coeff * f64_data) / G_t) / Vexc)

    def get_ECT_array(self):
        if self.halfArrayFlag:
            self.send_request(dev_instruction_set.DSP_INST_GET_CAP_ARRAY_HALF)
            for (num, data) in enumerate(self.usb.read(size=8 * self.dspHalfArraySize)):  # 8 = bytesize of float64_t
                i = num % 8
                self.float64_packet[i] = data
                if i == 7:
                    (f64_data,) = struct.unpack('d', struct.pack('<8B', self.float64_packet[0], self.float64_packet[1],
                                                                 self.float64_packet[2], self.float64_packet[3],
                                                                 self.float64_packet[4],
                                                                 self.float64_packet[5], self.float64_packet[6],
                                                                 self.float64_packet[7]))
                    self.dspHalfArray[num // 8] = self.convert2Analog(f64_data)
                    if self.dspArrayAvgBufferEmpty:
                        self.dspArrayAvgBuffer = self.dspHalfArray
                    else:
                        self.dspArrayAvgBuffer = (self.dspArrayAvgBuffer + self.dspHalfArray)/2
                    # np.savetxt('Measured_C_plastic_side.txt',self.dspHalfArray[:],delimiter=',')
        else:
            self.send_request(dev_instruction_set.DSP_INST_GET_CAP_ARRAY_FULL)
            for (num, data) in enumerate(self.usb.read(size=8 * self.dspFullArraySize)):  # 8 = bytesize of float64_t
                i = num % 8
                self.float64_packet[i] = data
                if i == 7:
                    (f64_data,) = struct.unpack('d', struct.pack('<8B', self.float64_packet[0], self.float64_packet[1],
                                                                 self.float64_packet[2], self.float64_packet[3],
                                                                 self.float64_packet[4],
                                                                 self.float64_packet[5], self.float64_packet[6],
                                                                 self.float64_packet[7]))
                    index = num // 8
                    self.dspFullArray[index // (self.electrodeNum - 1)][index % (self.electrodeNum - 1)] = self.convert2Analog(f64_data)
        #self.get_dwt_cnt()

    def get_adc_output(self):
        self.send_request(dev_instruction_set.DSP_INST_GET_ADC_DATA, self.adc.data_capture_channel)
        for (num, data) in enumerate(self.usb.read(size=8 * self.dspBlockSize)):
            i = num % 8
            self.float64_packet[i] = data
            if i == 7:
                (f64_data,) = struct.unpack('d', struct.pack('<8B', self.float64_packet[0], self.float64_packet[1],
                                                             self.float64_packet[2], self.float64_packet[3],
                                                             self.float64_packet[4],
                                                             self.float64_packet[5], self.float64_packet[6],
                                                             self.float64_packet[7]))
                self.rawSigBuffer[num // 8] = f64_data
        #self.get_dwt_cnt()

    def fft(self):
        self.fft_ry = np.abs(np.fft.fft(self.rawSigBuffer))
        self.fft_rfreq = self.adc.samplingFreq * np.fft.fftfreq(len(self.discreteTimeRef), np.diff(self.discreteTimeRef)[0])

    def update_Vref_data(self):
        self.send_request(dev_instruction_set.DSP_INST_UPDATE_VREF_DATA)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('Request fail !! -> update_Vref_data()')

    def usb_read_f64_data(self):
        for (num, data) in enumerate(self.usb.read(size=8)):
            i = num % 8
            self.float64_packet[i] = data
            if i == 7:
                (f64_data,) = struct.unpack('d', struct.pack('<8B', self.float64_packet[0], self.float64_packet[1],
                                                         self.float64_packet[2], self.float64_packet[3],
                                                         self.float64_packet[4], self.float64_packet[5],
                                                         self.float64_packet[6], self.float64_packet[7]))
                return f64_data

    def get_dwt_cnt(self):
        self.send_request(dev_instruction_set.DSP_INST_GET_DWT_CNT)
        for (num, data) in enumerate(self.usb.read(size=4)):
            i = num % 4
            self.dwt_cnt_packet[i] = data
            if i == 3:
                (dwt_cnt,) = struct.unpack('I', struct.pack('<4B', self.dwt_cnt_packet[0], self.dwt_cnt_packet[1],
                                                            self.dwt_cnt_packet[2], self.dwt_cnt_packet[3]))
                print(dwt_cnt)


    # For digital filter design
    def lock_in_amplifier(self):
        self.send_request(dev_instruction_set.DSP_INST_LIA, self.adc.data_capture_channel, self.debug_filter_output)
        for (num, data) in enumerate(self.usb.read(size=8*self.dspRawSigBufferSize)):
            i = num % 8
            self.float64_packet[i] = data
            if i == 7:
                (f64_data,) = struct.unpack('d', struct.pack('<8B', self.float64_packet[0], self.float64_packet[1],
                                                          self.float64_packet[2], self.float64_packet[3], self.float64_packet[4],
                                                          self.float64_packet[5], self.float64_packet[6], self.float64_packet[7]))
                self.rawSigBuffer[num // 8] = f64_data
        #np.savetxt('demod_ex1_fexc10M_fs78.1250M.txt',self.rawSigBuffer,delimiter=',')
        #np.savetxt('demod_ex1_fexc10M_fs78.1250Mto7.8125M_after_Deci1.txt',self.rawSigBuffer,delimiter=',')
        #np.savetxt('demod_ex1_fexc10M_fs7.8125Mto781.25k_after_Deci2.txt.txt',self.rawSigBuffer,delimiter=',')
        print(self.rawSigBuffer)

    def raw_signal_monitor_init(self):
        # Define raw ADC data buffer array
        self.rawSigBuffer = np.zeros(self.dspBlockSize, dtype=np.float64)
        # Define time-scale data
        self.discreteTimeRef = np.arange(self.dspBlockSize, dtype=np.uint16)

    def digital_filter_design_init(self, debug_filter_output):
        self.debug_filter_output = debug_filter_output
        # Define digital filter block sizes
        self.dspDeciBlockSize1 = (int)(((self.dspBlockSize - 1) / 10) + 1)
        self.dspDeciBlockSize2 = (int)(((self.dspDeciBlockSize1 - 1) / 10) + 1)
        # Digital filter design instruction
        # Demodulation signal = 0x00, LPF1 = 0x01
        # Decimation1 = 0x02, LPF2 = 0x03
        # Decimation2 = 0x04, LPF3 = 0x05
        # Average amplitude = 0x06
        if (self.debug_filter_output == 0x00) or (self.debug_filter_output == 0x01):
            self.dspRawSigBufferSize = self.dspBlockSize
        elif (self.debug_filter_output == 0x02) or (self.debug_filter_output == 0x03):
            self.dspRawSigBufferSize = self.dspDeciBlockSize1
        elif (self.debug_filter_output == 0x04) or (self.debug_filter_output == 0x05):
            self.dspRawSigBufferSize = self.dspDeciBlockSize2
        else:
            self.dspRawSigBufferSize = 1
        # Define data buffer array for DSP processing
        self.rawSigBuffer = np.zeros(self.dspRawSigBufferSize, dtype=np.float64)
        self.discreteTimeRef = np.arange(self.dspRawSigBufferSize, dtype=np.uint16)
