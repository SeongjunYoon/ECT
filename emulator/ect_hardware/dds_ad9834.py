import dev_instruction_set


class DDS:
    def __init__(self, ect_controller):
        self.ect_controller = ect_controller
        self.usb = self.ect_controller.usb
        self.packet = [dev_instruction_set.AD9834_DDS_INST, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    def send_request(self, instruction, data1=0x00, data2=0x00, data3=0x00, data4=0x00, data5=0x00):
        self.packet[1] = instruction
        self.packet[2] = data1
        self.packet[3] = data2
        self.packet[4] = data3
        self.packet[5] = data4
        self.packet[6] = data5
        self.usb.write(data=self.packet)

    def select_output_register(self, f_sel, p_sel):
        if (f_sel != 0) and (f_sel != 1):
            print('Invalid FSEL Register')
        else:
            if (p_sel != 0) and (p_sel != 1):
                print('Invalid PSEL Register')
            else:
                self.send_request(dev_instruction_set.AD9834_DDS_INST_SET_OUTPUT_REG, f_sel, p_sel)
                if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
                    print('Request fail !! -> select_output_register()')

    def set_frequency(self, freq_val, reg_val=0):
        if (freq_val < 0) or (freq_val > 35000000):
            print('Input Frequency = Out-of-Range')
        else:
            if (reg_val != 0) and (reg_val != 1):
                print('Invalid Target Register')
            else:
                # Split uint32_t freq_val to 4 x uint8_t data format
                self.f = bytearray(4)
                self.f[0] = (freq_val & 0xFF000000) >> 24
                self.f[1] = (freq_val & 0x00FF0000) >> 16
                self.f[2] = (freq_val & 0x0000FF00) >> 8
                self.f[3] = (freq_val & 0x000000FF)
                self.send_request(dev_instruction_set.AD9834_DDS_INST_SET_FREQ, self.f[0], self.f[1], self.f[2], self.f[3], reg_val)
                if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
                    print('Request fail !! -> set_frequency()')

    def set_phase(self, phase_val, reg_val):
        self.p = bytearray(2)
        self.p[0] = (phase_val & 0xFF00) >> 8
        self.p[1] = (phase_val & 0x00FF)
        self.send_request(dev_instruction_set.AD9834_DDS_INST_SET_PHASE, self.p[0], self.p[1], reg_val)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('Request fail !! -> set_phase()')

    def change_waveform(self, waveform):
        # waveform = 0x00 : Sinusoidal output
        # waveform = 0x01 : Triangular output
        self.send_request(dev_instruction_set.AD9834_DDS_INST_SET_WAVEFORM, waveform)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('Request fail !! -> change_waveform()')

    def sleep(self, sleep_mode):
        # AD9834 DDS Sleep Mode
        # sleep_mode = 0x00 : MCLK Enabled, DAC Active
        # sleep_mode = 0x01 : MCLK Enabled, DAC Power Down
        # sleep_mode = 0x02 : MCLK Disabled, DAC Active
        # sleep_mode = 0x03 : MCLK Disabled, DAC Power Down
        self.send_request(dev_instruction_set.AD9834_DDS_INST_SLEEP, sleep_mode)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('Request fail !! -> sleep()')

    def dds_init(self):
        default_freq = 1000000
        if self.ect_controller.adc.adc_speed_grade == 20:
            default_freq = 1000000
        elif self.ect_controller.adc.adc_speed_grade == 80:
            default_freq = 10000000
        else:
            print("Not valid ADC speed grade (during DDS init)")

        self.set_frequency(freq_val=default_freq, reg_val=0x00)
        self.set_phase(phase_val=0, reg_val=0x00)
        self.select_output_register(f_sel=0x00, p_sel=0x00)
        self.change_waveform(waveform=0x00)


