"""
[AD9269 16-Bit 20-80 MSPS 2-Channel ADC]

=> Analog input voltage (V) to ADC = -V_REF + (V_REF * 2 / 65536) * ADC_Output_Value
where, V_REF = 0.993 V

=> Analog input amplitude to ADC = (V_REF * 2 / 65536) * ADC_Output_Value
"""
import dev_instruction_set


class ADC:
    def __init__(self, ect_controller):
        self.ect_controller = ect_controller
        self.usb = self.ect_controller.usb
        self.packet = [dev_instruction_set.AD9269_INST, 0x00, 0x00, 0x00, 0x00]
        # Get Initial ADC Sampling Information
        self.check_adc_communication()
        # Set Initial Variables
        self.adc_speed_grade = 0
        self.adc_master_clock = 0
        self.adc_default_clk_div_20MSPS = 5
        self.adc_default_clk_div_80MSPS = 2
        self.get_speed_grade()
        # Get ADC Sampling Frequency Information
        if self.adc_speed_grade == 20:
            self.samplingFreq = self.adc_master_clock / self.adc_default_clk_div_20MSPS
        elif self.adc_speed_grade == 80:
            self.samplingFreq = self.adc_master_clock / self.adc_default_clk_div_80MSPS
        else:
            print('Error During ADC Sampling Frequency Initialization !!')
        # ADC Data Output Variables
        self.data_capture_channel = dev_instruction_set.AD9269_CHANNEL_A

    def send_request(self, adc_instruction, channel=0x01, target_reg=0x01, reg_val=0x00):
        # Packet Structure
        # packet[0] = Global instruction indicator
        # packet[1] = ADC instruction indicator
        # packet[2] = ADC target channel (A or B)
        # packet[3] = ADC target register address
        # packet[4] = ADC new register value
        self.packet[1] = adc_instruction
        self.packet[2] = channel
        self.packet[3] = target_reg
        self.packet[4] = reg_val
        self.usb.write(data=self.packet)

    def check_adc_communication(self):
        self.send_request(dev_instruction_set.AD9269_INST_CHECK_COMM)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('AD9269 ADC Communication Fail !!')

    def get_speed_grade(self):
        self.send_request(dev_instruction_set.AD9269_INST_GET_SPEED_GRADE)
        self.adc_speed_grade = self.usb.read(size=1)[0]
        # Update ADC Master Clock Information
        if self.adc_speed_grade == 20:  # Unit = MSPS
            self.adc_master_clock = 100000000   # Master Clock = 100MHz for 20 MSPS ADC
        elif self.adc_speed_grade == 80:    # Unit = MSPS
            self.adc_master_clock = 156250000   # Master Clock = 156.25MHz for 80 MSPS ADC
        else:
            print('Cannot Identify ADC Master Clock Information !!')

    # 'ADC Sampling Clock = {} MHz'.format(self.adc_master_clock / clk_div)
    def clock_divider(self, clk_div):
        if (clk_div) < 1 or (clk_div) > 6:
            print('Invalid clk_div !! (clk_div out of range')
        elif (self.adc_master_clock / clk_div) > (self.adc_speed_grade * 1000000):
            print('Invalid clk_div !! (Sampling Frequency > Speed Grade) !!!')
        else:
            self.send_request(dev_instruction_set.AD9269_INST_REG_UPDATE, dev_instruction_set.AD9269_CHANNEL_A, dev_instruction_set.AD9269_CLOCK_DIVIDER_REG, clk_div - 1)
            if self.usb.read(size=1)[0] == dev_instruction_set.ACK:
                self.samplingFreq = self.adc_master_clock / clk_div
            else:
                print('Request fail !! -> clock_divider()')


    # ADC Duty Cycle Stabilizer Enable
    def DCS_enable(self):
        self.send_request(dev_instruction_set.AD9269_INST_REG_UPDATE, dev_instruction_set.AD9269_CHANNEL_A, dev_instruction_set.AD9269_CLOCK_REG, 0x01)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('Request fail !! -> DCS_enable()')

    # ADC Duty Cycle Stabilizer Disable
    def DCS_disable(self):
        self.send_request(dev_instruction_set.AD9269_INST_REG_UPDATE, dev_instruction_set.AD9269_CHANNEL_A, dev_instruction_set.AD9269_CLOCK_REG, 0x00)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('Request fail !! -> DCS_disable()')

    # ADC Output Delay (Default = 0 ns)
    def output_delay(self, channel, reg_val=0x00):
        self.send_request(dev_instruction_set.AD9269_INST_REG_UPDATE, channel, dev_instruction_set.AD9269_OUTPUT_DELAY_REG, reg_val)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('Request fail !! -> output_delay()')

    # ADC Output Adjust (Default = 3.3V DCO & Data Drive Strength : 4 stripes
    def output_adjust(self, channel, reg_val=0xEE):
        self.send_request(dev_instruction_set.AD9269_INST_REG_UPDATE, channel, dev_instruction_set.AD9269_OUTPUT_ADJUST_REG, reg_val)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('Request fail !! -> output_adjust()')

    # ADC Output Mode (Default = 3.3V CMOS Output Enabled)
    def output_mode(self, channel, reg_val=0x00):
        self.send_request(dev_instruction_set.AD9269_INST_REG_UPDATE, channel, dev_instruction_set.AD9269_OUTPUT_MODE_REG, reg_val)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('Request fail !! -> output_mode()')

    # Change Target ADC Output Data Capture Channel
    def set_data_capture_channel(self, channel='A'):
        if channel == 'A':
            self.data_capture_channel = dev_instruction_set.AD9269_CHANNEL_A
        elif channel == 'B':
            self.data_capture_channel = dev_instruction_set.AD9269_CHANNEL_B
        else:
            print('Invalid Channel Input')

    # ADC Initialization
    def adc_init(self):
        if self.adc_speed_grade == 20:
            self.clock_divider(clk_div=self.adc_default_clk_div_20MSPS)
        elif self.adc_speed_grade == 80:
            self.clock_divider(clk_div=self.adc_default_clk_div_80MSPS)
        else:
            print("Error During Initialization -> clock_divider()")

        self.DCS_enable()
        self.output_delay(dev_instruction_set.AD9269_CHANNEL_A)
        self.output_delay(dev_instruction_set.AD9269_CHANNEL_B)
        self.output_adjust(dev_instruction_set.AD9269_CHANNEL_A)
        self.output_adjust(dev_instruction_set.AD9269_CHANNEL_B)
        self.output_mode(dev_instruction_set.AD9269_CHANNEL_A)
        self.output_mode(dev_instruction_set.AD9269_CHANNEL_B)
        self.set_data_capture_channel('A')





