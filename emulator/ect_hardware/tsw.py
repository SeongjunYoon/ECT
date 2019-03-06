import dev_instruction_set


class T_SW:
    def __init__(self, ect_controller):
        self.ect_controller = ect_controller
        self.usb = self.ect_controller.usb
        self.packet = [dev_instruction_set.T_SW_INST, 0x00, 0x00]
        # T-Switch Status
        # tsw_status[0] = Current Excitation Electrode Flag
        # tsw_status[1] = Current Detection Electrode Flag
        # tsw_status[2] = EXC_SWE
        # tsw_status[3] = EXC_SWG
        # tsw_status[4] = DET_SWE
        # tsw_status[5] = DET_SWG
        self.tsw_status = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.get_tsw_status()

    def send_request(self, instruction, data=0x00):
        self.packet[1] = instruction
        self.packet[2] = data
        self.usb.write(data=self.packet)

    def get_tsw_status(self):
        self.send_request(dev_instruction_set.T_SW_GET_STATUS)
        self.tsw_status = self.usb.read(size=6)
        # print('Excitation Electrode = ',self.tsw_status[0])
        # print('Detection Electrode = ',self.tsw_status[1])

    def tsw_reset(self):
        self.send_request(dev_instruction_set.T_SW_INST_RESET)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('Request fail !! -> tsw_reset()')
        self.get_tsw_status()

    def exc_tsw_control(self, exc_electrode):
            self.send_request(dev_instruction_set.T_SW_INST_EXC_ELEC, exc_electrode)
            if self.usb.read(size=1)[0] == dev_instruction_set.ACK:
                self.get_tsw_status()
            else:
                print('Request fail !! -> exc_tsw_control()')

    def det_tsw_control(self, det_electrode):
            self.send_request(dev_instruction_set.T_SW_INST_DET_ELEC, det_electrode)
            if self.usb.read(size=1)[0] == dev_instruction_set.ACK:
                self.get_tsw_status()
            else:
                print('Request fail !! -> det_tsw_control()')



