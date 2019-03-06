import dev_instruction_set


class AFE():
    def __init__(self, ect_controller):
        self.ect_controller = ect_controller
        self.usb = self.ect_controller.usb
        self.tsw = self.ect_controller.tsw
        self.packet = [dev_instruction_set.AFE_INST, 0x00]

    def send_request(self, instruction):
        self.packet[1] = instruction
        self.usb.write(data=self.packet)

    def afe_init(self):
        self.send_request(dev_instruction_set.AFE_INST_INIT)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('AFE Initialization Failed !!')
        self.tsw.get_tsw_status()

    def afe_deinit(self):
        self.send_request(dev_instruction_set.AFE_INST_DEINIT)
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('AFE De-initialization Failed !!')
