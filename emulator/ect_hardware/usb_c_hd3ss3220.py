import dev_instruction_set


class USB_TypeC_IC():
    def __init__(self, ect_controller):
        self.ect_controller = ect_controller
        self.usb = self.ect_controller.usb
        self.packet = [dev_instruction_set.HD3SS3220_INST, 0x00]
        # Connection Status Register (CSR)
        self.csr = None
        # Connection Status and Control register (CSCR)
        self.cscr = None
        # Register Data Validity Check (0x00 : Invalid, 0x01 : Valid)
        self.error = 0x00
        # Status Variables
        self.current_mode = None
        self.active_cable_detection = None
        self.attached_state = None
        self.cable_direction = None
        self.vconn_fault = None
        self.update()

    def send_request(self, instruction):
        self.packet[1] = instruction
        self.usb.write(data=self.packet)

    def update(self):
        self.get_register_status()
        if not self.error == 0x00:
            self.current_mode = self.check_detected_current_mode()
            self.active_cable_detection = self.check_active_cable_detection()
            self.attached_state = self.check_attached_state()
            self.cable_direction = self.check_cable_direction()
            self.vconn_fault = self.check_vconn_fault()
        else:
            self.current_mode = None
            self.active_cable_detection = None
            self.attached_state = None
            self.cable_direction = None
            self.vconn_fault = None

    def get_register_status(self):
        self.send_request(dev_instruction_set.HD3SS3220_INST_GET_REG_STATUS)
        self.temp = self.usb.read(size=3)
        # 3rd byte of USB RX = Validity check data
        self.csr = self.temp[0]
        self.cscr = self.temp[1]
        self.error = self.temp[2]  # self.error = {0x00 : Invalid data, 0x01 : Valid data}

    # Detected Type-C Current Mode
    def check_detected_current_mode(self):
        return {
            0x00: 'Default (500 mA / 900 mA)',
            0x01: 'Medium (1500 mA)',
            0x02: 'Charge through accessory (500 mA)',
            0x03: 'High (3000 mA)',
        }.get((self.csr >> 4) & 0x03, 'Error')

    # Active Cable Detection
    def check_active_cable_detection(self):
        return {
            0x00: 'No active cable',
            0x01: 'Active cable attached',
        }.get(self.csr & 0x01, 'Error')

    # Attached State
    def check_attached_state(self):
        return {
            0x00: 'Not attached',
            0x01: 'Attached.src (DFP)',
            0x02: 'Attached.snk (UFP)',
            0x03: 'Attached to accessory',
        }.get((self.cscr >> 6) & 0x03, 'Error')

    # USB Cable Direction
    def check_cable_direction(self):
        return {
            0x00: 'CC2',
            0x01: 'CC1',
        }.get((self.cscr >> 5) & 0x01, 'Error')

    # VCONN Fault
    def check_vconn_fault(self):
        return {
            0x00: 'Clear',
            0x01: 'Fault detected (Over-current Limit)',
        }.get((self.cscr >> 3) & 0x01, 'Error')
