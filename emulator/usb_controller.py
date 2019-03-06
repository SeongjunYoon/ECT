import usb.util


class USB_Controller:
    def __init__(self):
        self.vid = 0x0483  # STMicroelectronics
        self.pid = 0x5740
        self.configuration_value = 1
        self.ep_out = 0x01
        self.ep_in = 0x81
        self.interface = 1
        #
        self.usb_lock = False
        #
        self.usb_init()

    def usb_init(self):
        self.dev = usb.core.find(idVendor=self.vid, idProduct=self.pid)
        if self.dev is None:
            raise ValueError('Device not found')
        if self.dev.is_kernel_driver_active(self.interface):
            self.dev.detach_kernel_driver(self.interface)

        # Check activated configuration
        cfg = self.dev.get_active_configuration()
        if cfg is None or cfg.bConfigurationValue != self.configuration_value:
            self.dev.set_configuration(self.configuration_value)

    def write(self, data):
        self.usb_lock = True
        return self.dev.write(endpoint=self.ep_out, data=data)

    def read(self, size):
        if self.usb_lock:
            dev_return = self.dev.read(endpoint=self.ep_in, size_or_buffer=size)
            self.usb_lock = False
            return dev_return
        else:
            dev_return = []
            for i in range(size):
                dev_return.append(0x00)
            return dev_return           # Return dummy list




