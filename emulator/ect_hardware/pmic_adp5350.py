import dev_instruction_set


class BatterySOC:
    def __init__(self, ect_controller):
        self.ect_controller = ect_controller
        self.usb = self.ect_controller.usb
        self.packet = [dev_instruction_set.ADP5350_INST, dev_instruction_set.ADP5350_INST_GET_STATUS]
        self.battery_status = []
        self.battery_soc = 0
        self.battery_voltage = 0
        self.VBUSOK = 0
        self.BATOK = 0
        self.PG1_LDO1 = 0
        self.CHG_MODE = 0
        self.CHDONE = 0
        self.VBUS_OV = 0
        self.VBUS_ILIM = 0
        self.THERM_LIM = 0
        self.V_BSNS = 0
        self.THR_STATUS = 0
        self.IPK_STAT = 0
        self.IND_PEAK_INT = 0
        self.THERM_LIM_INT = 0
        self.WD_INT = 0
        self.TSD_INT = 0
        self.THR_INT = 0
        self.BAT_INT = 0
        self.CHG_INT = 0
        self.VIN_INT = 0
        self.BAT_SHR = 0
        self.IND_PEAK = 0
        self.TSD_130 = 0
        self.TSD_140 = 0
        self.update()

    def send_soc_request(self):
        self.usb.write(data=self.packet)
        self.battery_status = self.usb.read(size=8)

    def update(self):
        self.send_soc_request()
        self.get_battery_soc()
        self.get_battery_voltage()
        self.get_pgood_status()
        self.get_charger_status1()
        self.get_charger_status2()
        self.get_interrupt_flags()
        self.get_fault_flags()

    def get_battery_soc(self):
        self.battery_soc = self.battery_status[0] & 0x7F

    def get_battery_voltage(self):
        try:
            self.battery_voltage = (self.battery_status[1] * 32 + self.battery_status[2] / 8) / 1000
        except Exception as e:
            print(e)
            print(self.battery_status)

    def get_pgood_status(self):
        self.VBUSOK = (self.battery_status[3] >> 3) & 0x01  # VBUSOK == 1 : USB Power Connected
        self.BATOK = (self.battery_status[3] >> 2) & 0x01  # BATOK == 1 : Battery Status Good
        self.PG1_LDO1 = self.battery_status[3] & 0x01  # PG1_LDO1 == 1 : 3.3V Always-on LDO Power Good
        """
        if self.VBUSOK == 1:
            print('USB Power = Connected')
        else:
            print('USB Power = Disconnected')

        if self.BATOK == 1:
            print('Battery Status = Good')
        else:
            print('Battery Status = Weak or No Battery !!')

        if self.PG1_LDO1 == 1:
            print('3.3V Always-on LDO Power = Good')
        else:
            print('3.3V Always-on LDO Power = Warning !!')
         """

    def battery_charger_mode_dict(self, argv):
        # Battery Status Bus @ CHARGER_STATUS1 Register
        # 000 = off    /   001 = trickle charge  /   010 = fast charge (CC mode)   /   011 = fast charge (CV mode)
        # 100 = charge complete    /    101 = suspend   / 110 = trickle, fast, or safety charge timer expired
        # 111 = battery detection
        return {
            0x00: 'Off',
            0x01: 'Trickle Charge',
            0x02: 'Fast Charge (CC Mode)',
            0x03: 'Fast Charge (CV Mode)',
            0x04: 'Charge Complete',
            0x05: 'Suspend',
            0x06: 'Trickle, Fast, or Safety Charge Timer Expired',
            0x07: 'Battery Detection'
        }.get(argv, 'Error')

    def get_charger_status1(self):
        self.CHG_MODE = self.battery_charger_mode_dict(self.battery_status[4] & 0x07)
        self.CHDONE = (self.battery_status[4] >> 3) & 0x01  # Charge Done Flag, CHDONE == 1 : End of Charge
        self.VBUS_OV = (self.battery_status[4] >> 7) & 0x01  # VBUS overvoltage flag, VBUS_OV == 1 : Overvoltage
        self.VBUS_ILIM = (self.battery_status[
                              4] >> 5) & 0x01  # ILIM by high voltage blocking FET flag, VBUS_ILIM == 1 : Limited I
        self.THERM_LIM = (self.battery_status[4] >> 4) & 0x01  # ILIM by high temp. flag, THERM_LIM == 1 : Limited I
        """
        if self.CHDONE == 1:
            print('Charging Status = End of Charge')
        else:
            print('Charging Status = Active')
        if self.VBUS_OV == 1:
            print('(Warning) V_BUS Overvoltage !!')
        if self.VBUS_ILIM == 1:
            print('(Warning) I_CHG Limited by High Voltage Blocking FET !!')
        if self.THERM_LIM == 1:
            print('(Warning) I_CHG Limited by High Temperature !!')
        """

    def battery_voltage_sense_dict(self, argv):
        # Battery Status Bus @ CHARGER_STATUS2 Register
        # 000 = Battery monitor off  /  001 = no battery  /  010 = V_BSNS < V_TRK  /  011 = V_TRK < V_BSNS < V_WEAK
        # 100 = V_BSNS > V_WEAK
        return {
            0x00: 'Monitor Off',
            0x01: 'No Battery',
            0x02: '(Warning) Battery Voltage < Trickle Voltage',
            0x03: '(Warning) Battery Voltage < Weak Voltage',
            0x04: 'Normal'
        }.get(argv, 'Error')

    def battery_temperature_dict(self, argv):
        # Thermistor Pin Status @ CHARGER_STATUS2 Register
        # 000 = Thermistor off    /   001 = Battery cold  /   010 = Battery cool   /   011 = Battery warm
        # 100 = Battery hot    /    111 = Battery normal
        return {
            0x00: 'Thermistor Off',
            0x01: 'Cold',
            0x02: 'Cool',
            0x03: 'Warm',
            0x04: 'Hot',
            0x07: 'Normal'
        }.get(argv, 'Error')

    def get_charger_status2(self):
        self.V_BSNS = self.battery_voltage_sense_dict(self.battery_status[5] & 0x07)
        self.THR_STATUS = self.battery_temperature_dict((self.battery_status[5] >> 5) & 0x07)
        self.IPK_STAT = (self.battery_status[5] >> 4) & 0x10  # IPK_STAT == 1 : Peak Inductor I Limit
        """
        print('Battery Voltage Sensing -> ', self.V_BSNS)
        print('Battery temperature = ', self.THR_STATUS)
        if self.IPK_STAT == 1:
            print('Inductor Peak Current = (WARNING) Peak Inductor Current Limit !!')
        else:
            print('Inductor Peak Current = Normal')
        """

    def get_interrupt_flags(self):
        self.IND_PEAK_INT |= ((self.battery_status[6] >> 7) & 0x80)
        self.THERM_LIM_INT |= ((self.battery_status[6] >> 6) & 0x40)
        self.WD_INT |= ((self.battery_status[6] >> 5) & 0x20)
        self.TSD_INT |= ((self.battery_status[6] >> 4) & 0x10)
        self.THR_INT |= ((self.battery_status[6] >> 3) & 0x08)
        self.BAT_INT |= ((self.battery_status[6] >> 2) & 0x04)
        self.CHG_INT |= ((self.battery_status[6] >> 1) & 0x02)
        self.VIN_INT |= (self.battery_status[6] & 0x01)
        """
        if self.IND_PEAK_INT == 1:
            print('Charger Interrupt Flag - Inductor Peak Current Limit')
        if self.THERM_LIM_INT == 1:
            print('Charger Interrupt Flag - Isothermal Charging')
        if self.WD_INT == 1:
            print('Charger Interrupt Flag - Watchdog Alarm')
        if self.TSD_INT == 1:
            print('Charger Interrupt Flag - Over-Temperature Fault')
        if self.THR_INT == 1:
            print('Charger Interrupt Flag - Battery Temperature Thresholds')
        if self.BAT_INT == 1:
            print('Charger Interrupt Flag - Battery Voltage Threshold')
        if self.CHG_INT == 1:
            print('Charger Interrupt Flag - Charger Mode Change')
        if self.VIN_INT == 1:
            print('Charger Interrupt Flag - VBUS Voltage Threshold')
        """

    def get_fault_flags(self):
        self.BAT_SHR = (self.battery_status[7] >> 3) & 0x01
        self.IND_PEAK = (self.battery_status[7] >> 2) & 0x01
        self.TSD_130 = (self.battery_status[7] >> 1) & 0x01
        self.TSD_140 = self.battery_status[7] & 0x01
        """
        if self.BAT_SHR == 1:
            print('Charger Fault - Battery Short')
        if self.IND_PEAK == 1:
            print('Charger Fault - Inductor Peak Current Limit')
        if self.TSD_130 == 1:
            print('Charger Fault - Over-temperature Early Warning')
        if self.TSD_140 == 1:
            print('Charger Fault - Chip Over-temperature')
        """


class BatteryChargerSetting:
    def __init__(self, ect_controller):
        self.ect_controller = ect_controller
        self.usb = self.ect_controller.usb
        self.packet = [dev_instruction_set.ADP5350_INST, dev_instruction_set.ADP5350_INST_SET_CHARGER, 0, 0]
        self.charger_setting_init()

    def send_update_request(self, target_reg, data):
        self.packet[2] = target_reg
        self.packet[3] = data
        self.usb.write(data=self.packet)

    def charger_setting_init(self):
        self.set_charger_vbus_ilim()
        self.set_charger_termination_settings()
        self.set_charger_current_setting()
        self.set_charger_voltage_threshold()
        self.set_charger_timer_setting()
        self.charger_on()
        self.set_fuel_gauge_mode()
        self.set_battery_short()
        self.set_battery_thermistor_control()
        self.set_v_soc()
        self.set_filter_setting1()
        self.set_filter_setting2()
        self.set_r_bat()
        self.set_k_rbat_charge()
        self.set_bat_temp()
        self.set_ntc47k_set()
        self.set_LDO_ctrl()
        self.set_LDO_cfg()
        self.set_VID_LDO12()
        self.set_pgood_mask()
        self.set_charger_interrupt_enable()

    def reset_all(self):
        self.send_update_request(dev_instruction_set.ADP5350_DEFAULT_SET, 0x7F)

    # EN_JEITA = 1 (Enable JEITA)
    # DIS_IPK_SD = 1 (Disable Automatic Shutdown Even Under Inductor Peak Current Limit Condition)
    # EN_BMON = 1 (Enable Battery Monitor Even When VBUS < VBUSOK_FALL)
    # EN_THR = 1 (Enable NTC Current Source Even When VBUS < VBUSOK_FALL)
    # EN_DCDC = 1 (Enable DC-DC Converter), EN_EOC = 1 (Allow End of Charge), EN_TRK = 1 (Enable TRK CHG)
    # EN_CHG = 1 (Enable Battery Charging)
    def charger_on(self, val=0xFF):
        self.send_update_request(dev_instruction_set.ADP5350_CHG_FUNCTION, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> charger_on()')

    # EN_JEITA = 1 (Enable JEITA)
    # DIS_IPK_SD = 1 (Disable Automatic Shutdown Even Under Inductor Peak Current Limit Condition)
    # EN_BMON = 1 (Enable Battery Monitor Even When VBUS < VBUSOK_FALL)
    # EN_THR = 1 (Enable NTC Current Source Even When VBUS < VBUSOK_FALL)
    # EN_DCDC = 1 (Enable DC-DC Converter), EN_EOC = 1 (Allow End of Charge), EN_TRK = 1 (Enable TRK CHG)
    # EN_CHG = 0 (Disable Battery Charging)
    def charger_off(self, val=0xFE):
        self.send_update_request(dev_instruction_set.ADP5350_CHG_FUNCTION, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> charger_off()')

    # SLEEP_UPDATE_TIME = 0 (5 min), FUEL_GAUGE_MODE = 1 (Enable Sleep Mode)
    # FUEL_GAUGE_ENABLE = 1 (Enable Fuel Gauge)
    def set_fuel_gauge_mode(self, val=0x03):
        self.send_update_request(dev_instruction_set.ADP5350_FUEL_GAUGE_MODE, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_fuel_gauge_mode()')

    # ILIM[3:0] @ CHARGER_VBUS_ILIM Register
    # 0000 = 100 mA  /  0001 = 150 mA  /  0010 = 200 mA  /  0011 = 300 mA  /  0100 = 400 mA  /  0101 = 500 mA
    # 0110 = 600 mA  /  0111 = 700 mA  /  1000 = 800 mA  /  1001 = 900 mA  /  1010 = 1000 mA  /  1011 = 1100 mA
    # 1100 = 1200 mA  /  1101 = 1300 mA  /  1110 = 1400 mA  /  1111 = 1500 mA
    def i_lim_dict(self, argv):
        return {
            100: 0x00,
            150: 0x01,
            200: 0x02,
            300: 0x03,
            400: 0x04,
            500: 0x05,
            600: 0x06,
            700: 0x07,
            800: 0x08,
            900: 0x09,
            1000: 0x0A,
            1100: 0x0B,
            1200: 0x0C,
            1300: 0x0D,
            1400: 0x0E,
            1500: 0x0F
        }.get(argv, 0x0F)

    def set_charger_vbus_ilim(self, val=1500):
        self.send_update_request(dev_instruction_set.ADP5350_CHG_VBUS_ILIM, self.i_lim_dict(val))
        if not self.usb.read(size=1)[0] == self.i_lim_dict(val):
            print('Request fail !! -> set_charger_vbus_ilim()')

    # EN_TEND = 1 (Enable Charge Complete Timer), EN_CHG_TIMER = 1 (Enable Trickle/Fast Charger Timer)
    # CHG_TMR_PERIOD[4:3] = 11 (Trickle/Fast Charger Timer Period = 60 min/600 min)
    # EN_WD = 1 (Enable Watchdog Timer Safety Timer)
    # WD_PERIOD = 0 (Watchdog Safety Timer Period = 32 sec to 40 min)
    # RESET_WD = 1 (Write only) => If RESET_WD = 1, Watchdog Safety Timer Resets and RESET_WD -> 0 Automatically
    def set_charger_timer_setting(self, val=0x7D):
        self.send_update_request(dev_instruction_set.ADP5350_CHG_TIMER, val)
        if not self.usb.read(size=1)[0] == val & 0xFE:
            print('Request fail !! -> set_charger_timer_setting()')

    def reset_charger_fault(self, val=0x00):
        self.send_update_request(dev_instruction_set.ADP5350_CHG_FAULT, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> reset_charger_fault()')

    # V_SOC_0 (Default = 0x7C (3.5V))
    # V_SOC_5 (Default = 0x91 (3.66V))
    # V_SOC_11 (Default = 0x94 (3.684V))
    # V_SOC_19 (Default = 0x99 (3.724V))
    # V_SOC_28 (Default = 0x9E (3.764V))
    # V_SOC_41 (Default = 0xA3 (3.804V))
    # V_SOC_55 (Default = 0xAB (3.868V))
    # V_SOC_69 (Default = 0xB5 (3.948V))
    # V_SOC_84 (Default = 0xC4 (4.068V))
    # V_SOC_100 (Default = 0xD5 (4.204V))
    def set_v_soc(self, v_soc_0=0x7C, v_soc_5=0x91, v_soc_11=0x94, v_soc_19=0x99, v_soc_28=0x9E, v_soc_41=0xA3,
                  v_soc_55=0xAB, v_soc_69=0xB5, v_soc_84=0xC4, v_soc_100=0xD5):
        self.send_update_request(dev_instruction_set.ADP5350_V_SOC_0, v_soc_0)
        if not self.usb.read(size=1)[0] == v_soc_0:
            print('Request fail !! -> set_v_soc_0()')

        self.send_update_request(dev_instruction_set.ADP5350_V_SOC_5, v_soc_5)
        if not self.usb.read(size=1)[0] == v_soc_5:
            print('Request fail !! -> set_v_soc_5()')

        self.send_update_request(dev_instruction_set.ADP5350_V_SOC_11, v_soc_11)
        if not self.usb.read(size=1)[0] == v_soc_11:
            print('Request fail !! -> set_v_soc_11()')

        self.send_update_request(dev_instruction_set.ADP5350_V_SOC_19, v_soc_19)
        if not self.usb.read(size=1)[0] == v_soc_19:
            print('Request fail !! -> set_v_soc_19()')

        self.send_update_request(dev_instruction_set.ADP5350_V_SOC_28, v_soc_28)
        if not self.usb.read(size=1)[0] == v_soc_28:
            print('Request fail !! -> set_v_soc_28()')

        self.send_update_request(dev_instruction_set.ADP5350_V_SOC_41, v_soc_41)
        if not self.usb.read(size=1)[0] == v_soc_41:
            print('Request fail !! -> set_v_soc_41()')

        self.send_update_request(dev_instruction_set.ADP5350_V_SOC_55, v_soc_55)
        if not self.usb.read(size=1)[0] == v_soc_55:
            print('Request fail !! -> set_v_soc_55()')

        self.send_update_request(dev_instruction_set.ADP5350_V_SOC_69, v_soc_69)
        if not self.usb.read(size=1)[0] == v_soc_69:
            print('Request fail !! -> set_v_soc_69()')

        self.send_update_request(dev_instruction_set.ADP5350_V_SOC_84, v_soc_84)
        if not self.usb.read(size=1)[0] == v_soc_84:
            print('Request fail !! -> set_v_soc_84()')

        self.send_update_request(dev_instruction_set.ADP5350_V_SOC_100, v_soc_100)
        if not self.usb.read(size=1)[0] == v_soc_100:
            print('Request fail !! -> set_v_soc_100()')

    # Resistance value = R_BAT_# * 32 mohm
    def set_r_bat(self, r_bat_0=0x02, r_bat_10=0x02, r_bat_20=0x02, r_bat_30=0x02, r_bat_40=0x02, r_bat_60=0x02):
        self.send_update_request(dev_instruction_set.ADP5350_RBAT_0, r_bat_0)
        if not self.usb.read(size=1)[0] == r_bat_0:
            print('Request fail !! -> set_r_bat_0()')

        self.send_update_request(dev_instruction_set.ADP5350_RBAT_10, r_bat_10)
        if not self.usb.read(size=1)[0] == r_bat_10:
            print('Request fail !! -> set_r_bat_10()')

        self.send_update_request(dev_instruction_set.ADP5350_RBAT_20, r_bat_20)
        if not self.usb.read(size=1)[0] == r_bat_20:
            print('Request fail !! -> set_r_bat_20()')

        self.send_update_request(dev_instruction_set.ADP5350_RBAT_30, r_bat_30)
        if not self.usb.read(size=1)[0] == r_bat_30:
            print('Request fail !! -> set_r_bat_30()')

        self.send_update_request(dev_instruction_set.ADP5350_RBAT_40, r_bat_40)
        if not self.usb.read(size=1)[0] == r_bat_40:
            print('Request fail !! -> set_r_bat_40()')

        self.send_update_request(dev_instruction_set.ADP5350_RBAT_60, r_bat_60)
        if not self.usb.read(size=1)[0] == r_bat_60:
            print('Request fail !! -> set_r_bat_60()')

    # K_RBAT_SOC = 0x00 (RBAT at 0% SOC = RBAT at 20% SOC), K_RBAT_CHARGE = 0x08 (RBAT Coefficient for Charging = 1)
    def set_k_rbat_charge(self, val=0x08):
        self.send_update_request(dev_instruction_set.ADP5350_K_RBAT_CHG, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_k_rbat_charge()')

    # Setting Each Bit to 1 Enables Each Interrupt
    # (Bit 7)	EN_IND_PEAK_INT
    # (Bit 6)	EN_THERM_LIM_INT
    # (Bit 5)	EN_WD_INT
    # (Bit 4)	EN_TSD_INT
    # (Bit 3)	EN_THR_INT
    # (Bit 2)	EN_BAT_INT
    # (Bit 1)	EN_CHG_INT
    # (Bit 0)	EN_VIN_INT
    def set_charger_interrupt_enable(self, val=0xFF):
        self.send_update_request(dev_instruction_set.ADP5350_CHG_INT_EN, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_charger_interrupt_enable()')

    # VTRM[5:0] = 100011 (4.20 V, default), IEND[1:0] = 01 (35 mA, default)
    def set_charger_termination_settings(self, val=0x8D):
        self.send_update_request(dev_instruction_set.ADP5350_CHG_TERM_SET, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_charger_termination_settings()')


    # C_20_EOC = C_10_EOC = 0 (default), ICHG[3:0] = 1100 (500 mA, default)
    # ITRK_DEAD[1:0] = 10 (20 mA, default)
    def set_charger_current_setting(self, val=0x32):
        self.send_update_request(dev_instruction_set.ADP5350_CHG_I_SET, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_charger_current_setting()')


    # VRCH[1:0] = 00 (80 mV), VTRK_DEAD[1:0] = 01 (2.5 V, default)
    # VWEAK[2:0] = 011 (3.0 V, default)
    def set_charger_voltage_threshold(self, val=0x0B):
        self.send_update_request(dev_instruction_set.ADP5350_CHG_VTH, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_charger_voltage_threshold()')


    # TBAT_SHR[2:0] = 100 (30 sec, default), VBAT_SHR[2:0] = 100 (2.4 V, default)
    def set_battery_short(self, val=0x84):
        self.send_update_request(dev_instruction_set.ADP5350_BAT_SHORT, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_battery_short()')


    # ILIM_JEITA_COOL = 0 (50% I_CHG, default), TBAT_LOW = 0 (0 degC, default)
    # TBAT_HIGH = 1 (60 degC, default), R_NTC = 1 (100 kohm or 47k ohm, default)
    # BETA_NTC = 1000 (3800, default)
    def set_battery_thermistor_control(self, val=0x38):
        self.send_update_request(dev_instruction_set.ADP5350_BAT_NTC, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_battery_thermistor_control()')


    # Set FILTER_CHARGE = 100 (1C), FILTER_DISCHARGE = 100 (1C)
    def set_filter_setting1(self, val=0x44):
        self.send_update_request(dev_instruction_set.ADP5350_FILTER_SET1, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_filter_setting1()')


    # Set FILTER_IDLE = 00 (FILTER_CHARGE/8)
    def set_filter_setting2(self, val=0x00):
        self.send_update_request(dev_instruction_set.ADP5350_FILTER_SET2, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_filter_setting2()')


    # BAT_TEMP_SOURCE = 0 (From THR input, default)
    def set_bat_temp(self, val=0x1B):
        self.send_update_request(dev_instruction_set.ADP5350_BAT_TEMP, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_bat_temp()')


    # NTC_47K = 1 (47 kohm at 25 degC, default)
    def set_ntc47k_set(self, val=0x01):
        self.send_update_request(dev_instruction_set.ADP5350_NTC47K_SET, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_ntc47k_set()')


    # EN_LDO1 = 1 (Always On LDO1, default)
    def set_LDO_ctrl(self, val=0x01):
        self.send_update_request(dev_instruction_set.ADP5350_LDO_CTRL, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_ldo_ctrl()')


    # MODE_LDO1 = 0 (LDO MODE)
    def set_LDO_cfg(self, val=0x00):
        self.send_update_request(dev_instruction_set.ADP5350_LDO_CFG, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_LDO_cfg()')


    # VID_LDO1 = 0x02 (LDO1 Output = 3.3 V)
    def set_VID_LDO12(self, val=0x02):
        self.send_update_request(dev_instruction_set.ADP5350_VID_LDO12, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_VID_LDO12()')


    # VBUSOK_MASK = 1 (Output VBUS Status to External PGOOD Pin, Factory Setting)
    def set_pgood_mask(self, val=0x08):
        self.send_update_request(dev_instruction_set.ADP5350_PGOOD_MASK, val)
        if not self.usb.read(size=1)[0] == val:
            print('Request fail !! -> set_pgood_mask()')



class Battery(BatterySOC, BatteryChargerSetting):
    def __init__(self, ect_controller):
        self.ect_controller = ect_controller
        self.usb = self.ect_controller.usb
        BatteryChargerSetting.__init__(self, self.ect_controller)
        BatterySOC.__init__(self, self.ect_controller)

    def check_pmic_communication(self):
        self.usb.write(data=[dev_instruction_set.ADP5350_INST, dev_instruction_set.ADP5350_INST_CHECK_COMM])
        if not self.usb.read(size=1)[0] == dev_instruction_set.ACK:
            print('ADP5350 I2C Communication Fail !!')

