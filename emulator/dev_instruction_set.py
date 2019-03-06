# USB ACK/NACK
NACK =	0x00
ACK = 0x01

# USB Packet Instruction
ADP5350_INST = 0x00
ADP5350_INST_CHECK_COMM = 0x00
ADP5350_INST_GET_STATUS = 0x01
ADP5350_INST_SET_CHARGER = 0x02

HD3SS3220_INST = 0x01
HD3SS3220_INST_GET_REG_STATUS = 0x00

AD9269_INST	= 0x02
AD9269_INST_CHECK_COMM = 0x00
AD9269_INST_GET_SPEED_GRADE = 0x01
AD9269_INST_REG_UPDATE = 0x02

AFE_INST = 0x03
AFE_INST_INIT = 0x00
AFE_INST_DEINIT = 0x01

AD9834_DDS_INST	= 0x04
AD9834_DDS_INST_SET_OUTPUT_REG = 0x00
AD9834_DDS_INST_SET_FREQ = 0x01
AD9834_DDS_INST_SET_PHASE =	0x02
AD9834_DDS_INST_SET_WAVEFORM = 0x03
AD9834_DDS_INST_SLEEP =	0x04

T_SW_INST = 0x05
T_SW_GET_STATUS = 0x00
T_SW_INST_RESET = 0x01
T_SW_INST_EXC_ELEC = 0x02
T_SW_INST_DET_ELEC = 0x03

DSP_INST = 0x06
DSP_INST_INIT = 0x00
DSP_INST_GET_CAP_ARRAY_FULL = 0x01
DSP_INST_GET_CAP_ARRAY_HALF = 0x02
DSP_INST_LIA = 0x03
DSP_INST_UPDATE_VREF_DATA = 0x04
DSP_INST_GET_ADC_DATA = 0x05
DSP_INST_GET_DWT_CNT = 0x06


# ADP5350 Battery PMIC Register Address
ADP5350_MODEL = 0x00
ADP5350_REV = 0x01
ADP5350_CHG_VBUS_ILIM = 0x02
ADP5350_CHG_TERM_SET = 0x03
ADP5350_CHG_I_SET = 0x04
ADP5350_CHG_VTH = 0x05
ADP5350_CHG_TIMER = 0x06
ADP5350_CHG_FUNCTION = 0x07
ADP5350_CHG_STATUS1 = 0x08
ADP5350_CHG_STATUS2 = 0x09
ADP5350_CHG_FAULT = 0x0A
ADP5350_BAT_SHORT = 0x0B
ADP5350_BAT_NTC = 0x0C
ADP5350_V_SOC_0 = 0x0D
ADP5350_V_SOC_5 = 0x0E
ADP5350_V_SOC_11 = 0x0F
ADP5350_V_SOC_19 = 0x10
ADP5350_V_SOC_28 = 0x11
ADP5350_V_SOC_41 = 0x12
ADP5350_V_SOC_55 = 0x13
ADP5350_V_SOC_69 = 0x14
ADP5350_V_SOC_84 = 0x15
ADP5350_V_SOC_100 = 0x16
ADP5350_FILTER_SET1 = 0x17
ADP5350_FILTER_SET2 = 0x18
ADP5350_RBAT_0 = 0x19
ADP5350_RBAT_10 = 0x1A
ADP5350_RBAT_20 = 0x1B
ADP5350_RBAT_30 = 0x1C
ADP5350_RBAT_40 = 0x1D
ADP5350_RBAT_60 = 0x1E
ADP5350_K_RBAT_CHG = 0x1F
ADP5350_BAT_TEMP = 0x20
ADP5350_BAT_SOC = 0x21
ADP5350_VBAT_READ_H = 0x22
ADP5350_VBAT_READ_L = 0x23
ADP5350_FUEL_GAUGE_MODE = 0x24
ADP5350_SOC_RESET = 0x25
ADP5350_BST_LED_CTRL = 0x26
ADP5350_BST_CFG = 0x27
ADP5350_LDO_CTRL = 0x32
ADP5350_LDO_CFG = 0x33
ADP5350_VID_LDO12 = 0x34
ADP5350_VID_LDO3 = 0x35
ADP5350_PGOOD_STATUS = 0x36
ADP5350_PGOOD_MASK = 0x37
ADP5350_CHG_INT_EN = 0x38
ADP5350_CHG_INT_FLAG = 0x39
ADP5350_BOOST_LDO_INT_EN = 0x3A
ADP5350_BOOST_LDO_INT_FLAG = 0x3B
ADP5350_DEFAULT_SET = 0x3C
ADP5350_NTC47K_SET = 0x3D


# AD9269 ADC Constants
AD9269_DATA_SIZE = 4096     # FIFO memory size = 4096

AD9269_CHANNEL_A =	0x01
AD9269_CHANNEL_B = 0x02
AD9269_CHANNEL_BOTH = 0x03

AD9269_SPI_PORT_CONFIG_REG = 0x00
AD9269_CHIP_ID_REG = 0x01
AD9269_CHIP_GRADE_REG = 0x02
AD9269_CHANNEL_INDEX_REG = 0x05
AD9269_TRANSFER_REG = 0xFF
AD9269_MODES_REG = 0x08
AD9269_CLOCK_REG = 0x09
AD9269_CLOCK_DIVIDER_REG = 0x0B
AD9269_TEST_MODE_REG = 0x0D
AD9269_BIST_ENABLE_REG = 0x0E
AD9269_OFFSET_ADJUST_REG = 0x10
AD9269_OUTPUT_MODE_REG = 0x14
AD9269_OUTPUT_ADJUST_REG = 0x15
AD9269_OUTPUT_PHASE_REG = 0x16
AD9269_OUTPUT_DELAY_REG = 0x17

