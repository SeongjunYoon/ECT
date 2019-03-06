/* Includes ------------------------------------------------------------------*/
#include "custom_t_switch.h"
#include "custom_usb.h"

/* =============== Functions =============== */
// ADSG1408 8:1 MUX Control
void MUX_Ctrl(uint8_t mux_part, uint8_t elec_num)
{
	/*
	 * Excitation Part MCU GPIO Pin-Map
	 * PE5 = EXC_MUX_A2
	 * PE6 = EXC_MUX_A1
	 * PF2 = EXC_MUX_A0
	 * PF3 = EXC_MUX_EN
	 *
	 * Detection Part MCU GPIO Pin-Map
	 * PF4 = DET_MUX_A2
	 * PE2 = DET_MUX_A1
	 * PE3 = DET_MUX_A0
	 * PE4 = DET_MUX_EN
	 *
	 * ADG1408 MUX to Electrode Mapping
	 * ELEC1 = EXC_MUX_S4 = DET_MUX_S4
	 * ELEC2 = EXC_MUX_S3 = DET_MUX_S3
	 * ELEC3 = EXC_MUX_S2 = DET_MUX_S2
	 * ELEC4 = EXC_MUX_S1 = DET_MUX_S1
	 * ELEC5 = EXC_MUX_S5 = DET_MUX_S5
	 * ELEC6 = EXC_MUX_S6 = DET_MUX_S6
	 * ELEC7 = EXC_MUX_S7 = DET_MUX_S7
	 * ELEC8 = EXC_MUX_S8 = DET_MUX_S8
	 */

	switch (mux_part)
	{

	// Excitation Part MUX: Amplified Signal -> 8 Electrodes
	case EXC_MUX:
		switch (elec_num)
		{
		// elec_num == 0: MUX Output Disable
		case 0:
			EXC_MUX_A2_GPIO_Port->BSRRH = EXC_MUX_A2_Pin;
			EXC_MUX_A1_GPIO_Port->BSRRH = EXC_MUX_A1_Pin;
			EXC_MUX_A0_GPIO_Port->BSRRH = EXC_MUX_A0_Pin;
			EXC_MUX_EN_GPIO_Port->BSRRH = EXC_MUX_EN_Pin;
			break;

			// elec_num == 1 - 9: MUX Output to Electrode Selection (Number Reference = Electrode)
		case 1:			// ELEC1 = EXC_MUX_S4 = DET_MUX_S4
			EXC_MUX_A2_GPIO_Port->BSRRH = EXC_MUX_A2_Pin;
			EXC_MUX_A1_GPIO_Port->BSRRL = EXC_MUX_A1_Pin;
			EXC_MUX_A0_GPIO_Port->BSRRL = EXC_MUX_A0_Pin;
			EXC_MUX_EN_GPIO_Port->BSRRL = EXC_MUX_EN_Pin;
			break;

		case 2:			// ELEC2 = EXC_MUX_S3 = DET_MUX_S3
			EXC_MUX_A2_GPIO_Port->BSRRH = EXC_MUX_A2_Pin;
			EXC_MUX_A1_GPIO_Port->BSRRL = EXC_MUX_A1_Pin;
			EXC_MUX_A0_GPIO_Port->BSRRH = EXC_MUX_A0_Pin;
			EXC_MUX_EN_GPIO_Port->BSRRL = EXC_MUX_EN_Pin;
			break;

		case 3:			// ELEC3 = EXC_MUX_S2 = DET_MUX_S2
			EXC_MUX_A2_GPIO_Port->BSRRH = EXC_MUX_A2_Pin;
			EXC_MUX_A1_GPIO_Port->BSRRH = EXC_MUX_A1_Pin;
			EXC_MUX_A0_GPIO_Port->BSRRL = EXC_MUX_A0_Pin;
			EXC_MUX_EN_GPIO_Port->BSRRL = EXC_MUX_EN_Pin;
			break;

		case 4:			// ELEC4 = EXC_MUX_S1 = DET_MUX_S1
			EXC_MUX_A2_GPIO_Port->BSRRH = EXC_MUX_A2_Pin;
			EXC_MUX_A1_GPIO_Port->BSRRH = EXC_MUX_A1_Pin;
			EXC_MUX_A0_GPIO_Port->BSRRH = EXC_MUX_A0_Pin;
			EXC_MUX_EN_GPIO_Port->BSRRL = EXC_MUX_EN_Pin;
			break;

		case 5:			// ELEC5 = EXC_MUX_S5 = DET_MUX_S5
			EXC_MUX_A2_GPIO_Port->BSRRL = EXC_MUX_A2_Pin;
			EXC_MUX_A1_GPIO_Port->BSRRH = EXC_MUX_A1_Pin;
			EXC_MUX_A0_GPIO_Port->BSRRH = EXC_MUX_A0_Pin;
			EXC_MUX_EN_GPIO_Port->BSRRL = EXC_MUX_EN_Pin;
			break;

		case 6:			// ELEC6 = EXC_MUX_S6 = DET_MUX_S6
			EXC_MUX_A2_GPIO_Port->BSRRL = EXC_MUX_A2_Pin;
			EXC_MUX_A1_GPIO_Port->BSRRH = EXC_MUX_A1_Pin;
			EXC_MUX_A0_GPIO_Port->BSRRL = EXC_MUX_A0_Pin;
			EXC_MUX_EN_GPIO_Port->BSRRL = EXC_MUX_EN_Pin;
			break;

		case 7:			// ELEC7 = EXC_MUX_S7 = DET_MUX_S7
			EXC_MUX_A2_GPIO_Port->BSRRL = EXC_MUX_A2_Pin;
			EXC_MUX_A1_GPIO_Port->BSRRL = EXC_MUX_A1_Pin;
			EXC_MUX_A0_GPIO_Port->BSRRH = EXC_MUX_A0_Pin;
			EXC_MUX_EN_GPIO_Port->BSRRL = EXC_MUX_EN_Pin;
			break;

		case 8:			// ELEC8 = EXC_MUX_S8 = DET_MUX_S8
			EXC_MUX_A2_GPIO_Port->BSRRL = EXC_MUX_A2_Pin;
			EXC_MUX_A1_GPIO_Port->BSRRL = EXC_MUX_A1_Pin;
			EXC_MUX_A0_GPIO_Port->BSRRL = EXC_MUX_A0_Pin;
			EXC_MUX_EN_GPIO_Port->BSRRL = EXC_MUX_EN_Pin;
			break;

		}

		break;

		// Detection Part MUX: 8 Electrodes -> CV Converter
	case DET_MUX:
		switch (elec_num)
		{
		// elec_num == 0: MUX Output Disable
		case 0:
			DET_MUX_A2_GPIO_Port->BSRRH = DET_MUX_A2_Pin;
			DET_MUX_A1_GPIO_Port->BSRRH = DET_MUX_A1_Pin;
			DET_MUX_A0_GPIO_Port->BSRRH = DET_MUX_A0_Pin;
			DET_MUX_EN_GPIO_Port->BSRRH = DET_MUX_EN_Pin;
			break;

			// elec_num == 1 - 9: Electrode to MUX Selection (Number Reference = Electrode)
		case 1:			// ELEC1 = EXC_MUX_S4 = DET_MUX_S4
			DET_MUX_A2_GPIO_Port->BSRRH = DET_MUX_A2_Pin;
			DET_MUX_A1_GPIO_Port->BSRRL = DET_MUX_A1_Pin;
			DET_MUX_A0_GPIO_Port->BSRRL = DET_MUX_A0_Pin;
			DET_MUX_EN_GPIO_Port->BSRRL = DET_MUX_EN_Pin;
			break;

		case 2:			// ELEC2 = EXC_MUX_S3 = DET_MUX_S3
			DET_MUX_A2_GPIO_Port->BSRRH = DET_MUX_A2_Pin;
			DET_MUX_A1_GPIO_Port->BSRRL = DET_MUX_A1_Pin;
			DET_MUX_A0_GPIO_Port->BSRRH = DET_MUX_A0_Pin;
			DET_MUX_EN_GPIO_Port->BSRRL = DET_MUX_EN_Pin;
			break;

		case 3:			// ELEC3 = EXC_MUX_S2 = DET_MUX_S2
			DET_MUX_A2_GPIO_Port->BSRRH = DET_MUX_A2_Pin;
			DET_MUX_A1_GPIO_Port->BSRRH = DET_MUX_A1_Pin;
			DET_MUX_A0_GPIO_Port->BSRRL = DET_MUX_A0_Pin;
			DET_MUX_EN_GPIO_Port->BSRRL = DET_MUX_EN_Pin;
			break;

		case 4:			// ELEC4 = EXC_MUX_S1 = DET_MUX_S1
			DET_MUX_A2_GPIO_Port->BSRRH = DET_MUX_A2_Pin;
			DET_MUX_A1_GPIO_Port->BSRRH = DET_MUX_A1_Pin;
			DET_MUX_A0_GPIO_Port->BSRRH = DET_MUX_A0_Pin;
			DET_MUX_EN_GPIO_Port->BSRRL = DET_MUX_EN_Pin;
			break;

		case 5:			// ELEC5 = EXC_MUX_S5 = DET_MUX_S5
			DET_MUX_A2_GPIO_Port->BSRRL = DET_MUX_A2_Pin;
			DET_MUX_A1_GPIO_Port->BSRRH = DET_MUX_A1_Pin;
			DET_MUX_A0_GPIO_Port->BSRRH = DET_MUX_A0_Pin;
			DET_MUX_EN_GPIO_Port->BSRRL = DET_MUX_EN_Pin;
			break;

		case 6:			// ELEC6 = EXC_MUX_S6 = DET_MUX_S6
			DET_MUX_A2_GPIO_Port->BSRRL = DET_MUX_A2_Pin;
			DET_MUX_A1_GPIO_Port->BSRRH = DET_MUX_A1_Pin;
			DET_MUX_A0_GPIO_Port->BSRRL = DET_MUX_A0_Pin;
			DET_MUX_EN_GPIO_Port->BSRRL = DET_MUX_EN_Pin;
			break;

		case 7:			// ELEC7 = EXC_MUX_S7 = DET_MUX_S7
			DET_MUX_A2_GPIO_Port->BSRRL = DET_MUX_A2_Pin;
			DET_MUX_A1_GPIO_Port->BSRRL = DET_MUX_A1_Pin;
			DET_MUX_A0_GPIO_Port->BSRRH = DET_MUX_A0_Pin;
			DET_MUX_EN_GPIO_Port->BSRRL = DET_MUX_EN_Pin;
			break;

		case 8:			// ELEC8 = EXC_MUX_S8 = DET_MUX_S8
			DET_MUX_A2_GPIO_Port->BSRRL = DET_MUX_A2_Pin;
			DET_MUX_A1_GPIO_Port->BSRRL = DET_MUX_A1_Pin;
			DET_MUX_A0_GPIO_Port->BSRRL = DET_MUX_A0_Pin;
			DET_MUX_EN_GPIO_Port->BSRRL = DET_MUX_EN_Pin;
			break;
		}
		break;
	}
}

void SPST_Ctrl(uint8_t sw_part, uint8_t sw_num, uint8_t elec_num, uint8_t elec_op, uint8_t *Tsw_Status)
{
	/*
	 * [ADG5401 Pin-Map]
	 * <Excitation Part MCU GPIO Pin-Map>
	 * EXC_SWE1 = PF6		/		EXC_SWG1 = PF5
	 * EXC_SWE2 = PF8		/		EXC_SWG2 = PF7
	 * EXC_SWE3 = PF10		/		EXC_SWG3 = PF9
	 * EXC_SWE4 = PC2		/		EXC_SWG4 = PC0
	 * EXC_SWE5 = PE9		/		EXC_SWG5 = PG0
	 * EXC_SWE6 = PE8		/		EXC_SWG6 = PF12
	 * EXC_SWE7 = PE7		/		EXC_SWG7 = PF11
	 * EXC_SWE8 = PG1		/		EXC_SWG8 = PC3
	 *
	 * <Detection Part MCU GPIO Pin-Map>
	 * DET_SWE1 = PE1		/		DET_SWG1 = PE0
	 * DET_SWE2 = PB9		/		DET_SWG2 = PB8
	 * DET_SWE3 = PB7		/		DET_SWG3 = PB6
	 * DET_SWE4 = PB5		/		DET_SWG4 = PB4
	 * DET_SWE5 = PG9		/		DET_SWG5 = PG13
	 * DET_SWE6 = PD7		/		DET_SWG6 = PG12
	 * DET_SWE7 = PG11		/		DET_SWG7 = PG15
	 * DET_SWE8 = PG10		/		DET_SWG8 = PG14
	 */
	// Send Individual SW Control Bit to ADG5401
	switch (sw_part)
	{
	case EXC_SPST:
		switch (sw_num)
		{
		case SPST_SWE:
			switch (elec_op)
			{
			case SPST_OFF:
				switch (elec_num)
				{
				case 1:
					Tsw_Status[2] = Tsw_Status[2] & 0xFE;
					EXC_SWE1_GPIO_Port->BSRRH = EXC_SWE1_Pin;
					break;	//ELEC1 = EXC_SWE1

				case 2:
					Tsw_Status[2] = Tsw_Status[2] & 0xFD;
					EXC_SWE2_GPIO_Port->BSRRH = EXC_SWE2_Pin;
					break;	//ELEC2 = EXC_SWE2

				case 3:
					Tsw_Status[2] = Tsw_Status[2] & 0xFB;
					EXC_SWE3_GPIO_Port->BSRRH = EXC_SWE3_Pin;
					break;  //ELEC3 = EXC_SWE3

				case 4:
					Tsw_Status[2] = Tsw_Status[2] & 0xF7;
					EXC_SWE4_GPIO_Port->BSRRH = EXC_SWE4_Pin;
					break;	//ELEC4 = EXC_SWE4

				case 5:
					Tsw_Status[2] = Tsw_Status[2] & 0xEF;
					EXC_SWE5_GPIO_Port->BSRRH = EXC_SWE5_Pin;
					break;	//ELEC5 = EXC_SWE5

				case 6:
					Tsw_Status[2] = Tsw_Status[2] & 0xDF;
					EXC_SWE6_GPIO_Port->BSRRH = EXC_SWE6_Pin;
					break;	//ELEC6 = EXC_SWE6

				case 7:
					Tsw_Status[2] = Tsw_Status[2] & 0xBF;
					EXC_SWE7_GPIO_Port->BSRRH = EXC_SWE7_Pin;
					break;	//ELEC7 = EXC_SWE7

				case 8:
					Tsw_Status[2] = Tsw_Status[2] & 0x7F;
					EXC_SWE8_GPIO_Port->BSRRH = EXC_SWE8_Pin;
					break;	//ELEC8 = EXC_SWE8
				}
				break;
			case SPST_ON:
				switch (elec_num)
				{
				case 1:
					Tsw_Status[2] = Tsw_Status[2] | 0x01;
					EXC_SWE1_GPIO_Port->BSRRL = EXC_SWE1_Pin;
					break;	//ELEC1 = EXC_SWE1

				case 2:
					Tsw_Status[2] = Tsw_Status[2] | 0x02;
					EXC_SWE2_GPIO_Port->BSRRL = EXC_SWE2_Pin;
					break;	//ELEC2 = EXC_SWE2

				case 3:
					Tsw_Status[2] = Tsw_Status[2] | 0x04;
					EXC_SWE3_GPIO_Port->BSRRL = EXC_SWE3_Pin;
					break;	//ELEC3 = EXC_SWE3

				case 4:
					Tsw_Status[2] = Tsw_Status[2] | 0x08;
					EXC_SWE4_GPIO_Port->BSRRL = EXC_SWE4_Pin;
					break;	//ELEC4 = EXC_SWE4

				case 5:
					Tsw_Status[2] = Tsw_Status[2] | 0x10;
					EXC_SWE5_GPIO_Port->BSRRL = EXC_SWE5_Pin;
					break;	//ELEC5 = EXC_SWE5

				case 6:
					Tsw_Status[2] = Tsw_Status[2] | 0x20;
					EXC_SWE6_GPIO_Port->BSRRL = EXC_SWE6_Pin;
					break;	//ELEC6 = EXC_SWE6

				case 7:
					Tsw_Status[2] = Tsw_Status[2] | 0x40;
					EXC_SWE7_GPIO_Port->BSRRL = EXC_SWE7_Pin;
					break;	//ELEC7 = EXC_SWE7

				case 8:
					Tsw_Status[2] = Tsw_Status[2] | 0x80;
					EXC_SWE8_GPIO_Port->BSRRL = EXC_SWE8_Pin;
					break;	//ELEC8 = EXC_SWE8
				}
				break;
			}
			break;
		case SPST_SWG:
			switch (elec_op)
			{
			case SPST_OFF:
				switch (elec_num)
				{
				case 1:
					Tsw_Status[3] = Tsw_Status[3] & 0xFE;
					EXC_SWG1_GPIO_Port->BSRRH = EXC_SWG1_Pin;
					break;	//ELEC1 = EXC_SWG1

				case 2:
					Tsw_Status[3] = Tsw_Status[3] & 0xFD;
					EXC_SWG2_GPIO_Port->BSRRH = EXC_SWG2_Pin;
					break;	//ELEC2 = EXC_SWG2

				case 3:
					Tsw_Status[3] = Tsw_Status[3] & 0xFB;
					EXC_SWG3_GPIO_Port->BSRRH = EXC_SWG3_Pin;
					break;	//ELEC3 = EXC_SWG3

				case 4:
					Tsw_Status[3] = Tsw_Status[3] & 0xF7;
					EXC_SWG4_GPIO_Port->BSRRH = EXC_SWG4_Pin;
					break;	//ELEC4 = EXC_SWG4

				case 5:
					Tsw_Status[3] = Tsw_Status[3] & 0xEF;
					EXC_SWG5_GPIO_Port->BSRRH = EXC_SWG5_Pin;
					break;	//ELEC5 = EXC_SWG5

				case 6:
					Tsw_Status[3] = Tsw_Status[3] & 0xDF;
					EXC_SWG6_GPIO_Port->BSRRH = EXC_SWG6_Pin;
					break;	//ELEC6 = EXC_SWG6

				case 7:
					Tsw_Status[3] = Tsw_Status[3] & 0xBF;
					EXC_SWG7_GPIO_Port->BSRRH = EXC_SWG7_Pin;
					break;	//ELEC7 = EXC_SWG7

				case 8:
					Tsw_Status[3] = Tsw_Status[3] & 0x7F;
					EXC_SWG8_GPIO_Port->BSRRH = EXC_SWG8_Pin;
					break;	//ELEC8 = EXC_SWG8
				}
				break;
			case SPST_ON:
				switch (elec_num)
				{
				case 1:
					Tsw_Status[3] = Tsw_Status[3] | 0x01;
					EXC_SWG1_GPIO_Port->BSRRL = EXC_SWG1_Pin;
					break;	//ELEC1 = EXC_SWG1

				case 2:
					Tsw_Status[3] = Tsw_Status[3] | 0x02;
					EXC_SWG2_GPIO_Port->BSRRL = EXC_SWG2_Pin;
					break;	//ELEC2 = EXC_SWG2

				case 3:
					Tsw_Status[3] = Tsw_Status[3] | 0x04;
					EXC_SWG3_GPIO_Port->BSRRL = EXC_SWG3_Pin;
					break;	//ELEC3 = EXC_SWG3

				case 4:
					Tsw_Status[3] = Tsw_Status[3] | 0x08;
					EXC_SWG4_GPIO_Port->BSRRL = EXC_SWG4_Pin;
					break;	//ELEC4 = EXC_SWG4

				case 5:
					Tsw_Status[3] = Tsw_Status[3] | 0x10;
					EXC_SWG5_GPIO_Port->BSRRL = EXC_SWG5_Pin;
					break;	//ELEC5 = EXC_SWG5

				case 6:
					Tsw_Status[3] = Tsw_Status[3] | 0x20;
					EXC_SWG6_GPIO_Port->BSRRL = EXC_SWG6_Pin;
					break;	//ELEC6 = EXC_SWG6

				case 7:
					Tsw_Status[3] = Tsw_Status[3] | 0x40;
					EXC_SWG7_GPIO_Port->BSRRL = EXC_SWG7_Pin;
					break;	//ELEC7 = EXC_SWG7

				case 8:
					Tsw_Status[3] = Tsw_Status[3] | 0x80;
					EXC_SWG8_GPIO_Port->BSRRL = EXC_SWG8_Pin;
					break;	//ELEC8 = EXC_SWG8
				}
				break;
			}
			break;
		}
		break;

	case DET_SPST:
		switch (sw_num)
		{
		case SPST_SWE:
			switch (elec_op)
			{
			case SPST_OFF:
				switch (elec_num)
				{
				case 1:
					Tsw_Status[4] = Tsw_Status[4] & 0xFE;
					DET_SWE1_GPIO_Port->BSRRH = DET_SWE1_Pin;
					break;    //ELEC1 = DET_SWE1

				case 2:
					Tsw_Status[4] = Tsw_Status[4] & 0xFD;
					DET_SWE2_GPIO_Port->BSRRH = DET_SWE2_Pin;
					break;    //ELEC2 = DET_SWE2

				case 3:
					Tsw_Status[4] = Tsw_Status[4] & 0xFB;
					DET_SWE3_GPIO_Port->BSRRH = DET_SWE3_Pin;
					break;    //ELEC3 = DET_SWE3

				case 4:
					Tsw_Status[4] = Tsw_Status[4] & 0xF7;
					DET_SWE4_GPIO_Port->BSRRH = DET_SWE4_Pin;
					break;    //ELEC4 = DET_SWE4

				case 5:
					Tsw_Status[4] = Tsw_Status[4] & 0xEF;
					DET_SWE5_GPIO_Port->BSRRH = DET_SWE5_Pin;
					break;    //ELEC5 = DET_SWE5

				case 6:
					Tsw_Status[4] = Tsw_Status[4] & 0xDF;
					DET_SWE6_GPIO_Port->BSRRH = DET_SWE6_Pin;
					break;    //ELEC6 = DET_SWE6

				case 7:
					Tsw_Status[4] = Tsw_Status[4] & 0xBF;
					DET_SWE7_GPIO_Port->BSRRH = DET_SWE7_Pin;
					break;    //ELEC7 = DET_SWE7

				case 8:
					Tsw_Status[4] = Tsw_Status[4] & 0x7F;
					DET_SWE8_GPIO_Port->BSRRH = DET_SWE8_Pin;
					break;    //ELEC8 = DET_SWE8
				}
				break;
			case SPST_ON:
				switch (elec_num)
				{
				case 1:
					Tsw_Status[4] = Tsw_Status[4] | 0x01;
					DET_SWE1_GPIO_Port->BSRRL = DET_SWE1_Pin;
					break;    //ELEC1 = DET_SWE1

				case 2:
					Tsw_Status[4] = Tsw_Status[4] | 0x02;
					DET_SWE2_GPIO_Port->BSRRL = DET_SWE2_Pin;
					break;    //ELEC2 = DET_SWE2

				case 3:
					Tsw_Status[4] = Tsw_Status[4] | 0x04;
					DET_SWE3_GPIO_Port->BSRRL = DET_SWE3_Pin;
					break;    //ELEC3 = DET_SWE3

				case 4:
					Tsw_Status[4] = Tsw_Status[4] | 0x08;
					DET_SWE4_GPIO_Port->BSRRL = DET_SWE4_Pin;
					break;    //ELEC4 = DET_SWE4

				case 5:
					Tsw_Status[4] = Tsw_Status[4] | 0x10;
					DET_SWE5_GPIO_Port->BSRRL = DET_SWE5_Pin;
					break;    //ELEC5 = DET_SWE5

				case 6:
					Tsw_Status[4] = Tsw_Status[4] | 0x20;
					DET_SWE6_GPIO_Port->BSRRL = DET_SWE6_Pin;
					break;    //ELEC6 = DET_SWE6

				case 7:
					Tsw_Status[4] = Tsw_Status[4] | 0x40;
					DET_SWE7_GPIO_Port->BSRRL = DET_SWE7_Pin;
					break;    //ELEC7 = DET_SWE7

				case 8:
					Tsw_Status[4] = Tsw_Status[4] | 0x80;
					DET_SWE8_GPIO_Port->BSRRL = DET_SWE8_Pin;
					break;    //ELEC8 = DET_SWE8
				}
				break;
			}
			break;
		case SPST_SWG:
			switch (elec_op)
			{
			case SPST_OFF:
				switch (elec_num)
				{
				case 1:
					Tsw_Status[5] = Tsw_Status[5] & 0xFE;
					DET_SWG1_GPIO_Port->BSRRH = DET_SWG1_Pin;
					break;	//ELEC1 = DET_SWG1

				case 2:
					Tsw_Status[5] = Tsw_Status[5] & 0xFD;
					DET_SWG2_GPIO_Port->BSRRH = DET_SWG2_Pin;
					break;	//ELEC2 = DET_SWG2

				case 3:
					Tsw_Status[5] = Tsw_Status[5] & 0xFB;
					DET_SWG3_GPIO_Port->BSRRH = DET_SWG3_Pin;
					break;	//ELEC3 = DET_SWG3

				case 4:
					Tsw_Status[5] = Tsw_Status[5] & 0xF7;
					DET_SWG4_GPIO_Port->BSRRH = DET_SWG4_Pin;
					break;	//ELEC4 = DET_SWG4

				case 5:
					Tsw_Status[5] = Tsw_Status[5] & 0xEF;
					DET_SWG5_GPIO_Port->BSRRH = DET_SWG5_Pin;
					break;	//ELEC5 = DET_SWG5

				case 6:
					Tsw_Status[5] = Tsw_Status[5] & 0xDF;
					DET_SWG6_GPIO_Port->BSRRH = DET_SWG6_Pin;
					break;	//ELEC6 = DET_SWG6

				case 7:
					Tsw_Status[5] = Tsw_Status[5] & 0xBF;
					DET_SWG7_GPIO_Port->BSRRH = DET_SWG7_Pin;
					break;	//ELEC7 = DET_SWG7

				case 8:
					Tsw_Status[5] = Tsw_Status[5] & 0x7F;
					DET_SWG8_GPIO_Port->BSRRH = DET_SWG8_Pin;
					break;	//ELEC8 = DET_SWG8
				}
				break;
			case SPST_ON:
				switch (elec_num)
				{
				case 1:
					Tsw_Status[5] = Tsw_Status[5] | 0x01;
					DET_SWG1_GPIO_Port->BSRRL = DET_SWG1_Pin;
					break;	//ELEC1 = DET_SWG1

				case 2:
					Tsw_Status[5] = Tsw_Status[5] | 0x02;
					DET_SWG2_GPIO_Port->BSRRL = DET_SWG2_Pin;
					break;	//ELEC2 = DET_SWG2

				case 3:
					Tsw_Status[5] = Tsw_Status[5] | 0x04;
					DET_SWG3_GPIO_Port->BSRRL = DET_SWG3_Pin;
					break;	//ELEC3 = DET_SWG3

				case 4:
					Tsw_Status[5] = Tsw_Status[5] | 0x08;
					DET_SWG4_GPIO_Port->BSRRL = DET_SWG4_Pin;
					break;	//ELEC4 = DET_SWG4

				case 5:
					Tsw_Status[5] = Tsw_Status[5] | 0x10;
					DET_SWG5_GPIO_Port->BSRRL = DET_SWG5_Pin;
					break;	//ELEC5 = DET_SWG5

				case 6:
					Tsw_Status[5] = Tsw_Status[5] | 0x20;
					DET_SWG6_GPIO_Port->BSRRL = DET_SWG6_Pin;
					break;	//ELEC6 = DET_SWG6

				case 7:
					Tsw_Status[5] = Tsw_Status[5] | 0x40;
					DET_SWG7_GPIO_Port->BSRRL = DET_SWG7_Pin;
					break;	//ELEC7 = DET_SWG7

				case 8:
					Tsw_Status[5] = Tsw_Status[5] | 0x80;
					DET_SWG8_GPIO_Port->BSRRL = DET_SWG8_Pin;
					break;	//ELEC8 = DET_SWG8
				}
				break;
			}
			break;
		}
		break;
	}
}

void Exc_Tsw_Ctrl(uint8_t EXC_ELEC, uint8_t *Tsw_Status)
{
	if ((EXC_ELEC != Tsw_Status[0]) && (EXC_ELEC != Tsw_Status[1]))
	{
		if (Tsw_Status[0] != 0)
		{
			// Make Old Electrode to GND
			MUX_Ctrl(EXC_MUX, 0);													// Disable Excitation MUX
			SPST_Ctrl(EXC_SPST, SPST_SWG, Tsw_Status[0], SPST_ON, Tsw_Status);		// Turn EXC_SWG ON
			SPST_Ctrl(DET_SPST, SPST_SWE, Tsw_Status[0], SPST_ON, Tsw_Status);		// Turn DET_SWE ON
			Tsw_Status[0] = 0;														// Initialize Excitation Electrode Flag
		}

		// Turn DET_SWE OFF
		SPST_Ctrl(DET_SPST, SPST_SWE, EXC_ELEC, SPST_OFF, Tsw_Status);

		// Turn EXC_SWG OFF
		SPST_Ctrl(EXC_SPST, SPST_SWG, EXC_ELEC, SPST_OFF, Tsw_Status);

		// Set MUX Connection
		MUX_Ctrl(EXC_MUX, EXC_ELEC);

		// Update New Excitation Electrode Flag
		Tsw_Status[0] = EXC_ELEC;
	}
}

void Det_Tsw_Ctrl(uint8_t DET_ELEC, uint8_t *Tsw_Status)
{
	if ((DET_ELEC != Tsw_Status[0]) && (DET_ELEC != Tsw_Status[1]))
	{
		if (Tsw_Status[1] != 0)
		{
			// Make Old Electrode to GND
			MUX_Ctrl(DET_MUX, 0);													// Disable Detection MUX
			SPST_Ctrl(DET_SPST, SPST_SWG, Tsw_Status[1], SPST_ON, Tsw_Status);		// Turn DET_SWG ON
			SPST_Ctrl(EXC_SPST, SPST_SWE, Tsw_Status[1], SPST_ON, Tsw_Status);		// Turn EXC_SWE ON
			Tsw_Status[1] = 0;														// Initialize Excitation Electrode Flag
		}

		// Turn EXC_SWE OFF
		SPST_Ctrl(EXC_SPST, SPST_SWE, DET_ELEC, SPST_OFF, Tsw_Status);

		// Turn DET_SWG OFF
		SPST_Ctrl(DET_SPST, SPST_SWG, DET_ELEC, SPST_OFF, Tsw_Status);

		// Set MUX Connection
		MUX_Ctrl(DET_MUX, DET_ELEC);

		// Update New Excitation Electrode Flag
		Tsw_Status[1] = DET_ELEC;
	}
}

void Tsw_Reset(uint8_t *Tsw_Status)
{
	// Disable MUX
	MUX_Ctrl(EXC_MUX, 0);	// EXC Part
	MUX_Ctrl(DET_MUX, 0);	// DET Part

	// Turn All SWG ON (Connect All Intermediate Signal Path to GND)
	for (uint8_t cnt = 1; cnt <= 8; cnt++)
	{
		SPST_Ctrl(DET_SPST, SPST_SWG, cnt, SPST_ON, Tsw_Status);	// DET Part, SWG, All SW, ON
	}

	for (uint8_t cnt = 1; cnt <= 8; cnt++)
	{
		SPST_Ctrl(EXC_SPST, SPST_SWG, cnt, SPST_ON, Tsw_Status);	// EXC Part, SWG, All SW, ON
	}

	// Turn ALL SWE ON (Connect All Electrode to GND)
	for (uint8_t cnt = 1; cnt <= 8; cnt++)
	{
		SPST_Ctrl(DET_SPST, SPST_SWE, cnt, SPST_ON, Tsw_Status);	// DET Part, SWE, All SW, ON
	}

	for (uint8_t cnt = 1; cnt <= 8; cnt++)
	{
		SPST_Ctrl(EXC_SPST, SPST_SWE, cnt, SPST_ON, Tsw_Status);	// EXC Part, SWE, All SW, ON
	}

	// Initialize Electrode Status Flags
	Tsw_Status[0] = 0;
	Tsw_Status[1] = 0;
}
