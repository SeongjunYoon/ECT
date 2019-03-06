/* Includes ------------------------------------------------------------------*/
#include "custom_usb.h"

/* =============== Variables =============== */
uint8_t nack[1] = {0x00};
uint8_t ack[1] = {0x01};

/* =============== Functions =============== */
// USB CDC Text Data Transmission
void USB_ACK()
{
	CDC_Transmit_FS(ack, 1);
}

void USB_NACK()
{
	CDC_Transmit_FS(nack, 1);
}
