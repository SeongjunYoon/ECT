#ifndef CUSTOM_USB_H_
#define CUSTOM_USB_H_

#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include "arm_math.h"

/* =============== Function Prototypes =============== */
void USB_ACK();
void USB_NACK();

#endif /* CUSTOM_USB_H_ */
