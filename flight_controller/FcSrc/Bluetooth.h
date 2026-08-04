#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__
#include "sysconfig.h"
#include "ANO_LX.h"


void Bluetooth_GetOneByte(uint8_t data);
static void Bluetooth_Data(uint8_t *data, uint8_t len);
void Bluetooth_speed_check(void);


#endif
