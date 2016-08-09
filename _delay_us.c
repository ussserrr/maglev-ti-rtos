/*
 * delay_us.c
 *
 *  Created on: 28 apr. 2016
 *      Author: Andrey Chufyrev
 */

#include "_delay_us.h"

void _delay_us(uint32_t us) {
	SysCtlDelay(SysCtlClockGet()/3000000*us);
}
