/*
 * delay_us.c
 *
 *  Created on: 28 apr. 2016
 *      Author: Andrey Chufyrev
 *
 *  Description: not accurate delay function
 */

#include "_delay_us.h"


void _delay_us(uint32_t us) {
	ROM_SysCtlDelay(SysCtlClockGet()/3000000*us);
}
