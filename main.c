/* XDCtools Header files */
#include <xdc/std.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driverlib/ssi.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"

#include "pid.h"
#include "utils/uartstdio.h"
#include "enc28j60.h"
#include "ip_arp_udp_tcp.h"

#define LED GPIO_PIN_1  // Red LED

// PWM
#define PWM_FREQUENCY 2000  // 0.5 ms period

// ADC
#define ADCSequencer 3  // One measurement
uint32_t ui32ADC0Value;

// PID
PIDdata PIDdataLightFlow;
volatile float Goal = 3413.0f;
volatile float ValueToPWM;

// IP/UDP-stack
uint8_t mymac[6] = {0x54,0x55,0x56,0x57,0x58,0x59};
uint8_t myip[4] = {192,168,1,102};
#define MYUDPPORT 1200
#define BUFFER_SIZE 100
#define STR_SIZE 50


/*
 *  ======== main ========
 */
void main(void) {
    /* Start BIOS */
    BIOS_start();
}


void initTask(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    FPUEnable();

//    // LED
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED);
//    GPIOPinWrite(GPIO_PORTF_BASE, LED, 0);

	// PWM configuration (PD0)
	SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)) {}
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMOutputInvert(PWM1_BASE, PWM_OUT_0_BIT, true);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, (((SysCtlClockGet()/128)/PWM_FREQUENCY)*2)-1);

	// ADC configuration (PE3)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ADCHardwareOversampleConfigure(ADC0_BASE, 64);
	ADCSequenceConfigure(ADC0_BASE, ADCSequencer, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, ADCSequencer, 0, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, ADCSequencer);
	// ADC interrupt
	ADCIntDisable(ADC0_BASE, ADCSequencer);
	ADCIntClear(ADC0_BASE, ADCSequencer);
	ADCIntEnable(ADC0_BASE, ADCSequencer);

	// SPI
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);  // BUT!!! CS for our project is PA6 (see enc28j60.c)
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
			GPIO_PIN_2);
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet()/2, SSI_FRF_MOTO_MODE_0,
			SSI_MODE_MASTER, 5000000, 8);
	SSIEnable(SSI0_BASE);

	enc28j60Init(mymac);
//    enc28j60clkout(2); // Change clkout from 6.25 MHz to 12.5 MHz
	enc28j60PhyWrite(PHLCON, 0x476);  // LED mode

	// PID configuration
    PID_Init(&PIDdataLightFlow);
	PID_SetPID(&PIDdataLightFlow, 1.3f, 15.0f, 0.0015f);

	// UART configuration
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTStdioConfig(0, 115200, SysCtlClockGet());
	UARTprintf("TM4C123GH6PM initialized. Clock frequency: %d Hz\n", SysCtlClockGet());
	UARTprintf("ENC28J660 initialized. Revision: %d\n", enc28j60getrev());
//	UARTprintf("PWM clock: %d\n", SysCtlPWMClockGet());

	// Enable UDP-server, PWM & global interrupts
	init_udp_or_www_server(mymac, myip);
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	IntMasterEnable();
}


void UDPserverTask(void) {

    char str[STR_SIZE], ErrMIN[10], ErrMAX[10];
	uint16_t plen;  // , dat_p;
	uint8_t buf[BUFFER_SIZE+1];

	while (1) {
		// Receive message via Ethernet
        plen = enc28j60PacketReceive(BUFFER_SIZE, buf);

        // Process ping request
        packetloop_arp_icmp_tcp(buf, plen);

        // If protocol is IP and IP-address is mine...
        if ( eth_type_is_ip_and_my_ip(buf, plen) != 0 ) {
        	// If protocol is UDP and UDP-port matched...
            if (buf[IP_PROTO_P]==IP_PROTO_UDP_V && buf[UDP_DST_PORT_H_P]==(MYUDPPORT>>8) &&
            		buf[UDP_DST_PORT_L_P]==(MYUDPPORT&0xff)) {

            	// Commands to read
            	if ( strncmp("u", (char *)&(buf[UDP_DATA_P]), 1) == 0 ) {
            		snprintf(str, STR_SIZE, "%d", ui32ADC0Value);
            		make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
            	}
            	else if ( strncmp("p", (char *)&(buf[UDP_DATA_P]), 1) == 0 ) {
            		snprintf(str, STR_SIZE, "%f", ValueToPWM);
            		make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
            	}
            	else if ( strncmp("GoalRead", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
            		snprintf(str, STR_SIZE, "%f", (Goal/4095.0f)*3.3f);
            		make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
            	}
            	else if ( strncmp("KpRead", (char *)&(buf[UDP_DATA_P]), 6) == 0 ) {
            		snprintf(str, STR_SIZE, "%f", PIDdataLightFlow.Kp);
            		make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
            	}
            	else if ( strncmp("KiRead", (char *)&(buf[UDP_DATA_P]), 6) == 0 ) {
            		snprintf(str, STR_SIZE, "%f", PIDdataLightFlow.Ki);
           			make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
           		}
           		else if ( strncmp("KdRead", (char *)&(buf[UDP_DATA_P]), 6) == 0 ) {
           			snprintf(str, STR_SIZE, "%f", PIDdataLightFlow.Kd);
           			make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
           		}

            	// Commands to write
            	if ( strncmp("GoalWrite", (char *)&(buf[UDP_DATA_P]), 9) == 0 ) {
            		// New value located in string, starts from 10th symbol and has length in 8 symbols:
            		strncpy(str, (char *)&(buf[UDP_DATA_P+10]), 8);
            		Goal = (atof(str)/3.3f)*4095.0f;
            	}
            	else if ( strncmp("KpWrite", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
            		strncpy(str, (char *)&(buf[UDP_DATA_P+8]), 8);
            		PIDdataLightFlow.Kp = atof(str);
            	}
            	else if ( strncmp("KiWrite", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
            		strncpy(str, (char *)&(buf[UDP_DATA_P+8]), 8);
            		PIDdataLightFlow.Ki = atof(str);
            	}
            	else if ( strncmp("KdWrite", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
            		strncpy(str, (char *)&(buf[UDP_DATA_P+8]), 8);
            		PIDdataLightFlow.Kd = atof(str);
            	}

            	// Commands to control values of PID errors
            	if ( strncmp("PerrLIMr", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
            		snprintf(str, STR_SIZE, "%16.3f %16.3f", PIDdataLightFlow.Perrmin, PIDdataLightFlow.Perrmax);
            		make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
            	}
            	else if ( strncmp("IerrLIMr", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
            		snprintf(str, STR_SIZE, "%16.3f %16.3f", PIDdataLightFlow.Ierrmin, PIDdataLightFlow.Ierrmax);
            		make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
            	}
            	else if ( strncmp("IerrRead", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
            		snprintf(str, STR_SIZE, "%f", PIDdataLightFlow.Ierr);
            		make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
            	}
            	else if ( strncmp("IerrRST", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
            		PID_ResetIerr(&PIDdataLightFlow);
            	}
            	else if ( strncmp("PerrLIMw", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
            		strncpy(ErrMIN, (char *)&(buf[UDP_DATA_P+9]), 16);
            		strncpy(ErrMAX, (char *)&(buf[UDP_DATA_P+9+16+1]), 16);
            		PID_SetLimitsPerr(&PIDdataLightFlow, atof(ErrMIN), atof(ErrMAX));
            	}
            	else if ( strncmp("IerrLIMw", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
            		strncpy(ErrMIN, (char *)&(buf[UDP_DATA_P+9]), 16);
            		strncpy(ErrMAX, (char *)&(buf[UDP_DATA_P+9+16+1]), 16);
            		PID_SetLimitsIerr(&PIDdataLightFlow, atof(ErrMIN), atof(ErrMAX));
            	}

            }
        }
	}

}


void ADCTimer(void) {
	ADCProcessorTrigger(ADC0_BASE, ADCSequencer);
}


void ADCHwi(void) {
	ADCIntClear(ADC0_BASE, ADCSequencer);
	ADCSequenceDataGet(ADC0_BASE, ADCSequencer, &ui32ADC0Value);

	ValueToPWM = PID_Update(&PIDdataLightFlow, Goal, (float)ui32ADC0Value);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, (uint32_t)((SysCtlClockGet()/16/PWM_FREQUENCY)*(ValueToPWM)/4095));
}
