/*
 * Add this in predefined symbols (project properties) for ROM_ functions
 *
       #define TARGET_IS_TM4C123_RB2

 *
 * Use DID0 and DID1 registers to get device-specific info (such as silicon revision):
 *
       HWREG(SYSCTL_DID0);
*/

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdbool.h>
#include <stdint.h>
#include <file.h>
#include <string.h>
#include <stdio.h>

#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/eeprom.h"
#include "driverlib/rom.h"

//#include "driverlib/uart.h"
//#include "uartstdio.h"

//#define RELEASE_VERSION
#ifdef RELEASE_VERSION
#include "driverlib/uart.h"
#include "Board.h"
#include "UARTUtils.h"
void consoleTask(void);
#endif

#include "pid.h"

#include "enc28j60.h"
#include "ip_arp_udp_tcp.h"

//#define LED GPIO_PIN_1  // Debug Red LED (PF1)

// PWM
//#define PWM_FREQUENCY 8000  // Hz (0.125ms period)
#define PWM_FREQUENCY 2000  // magic
//volatile uint32_t ui32PWMLoad;
//volatile uint32_t ui32PWMClock;

// ADC
#define ADCSequencer 3  // One measurement
uint32_t ui32ADC0Value;

// PID
#define WRITE_TO_EEPROM
#ifdef WRITE_TO_EEPROM
PIDdata PIDdataMagLev_EEPROM;
#endif
#define PIDdata_EEPROM_ADDR 0x0
PIDdata PIDdataMagLev;
volatile float ValueToPWM;

// IP/UDP-stack
#define IP_EEPROM_ADDR 0x50
#define PORT_EEPROM_ADDR 0x100
#ifdef WRITE_TO_EEPROM
uint8_t myip_EEPROM[4] = {192,168,0,110};
uint32_t udpport_EEPROM = 1200;
#endif
uint32_t udpport;
uint8_t myip[4];
#define BUFFER_SIZE 100
#define STR_SIZE 50
bool offline_mode = false;

// Prototypes for dynamic creation
void UDPserverTask(void);
void initTask(void);


/*
 *  ======== main ========
 */
void main(void) {
    Task_Params initTask_params;
    Task_Params_init(&initTask_params);
    initTask_params.priority = 15;
    initTask_params.stackSize = 1024;
    initTask_params.vitalTaskFlag = false;
    Task_create((Task_FuncPtr)initTask, &initTask_params, NULL);

    /*
     * Use this for checking task existence (see Task_Mode enum for information):
     *
       Task_Handle initTask_handle;
       initTask_handle = Task_create((Task_FuncPtr)initTask, &initTask_params, NULL);
       Task_Stat initTask_stat;
       Task_stat(initTask_handle, &initTask_stat);
       System_printf("Init task mode: %d\n", initTask_stat.mode);
       System_flush();
    */

    /* Start BIOS */
    BIOS_start();
}


void initTask(void) {
    ROM_IntMasterDisable();

    // 40 MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // UART configuration
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
//    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
//    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
//    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
//    UARTStdioConfig(0, 115200, ROM_SysCtlClockGet());

#ifdef RELEASE_VERSION
    /*
     *  Add the UART device to the system (speed: 9600, length: 8 bits, stop-bit: 1, parity: none)
     *  All UART peripherals must be setup and the module must be initialized
     *  before opening.  This is done by Board_initUART(). The functions used
     *  are implemented in UARTUtils.c.
     */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();
    add_device("UART", _MSA, UARTUtils_deviceopen,
               UARTUtils_deviceclose, UARTUtils_deviceread,
               UARTUtils_devicewrite, UARTUtils_devicelseek,
               UARTUtils_deviceunlink, UARTUtils_devicerename);
    // Open UART0 for writing to stdout and set buffer
    freopen("UART:0", "w", stdout);
    setvbuf(stdout, NULL, _IOLBF, 128);
    // Open UART0 for reading from stdin and set buffer
    freopen("UART:0", "r", stdin);
    setvbuf(stdin, NULL, _IOLBF, 128);
    /*
     *  Initialize UART port 0 used by SysCallback.  This and other SysCallback
     *  UART functions are implemented in UARTUtils.c. Calls to System_printf()
     *  will go to UART0, the same as printf().
     */
    UARTUtils_systemInit(0);
#endif

    ROM_FPUEnable();

//    // PID configuration
//    PID_Init(&PIDdataMagLev);
    // EEPROM block size may differ depends on device
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    ROM_EEPROMInit();
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0)) {}
#ifdef WRITE_TO_EEPROM
    ROM_EEPROMMassErase();
    PIDdata PIDdataMagLev_EEPROM;
    PID_Init(&PIDdataMagLev_EEPROM);
    PID_SetPID(&PIDdataMagLev_EEPROM, 3413.0f, 1.3f, 15.0f, 0.0015f);  // 2.70 V
    ROM_EEPROMProgram((uint32_t *)&PIDdataMagLev_EEPROM, PIDdata_EEPROM_ADDR, sizeof(PIDdata));

    ROM_EEPROMProgram((uint32_t *)&myip_EEPROM, IP_EEPROM_ADDR, sizeof(myip_EEPROM));
    ROM_EEPROMProgram((uint32_t *)&udpport_EEPROM, PORT_EEPROM_ADDR, sizeof(udpport_EEPROM));
#endif
    ROM_EEPROMRead((uint32_t *)&PIDdataMagLev, PIDdata_EEPROM_ADDR, sizeof(PIDdata));
    ROM_EEPROMRead((uint32_t *)&myip, IP_EEPROM_ADDR, sizeof(myip));
    ROM_EEPROMRead((uint32_t *)&udpport, PORT_EEPROM_ADDR, sizeof(udpport));

    // LED (for any debug purposes)
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED);
//    ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED, LED);
//    ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED, 0);

    // PWM configuration (PD0) (electromagnet driver)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)) {}
//    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {}
    ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinConfigure(GPIO_PD0_M1PWM0);
    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);
    ROM_PWMOutputInvert(PWM1_BASE, PWM_OUT_0_BIT, true);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, (((SysCtlClockGet()/128)/PWM_FREQUENCY)*2)-1);
//    ui32PWMClock = ROM_SysCtlClockGet()/8;
//    ui32PWMLoad = (ui32PWMClock/PWM_FREQUENCY) - 1;
//    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32PWMLoad);

    // ADC configuration (PE3) (Hall sensor)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}
    ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 64);
    ROM_ADCSequenceConfigure(ADC0_BASE, ADCSequencer, ADC_TRIGGER_PROCESSOR, 0);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, ADCSequencer, 0, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);
    ROM_ADCSequenceEnable(ADC0_BASE, ADCSequencer);
    // ADC interrupt
    ROM_ADCIntDisable(ADC0_BASE, ADCSequencer);
    ROM_ADCIntClear(ADC0_BASE, ADCSequencer);
    ROM_ADCIntEnable(ADC0_BASE, ADCSequencer);

    System_printf("TM4C123GH6PM initialized. Clock frequency: %f MHz\n", (float)ROM_SysCtlClockGet()/1000000);
    System_flush();

    // SPI (communication with ENC28J60)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0)) {}
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    ROM_GPIOPinConfigure(GPIO_PA3_SSI0FSS);  // BUT!!! CS for our project is PA6 (see enc28j60.c)
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
    ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
    ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);
    ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2);
    ROM_SSIConfigSetExpClk(SSI0_BASE, ROM_SysCtlClockGet()/2, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 5000000, 8);
    ROM_SSIEnable(SSI0_BASE);

    uint8_t mymac[6] = {0x54,0x55,0x56,0x57,0x58,0x59};
    enc28j60Init(mymac);
    uint8_t enc28j60_rev = enc28j60getrev();
    if (enc28j60_rev) {
        System_printf("ENC28J660 initialized. Revision: %d\n", enc28j60_rev);
        System_flush();

        Task_Params UDPserver_params;
        Task_Params_init(&UDPserver_params);
        UDPserver_params.stackSize = 1024;
        UDPserver_params.vitalTaskFlag = true;
        Task_create((Task_FuncPtr)UDPserverTask, &UDPserver_params, NULL);

        init_udp_or_www_server(mymac, myip);
        System_printf("UDP server started. MAC: 0x%x.0x%x.0x%x.0x%x.0x%x.0x%x, IP/PORT: %d.%d.%d.%d/%d\n",
                      mymac[0], mymac[1], mymac[2], mymac[3], mymac[4], mymac[5],
                      myip[0], myip[1], myip[2], myip[3], udpport);
        System_flush();
    }
    else {
        offline_mode = true;
        System_printf("Offline mode\n");
        System_flush();
    }

#ifdef RELEASE_VERSION
    if (offline_mode) {
        System_printf("Current IP/UDPport pair is: %d.%d.%d.%d/%d. Send 'XXX.XXX.XXX.XXX/XXXXX' for entering new one (for example, 192.168.1.110/1200)\n",
                      myip[0], myip[1], myip[2], myip[3], udpport);

        Task_Params consoleTaskParams;
        Task_Params_init(&consoleTaskParams);
        consoleTaskParams.stackSize = 1536;
        consoleTaskParams.vitalTaskFlag = true;
        Task_create((Task_FuncPtr)consoleTask, &consoleTaskParams, NULL);
    }
#endif

    System_printf("PID controller started. Goal: %fV, Kp: %f, Ki: %f, Kd: %f\n",
                  PIDdataMagLev.setpoint*3.3/4095.0, PIDdataMagLev.Kp, PIDdataMagLev.Ki, PIDdataMagLev.Kd);
    System_flush();

    // Enable PWM & global interrupts
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    ROM_IntMasterEnable();
}


#ifdef RELEASE_VERSION
void consoleTask(void) {
    uint8_t ip[4] = {0,0,0,0};
    uint32_t port = 0;
    int input_result;

    // Loop forever receiving commands
    while (1) {
        printf("Input: ");
        // Get the user's input
        input_result = scanf("%u.%u.%u.%u/%u", &ip[0], &ip[1], &ip[2], &ip[3], &port);
        // Flush the remaining characters from stdin since they are not used
        fflush(stdin);

        if (input_result<5 || input_result==EOF) {
            printf("Wrong input! Try again\n");
        }
        else {
            printf("You entered: IP='%u.%u.%u.%u', PORT=%u\n", ip[0], ip[1], ip[2], ip[3], port);
            if ( !ROM_EEPROMProgram((uint32_t *)&ip, IP_EEPROM_ADDR, sizeof(ip)) &&
                 !ROM_EEPROMProgram((uint32_t *)&port, PORT_EEPROM_ADDR, sizeof(port)) )
                printf("saved\n");
            else
                printf("error while saving\n");
            ROM_EEPROMRead((uint32_t *)&myip, IP_EEPROM_ADDR, sizeof(myip));
            ROM_EEPROMRead((uint32_t *)&udpport, PORT_EEPROM_ADDR, sizeof(udpport));
            printf("Current values: IP='%u.%u.%u.%u', PORT=%u. Reboot MCU for server start\n", myip[0], myip[1], myip[2], myip[3], udpport);
        }
    }

}
#endif


void ADCTimer(void) {
    ROM_ADCProcessorTrigger(ADC0_BASE, ADCSequencer);
}


void ADCHwi(void) {
    ROM_ADCIntClear(ADC0_BASE, ADCSequencer);
    ROM_ADCSequenceDataGet(ADC0_BASE, ADCSequencer, &ui32ADC0Value);

    ValueToPWM = PID_Update(&PIDdataMagLev, (float)ui32ADC0Value);

//    static uint16_t cnt=0;
//    if (++cnt == 250) {
//        cnt = 0;
//        UARTprintf("pulse width: %d\n", ROM_PWMPulseWidthGet(PWM1_BASE, PWM_OUT_0));
//        UARTprintf("ADC: %d\n", ui32ADC0Value);
//    }

//    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, (uint32_t)(ui32PWMLoad*ValueToPWM/4095.0f));
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, (uint32_t)((SysCtlClockGet()/16/PWM_FREQUENCY)*(ValueToPWM)/4095));
//  uint32_t ui32NewPulseWidth;
//  ui32NewPulseWidth = (float)ui32Load*(1.0-(ValueToPWM/4095.0)/2.0);
//  UARTprintf("%d\n", ui32NewPulseWidth);
//  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, (uint32_t)((float)ui32Load*(ValueToPWM/4095.0)-1));
}


void UDPserverTask(void) {
    char str[STR_SIZE], ErrMIN[10], ErrMAX[10];
    uint16_t plen;
    uint8_t buf[BUFFER_SIZE+1];

    while (1) {
        // Receive message via Ethernet
        plen = enc28j60PacketReceive(BUFFER_SIZE, buf);

        // Process ping request
        packetloop_arp_icmp_tcp(buf, plen);

        // If protocol is IP and IP-address is mine...
        if ( eth_type_is_ip_and_my_ip(buf, plen) != 0 ) {
            // If protocol is UDP and UDP-port matched...
            if (buf[IP_PROTO_P]==IP_PROTO_UDP_V && buf[UDP_DST_PORT_H_P]==(udpport>>8) &&
                buf[UDP_DST_PORT_L_P]==(udpport&0xff)) {

                // Commands to read
                if ( strncmp("u", (char *)&(buf[UDP_DATA_P]), 1) == 0 ) {
                    snprintf(str, STR_SIZE, "%d", ui32ADC0Value);
                    make_udp_reply_from_request(buf, str, strlen(str), udpport);
                }
                else if ( strncmp("p", (char *)&(buf[UDP_DATA_P]), 1) == 0 ) {
                    snprintf(str, STR_SIZE, "%f", ValueToPWM);
                    make_udp_reply_from_request(buf, str, strlen(str), udpport);
                }
                else if ( strncmp("spRead", (char *)&(buf[UDP_DATA_P]), 6) == 0 ) {
                    snprintf(str, STR_SIZE, "%f", (PIDdataMagLev.setpoint/4095.0f)*3.3f);
                    make_udp_reply_from_request(buf, str, strlen(str), udpport);
                }
                else if ( strncmp("KpRead", (char *)&(buf[UDP_DATA_P]), 6) == 0 ) {
                    snprintf(str, STR_SIZE, "%f", PIDdataMagLev.Kp);
                    make_udp_reply_from_request(buf, str, strlen(str), udpport);
                }
                else if ( strncmp("KiRead", (char *)&(buf[UDP_DATA_P]), 6) == 0 ) {
                    snprintf(str, STR_SIZE, "%f", PIDdataMagLev.Ki);
                    make_udp_reply_from_request(buf, str, strlen(str), udpport);
                }
                else if ( strncmp("KdRead", (char *)&(buf[UDP_DATA_P]), 6) == 0 ) {
                    snprintf(str, STR_SIZE, "%f", PIDdataMagLev.Kd);
                    make_udp_reply_from_request(buf, str, strlen(str), udpport);
                }

                // Commands to write
                if ( strncmp("spWrite", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
                    // New value located in string, starts from 8th symbol and has length in 8 symbols:
                    strncpy(str, (char *)&(buf[UDP_DATA_P+8]), 8);
                    PIDdataMagLev.setpoint = (atof(str)/3.3f)*4095.0f;
                }
                else if ( strncmp("KpWrite", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
                    strncpy(str, (char *)&(buf[UDP_DATA_P+8]), 8);
                    PIDdataMagLev.Kp = atof(str);
                }
                else if ( strncmp("KiWrite", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
                    strncpy(str, (char *)&(buf[UDP_DATA_P+8]), 8);
                    PIDdataMagLev.Ki = atof(str);
                }
                else if ( strncmp("KdWrite", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
                    strncpy(str, (char *)&(buf[UDP_DATA_P+8]), 8);
                    PIDdataMagLev.Kd = atof(str);
                }

                // Commands to control values of PID errors
                if ( strncmp("PerrLIMr", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
                    snprintf(str, STR_SIZE, "%16.3f %16.3f", PIDdataMagLev.Perrmin, PIDdataMagLev.Perrmax);
                    make_udp_reply_from_request(buf, str, strlen(str), udpport);
                }
                else if ( strncmp("IerrLIMr", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
                    snprintf(str, STR_SIZE, "%16.3f %16.3f", PIDdataMagLev.Ierrmin, PIDdataMagLev.Ierrmax);
                    make_udp_reply_from_request(buf, str, strlen(str), udpport);
                }
                else if ( strncmp("IerrRead", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
                    snprintf(str, STR_SIZE, "%f", PIDdataMagLev.Ierr);
                    make_udp_reply_from_request(buf, str, strlen(str), udpport);
                }
                else if ( strncmp("IerrRST", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
                    PID_ResetIerr(&PIDdataMagLev);
                }
                else if ( strncmp("PerrLIMw", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
                    strncpy(ErrMIN, (char *)&(buf[UDP_DATA_P+9]), 16);
                    strncpy(ErrMAX, (char *)&(buf[UDP_DATA_P+9+16+1]), 16);
                    PID_SetLimitsPerr(&PIDdataMagLev, atof(ErrMIN), atof(ErrMAX));
                }
                else if ( strncmp("IerrLIMw", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
                    strncpy(ErrMIN, (char *)&(buf[UDP_DATA_P+9]), 16);
                    strncpy(ErrMAX, (char *)&(buf[UDP_DATA_P+9+16+1]), 16);
                    PID_SetLimitsIerr(&PIDdataMagLev, atof(ErrMIN), atof(ErrMAX));
                }

                // Save PID configuration to EEPROM
                if ( strncmp("SaveToEEPROM", (char *)&(buf[UDP_DATA_P]), 12) == 0 ) {
                    ROM_EEPROMMassErase();
                    if (!ROM_EEPROMProgram((uint32_t *)&PIDdataMagLev, PIDdata_EEPROM_ADDR, sizeof(PIDdata)))
                        snprintf(str, STR_SIZE, "success");
                    else
                        snprintf(str, STR_SIZE, "failure");
                    make_udp_reply_from_request(buf, str, strlen(str), udpport);
                }

            }
        }
    }

}
