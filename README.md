# maglev-ti-rtos
Example of PID controller usage applied to magnetic levitation.

## Overview
Magnetic levitator on the base of Texas Instruments TIVA-C TM4C123GH6PM ARM Cortex-M4F MCU (EK-TM4C123GXL board). Regulation is performed by PID algorithm. Remote monitoring and control is provided by external Ethernet controller chip Microchip ENC28J60 (UDP commands).

## Software
### MCU firmware (server)
This version of maglev use TI-RTOS (v.2.16.01.14 at the moment) real-time operating system to manages all tasks:
  - `initTask` task. Performs initial setup of clocking system, GPIO, EEPROM, PWM, PID, ADC, SPI, UART, etc;
  - `consoleTask` task. Used for setting IP/Port pair through the UART interface;
  - `ADCTimer` timer. TI-RTOS' `Timer` instance firing every 1 ms and triggering ADC to start measurements;
  - `ADCHwi` hwi. TI-RTOS' `hwi` (hardware interrupt) instance managing getting ADC results and also calling PID mechanism;
  - `UDPserverTask` task. Vital task that constantly looking for UDP commands;

Generally, have been used the approach of dynamic creation/deletion of threads to reduce memory usage. For example, `initTask` is created before RTOS starts and then removed after doing its jobs.
For remote control have been used ported from AVR lightweight driver and network stack for ENC28J60.

### PC/Mac client
For remote control client see [maglev-client](https://github.com/ussserrr/maglev-client) (Python-based).

## Installation
Due to not always proper work of TI CCS (Code Composer Studio) (exporting/importing projects) this repository is just set of necessary files. It's recommended to create brand new clean TI-RTOS project and simply put all these files (except `.cfg` one) into it. You better manually set tasks and SYS/BIOS parameters.
There are 2 available build configurations: Debug and Release. Besides obvious compiler optimizations and all other jobs, Release and Debug configurations also differentiates in main code and `.cfg` script. Debug one uses semihosting mechanism because it's more handy to print all debug information in the same window while Release version route all strings to the UART (USB virtual COM port in case of EK board). Also, function of setting IP/Port in EEPROM is enabled only in Release because otherwise you can just edit code to change them. To switch between configurations use `RELEASE_VERSION` directive and see also "System configuration" part of `maglev.cfg` file.

## Known issues
Besides probably not ideal SW architecture :), there is a strange behavior of PWM module. All current values of registers indicate that levitation shouldn't actually work, but it does. See forward to understand this fact.

## Hardware
For schematic and PCB see [maglev-hardware](https://github.com/ussserrr/maglev-hardware) (KiCad EDA).

## Copyright
All third-party software components (ip_arp_udp_tcp stack, enc28j60 driver and so on) belongs to their authors.
