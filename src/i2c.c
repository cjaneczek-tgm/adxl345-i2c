/*
  Name of file  : i2c.c
  Author        : Christian Janeczek
  Version       : 2015-01-12
  Description   : Reading ADXL-sensor values via Inter-Integrated Circuit
                  One file to rule them all...
                  - Sometimes, UART is a b**ch
                  - For God so loved the world that he gave his one and only Son,
                    that whoever believes in him shall not perish but have eternal life.
*/
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_i2c.h"

#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"

#include "utils/uartstdio.h"

//  ADXL DOCUMENTATION PAGE 23
//  REGISTER MAP 

//	Page 18
#define SLAVE_ADRESS 0x53
#define ID 0x00
//	Page 23
#define P_MODE 0x3f
//	Config on page 25
#define P_SETTINGS 0x2d
//	Adresses for raw data are located at page 23
#define X_AXIS 0
#define DATA_X0 0x32
#define DATA_X1 0x33

#define Y_AXIS 1
#define DATA_Y0 0x34
#define DATA_Y1 0x35

#define Z_AXIS 2
#define DATA_Z0 0x36
#define DATA_Z1 0x37

//	Declaration of the prototypes
void InitI2C(void);
void InitConsole(void);
int32_t shiftProcess(int8_t, int8_t);
int8_t readProcess(uint32_t, uint32_t);
void writeProcess(uint32_t, uint32_t, uint32_t);

int main(void)
{
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    InitI2C();
    InitConsole();

    UARTprintf("ID: %i\n", readProcess(SLAVE_ADRESS, ID));

    // Without writing to the P_MODE, the sensor won't start to measure
    // We have to write the power-settings to the power-mode register
    writeProcess(SLAVE_ADRESS, P_SETTINGS, P_MODE);

    while (true) {
        UARTprintf("DATA_X: %i -", shiftProcess(readProcess(SLAVE_ADRESS, DATA_X0), readProcess(SLAVE_ADRESS, DATA_X1)));
        UARTprintf("DATA_Y: %i -", shiftProcess(readProcess(SLAVE_ADRESS, DATA_Y0), readProcess(SLAVE_ADRESS, DATA_Y1)));
        UARTprintf("DATA_Z: %i\n", shiftProcess(readProcess(SLAVE_ADRESS, DATA_Z0), readProcess(SLAVE_ADRESS, DATA_Z1)));
        //Provides a small delay.
        // 1/10 of the processor clock rate.
	    SysCtlDelay(SysCtlClockGet()/10);
    }
    return(0);
}

//	Initializing the communication with I2C
void InitI2C(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    //Initializes the I2C Master block.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
}

//	Initializing the Console for UART-output
void InitConsole(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}

//	The sensor delivers 8-bit data, so it is necessary to join the two 8-bit values
//	to one 16-bit information.

int32_t shiftProcess(int8_t left, int8_t right)
{
	//	Bitshifting to the left
    int16_t shifted = (int16_t) ((right << 8) | (left & 0xff));
    //	Floating point info through multiplication
    return (int32_t) shifted * 4;
}

//	The reading process of i2c, as described in chapter 17 of the
//	SW-TM4-DRL-UG
//	Tiva-Ware Peripheral Driver Library Users Guide
int8_t readProcess(uint32_t slaveAddress, uint32_t registerAddress)
{
	//Sets the address that the I2C Master places on the bus.
	//This function configures the address that the I2C Master places on the bus when initiating a
	//transaction. When the bReceive parameter is set to true, the address indicates that the I2C
    //Master is initiating a read from the slave; otherwise the address indicates that the I2C Master
	//is initiating a write to the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddress, false);
    //Transmits a byte from the I2C Master.
    I2CMasterDataPut(I2C0_BASE, registerAddress);
    //Controls the state of the I2C Master module.
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    //Indicates whether or not the I2C Master is busy.
    while(I2CMasterBusy(I2C0_BASE)) {
    	//Continue as soon as the I2C Master isn't busy anymore.
    }
    int8_t data = 0;
	//Sets the address that the I2C Master places on the bus.
    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddress, true);
    //Controls the state of the I2C Master module.
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    //Indicates whether or not the I2C Master is busy.
    while(I2CMasterBusy(I2C0_BASE)) {
    	//Continue as soon as the I2C Master isn't busy anymore.
    }
    //Receives a byte that has been sent to the I2C Master.
    data = (int8_t)I2CMasterDataGet(I2C0_BASE);
    return data;
}

void writeProcess(uint32_t slaveAddress, uint32_t registerAddress, uint32_t writeProcess)
{
	//Sets the address that the I2C Master places on the bus.
	//This function configures the address that the I2C Master places on the bus when initiating a
	//transaction. When the bReceive parameter is set to true, the address indicates that the I2C
    //Master is initiating a read from the slave; otherwise the address indicates that the I2C Master
	//is initiating a write to the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddress, false);
    //Transmits a byte from the I2C Master.
    I2CMasterDataPut(I2C0_BASE, registerAddress);
    //Controls the state of the I2C Master module.
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    //Indicates whether or not the I2C Master is busy.
    while(I2CMasterBusy(I2C0_BASE)) {
    	//Continue as soon as the I2C Master isn't busy anymore.
    }
   	//Sets the address that the I2C Master places on the bus.
    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddress, false);
	//Transmits a byte from the I2C Master.
    I2CMasterDataPut(I2C0_BASE, writeProcess);
    //Controls the state of the I2C Master module.
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C0_BASE)) {
    	//Continue as soon as the I2C Master isn't busy anymore.
    }
}
