/*
 * spi_dac.c
 *
 *  Created on: 19/03/2013
 *      Author: Joel
 */

#include "spi_4810.h"


void
initDACSSI()
{
	//
	// Initialize the 3 pins we will need for SPI communication with the LCD
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    // Connect SPI to PB4 (clock) and PB7(TX)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	GPIOPinConfigure(GPIO_PB4_SSI2CLK);
	GPIOPinConfigure(GPIO_PB7_SSI2TX);
	GPIOPinConfigure(GPIO_PB5_SSI2FSS);

	GPIOPinTypeSSI(GPIO_PORTB_BASE, SSI_DAC_CLK | SSI_DAC_TX | SSI_DAC_CS);


	//
	// Configure SSI2
	//
	SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,  SSI_MODE_MASTER, SysCtlClockGet()/4, 16);

	//
	// Enable the SSI module.
	//
	SSIEnable(SSI2_BASE);


}



void SPIWriteDAC(unsigned short cmd, unsigned short data)
{

	SSIDataPut(SSI2_BASE, ((cmd << 12) | (data & 0x0FFF) ) );
	//
	// Wait until SSI is done transferring all the data in the transmit FIFO
	//
	while(SSIBusy(SSI2_BASE))
	{
	}

}
