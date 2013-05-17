/*
 * spi_4810.h
 *
 *  Created on: 19/03/2013
 *      Author: Joel
 */


#ifndef SPI_4810_H_
#define SPI_4810_H_

 #ifdef __cplusplus
extern "C"
{
#endif

#include "engg4810.h"

#define SSI_DAC_CLK GPIO_PIN_4 //port B
#define SSI_DAC_TX GPIO_PIN_7 //port B
#define SSI_DAC_CS GPIO_PIN_5 //port B

void initDACSSI();
void SPIWriteDAC(unsigned short cmd, unsigned short data);

#ifdef __cplusplus
}
#endif

#endif /* SPI_4810_H_ */
