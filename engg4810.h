#ifndef ENGG4810_H_
#define ENGG4810_H_

#include <stdio.h>
#include <stdarg.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "ff.h"
#include "diskio.h"
#include "driverlib/uart.h"
#include "uartstdio.h"
#include "string.h"

#include "inc/hw_ssi.h"
#include "driverlib/ssi.h"
#include "driverlib/adc.h"


//CMSIS
#include "CMSIS-3.20/CMSIS/Include/arm_math.h"

#include "spi_4810.h"
#include "audiomanager.h"
#include "adcmanager.h"
#include "buttonmanager.h"
#include "midimanager.h"
#include "transfermanager.h"

//forward declarations
void buttonPressed(int button);
void buttonReleased(int button);



//#include "handel_4810.h"

#define AUDIO_FREQ 44100

#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

//tempo
#define TEMPO_LED GPIO_PIN_6

//PORT C
#define ROW_4 GPIO_PIN_4
#define ROW_3 GPIO_PIN_5
#define ROW_2 GPIO_PIN_6
#define ROW_1 GPIO_PIN_7

//PORT E
#define COLUMN_1 GPIO_PIN_1
#define COLUMN_2 GPIO_PIN_2
#define COLUMN_3 GPIO_PIN_3
#define COLUMN_4 GPIO_PIN_4

//LEDS PORT D
#define RED_ROW_1 GPIO_PIN_0
#define RED_ROW_2 GPIO_PIN_1
#define RED_ROW_3 GPIO_PIN_2
#define RED_ROW_4 GPIO_PIN_3

#define MAX_ADC_VAL 4096

#define EMPTY_SLOT 0
#define BITCRUSHER_DECIMATOR 1
#define BITWISE_KO 2
#define FIR_DELAY 3
#define IIR_ECHO 4
#define LOWPASS_FILT 5
#define HIGHPASS_FILT 6
#define BANDPASS_FILT 7
#define NOTCH_FILT 8
#define SAMPLE_SHIFT 9 

#define LOOP_ON 1
#define LOOP_OFF 0

#define LATCH_MODE 0
#define HOLD_MODE 1

#define FREESTYLE_MODE 0
#define BEAT_GRID_MODE 1
#define QUANTISE_MODE 2

#define NUM_BUTTONS 16

#define FN_BUTTON GPIO_PIN_6
#define LOOP_BUTTON GPIO_PIN_7

#define TRANSFER_BAUD 921600
//#define TRANSFER_BAUD 115200
#define MIDI_BAUD 115200

char playbackMode;

#define BUTTON_POLL_TIME 10

//IMPORTANT: OUT_BUFF_LEN MUST BE A MULTIPLE OF AUD_BUFF_LEN!!
#define AUD_BUFF_LEN 2000
#define OUT_BUFF_LEN 4000
#define EFFECT_BUFF_LEN 0



short audBuff1[AUD_BUFF_LEN];
short audBuff2[AUD_BUFF_LEN];
short outBuff[OUT_BUFF_LEN];
float32_t floatBuff[AUD_BUFF_LEN];

char loopInterval[16];
char buttonMode[16];
FIL* buttonFil[16];



int lfoEnabled;
int lutValue;
int midiEnabled;

int bpm;
char beatGrid;


unsigned long timestamp;
unsigned int buttonPollCnt;

//file system object.
FATFS Fatfs;    

FIL audFil1;    //file object, used for storing logging file info.
FIL audFil2;    //file object, used for storing logging file info.
UINT bw;    //store the number of bytes written thrugh f_write. 
UINT bw2;    //store the number of bytes written thrugh f_write. 
FRESULT rc; //stores the result of file operations to check for errors.

FILINFO fno;        //stores the status of a file.

FIL fil;
UINT bw;

int currRowPin;
int currColumnPin;

int adcReadings[4];


unsigned long debounceCnt;




int cAtoi (char * str, int len);
int loadConfig();

#endif
	
