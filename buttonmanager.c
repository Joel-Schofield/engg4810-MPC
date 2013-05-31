#include "buttonmanager.h"




int buttonStat;

int latestRow;
int latestColumn;
int prevDebounceCnt;

int n = 0;
int lutSampleCnt=0;

static const unsigned short lutTable[] = {2047,1947,1846,1747,1648,1549,1453,1357,
    1263,1172,1082,994,909,827,748,672,599,530,464,402,345,291,241,196,
    155,119,88,61,39,22,9,2,0,2,9,22,39,61,88,119,155,196,241,291,345,
    402,464,530,599,672,748,827,909,994,1082,1172,1263,1357,1453,1549,
    1648,1747,1846,1947,2047,2147,2248,2347,2446,2545,2641,2737,2831,2922,
    3012,3100,3185,3267,3346,3422,3495,3564,3630,3692,3749,3803,3853,3898,
    3939,3975,4006,4033,4055,4072,4085,4092,4095,4092,4085,4072,4055,4033,4006,
    3975,3939,3898,3853,3803,3749,3692,3630,3564,3495,3422,3346,3267,3185,3100,
    3012,2922,2831,2737,2641,2545,2446,2347,2248,2147
}; //length = 128


char tempoLedStat = 0;


void Timer1ButtonHandler(void) {

	//Clear the timer interrupt.
    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    
    //Disable interrupts from occuring while processing.
    ROM_IntMasterDisable();

    //send the sample to the DAC.

    if (lfoEnabled) {
        /*
        SPIWriteDAC(
            0x0007,  
            (unsigned short) ( ((long)*outReadPtr) - ( ( ( (long)*outReadPtr - INT16_MID) * lutTable[n] ) / 4096 ) )
            //(unsigned short) ( ((long)outReadPtr) - ( ( ( (long)outReadPtr - INT16_MID) * lutTable[n] ) / 4096 ) )
            
            );
            */

        adcReadings[0] = (short)((float)lutTable[n] * 0.8f + 100.0f);

        if (lutSampleCnt++ + 1 >= (short)(lutValue) ) {
            lutSampleCnt = 0;
            n += 2;
            if (n==126) n=0;
        }
    }

    timestamp++;
    buttonPollCnt++;

    if ( (timestamp % bpm) < (bpm / 5) )
            GPIOPinWrite(GPIO_PORTA_BASE, TEMPO_LED, TEMPO_LED);
    else
    	GPIOPinWrite(GPIO_PORTA_BASE, TEMPO_LED, 0);

    ROM_IntMasterEnable();

}