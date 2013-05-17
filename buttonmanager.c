#include "buttonmanager.h"




int buttonStat;

int latestRow;
int latestColumn;
int prevDebounceCnt;


char tempoLedStat = 0;


void Timer1ButtonHandler(void) {

	//Clear the timer interrupt.
    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    
    //Disable interrupts from occuring while processing.
    ROM_IntMasterDisable();

    timestamp++;
    buttonPollCnt++;

    if ( (timestamp % bpm) < (bpm / 5) )
            GPIOPinWrite(GPIO_PORTA_BASE, TEMPO_LED, TEMPO_LED);
    else
    	GPIOPinWrite(GPIO_PORTA_BASE, TEMPO_LED, 0);

    ROM_IntMasterEnable();

}