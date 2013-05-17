//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 9453 of the EK-LM4F120XL Firmware Package.
//
//*****************************************************************************

#include "engg4810.h"


#define DEBOUNCE_TIME 50

#define NUM_BUTTONS 16

#define END_OF_SEQUENCE 255
#define BUTTON_PUSH 1
#define BUTTON_RELEASE 0

int buttonStat;
int loopStat;
int fnStat;

int latestRow;
int latestColumn;
int prevDebounceCnt;


int midiVarOffset = 36;


#define LATCH_MODE 0
#define HOLD_MODE 1

char loopInterval[16];
char buttonMode[16];
char prevStat[16]; // records the previous state.
unsigned long lastEvent[16]; //records timestamps that the last time a button changed states.
FIL* buttonFil[16];

char charBuff[] = "a.wav";

char buttonEventBuff[100];
char buttonEventBuffType[100];
unsigned long timeEventBuff[100];
char recordingFreestyle = 0;
unsigned long startTime;

int playback = 0;

int eventCnt = 0;

char rows[] = {ROW_1, ROW_2, ROW_3, ROW_4};
char columns[] = {COLUMN_1, COLUMN_2, COLUMN_3, COLUMN_4};

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hello World (hello)</h1>
//!
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the UART and is a starting point for more complicated applications.
//!
//! UART0, connected to the Stellaris Virtual Serial Port and running at 
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif


//*****************************************************************************
//
// MAIN
//
//*****************************************************************************
int
main(void)
{

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);        //enable port A for GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);        //enable port C for GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);        //enable port E for GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);        //enable port F for GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);        //enable port D for GPIO

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);   //enable timer0.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);   //enable timer0.

    initDACSSI(); //setup SPI pins for DAC.

    disk_initialize(0); //setup SD Card SPI

    ROM_IntMasterEnable(); //Enable processor interrupts.

    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //Configure the two 32-bit periodic timers.
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); //Configure the two 32-bit periodic timers.

    // Configure the 32-bit periodic timer.
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() / AUDIO_FREQ); // 1sec / 44100 = 44.1kHz
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet() / 1000); // 1sec / 44100 = 44.1kHz

    // Setup the interrupts for the timer timeouts.
    ROM_IntEnable(INT_TIMER0A);
    ROM_IntEnable(INT_TIMER1A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //enable the timer
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
    ROM_TimerEnable(TIMER1_BASE, TIMER_A);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);         //stellaris led
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, TEMPO_LED);         //tempo led


    // Set each of the button GPIO pins as an input with a pull-up.
    ROM_GPIODirModeSet(GPIO_PORTC_BASE, ROW_1|ROW_2|ROW_3|ROW_4, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(GPIO_PORTC_BASE, ROW_1|ROW_2|ROW_3|ROW_4,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    ROM_GPIODirModeSet(GPIO_PORTE_BASE, COLUMN_1|COLUMN_2|COLUMN_3|COLUMN_4, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(GPIO_PORTE_BASE, COLUMN_1|COLUMN_2|COLUMN_3|COLUMN_4,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    ROM_GPIODirModeSet(GPIO_PORTA_BASE, LOOP_BUTTON, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(GPIO_PORTA_BASE, LOOP_BUTTON,
                        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    ROM_GPIODirModeSet(GPIO_PORTB_BASE, FN_BUTTON, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE, FN_BUTTON,
                        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //configure board leds
    //GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, RED_ROW_1|RED_ROW_2|RED_ROW_3|RED_ROW_4); //board leds
    //GPIOPinWrite(GPIO_PORTD_BASE, RED_ROW_1|RED_ROW_2|RED_ROW_3|RED_ROW_4, 0);

    //setup ADCs.
    adc_init();



    //HARDWARE CONFIGURED. SETUP INIT DATA VALUES.

    initAudioBuffers();

    currRowPin = ROW_1;
    currColumnPin = COLUMN_1;

    lutEnabled = 0;
    midiEnabled = 0;


    for (int i=0; i < NUM_BUTTONS; i++) {
        prevStat[i] = 0;
        lastEvent[i] = 0;
        if (i < 4) {
            buttonMode[i] = LATCH_MODE;
            loopInterval[i] = 1;
        }
        else {
            buttonMode[i] = HOLD_MODE;
            loopInterval[i] = 4;
        }

        
    }

    timestamp = 0;
    bpm = 60000 / 135 * 16; //divide by 4 so can get full samples current setup board.

    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, GREEN_LED);

    /* Register volume work area (never fails) */
    f_mount(0, &Fatfs); 

    //
    // Initialize the UART.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioInit(0);



    //startAudio("1.raw");
    //startAudio("wolf16.wav", LOOP_OFF);
    //startAudio("ram16.wav", LOOP_ON);
    //startEffect(EFFECT_LUT, 0);
    //startEffect(HIGHPASS_FILT, 0);
    //startEffect(BITWISE_KO, 0);
    //startEffect(BITCRUSHER_DECIMATOR, 0);
    //startEffect(BITCRUSHER_DECIMATOR, 0);
    //startEffect(IIR_ECHO, 0);
    //startAudio("RAM16B.wav");


    while(1) {

        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, GREEN_LED);


        stepAudioProcess();

        if (playback) {


            if (timeEventBuff[eventCnt] >= timestamp - startTime ) {

                if (buttonEventBuffType[eventCnt] == END_OF_SEQUENCE) {
                    playback = 0;
                    while(1);
                }
                else if(buttonEventBuffType[eventCnt] == BUTTON_PUSH){
                    buttonPressed(buttonEventBuff[eventCnt]);
                    //while(1);
                }
                else{
                    buttonReleased(buttonEventBuff[eventCnt]);
                }

                eventCnt++;
            }
        }

        switch(currColumnPin) {
        case COLUMN_1:
            currColumnPin = COLUMN_2;
            break;
        case COLUMN_2:
            currColumnPin = COLUMN_3;
            break;
        case COLUMN_3:
            currColumnPin = COLUMN_4;
            break;
        case COLUMN_4:
            currColumnPin = COLUMN_1;
            switch(currRowPin) {
                case ROW_1:
                    currRowPin = ROW_2;
                    break;
                case ROW_2:
                    currRowPin = ROW_3;
                    break;
                case ROW_3:
                    currRowPin = ROW_4;
                    break;
                case ROW_4:
                    currRowPin = ROW_1;
                    break;
            }
            break;
        }
    

        //select a column: 
        GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, currColumnPin);
        GPIOPinWrite(GPIO_PORTE_BASE, currColumnPin, 0);

        //check row:
        buttonStat = GPIOPinRead(GPIO_PORTC_BASE, currRowPin);
        // && !(latestColumn == currColumnPin && latestRow == currRowPin ) && (debounceCnt - prevDebounceCnt) < 5000

        /*
        if (currRowPin == ROW_4 && currColumnPin == COLUMN_4) {
            if (!buttonStat)
                startEffect(EFFECT_LUT);
            else
                stopEffect(EFFECT_LUT);
        }
        */

        if (GPIOPinRead(GPIO_PORTA_BASE, LOOP_BUTTON))
            loopStat = 0;
        else 
            loopStat = 1;

        if (GPIOPinRead(GPIO_PORTB_BASE, FN_BUTTON))
            fnStat = 0;
        else 
            fnStat = 1;


        for (int i=0; i < 4; i++) {
            for (int j=0; j < 4; j++) {
                if (rows[i] == currRowPin && columns[j] == currColumnPin) {
                    if (prevStat[j*4+i] != buttonStat) {
                        if (timestamp > lastEvent[j*4+i] + DEBOUNCE_TIME) {
                            if (!buttonStat)
                                buttonPressed(j*4+i);
                            else
                                buttonReleased(j*4+i);

                            prevStat[j*4+i] = buttonStat;
                            lastEvent[j*4+i] = timestamp;
                        }
                    }

                }
            }
        }

        //reset to init state:
        GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, COLUMN_1|COLUMN_2|COLUMN_3|COLUMN_4);
        
        //adc
        ADCProcessorTrigger(ADC0_BASE, 0);

        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, BLUE_LED);
    
    }

}



inline void buttonPressed(int button) {

    if (midiEnabled && fnStat && button >= 4 && button <= 11) {
        midiVarOffset = 12 * (button - 4);
    }
    else if (fnStat) {
        if (button == 0){
            //Enable/Disable USB Midi
            midiEnabled ^= 1;
        }
        else if (button == 1){
            //Enable/Disable LFO
            lutEnabled ^= 1;
        }
        else if (button == 2){
            recordingFreestyle ^= 1;
            if (recordingFreestyle == 1) {
                startTime = timestamp;
                eventCnt = 0;
            }
            else {
                buttonEventBuff[eventCnt] = END_OF_SEQUENCE;
            }
        }
        else if (button == 3){
            playback = 1;
            eventCnt = 0;
            startTime = timestamp;
        }
        else if (button == 4) {
            if (getEffect(0) == BITCRUSHER_DECIMATOR)
                stopEffect(0);
            else
                startEffect(BITCRUSHER_DECIMATOR, 0);
        }
        else if (button == 5) {
            if (getEffect(0) == IIR_ECHO)
                stopEffect(0);
            else
                startEffect(IIR_ECHO, 0);
        }
        else if (button == 6) {
            if (getEffect(0) == HIGHPASS_FILT)
                stopEffect(0);
            else
                startEffect(HIGHPASS_FILT, 0);
        }
        else if (button == 7) {
            if (getEffect(0) == LOWPASS_FILT)
                stopEffect(0);
            else
                startEffect(LOWPASS_FILT, 0);
        }


    }
    else if (midiEnabled) {

        midi_t m;

        //+MIDI_BUTTON_OFFSET as otherwise the notes are just bass.
        midiMessage(&m, MIDI_STATUS_NOTE_ON, 0, button + midiVarOffset + MIDI_BUTTON_OFFSET, MIDI_VALUE_NOTE_VELOCITY); 

        UARTwrite(&m, sizeof(m));
        //midi_t midiMessage(uint8_t stat, uint8_t ch, uint8_t ctrl, uint8_t val) {
        //UARTwrite( &);
    }
    else {
        charBuff[0] = button + 97;
        //if (buttonMode[button] == HOLD_MODE)
        //    loopStat = 1;
        buttonFil[button] = startAudio(charBuff, loopStat, loopInterval[button]);

        if (recordingFreestyle) {
            buttonEventBuff[eventCnt] = button;
            buttonEventBuffType[eventCnt] = BUTTON_PUSH;
            timeEventBuff[eventCnt] = timestamp - startTime;
            eventCnt++;
        }
    }

}

inline void buttonReleased(int button) {

    if (midiEnabled) {

        midi_t m;

        midiMessage(&m, MIDI_STATUS_NOTE_OFF, 0, button + midiVarOffset + MIDI_BUTTON_OFFSET, MIDI_VALUE_NOTE_VELOCITY);

        UARTwrite(&m, sizeof(m));
        //midi_t midiMessage(uint8_t stat, uint8_t ch, uint8_t ctrl, uint8_t val) {
        //UARTwrite( &);
    }

    else {
        if (buttonMode[button] == HOLD_MODE) {
            charBuff[0] = button + 97;
            stopAudio(buttonFil[button]);
            buttonFil[button] = 0;
        }

        if (!fnStat && recordingFreestyle) {
                buttonEventBuff[eventCnt] = button;
                buttonEventBuffType[eventCnt] = BUTTON_RELEASE;
                timeEventBuff[eventCnt] = timestamp - startTime;
                eventCnt++;
        }
    }
}



