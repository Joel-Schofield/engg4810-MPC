
#include "adcmanager.h"


void adc_init(void)
{
    unsigned long ulDummy = 0;

    // Enable the ADC hardware
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    

    // Configure the pin as analog input
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1); // PIN D3/2/1 as ADC.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5); // PIN E5 as ADC.
    //GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_5 | GPIO_PIN_4);                           // U_AN{0..1}



    

    // for 6 ADCs
    // use Sample Sequencer 0 since it is the only one able to handle more than four ADCs
    ADCSequenceDisable(ADC0_BASE, 0);
    // configure Sequencer to trigger from processor with priority 0
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    // do NOT use TRIGGER_TIMER because of malfunction in the touch screen handler
    //ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);

    // configure the steps of the Sequencer
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH4);   //CH4 = PD3  
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH5);   //CH5 = PD2    
    ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH6);   //CH5 = PD1    
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH8 | ADC_CTL_IE | ADC_CTL_END);//CH8 = PE4
    //ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH11);                    // U_AN1
    //ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH4);                     // U_AN2
    //ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH5);                     // U_AN3
    //ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH6);                     // U_AN4
    //ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH7 | ADC_CTL_IE | ADC_CTL_END);       // U_AN5
    ADCSequenceEnable(ADC0_BASE, 0);

    // flush the ADC
    ADCSequenceDataGet(ADC0_BASE, 0, &ulDummy);

    // Enable Interrupt for ADC0 Sequencer 0
    ADCIntEnable(ADC0_BASE, 0);
    IntEnable(INT_ADC0);


   	

}


void ADCIntHandler(void)
{
    int i;
    unsigned long ulADCval[4];
    long lSampleCount = 0;             // number of samples taken

    // Clear the ADC interrupt.
    ADCIntClear(ADC0_BASE, 0);



    // Get the 4 new values from ADC
    lSampleCount = ADCSequenceDataGet(ADC0_BASE, 0, &(ulADCval[0]));

    if (lutEnabled)
        lutValue = ulADCval[0];
    else
        adcReadings[0] = ulADCval[0];

    adcReadings[1] = ulADCval[1];
    adcReadings[2] = ulADCval[2];
    adcReadings[3] = ulADCval[3];
    //adcReadings[2] = ulADCval[1];

}





