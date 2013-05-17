
#include "audiomanager.h"

#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

#define AUD_BUFF_ONE 1
#define AUD_BUFF_TWO 2
#define AUD_BUFF_EMPTY 0
#define AUD_BUFF_FULL 1
#define AUD_BUFF_DONE 2



#define DAC_MID_OFFSET 0 //8 bit dac, (2^12)/2
//#define DAC_MID_OFFSET 2047 //12 bit dac, (2^12)/2
#define DAC_MAX 32767
#define DAC_MIN -32767 


int fx[2];

float32_t pState[4];
float32_t pCoeffs[5];

char loop1;
char loop2;

char interval1;
char interval2;

unsigned long startStamp1;
unsigned long startStamp2;

int pos = 0;

//IMPORTANT: OUT_BUFF_LEN MUST BE A MULTIPLE OF AUD_BUFF_LEN!!
#define AUD_BUFF_LEN 1000
#define OUT_BUFF_LEN 8000
#define EFFECT_BUFF_LEN 0

short audBuff1[AUD_BUFF_LEN];
short audBuff2[AUD_BUFF_LEN];
short outBuff[OUT_BUFF_LEN];
float32_t testBuff[AUD_BUFF_LEN];

int decValPrev = DAC_MID_OFFSET;
int decValCurr = DAC_MID_OFFSET;
int decCount = 0;


#define NUM_TAPS 38
#define FILT_BLOCK_SIZE AUD_BUFF_LEN

arm_biquad_casd_df1_inst_f32 S;


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


int n = 0;
int lutSampleCnt=0;

int lead = 0;

int nextBuf = AUD_BUFF_ONE;
int audBuff1Stat = AUD_BUFF_DONE;
int audBuff2Stat = AUD_BUFF_DONE;
short* outWritePtr = outBuff;
short* outReadPtr = outBuff;

int initAudioBuffers() {

	memset(audBuff1, DAC_MID_OFFSET, AUD_BUFF_LEN); 
	memset(audBuff2, DAC_MID_OFFSET, AUD_BUFF_LEN); 
	memset(outBuff, DAC_MID_OFFSET, OUT_BUFF_LEN); 
	//memset(testBuff, DAC_MID_OFFSET, AUD_BUFF_LEN); 

	fx[0] = 0;
	fx[1] = 0;

	//biquadGen(HIGHPASS_FILT, 6000.0, 0.707, 44100.0, pCoeffs);

	arm_biquad_cascade_df1_init_f32(&S, 1, pCoeffs, pState);

    return 0;
}


FIL* startAudio(char* fileNameStr, int loop, char interval){

	FIL* fil;

	if(nextBuf == AUD_BUFF_ONE) {
		rc = f_open(&audFil1, fileNameStr, FA_READ);
		f_lseek(&audFil1, 150); //skip WAV header.
		if (rc) { //if not successful open:
			audBuff1Stat = AUD_BUFF_DONE;
		}
		else { //open succeeded:
			audBuff1Stat = AUD_BUFF_EMPTY;
			loop1 = loop;
			interval1 = interval;
			startStamp1 = timestamp;
			fil = &audFil1;
		}

    	nextBuf = AUD_BUFF_TWO;
	}
	else {
		rc = f_open(&audFil2, fileNameStr, FA_READ);
		f_lseek(&audFil2, 150);
		if (rc) { //if not successful open:
			audBuff2Stat = AUD_BUFF_DONE;
		}
		else { //open succeeded:
			audBuff2Stat = AUD_BUFF_EMPTY;
			loop2 = loop;
			interval2 = interval;
			startStamp2 = timestamp;
			fil = &audFil2;
		}

		nextBuf = AUD_BUFF_ONE;
	}

	return fil;
}

int stopAudio(FIL* fil) {

	//while(1);

	if (fil == &audFil1) {
		audBuff1Stat = AUD_BUFF_DONE;
		nextBuf = AUD_BUFF_ONE;
	}
	else if (fil == &audFil2) {
		audBuff2Stat = AUD_BUFF_DONE;
		nextBuf = AUD_BUFF_TWO;
	}
	else
		return -1;

	return 0;
}


int startEffect(int effect, int fxSlot){

	if (fxSlot <= 1)
		fx[fxSlot] = effect;	

	return 0;	
}

int stopEffect(int fxSlot) {

	if (fxSlot <= 1)
		fx[fxSlot] = 0;	

	return 0;
}

int getEffect(int fxSlot) {

	if (fxSlot <= 1)
		return fx[fxSlot];
}


int stepAudioProcess(){

	long res;
	unsigned long samplesWritten;
	short* refOutWritePtr = outWritePtr;

	
	if (lead >= AUD_BUFF_LEN) {
		return 0;
	}
	


	if (audBuff1Stat == AUD_BUFF_EMPTY) {
		rc = f_read(&audFil1, (char*)audBuff1, AUD_BUFF_LEN*2, &bw);
		if (bw == AUD_BUFF_LEN*2) {
			audBuff1Stat = AUD_BUFF_FULL;
			if (loop1 == LOOP_ON && timestamp >= startStamp1 + (bpm / interval1) ) {
				f_lseek(&audFil1, 150);
				startStamp1 = startStamp1 + (bpm / interval1);;
			}
		}
		else if (loop1 == LOOP_ON) {
			audBuff1Stat = AUD_BUFF_FULL;
			for(int i=0; i < AUD_BUFF_LEN - bw/2; i++)
				audBuff1[bw/2] = DAC_MID_OFFSET;
			if (loop1 == LOOP_ON && timestamp >= startStamp1 + (bpm / interval1) ) {
				f_lseek(&audFil1, 150);
				startStamp1 = startStamp1 + (bpm / interval1);
			}
		}
		else {
			audBuff1Stat = AUD_BUFF_DONE;
			nextBuf = AUD_BUFF_ONE; // can reuse this buff as its finished with.
		}
	}
	if (audBuff2Stat == AUD_BUFF_EMPTY) {
		rc = f_read(&audFil2, (char*)audBuff2, AUD_BUFF_LEN*2, &bw);
		if (bw == AUD_BUFF_LEN*2) {
			audBuff2Stat = AUD_BUFF_FULL;
			if (loop2 == LOOP_ON && timestamp >= startStamp2 + (bpm / interval2) ) {
				f_lseek(&audFil2, 150);
				startStamp2 = startStamp2 + (bpm / interval2);
			}
		}
		else if (loop2 == LOOP_ON) {
			audBuff2Stat = AUD_BUFF_FULL;
			for(int i=0; i < AUD_BUFF_LEN - bw/2; i++)
				audBuff2[bw/2] = DAC_MID_OFFSET;
			if (loop2 == LOOP_ON && timestamp >= startStamp2 + (bpm / interval2) ) {
				f_lseek(&audFil2, 150);
				startStamp2 = startStamp2 + (bpm / interval2);
			}
		}
		else {
			audBuff2Stat = AUD_BUFF_DONE;
			nextBuf = AUD_BUFF_TWO; 
		}
	}


	
	//mix two samples together:
	if (audBuff1Stat == AUD_BUFF_FULL && audBuff2Stat == AUD_BUFF_FULL) {
		
		for (int i=0; i < AUD_BUFF_LEN; i++) {
			res = (audBuff1[i]>>3) + (audBuff2[i]>>3);

			if (res > DAC_MAX)
				*outWritePtr = DAC_MAX;
			else if (res < DAC_MIN)
				*outWritePtr = DAC_MIN;
			else 
				*outWritePtr = res;

			outWritePtr++;
		}

		audBuff1Stat = AUD_BUFF_EMPTY;
		audBuff2Stat = AUD_BUFF_EMPTY;
		
	}
	//if only one sound playing, just copy audBuff sample to outBuff.
	else if (audBuff1Stat == AUD_BUFF_FULL) {
		//memcpy(outWritePtr, audBuff1, AUD_BUFF_LEN*2); 
		for(int i=0; i < AUD_BUFF_LEN; i++)
			outWritePtr[i] = audBuff1[i] >> 3;
		outWritePtr += AUD_BUFF_LEN;
		audBuff1Stat = AUD_BUFF_EMPTY;
	}
	else if (audBuff2Stat == AUD_BUFF_FULL) {
		//memcpy(outWritePtr, audBuff2, AUD_BUFF_LEN*2); 
		for(int i=0; i < AUD_BUFF_LEN; i++)
			outWritePtr[i] = audBuff2[i] >> 3;
		outWritePtr += AUD_BUFF_LEN;
		audBuff2Stat = AUD_BUFF_EMPTY;
	}
	//if nothing playing set the buffer to mid val.
	else {
		for (int i=0; i < AUD_BUFF_LEN; i++)
			outWritePtr[i] = DAC_MID_OFFSET;
		outWritePtr += AUD_BUFF_LEN;
	}



	//perform effects.
	samplesWritten = outWritePtr - refOutWritePtr;

	for (int i=0; i < 2; i++) {

		if (fx[i] == BITCRUSHER_DECIMATOR) {

			int bitAmount = adcReadings[i*2+1];
			int decAmount = adcReadings[i*2] / 50;
			
			//bitcrushing.
			for (int j=0; j < samplesWritten; j++) {
				
				//if (adcReadings[i*2] != 0) 
				refOutWritePtr[j] = (refOutWritePtr[j] >> (bitAmount / 274)  ) << (bitAmount / 274);

			}

			
			//decimating.
			for (int j=0; j < samplesWritten; j++) {

				if (decCount >= decAmount) {
					decValPrev = decValCurr;
					decValCurr = refOutWritePtr[j];
					decCount = 0;
				}
				
				refOutWritePtr[j] = decValPrev + ( (decValCurr - decValPrev) / decAmount) * decCount;

				decCount++;
			}
		}
		else if (fx[i] == BITWISE_KO) {

			int andVal = adcReadings[i*2] * (1 << 4);
			int xorVal = adcReadings[i*2+1] * (1 << 4);
			for (int j=0; j < samplesWritten; j++) {
				refOutWritePtr[j] = refOutWritePtr[j] & andVal;
				refOutWritePtr[j] = refOutWritePtr[j] ^ xorVal;

			}
		}
		else if (fx[i] == FIR_DELAY){

			int delayVal = adcReadings[i*2];
			int ampVal = adcReadings[i*2+1];
			long res;

		}
		else if (fx[i] == IIR_ECHO) {

			int delayVal = adcReadings[i*2];
			int ampVal = adcReadings[i*2+1];
			long res;
			
			for (int j=0; j < samplesWritten; j++) {

				if (&refOutWritePtr[j] - outBuff < delayVal)
					//refOutWritePtr[j] += refOutWritePtr[OUT_BUFF_LEN - (delayVal - (&refOutWritePtr[j] - outBuff))] * ampVal;
					//refOutWritePtr[j] = refOutWritePtr[j];
					res = ((long)refOutWritePtr[j]) + ( ( (long)refOutWritePtr[OUT_BUFF_LEN - (delayVal - (&refOutWritePtr[j] - outBuff) )] - DAC_MID_OFFSET) * ampVal ) / MAX_ADC_VAL;
				else {

					res = ((long)refOutWritePtr[j]) + ( ( (long)refOutWritePtr[j - delayVal] - DAC_MID_OFFSET) * ampVal ) / MAX_ADC_VAL;
				}

				
				if (res >= DAC_MAX)
					refOutWritePtr[j] = DAC_MAX;
				else if (res < DAC_MIN)
					refOutWritePtr[j] = DAC_MIN;
				else
					refOutWritePtr[j] = (unsigned short) res;
			}
		}
		else if (fx[i] == LOWPASS_FILT || fx[i] == HIGHPASS_FILT || fx[i] == BANDPASS_FILT || fx[i] == NOTCH_FILT) {

			int fcVal = adcReadings[i*2];
			int qVal = adcReadings[i*2+1];


			//for (int j=0; j < samplesWritten; j++) {
			//	refOutWritePtr[j] = refOutWritePtr[j] >> 2;
			//}

			for (int j=0; j < samplesWritten; j++) {
				testBuff[j] = ( (float32_t)refOutWritePtr[j] ) / 32768.0 ;
			}

			//biquadGen(fx[i], (float)(fcVal*5), 0.7, 44100.0, pCoeffs);
			//biquadGen(fx[i], 20.0, 0.7, 44.0, pCoeffs);

			biquadGen(fx[i], ((float)fcVal)*5.0, 4096.0 / ((float)qVal), 44100.0, pCoeffs);
			//biquadGen(fx[i], ((float)fcVal)*5.0, 0.9, 44100.0, pCoeffs);

			arm_biquad_cascade_df1_f32(&S, testBuff, testBuff, FILT_BLOCK_SIZE);

			for (int j=0; j < samplesWritten; j++) {
				refOutWritePtr[j] = (short)(testBuff[j] * 32768.0);
			}

		}
	}

	


	lead += (outWritePtr - refOutWritePtr);

	

		//loop to start of outBuff if at the end.
	if (outWritePtr == (&outBuff[OUT_BUFF_LEN]) ) { //at last value of outBuff:
		outWritePtr = outBuff;
		//GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, BLUE_LED);
		//while(1);
	}

	return 0;
}


//Timer interrupt configured to occur at sample rate (44kHz)
void Timer0AudioHandler(void) {

    //Clear the timer interrupt.
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
    //Disable interrupts from occuring while processing.
    ROM_IntMasterDisable();
    	
    //check that the buffer has an available audio sample to output.
    if (lead > 0) { //TODO: CAN CREATE DIRTY EFFECT BUFFER BY MAKING THIS LEAD > EFFECT_BUFF_LENGTH

    	//send the sample to the DAC.

    	if (lutEnabled) {
    		/*
	        SPIWriteDAC(
	        	0x0007,  
	        	(unsigned short) ( ((long)*outReadPtr) - ( ( ( (long)*outReadPtr - DAC_MID_OFFSET) * lutTable[n] ) / 4096 ) )
	        	//(unsigned short) ( ((long)outReadPtr) - ( ( ( (long)outReadPtr - DAC_MID_OFFSET) * lutTable[n] ) / 4096 ) )
	        	
	        	);
	        	*/

	        adcReadings[0] = lutTable[n];

	        if (lutSampleCnt++ >= (lutValue / 10) ) {
	        	lutSampleCnt = 0;
	        	n++;
	        	if (n==127) n=0;
	        }
	    }
	    
	    SPIWriteDAC(0x0007, (unsigned short)(outReadPtr[pos] + DAC_MAX) >> 4);
	    
        

        pos++;
        lead--;

        //if at end of output buffer, loop back to start.
        if(pos >= OUT_BUFF_LEN) {
            pos = 0;
        }
    }
    
    ROM_IntMasterEnable();
}



//good values is 0.7
int biquadGen(int type, float32_t Fc, float32_t Q, float32_t Fs, float32_t* pCoeffs) {

    float32_t w0;
    float32_t c1;
    float32_t alpha;

    float32_t b0;
    float32_t b1;
    float32_t b2;

    float32_t a0;
    float32_t a1;
    float32_t a2;

    float32_t res0;
    float32_t res1;
    float32_t res2;
    float32_t res3;
    float32_t res4;

    w0 = 2.0*3.14*(Fc/Fs); // digital cutoff frequency
    c1 = arm_cos_f32(w0);
    alpha = arm_sin_f32(w0) / (2.0 * Q);

    //UARTprintf("Hello, world! w0, c1, alpha: %d, %d, %d\n", w0, c1, alpha);

    switch (type) {
        case LOWPASS_FILT:
            b0 = (1.0 - c1)/2.0;
            b1 = 1.0 - c1;
            b2 = (1.0-c1)/2.0;
            a0 = 1.0 + alpha;
            a1 = -2.0 * c1;
            a2 = 1.0 - alpha;
            break;
        case HIGHPASS_FILT:
            b0 = (1.0 + c1)/2.0;
            b1 = -1.0 - c1;
            b2 = (1.0 + c1)/2.0;
            a0 = 1.0 + alpha;
            a1 = -2.0 * c1;
            a2 = 1.0 - alpha;
            break;
        case BANDPASS_FILT:
            b0 = alpha;
            b1 = 0.0;
            b2 = -1.0 * alpha;
            a0 = 1.0 + alpha;
            a1 = -2.0 * c1;
            a2 = 1.0 - alpha;
        case NOTCH_FILT:
            b0 = 1.0;
            b1 = -2.0 * c1;
            b2 = 1.0;
            a0 = 1.0 + alpha;
            a1 = -2.0 * c1;
            a2 = 1.0 - alpha;
    }

    //{b10, 0, b11, b12, a11, a12, b20, 0, b21, b22, a21, a22, ...}  

    //UARTprintf("Hello, world! beforecoeffs: %d, %d, %d, %d, %d, %d\n", round(b0),round(b1),round(b2),round(a0),round(a1),round(a2));
    //UARTprintf("Hello, world! beforecoeffs: %d, %d\n", round(b0*1000),round(a0*1000));

    //pCoeffs[4] = 1.0;
    
    pCoeffs[0] = b0/a0;
    pCoeffs[1] = b1/a0;
    pCoeffs[2] = b2/a0;
	pCoeffs[3] = (-1.0*a1)/a0;
    pCoeffs[4] = (-1.0*a2)/a0;
    

    /* 10800
    pCoeffs[0] = 0.2824;
    pCoeffs[1] = 0.5648;
    pCoeffs[2] = 0.2824;
    pCoeffs[3] = 0.0374;
    pCoeffs[4] = 0.1669;
    */

    /*	1000
    pCoeffs[0] = 0.0046;
    pCoeffs[1] = 0.0092;
    pCoeffs[2] = 0.0046;
    pCoeffs[3] = 1.7974;
    pCoeffs[4] = -0.8158;
    */

    /*
    pCoeffs[0] = 0.3356;
    pCoeffs[1] = -0.6712;
    pCoeffs[2] = 0.3356;
    pCoeffs[3] = 0.1705; 
    pCoeffs[4] = -0.1719; 
    */


    /*
    UARTprintf("res:");
    UARTprintf("%d ", res0 );
    UARTprintf("%d ", res1);
    UARTprintf("%d ", res2);
    UARTprintf("%d ", res3);
    UARTprintf("%d\n", res4);
    */
   
    //pCoeffs[0] =  (q15_t) ( (round(b0/a0) ) * DAC_MAX);
    /*
    pCoeffs[0] =  (q15_t) ( round(res0*10000) );
    pCoeffs[1] = 0;
    pCoeffs[2] = (q15_t) ( round(res1*10000) );
    pCoeffs[3] = (q15_t) ( round(res2*10000) );

    pCoeffs[4] = (q15_t) ( round(res3*10000) );
    pCoeffs[5] = (q15_t) ( round(res4*10000) );
    */

    
    //UARTprintf("co:");
    //for (int i=0; i < 4; i++)
    //	UARTprintf("%d ", pCoeffs[i]);
    //UARTprintf("%d ", pCoeffs[1]);
    


    return 0;

}
