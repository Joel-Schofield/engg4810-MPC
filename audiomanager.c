
#include "audiomanager.h"

#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

#define AUD_BUFF_ONE 1
#define AUD_BUFF_TWO 2
#define AUD_BUFF_EMPTY 0
#define AUD_BUFF_FULL 1
#define AUD_BUFF_DONE 2

#define SHIFT_CONST 4



#define INT16_MID 0 //8 bit dac, (2^12)/2
//#define INT16_MID 2047 //12 bit dac, (2^12)/2
#define INT16_MAX 32767
#define INT16_MIN -32767 

#define INT12_MAX 2048
#define INT12_MIN -2048

#define DAC_MAX 3000
#define DAC_MIN 0

unsigned short test = 0;

int fx[2];

float32_t pState1[4];
float32_t pState2[4];
float32_t pCoeffs[5];

char loop1;
char loop2;

char interval1;
char interval2;

unsigned long startStamp1;
unsigned long startStamp2;

int pos = 0;

int decValPrev[2] = {INT16_MID, INT16_MID};
int decValCurr[2] = {INT16_MID, INT16_MID};
int decCount[2] = {0,0};


#define NUM_TAPS 38
#define FILT_BLOCK_SIZE AUD_BUFF_LEN

arm_biquad_casd_df1_inst_f32 S1;
arm_biquad_casd_df1_inst_f32 S2;

int lead = 0;

int nextBuf = AUD_BUFF_ONE;
int audBuff1Stat = AUD_BUFF_DONE;
int audBuff2Stat = AUD_BUFF_DONE;
short* outWritePtr = outBuff;
short* outReadPtr = outBuff;

int initAudioBuffers() {

	memset(audBuff1, INT16_MID, AUD_BUFF_LEN); 
	memset(audBuff2, INT16_MID, AUD_BUFF_LEN); 
	memset(outBuff, INT16_MID, OUT_BUFF_LEN); 
	//memset(floatBuff, INT16_MID, AUD_BUFF_LEN); 

	fx[0] = 0;
	fx[1] = 0;

	biquadGen(HIGHPASS_FILT, 6000.0, 0.707, 44100.0, pCoeffs);

	arm_biquad_cascade_df1_init_f32(&S1, 1, pCoeffs, pState1);
	arm_biquad_cascade_df1_init_f32(&S2, 1, pCoeffs, pState2);

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
	
	if (loop1 == LOOP_ON) {
		if (timestamp >= startStamp1 + (bpm / interval1) ) {
			f_lseek(&audFil1, 150);
			audBuff1Stat = AUD_BUFF_EMPTY;
			startStamp1 = startStamp1 + (bpm / interval1);
		}
	}

	if (loop2 == LOOP_ON) {
		if (timestamp >= startStamp2 + (bpm / interval2) ) {
			f_lseek(&audFil2, 150);
			audBuff1Stat = AUD_BUFF_EMPTY;
			startStamp2 = startStamp2 + (bpm / interval2);
		}
	}

	if (audBuff1Stat == AUD_BUFF_EMPTY) {
		rc = f_read(&audFil1, (char*)audBuff1, AUD_BUFF_LEN*2, &bw);
		if (bw == AUD_BUFF_LEN*2) {
			audBuff1Stat = AUD_BUFF_FULL;
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
		}
		else {
			audBuff2Stat = AUD_BUFF_DONE;
			nextBuf = AUD_BUFF_TWO; 
		}
	}


	
	//mix two samples together:
	if (audBuff1Stat == AUD_BUFF_FULL && audBuff2Stat == AUD_BUFF_FULL) {
		
		for (int i=0; i < AUD_BUFF_LEN; i++) {
			res = (audBuff1[i]>>SHIFT_CONST) + (audBuff2[i]>>SHIFT_CONST);

			if (res > INT16_MAX)
				*outWritePtr = INT16_MAX;
			else if (res < INT16_MIN)
				*outWritePtr = INT16_MIN;
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
			outWritePtr[i] = audBuff1[i] >> SHIFT_CONST;
		outWritePtr += AUD_BUFF_LEN;
		audBuff1Stat = AUD_BUFF_EMPTY;
	}
	else if (audBuff2Stat == AUD_BUFF_FULL) {
		//memcpy(outWritePtr, audBuff2, AUD_BUFF_LEN*2); 
		for(int i=0; i < AUD_BUFF_LEN; i++)
			outWritePtr[i] = audBuff2[i] >> SHIFT_CONST;
		outWritePtr += AUD_BUFF_LEN;
		audBuff2Stat = AUD_BUFF_EMPTY;
	}
	//if nothing playing set the buffer to mid val.
	else {
		for (int i=0; i < AUD_BUFF_LEN; i++)
			outWritePtr[i] = INT16_MID;
		outWritePtr += AUD_BUFF_LEN;
	}



	//perform effects.
	samplesWritten = outWritePtr - refOutWritePtr;

	for (int i=0; i < 2; i++) {

		if (fx[i] == BITCRUSHER_DECIMATOR) {

			int bitAmount = adcReadings[i*2+1] / 256;
			int decAmount = adcReadings[i*2] / 200;
			
			//bitcrushing.
			for (int j=0; j < samplesWritten; j++) {
				
				//if (adcReadings[i*2] != 0) 
				refOutWritePtr[j] = (refOutWritePtr[j] >> bitAmount) << bitAmount;

			}

			
			//decimating.
			for (int j=0; j < samplesWritten; j++) {

				if (decCount[i] >= decAmount) {
					decValPrev[i] = decValCurr[i];
					decValCurr[i] = refOutWritePtr[j];
					decCount[i] = 0;
				}
				
				refOutWritePtr[j] = decValPrev[i] + ( (decValCurr[i] - decValPrev[i]) / decAmount) * decCount[i];

				decCount[i]++;
			}
		}
		else if (fx[i] == BITWISE_KO) {

			short andVal = adcReadings[i*2] * (1 << 1);
			short xorVal = adcReadings[i*2+1] * (1 << 1);
			for (int j=0; j < samplesWritten; j++) {
				refOutWritePtr[j] = refOutWritePtr[j] & andVal;
				refOutWritePtr[j] = refOutWritePtr[j] ^ xorVal;

			}
		}
		else if (fx[i] == FIR_DELAY){

			int delayVal = adcReadings[i*2];
			int ampVal = adcReadings[i*2+1];



			FIL* currFil;
			/* Forward 3000 bytes */
			if (audBuff1Stat == AUD_BUFF_EMPTY) {
    			currFil = &audFil1;

	    		rc = f_lseek(currFil, f_tell(currFil) + delayVal*10);

	    		if (f_size(currFil) != f_tell(currFil)) {


		    		rc = f_read(currFil, (char*)floatBuff, samplesWritten*2, &bw);

		    		for(int j=0; j < samplesWritten; j++) {

		    			refOutWritePtr[j] += ( ((short*)floatBuff)[j] / 10);
		    		}

		    		rc = f_lseek(currFil, f_tell(currFil) - delayVal*10 - samplesWritten*2);
	    		}

	    	}

	    	
    		if (audBuff2Stat == AUD_BUFF_EMPTY) {
    			currFil = &audFil2;

    			rc = f_lseek(currFil, f_tell(currFil) + delayVal*10);

    			if (f_size(currFil) != f_tell(currFil)) {

		    		rc = f_read(currFil, (char*)floatBuff, samplesWritten*2, &bw);

		    		for(int j=0; j < samplesWritten; j++) {

		    			refOutWritePtr[j] += ( (float)((short*)floatBuff)[j] * (float)ampVal/40000.0f );
		    		}

		    		rc = f_lseek(currFil, f_tell(currFil) - delayVal*10 - samplesWritten*2);
		    	}

    		}
    		


		}
		else if (fx[i] == IIR_ECHO) {

			int delayVal = adcReadings[i*2] / 3;
			int ampVal = adcReadings[i*2+1] / 2;
			long res;
			
			for (int j=0; j < samplesWritten; j++) {

				if (&refOutWritePtr[j] - outBuff < delayVal) {
					//refOutWritePtr[j] += refOutWritePtr[OUT_BUFF_LEN - (delayVal - (&refOutWritePtr[j] - outBuff))] * ampVal;
					//refOutWritePtr[j] = refOutWritePtr[j];
					res = ((long)refOutWritePtr[j]) + ( ( (long)refOutWritePtr[OUT_BUFF_LEN - (delayVal - (&refOutWritePtr[j] - outBuff) )]) * ampVal ) / MAX_ADC_VAL;
				}
				else {

					res = ((long)refOutWritePtr[j]) + ( ( (long)refOutWritePtr[j - delayVal]) * ampVal ) / MAX_ADC_VAL;
				}

				
				if (res >= INT16_MAX)
					refOutWritePtr[j] = INT16_MAX;
				else if (res < INT16_MIN)
					refOutWritePtr[j] = INT16_MIN;
				else
					refOutWritePtr[j] = (short) res;
			}
		}
		else if (fx[i] == LOWPASS_FILT || fx[i] == HIGHPASS_FILT || fx[i] == BANDPASS_FILT || fx[i] == NOTCH_FILT) {

			arm_biquad_casd_df1_inst_f32* currState;

			if (i == 0)
				currState = &S1;
			else if(i==1)
				currState = &S2;

			int fcVal = adcReadings[i*2];
			int qVal = (int)adcReadings[i*2+1];


			if (qVal > 0) {
				//while(1);
				qVal = qVal;
			}
			else 
				qVal = 4000;


			//for (int j=0; j < samplesWritten; j++) {
			//	refOutWritePtr[j] = refOutWritePtr[j] >> 2;
			//}

			for (int j=0; j < samplesWritten; j++) {
				floatBuff[j] = ( (float32_t)refOutWritePtr[j] ) / 32768.0 ;
			}

			if (fx[i] == HIGHPASS_FILT)
				biquadGen(fx[i], ((float)fcVal+4.0f)*5.0f, ((float)qVal * 20.0f + 1000.0f) / 4096.0f , 44100.0, pCoeffs);
			else if (fx[i] == LOWPASS_FILT)
				biquadGen(fx[i], 8250.0f - ((float)fcVal+4.0f)*2.0, ((float)qVal * 20.0f + 1000.0f) / 4096.0f , 44100.0, pCoeffs);
			else if(fx[i] == NOTCH_FILT)
				biquadGen(fx[i], ((float)fcVal+4.0f)*5.0f, ((float)qVal + 20.0f) / 4096.0f , 44100.0, pCoeffs);
			else if(fx[i] == BANDPASS_FILT)
				biquadGen(fx[i], ((float)fcVal+4.0f)*5.0f, ((float)qVal + 20.0f) / 4096.0f , 44100.0, pCoeffs);
			//biquadGen(fx[i], ((float)fcVal+4.0f)*5.0f, 100.0f, 44100.0, pCoeffs);
			//biquadGen(fx[i], ((float)fcVal)*5.0, 0.9, 44100.0, pCoeffs);

			arm_biquad_cascade_df1_f32(currState, floatBuff, floatBuff, FILT_BLOCK_SIZE);

			for (int j=0; j < samplesWritten; j++) {

				refOutWritePtr[j] = (short)(floatBuff[j] * 32768.0);
			}
		}
		else if (fx[i] == SAMPLE_SHIFT) {

			unsigned int sampleVal = (unsigned int)((float)adcReadings[i*2] * 2.0f * 44100.0f / 4096.0f);

			if (sampleVal < 100 || sampleVal > 88000)
				sampleVal = 44100;

			ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() / sampleVal );
		}

		

		//NORMALIZE!

		/*
		short maxVal = 0;

		for (int j=0; j < samplesWritten; j++) {

			if (maxVal < refOutWritePtr[j])
				maxVal = refOutWritePtr[j];
		}

		if (maxVal > 5) {

			for (int j=0; j < samplesWritten; j++) {

				refOutWritePtr[j] =  (short)( (float)refOutWritePtr[j] * ( 30000.0f / (float)maxVal) );
			}
		}
		*/

	

	}

	


	lead += (outWritePtr - refOutWritePtr);

	

		//loop to start of outBuff if at the end.
	if (outWritePtr == (&outBuff[OUT_BUFF_LEN]) ) { //at last value of outBuff:
		outWritePtr = outBuff;

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

	    outReadPtr[pos] >>= 2;

	    
	    if (outReadPtr[pos] >= INT12_MAX)
	    	outReadPtr[pos] = INT12_MAX - 1;
	    else if (outReadPtr[pos] <= INT12_MIN)
	    	outReadPtr[pos] = INT12_MIN + 1;
	    	
	    
	    SPIWriteDAC(0x0007, (unsigned short)outReadPtr[pos] + (unsigned short)INT12_MAX );

	    /*
	    	SPIWriteDAC(0x0003, test++ );
	 		if (test == 4095)
	 			test = 0;
	 			*/
	 			
	    //SPIWriteDAC(0x0007, 0xAAAA);
	    
        

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


    w0 = 2.0*3.14*(Fc/Fs); // digital cutoff frequency
    c1 = arm_cos_f32(w0);
    alpha = arm_sin_f32(w0) / (2.0 * Q);


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
    
    pCoeffs[0] = b0/a0;
    pCoeffs[1] = b1/a0;
    pCoeffs[2] = b2/a0;
	pCoeffs[3] = (-1.0*a1)/a0;
    pCoeffs[4] = (-1.0*a2)/a0;
    

    return 0;

}
