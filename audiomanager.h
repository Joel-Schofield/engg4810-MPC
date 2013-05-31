#ifndef AUDIOMANAGER_H_
#define AUDIOMANAGER_H_

#include "engg4810.h"

int initAudioBuffers();
FIL* startAudio(char* fileNameStr, int loop, char interval);
int startEffect(int effect, int fxSlot);
int stopEffect(int fxSlot);
int getEffect(int fxSlot);
int stepAudioProcess();
int biquadGen(int type, float Fc, float Q, float Fs, float32_t* pCoeffs);

#endif
