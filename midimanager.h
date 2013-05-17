#ifndef MIDI_H
#define MIDI_H

#include "engg4810.h"

typedef struct {
  uint8_t msg[3];
} midi_t;




/** midi packet constants for the MPC
    see http://home.roadrunner.com/~jgglatt/tech/midispec.htm for more info
**/
#define MIDI_STATUS_NOTE_ON 0x9
#define MIDI_STATUS_NOTE_OFF 0x8

#define MIDI_CHANNEL_0 0x0

#define MIDI_CONTROL_NOTE(button) (button)

#define MIDI_VALUE_NOTE_VELOCITY 100

#define MIDI_BUTTON_OFFSET 24


    
void midiMessage(midi_t* m, uint8_t stat, uint8_t ch, uint8_t ctrl, uint8_t val);



/** 32-bit midi message: a 3-character payload and a null terminator.
    can be safely cast to (uint8_t *) for serial transmission
**/

#endif

