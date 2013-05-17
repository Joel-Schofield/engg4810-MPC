#include "midimanager.h"



void midiMessage(midi_t* m, uint8_t stat, uint8_t ch, uint8_t ctrl, uint8_t val) {
/** returns a midi message struct.
    
    stat: MIDI status [0..16]
      when a button is pressed, use 9 (NOTE_ON).
      when a button is released,use 8 (NOTE_OFF).
      
    ch:   MIDI channel [0..16] - MPC will use channel 0
    
    ctrl: MIDI feature (for NOTE_ON or NOTE_OFF, this is a note identifier)
                       (MPC will use notes 0..15, to correspond with buttons)
                                 
    val:  value to apply [0..127] 
      for NOTE_ON and NOTE_OFF, this is the attack/decay rate.
      Our project doesn't implement this; just set it to default (64)
**/
  //midi_t m;
  m->msg[0] = (uint8_t)(((stat&0x0F)<<4) | (ch&0X0F));
  m->msg[1] = (uint8_t)( ctrl&0x7F);
  m->msg[2] = (uint8_t)( val &0x7F);
  //m->msg[3] = 0x0;
  //return m;
}
