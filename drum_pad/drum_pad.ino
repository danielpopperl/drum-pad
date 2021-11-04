#include <MIDIUSB.h>
#include "Controller.h"


/*************************************************************
  MIDI CONTROLLER

  by Notes and Volts
  www.notesandvolts.com

  Version 1.2 **Arduino UNO ONLY!**
 *************************************************************/


//************************************************************
//***SET THE NUMBER OF CONTROLS USED**************************
//************************************************************
//---How many buttons are connected directly to pins?---------
byte NUMBER_BUTTONS = 0;
//---How many potentiometers are connected directly to pins?--
byte NUMBER_POTS = 0;
//---How many buttons are connected to a multiplexer?---------
byte NUMBER_MUX_BUTTONS = 0;
//---How many potentiometers are connected to a multiplexer?--
byte NUMBER_MUX_POTS = 2;
//************************************************************

//***ANY MULTIPLEXERS? (74HC4067)************************************
//MUX address pins must be connected to Arduino UNO pins 2,3,4,5
//A0 = PIN2, A1 = PIN3, A2 = PIN4, A3 = PIN5
//*******************************************************************
//Mux NAME (OUTPUT PIN, , How Many Mux Pins?(8 or 16) , Is It Analog?);


//Mux M1(10, 16, false); //Digital multiplexer on Arduino pin 10
Mux M2(A0, 16, true); //Analog multiplexer on Arduino analog pin A0
//*******************************************************************


//***DEFINE DIRECTLY CONNECTED POTENTIOMETERS************************
//Pot (Pin Number, Command, CC Control, Channel Number)
//**Command parameter is for future use**

//Pot PO1(A0, 0, 1, 1);
//Pot PO2(A1, 0, 10, 1);
//Pot PO3(A2, 0, 22, 1);
//Pot PO4(A3, 0, 118, 1);
//Pot PO5(A4, 0, 30, 1);
//Pot PO6(A5, 0, 31, 1);
//*******************************************************************
//Add pots used to array below like this->  Pot *POTS[] {&PO1, &PO2, &PO3, &PO4, &PO5, &PO6};
Pot *POTS[] {};
//*******************************************************************


//***DEFINE DIRECTLY CONNECTED BUTTONS*******************************
//Button (Pin Number, Command, Note Number, Channel, Debounce Time)
//** Command parameter 0=NOTE  1=CC  2=Toggle CC **

//Button BU1(2, 0, 60, 1, 5 );
//Button BU2(3, 0, 61, 1, 5 );
//Button BU3(4, 0, 62, 1, 5 );
//Button BU4(5, 0, 63, 1, 5 );
//Button BU5(6, 0, 64, 1, 5 );
//Button BU6(7, 0, 65, 1, 5 );
//Button BU7(8, 1, 64, 1, 5 );
//Button BU8(9, 2, 64, 1, 5 );
//*******************************************************************
//Add buttons used to array below like this->  Button *BUTTONS[] {&BU1, &BU2, &BU3, &BU4, &BU5, &BU6, &BU7, &BU8};
Button *BUTTONS[] {};
//*******************************************************************


//***DEFINE BUTTONS CONNECTED TO MULTIPLEXER*************************
//Button::Button(Mux mux, byte muxpin, byte command, byte value, byte channel, byte debounce)
//** Command parameter 0=NOTE  1=CC  2=Toggle CC **

//Button MBU1(M1, 0, 0, 70, 1, 5);
//Button MBU2(M1, 1, 1, 71, 1, 5);
//Button MBU3(M1, 2, 2, 72, 1, 5);
//Button MBU4(M1, 3, 0, 73, 1, 5);
//Button MBU5(M1, 4, 0, 74, 1, 5);
//Button MBU6(M1, 5, 0, 75, 1, 5);
//Button MBU7(M1, 6, 0, 76, 1, 5);
//Button MBU8(M1, 7, 0, 77, 1, 5);
//Button MBU9(M1, 8, 0, 78, 1, 5);
//Button MBU10(M1, 9, 0, 79, 1, 5);
//Button MBU11(M1, 10, 0, 80, 1, 5);
//Button MBU12(M1, 11, 0, 81, 1, 5);
//Button MBU13(M1, 12, 0, 82, 1, 5);
//Button MBU14(M1, 13, 0, 83, 1, 5);
//Button MBU15(M1, 14, 0, 84, 1, 5);
//Button MBU16(M1, 15, 0, 85, 1, 5);
//*******************************************************************
////Add multiplexed buttons used to array below like this->  Button *MUXBUTTONS[] {&MBU1, &MBU2, &MBU3, &MBU4, &MBU5, &MBU6.....};
Button *MUXBUTTONS[] {};

//*******************************************************************


//***DEFINE POTENTIOMETERS CONNECTED TO MULTIPLEXER*******************
//Pot::Pot(Mux mux, byte muxpin, byte command, byte control/note, byte channel, byte type)
//** Command parameter 0=NOTE  1=CC **
//** Type parameter 0=Potentiometer 1=Piezzo **

Pot MPO1(M2, 0, 0, 48, 1, 1);
Pot MPO2(M2, 1, 1, 77, 1, 1);
//Pot MPO3(M2, 2, 0, 50, 1);
//Pot MPO4(M2, 3, 0, 55, 2);
//Pot MPO5(M2, 4, 0, 50, 1);
//Pot MPO6(M2, 5, 0, 55, 2);
//Pot MPO7(M2, 6, 0, 50, 1);
//Pot MPO8(M2, 7, 0, 55, 2);
//Pot MPO9(M2, 8, 0, 50, 1);
//Pot MPO10(M2, 9, 0, 55, 2);
//Pot MPO11(M2, 10, 0, 50, 1);
//Pot MPO12(M2, 11, 0, 55, 2);
//Pot MPO13(M2, 12, 0, 50, 1); &MPO5, &MPO6.....};
Pot *MUXPOTS[] {&MPO1, &MPO2};
//*******************************************************************


void setup() {
  //Serial1.begin(115200);
  Serial1.begin(9600);
  pinMode(6, OUTPUT);

  delay(200);
  pinMode(13, OUTPUT);
  Serial1.println("Hello Arduino Micro!");
}

void loop() {

  if (NUMBER_BUTTONS != 0) updateButtons();
  if (NUMBER_POTS != 0) updatePots();
  if (NUMBER_MUX_BUTTONS != 0) updateMuxButtons();
  if (NUMBER_MUX_POTS != 0) updateMuxPots();
}


//*****************************************************************
void updateButtons() {

  // Cycle through Button array
  for (int i = 0; i < NUMBER_BUTTONS; i = i + 1) {
    byte message = BUTTONS[i]->getValue();

    //  Button is pressed
    if (message == 0) {
      switch (BUTTONS[i]->Bcommand) {
        case 0: //Note
          noteOn(BUTTONS[i]->Bvalue, 127, BUTTONS[i]->Bchannel);
          MidiUSB.flush();
          break;
        case 1: //CC
          controlChange(BUTTONS[i]->Bvalue, 127, BUTTONS[i]->Bchannel);
          MidiUSB.flush();
          break;
        case 2: //Toggle
          if (BUTTONS[i]->Btoggle == 0) {
            controlChange(BUTTONS[i]->Bvalue, 127, BUTTONS[i]->Bchannel);
            MidiUSB.flush();
            BUTTONS[i]->Btoggle = 1;
          }
          else if (BUTTONS[i]->Btoggle == 1) {
            controlChange(BUTTONS[i]->Bvalue, 0, BUTTONS[i]->Bchannel);
            MidiUSB.flush();
            BUTTONS[i]->Btoggle = 0;
          }
          break;
      }
    }

    //  Button is not pressed
    if (message == 1) {
      switch (BUTTONS[i]->Bcommand) {
        case 0:
          noteOff(BUTTONS[i]->Bvalue, 0, BUTTONS[i]->Bchannel);
          MidiUSB.flush();
          break;
        case 1:
          controlChange(BUTTONS[i]->Bvalue, 0, BUTTONS[i]->Bchannel);
          MidiUSB.flush();
          break;
      }
    }
  }
}
//*******************************************************************
void updateMuxButtons() {

  // Cycle through Mux Button array
  for (int i = 0; i < NUMBER_MUX_BUTTONS; i = i + 1) {

    MUXBUTTONS[i]->muxUpdate();
    byte message = MUXBUTTONS[i]->getValue();

    //  Button is pressed
    if (message == 0) {
      switch (MUXBUTTONS[i]->Bcommand) {
        case 0: //Note
          noteOn(MUXBUTTONS[i]->Bvalue, 127, MUXBUTTONS[i]->Bchannel);
          MidiUSB.flush();
          break;
        case 1: //CC
          controlChange(MUXBUTTONS[i]->Bvalue, 127, MUXBUTTONS[i]->Bchannel);
          MidiUSB.flush();
          break;
        case 2: //Toggle
          if (MUXBUTTONS[i]->Btoggle == 0) {
            controlChange(MUXBUTTONS[i]->Bvalue, 127, MUXBUTTONS[i]->Bchannel);
            MidiUSB.flush();
            MUXBUTTONS[i]->Btoggle = 1;
          }
          else if (MUXBUTTONS[i]->Btoggle == 1) {
            controlChange(MUXBUTTONS[i]->Bvalue, 0, MUXBUTTONS[i]->Bchannel);
            MidiUSB.flush();
            MUXBUTTONS[i]->Btoggle = 0;
          }
          break;
      }
    }
    //  Button is not pressed
    if (message == 1) {
      switch (MUXBUTTONS[i]->Bcommand) {
        case 0:
          noteOff(MUXBUTTONS[i]->Bvalue, 0, MUXBUTTONS[i]->Bchannel);
          MidiUSB.flush();
          break;
        case 1:
          controlChange(MUXBUTTONS[i]->Bvalue, 0, MUXBUTTONS[i]->Bchannel);
          MidiUSB.flush();
          break;
      }
    }
  }
}
//***********************************************************************
void updatePots() {
  for (int i = 0; i < NUMBER_POTS; i = i + 1) {
    byte potmessage = POTS[i]->getValue(8);
    if (potmessage != 255) {
      controlChange(POTS[i]->Pcontrol, potmessage, POTS[i]->Pchannel);
      MidiUSB.flush();
    }
  }
}
//***********************************************************************
void updateMuxPots() {
  byte potmessage;

  for (int i = 0; i < NUMBER_MUX_POTS; i = i + 1) {
    MUXPOTS[i]->muxUpdate();
 
    if (MUXPOTS[i]->Ptype = 0) {
      potmessage = MUXPOTS[i]->getValue(8);
    }
    if (MUXPOTS[i]->Ptype = 1) {
      potmessage = MUXPOTS[i]->getValue(80);
    }

    //  Pot/Piezzo turn/hit
    if (potmessage != 255) {
      switch (MUXPOTS[i]->Pcommand) {
        case 0: //Note
          noteOn(MUXPOTS[i]->Pcontrol, 127, MUXPOTS[i]->Pchannel);
          MidiUSB.flush();
          break;
        case 1: //CC
          controlChange(MUXPOTS[i]->Pcontrol, potmessage, MUXPOTS[i]->Pchannel);
          MidiUSB.flush();
          break;
      }
    }
  }
}

//***********************************************************************
// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {pitch, velocity, 0x09, 0x90 | channel};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {pitch, velocity, 0x08, 0x80 | channel};
  MidiUSB.sendMIDI(noteOff);
}

// First parameter is the event type (0x0B = control change).
// Second parameter is the event type, combined with the channel.
// Third parameter is the control number number (0-119).
// Fourth parameter is the control value (0-127).

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {control, value, 0x0B, 0xB0 | channel};
  MidiUSB.sendMIDI(event);
}
