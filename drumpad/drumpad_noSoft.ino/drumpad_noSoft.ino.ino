#include <Arduino.h>
#include <TimerOne.h>

#define MUX_POT  A0
#define MUX_PIE  A3

//************************************************************
//---How many buttons are connected to a multiplexer?---------
#define NUMBER_MUX_BUTTONS 0;

//---How many potentiometers/piezzos are connected to a multiplexer?--
#define NUMBER_MUX_POTS 2
#define NUMBER_MUX_PIEZ 3
//************************************************************

byte potMessage[NUMBER_MUX_POTS] = {0};

int _valuePot[NUMBER_MUX_POTS] = {0};
int _oldValuePot[NUMBER_MUX_POTS] = {0};

int _valuePie[NUMBER_MUX_PIEZ] = {0};

const byte PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler

//***********************************************************************
int analogValuePot[NUMBER_MUX_POTS] = {0};
int analogValuePie[NUMBER_MUX_PIEZ] = {0};

// Valores em binario
byte ci[16][4] = {
  {B00000000, B00000000, B00000000, B00000000},
  {B00000000, B00000000, B00000000, B00010000},
  {B00000000, B00000000, B00100000, B00000000},
  {B00000000, B00000000, B00100000, B00010000},
  {B00000000, B01000000, B00000000, B00000000},
  {B00000000, B01000000, B00000000, B00010000},
  {B00000000, B01000000, B00100000, B00000000},
  {B00000000, B01000000, B00100000, B00010000},
  {B10000000, B00000000, B00000000, B00000000},
  {B10000000, B00000000, B00000000, B00010000},
  {B10000000, B00000000, B00100000, B00000000},
  {B10000000, B00000000, B00100000, B00010000},
  {B10000000, B01000000, B00000000, B00000000},
  {B10000000, B01000000, B00000000, B00010000},
  {B10000000, B01000000, B00100000, B00000000},
  {B10000000, B01000000, B00100000, B00010000},
  // PORT 11   PORT 10     PORT 9     PORT 8
};
//***********************************************************************



//***********************************************************************
class Mux
{
  public:
    Mux(byte outpin_, byte numPins_, bool analog_);
    byte outpin;
    byte numPins;
    bool analog;
};

class Pot
{
  public:
    Pot(byte pin, byte command, byte control, byte channel, byte type);
    Pot(Mux mux, byte muxpin , byte command, byte control, byte channel, byte type, unsigned long debounce, unsigned long timer);
    byte getValue(byte v);
    byte getCommand();
    byte getControl();
    byte getTypeP();
    byte getDebounce(byte d);
    byte Pcommand;
    byte Pcontrol;
    byte Pchannel;
    byte Ptype;
    unsigned long _debounce;
    unsigned long _timer;

  private:
    byte _pin;
    byte _muxpin;
    byte _numMuxPins;
    byte _control;
};
//*************************************************************************


//****************************************************************************************
Mux::Mux(byte outpin_, byte numPins_, bool analog_)
{
  outpin = outpin_;
  numPins = numPins_;
  analog = analog_;
  if (analog == false) pinMode(outpin, INPUT_PULLUP);
  else pinMode(outpin, INPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  if (numPins > 8) pinMode(11, OUTPUT);
}
//****************************************************************************************


//****************************************************************************************
Pot::Pot(Mux mux, byte muxpin, byte command, byte control, byte channel, byte type, unsigned long debounce, unsigned long timer)
{
  _pin = mux.outpin;
  _numMuxPins = mux.numPins;
  _muxpin = muxpin;
  _control = control;
  _timer = timer;
  _debounce = debounce;
  Pcommand = command;
  Pcontrol = control;
  Pchannel = channel;
  Ptype = type;
}
//****************************************************************************************


//***ANY MULTIPLEXERS? (74HC4067)************************************
//MUX address pins must be connected to Arduino Micro 4, 5, 6, 7
//Mux NAME (OUTPUT PIN, , How Many Mux Pins?(8 or 16) , Is It Analog?);

Mux M2(MUX_POT, 16, true); //MUX POTS
Mux M1(MUX_PIE, 16, true); //MIX PIEZZOS
//*******************************************************************


//***DEFINE POTENTIOMETERS CONNECTED TO MULTIPLEXER*******************
//Pot::Pot(Mux mux, byte muxpin, byte command, byte control/note, byte channel, byte type, unsigned long threshold, unsigned long timer)
//** Command parameter 0=NOTE  1=CC **
//** Type parameter 0=Potentiometer 1=Piezzo **

Pot MUXPIEZ[] = {
  Pot(M1, 0, 0, 48, 0, 1, 100000, 100000),
  Pot(M1, 1, 0, 58, 0, 1, 100000, 100000),
  Pot(M1, 2, 0, 49, 0, 1, 100000, 100000),
};

Pot MUXPOTS[] = {
  Pot(M2, 0, 1, 10, 0, 0, 99999, 100000),
  Pot(M2, 1, 1, 20, 0, 0, 99999, 100000),
};
//*******************************************************************


void setup() {

  delay(1000);

  //  PRR1 |= 1 << PRUSB;

  ADCSRA |= PS_128; //Pre scaler 128

  Timer1.initialize(1000);
  Timer1.attachInterrupt(multiplexReadPorts);
}

void loop() {
  //    for (int x2 = 0; x2 <= 15; x2++) {
  //      Serial.print("Pino ");
  //      Serial.print(x2);
  //      Serial.print(" = ");
  //      Serial.println(analogicValue[x2] >> 3);
  //    }
  //    Serial.println("");
  //    delay(10);

  if (NUMBER_MUX_POTS != 0) updateMuxPots();
  if (NUMBER_MUX_PIEZ != 0) updateMuxPiezzos();
}


//***********************************************************************
// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).

void noteOn(byte channel, byte pitch, byte velocity) {
  MIDIEvent  noteOn = { 0x09, 0x90 | channel, pitch, velocity };
  MIDIUSB.write(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  MIDIEvent  noteOff = {0x08, 0x80 | channel, pitch, velocity };
  MIDIUSB.write(noteOff);
}

// First parameter is the event type (0x0B = control change).
// Second parameter is the event type, combined with the channel.
// Third parameter is the control number number (0-119).
// Fourth parameter is the control value (0-127).

void controlChange(byte channel, byte control, byte value) {
  MIDIEvent  event = {0x0B, 0xB0 | channel, control, value };
  MIDIUSB.write(event);
}


//***********************************************************************
byte getValue(byte z) {
  _valuePot[z] = analogValuePot[z];

  int tmp = (_oldValuePot[z] - _valuePot[z]);

  if (tmp >= 8 || tmp <= -(8)) {
    _oldValuePot[z] = _valuePot[z] >> 3;
    _oldValuePot[z] = _oldValuePot[z] << 3;

    return _valuePot[z] >> 3;
  }

  return 255;
}

//byte getDebounce(int w) {
//  _value2 = analogicValue2[w] >> 3;
//
//  if ( micros() - MUXPOTS[w]._timer >= MUXPOTS[w]._debounce)
//  {
//    if ( _value2 >= 90  && _value2 <= 127)
//    {
//      MUXPOTS[w]._timer = micros();
//
//      return 1;
//    }
//  }
//
//  return 255;
//}

byte Pot::getCommand() {
  return Pcommand;
}

byte Pot::getTypeP() {
  return Ptype;
}


//void Pot::muxUpdate()
//{
//  byte temp = _muxpin;
//  temp = temp << 2;
//
//  if (_numMuxPins > 8) PORTD = PORTD & B11000011;
//  else PORTD = PORTD & B11100011;
//  //PORTD = PORTD & B11000011;
//
//  PORTD = PORTD | temp;
//}

void updateMuxPots() {
  for (byte x = 0; x <= NUMBER_MUX_POTS - 1; x++) {
    byte gType = MUXPOTS[x].getTypeP();
    potMessage[x] = getValue(x);

    if (MUXPOTS[x].getTypeP() == 0 && potMessage[x] != 255) {
      controlChange(MUXPOTS[x].Pchannel, MUXPOTS[x].Pcontrol, potMessage[x]);
      MIDIUSB.flush();
    }
  }
}

void updateMuxPiezzos() {
  for (byte x = 0; x <= NUMBER_MUX_PIEZ - 1; x++)
  {
    if ( MUXPIEZ[x].getCommand() == 0 ) {// NOTE
      _valuePie[x] = analogValuePie[x] >> 3;

      if ( micros() - MUXPIEZ[x]._timer >= MUXPIEZ[x]._debounce)
      {     
        if ( _valuePie[x] >= 70 )
        {
          MUXPIEZ[x]._timer = micros();
          noteOn(MUXPIEZ[x].Pchannel, MUXPIEZ[x].Pcontrol, 127);
          MIDIUSB.flush();
        }
      }
    }
  }
}

//***********************************************************************
void multiplexReadPorts() {
  for (byte x = 0; x <= NUMBER_MUX_PIEZ - 1; x++) {
    for (byte y = 0; y <= 3; y++) {
      PORTB |= ci[x][y];
    }

    analogValuePie[x] = analogRead(MUX_PIE);

    clearBytes();
  }
  
  for (byte x = 0; x <= NUMBER_MUX_POTS - 1; x++) {
    for (byte y = 0; y <= 3; y++) {
      PORTB |= ci[x][y];
    }

    analogValuePot[x] = analogRead(MUX_POT);

    clearBytes();
  }
}

void clearBytes(){
  //    Set PORTS 8, 9, 10 and 11 to LOW
  PORTB &= ~(1 << 4);
  PORTB &= ~(1 << 5);
  PORTB &= ~(1 << 6);
  PORTB &= ~(1 << 7);
}
