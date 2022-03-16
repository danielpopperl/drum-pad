#include <USB-MIDI.h>
#include <Arduino.h>
#include <TimerOne.h>


USBMIDI_CREATE_DEFAULT_INSTANCE();

#define MUX1  A0
#define MUX2  A3

//************************************************************
//---How many buttons are connected directly to pins?---------
byte NUMBER_BUTTONS = 0;
//---How many potentiometers are connected directly to pins?--
byte NUMBER_POTS = 0;
//---How many buttons are connected to a multiplexer?---------
byte NUMBER_MUX_BUTTONS = 0;
//---How many potentiometers are connected to a multiplexer?--
int NUMBER_MUX_POTS = 2;
int NUMBER_MUX_PIES = 3;
//************************************************************

int potMessage = 0;

int _valuePot[2] = {0};
int _oldValuePot[2] = {0};

int _valuePie[3] = {0};


const byte PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler


//***********************************************************************
int analogicValue[2] = {0};
int analogicValue2[3] = {0};

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
    void muxUpdate();
    void newValue(byte command, byte value, byte channel);
    byte getValue(int i);
    byte getCommand();
    byte getControl();
    byte getTypeP();
    byte getDebounce(int i);
    byte Pcommand;
    byte Pcontrol;
    byte Pchannel;
    byte Ptype;
    byte MUXPOTS;
    unsigned long _debounce;
    unsigned long _timer;

  private:
    byte _pin;
    byte _muxpin;
    byte _numMuxPins;
    byte _control;
    bool _changed;
    byte _enablepin;
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

Mux M2(A0, 16, true); //MUX POTS
Mux M1(A3, 16, true); //MIX PIEZZOS
//*******************************************************************


//***DEFINE POTENTIOMETERS CONNECTED TO MULTIPLEXER*******************
//Pot::Pot(Mux mux, byte muxpin, byte command, byte control/note, byte channel, byte type, unsigned long threshold, unsigned long timer)
//** Command parameter 0=NOTE  1=CC **
//** Type parameter 0=Potentiometer 1=Piezzo **

Pot MUXPIES[] = {
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
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  MIDI.begin(1);

  delay(1000);

  //  PRR1 |= 1 << PRUSB;

  ADCSRA |= PS_128; //Pre scaler 128

  Timer1.initialize(2000);
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

  //  if (NUMBER_BUTTONS != 0) updateButtons();
  //  if (NUMBER_POTS != 0) updatePots();
  //  if (NUMBER_MUX_BUTTONS != 0) updateMuxButtons();

  if (NUMBER_MUX_POTS != 0) updateMuxPots();
  if (NUMBER_MUX_PIES != 0) updateMuxPiezzos();
}


//***********************************************************************
// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = { 0x09, 0x90 | channel, pitch, velocity };
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity };
  MidiUSB.sendMIDI(noteOff);
}

// First parameter is the event type (0x0B = control change).
// Second parameter is the event type, combined with the channel.
// Third parameter is the control number number (0-119).
// Fourth parameter is the control value (0-127).

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value };
  MidiUSB.sendMIDI(event);
}


//***********************************************************************
byte getValue(int z) {
  _valuePot[z] = analogicValue[z];

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
  for (int x = 0; x <= NUMBER_MUX_POTS-1; x++) {
    int gType = MUXPOTS[x].getTypeP();
    potMessage = getValue(x);

    if (MUXPOTS[x].getTypeP() == 0 && potMessage != 255) {
      controlChange(MUXPOTS[x].Pchannel, MUXPOTS[x].Pcontrol, potMessage);
      MidiUSB.flush();
    }
  }
}

void updateMuxPiezzos() {
  for (int x = 0; x <= NUMBER_MUX_PIES-1; x++)
  {
    if ( MUXPIES[x].getCommand() == 0 ) {// NOTE
      _valuePie[x] = analogicValue2[x] >> 3;

      if ( micros() - MUXPIES[x]._timer >= MUXPIES[x]._debounce)
      {     
        if ( _valuePie[x] >= 50 )
        {
          MUXPIES[x]._timer = micros();
          noteOn(MUXPIES[x].Pchannel, MUXPIES[x].Pcontrol, 127);
          MidiUSB.flush();
        }
      }
    }
  }
}

//***********************************************************************
void multiplexReadPorts() {
  for (int x = 0; x <= 2; x++) {
    for (int y = 0; y <= 3; y++) {
      PORTB |= ci[x][y];
    }

    analogicValue[x] = analogRead(MUX1);
    analogicValue2[x] = analogRead(MUX2);

    //    Set PORTS 8, 9, 10 and 11 to LOW
    PORTB &= ~(1 << 4);
    PORTB &= ~(1 << 5);
    PORTB &= ~(1 << 6);
    PORTB &= ~(1 << 7);
  }
}
