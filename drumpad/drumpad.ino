//#include <MIDIUSB.h>
#include <Arduino.h>
#include <TimerOne.h>

#define SIG  A5


const byte PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler


//***********************************************************************
int analogicValue[8];

//PORTB = B00010000 // port 8 digital
//PORTB = B00100000 // port 9 digital
//PORTB = B01000000 // port 10 digital

// Valores em binario
volatile byte ci[8][3] = {
  {B00000000, B00000000, B00000000},
  {B00000000, B00000000, B00010000},
  {B00000000, B00100000, B00000000},
  {B00000000, B00100000, B00010000},
  {B01000000, B00000000, B00000000},
  {B01000000, B00000000, B00010000},
  {B01000000, B00100000, B00000000},
  {B01000000, B00100000, B00010000},
  // PORT 10     PORT 9     PORT 8
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
    byte getDebounceV();
    byte getCommand();
    byte getTypeP();
    byte getMuxPin();
    byte getDebounce(int i);
    byte Pcommand;
    byte Pcontrol;
    byte Pchannel;
    byte Ptype;
    byte MUXPOTS;

  private:
    byte _pin;
    byte _muxpin;
    byte _numMuxPins;
    byte _control;
    int _value;
    int _oldValue;
    bool _changed;
    byte _enablepin;
    unsigned long _debounce;
    unsigned long _timer;
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
Pot::Pot(byte pin, byte command, byte control, byte channel, byte type)
{
  _pin = pin;
  _control = control;
  //  _value = analogRead(_pin);
  //  _value = _value >> 3;
  //  _oldValue = _value << 3;
  //  _value = _value << 3;
  Pcommand = command;
  Pcontrol = control;
  Pchannel = channel;
  Ptype = type;
}

Pot::Pot(Mux mux, byte muxpin, byte command, byte control, byte channel, byte type, unsigned long debounce, unsigned long timer)
{
  _pin = mux.outpin;
  _numMuxPins = mux.numPins;
  _muxpin = muxpin;
  _control = control;
  _timer = timer;
  _debounce = debounce;
  //  muxUpdate();
  //  _value = analogRead(_pin);
  //  _value = _value >> 3;
  //  _oldValue = _value << 3;
  //  _value = _value << 3;
  Pcommand = command;
  Pcontrol = control;
  Pchannel = channel;
  Ptype = type;
}
//****************************************************************************************


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
//MUX address pins must be connected to Arduino Micro 4, 5, 6, 7
//Mux NAME (OUTPUT PIN, , How Many Mux Pins?(8 or 16) , Is It Analog?);

//Mux M1(10, 16, false); //Digital multiplexer on Arduino pin 10
Mux M2(A5, 8, true); //Analog multiplexer on Arduino analog pin A5
//*******************************************************************


//***DEFINE POTENTIOMETERS CONNECTED TO MULTIPLEXER*******************
//Pot::Pot(Mux mux, byte muxpin, byte command, byte control/note, byte channel, byte type, unsigned long threshold, unsigned long timer)
//** Command parameter 0=NOTE  1=CC **
//** Type parameter 0=Potentiometer 1=Piezzo **

Pot MUXPOTS[] = {
  Pot(M2, 0, 0, 10, 1, 1, 300, 2000),
  Pot(M2, 1, 1, 48, 1, 1, 300, 2000),
  Pot(M2, 2, 1, 55, 1, 0, 0, 2000)
};
//*******************************************************************



void setup() {
  //  Serial1.begin(9600);
  Serial1.begin(115200);
  //  pinMode(LED_BUILTIN, OUTPUT);

  ADCSRA |= PS_128;

  Timer1.initialize(3000);
  Timer1.attachInterrupt(multiplexReadPorts);
}

void loop() {

  //  for (int x = 0; x <= 1; x++) {
  //    Serial1.print("Pino ");
  //    Serial1.print(x);
  //    Serial1.print(" = ");
  //    Serial1.println(analogicValue[x]);
  //  }
  //  Serial1.println("");
  //  delay(50);

  //  if (NUMBER_BUTTONS != 0) updateButtons();
  //  if (NUMBER_POTS != 0) updatePots();
  //  if (NUMBER_MUX_BUTTONS != 0) updateMuxButtons();
  if (NUMBER_MUX_POTS != 0) updateMuxPots();
}



//***********************************************************************
// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).

//void noteOn(byte channel, byte pitch, byte velocity) {
//  midiEventPacket_t noteOn = { 0x09, 0x90 | channel, pitch, velocity };
//  MidiUSB.sendMIDI(noteOn);
//}
//
//void noteOff(byte channel, byte pitch, byte velocity) {
//  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity };
//  MidiUSB.sendMIDI(noteOff);
//}
//
//// First parameter is the event type (0x0B = control change).
//// Second parameter is the event type, combined with the channel.
//// Third parameter is the control number number (0-119).
//// Fourth parameter is the control value (0-127).
//
//void controlChange(byte channel, byte control, byte value) {
//  midiEventPacket_t event = {0x0B, channel, control, value };
//  MidiUSB.sendMIDI(event);
//}


//***********************************************************************
byte Pot::getCommand() {
  return Pcommand;
}

byte Pot::getTypeP() {
  return Ptype;
}

byte Pot::getMuxPin() {
  return _muxpin;
}

byte Pot::getDebounceV() {
  return _debounce;
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
  byte potmessage;
  for (int i = 0; i < NUMBER_MUX_POTS; i = i + 1) {

    //    MUXPOTS[i].muxUpdate();

    if (MUXPOTS[i].getTypeP() == 0) {
      potmessage = MUXPOTS[i].getValue(i);
    }

    if (MUXPOTS[i].getTypeP() == 1) {
      potmessage = MUXPOTS[i].getDebounce(i);
    }

    //  Pot/Piezzo turn/hit
    if (potmessage != 255) {
      if (MUXPOTS[i].getCommand() == 0 ) {
        Serial1.println("NOTE " + (String)i);
        //        noteOn(MUXPOTS[i]->Pchannel, MUXPOTS[i]->Pcontrol, 127);
        //        MidiUSB.flush();
      }

      if (MUXPOTS[i].getCommand() == 1 ) {
        Serial1.println("CC " + (String) i);
        //          controlChange(MUXPOTS[i]->Pchannel, MUXPOTS[i]->Pcontrol, potmessage);
        //          MidiUSB.flush();
      }
    }
  }
}

byte Pot::getValue(int i) {
  _value = analogicValue[i];
  //  _value = analogRead(_pin); //0b01100000

  int tmp = (_oldValue - _value);

  if (tmp >= 8 || tmp <= -(8)) {
    _oldValue = _value >> 3; //0b00001100
    _oldValue = _oldValue << 3; //0b01100000
    return _value >> 3;
  }

  return 255;
}

byte Pot::getDebounce(int i) {
  _value = analogicValue[i];
  _value = _value >> 3;
  //  _value = analogRead(_pin);

  if ( _value > 70 )
  {
    if ( millis() - _timer >= _debounce)
    {
      _timer = millis();
      return 1;
    }
  }

  return 255;
}


//***********************************************************************
void multiplexReadPorts() {

  //    for(int x=0; x <= 7; x++){
  //     for (int y = 0; y <= 2; y++){
  //        digitalWrite (mix[y], ci[x][y]);
  //      }
  //      analogicValue[x] = analogRead(SIG);
  //    }

  for (int x = 0; x <= 1; x++) {
    for (int y = 0; y <= 2; y++) {
      PORTB = PORTB | ci[x][y];
    }
    analogicValue[x] = analogRead(SIG);

    // Set PORTS 8, 9 and 10 to LOW
    PORTB &= ~(1 << 4);
    PORTB &= ~(1 << 5);
    PORTB &= ~(1 << 6);
  }
}
