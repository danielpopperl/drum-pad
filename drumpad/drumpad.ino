#include <MIDIUSB.h>
#include <Arduino.h>
#include <TimerOne.h>

#define SIG  0

int potMessage = 0;
int potMessage2 = 0;

int _value = 0;
int _value2 = 0;
int _oldValue = 0;

unsigned long timer2 = 0;


const byte PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler


//***********************************************************************
int analogicValue[2] = {0};

//PORTB = B00010000 // port 8 digital
//PORTB = B00100000 // port 9 digital
//PORTB = B01000000 // port 10 digital
//PORTB = B10000000 // port 11 digital

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
    byte getChannel2();
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
Mux M2(0, 16, true); //Analog multiplexer on Arduino analog pin A5
//*******************************************************************


//***DEFINE POTENTIOMETERS CONNECTED TO MULTIPLEXER*******************
//Pot::Pot(Mux mux, byte muxpin, byte command, byte control/note, byte channel, byte type, unsigned long threshold, unsigned long timer)
//** Command parameter 0=NOTE  1=CC **
//** Type parameter 0=Potentiometer 1=Piezzo **

Pot MUXPOTS[] = {
  Pot(M2, 0, 0, 48, 1, 1, 200000, 100000),
  Pot(M2, 1, 1, 10, 1, 0, 100000, 100000),
};
//*******************************************************************


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  //  PRR1 |= 1 << PRUSB;
  //  pinMode(LED_BUILTIN, OUTPUT);

  //  ADCSRA |= PS_128; //Pre scaler 128

//  Timer1.initialize(1000);
//  Timer1.attachInterrupt(updateMuxPots);

  // Set PORTS 8, 9, 10 and 11 to LOW
  PORTB &= ~(1 << 4);
  PORTB &= ~(1 << 5);
  PORTB &= ~(1 << 6);
  PORTB &= ~(1 << 7);
}

void loop() {
//    multiplexReadPorts();

    for (int x2 = 0; x2 <= 1; x2++) {
      multiplexReadPorts(x2);
      Serial.print("Pino ");
      Serial.print(x2);
      Serial.print(" = ");
      Serial.println(analogicValue[x2] >> 3);
    }
    Serial.println("");
    delay(10);

  //  if (NUMBER_BUTTONS != 0) updateButtons();
  //  if (NUMBER_POTS != 0) updatePots();
  //  if (NUMBER_MUX_BUTTONS != 0) updateMuxButtons();
  //  if (NUMBER_MUX_POTS != 0) updateMuxPots();

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
  multiplexReadPorts(z);
  _value = analogicValue[z];

  int tmp = (_oldValue - _value);

  if (tmp >= 8 || tmp <= -(8)) {
    _oldValue = _value >> 3;
    _oldValue = _oldValue << 3;

    return _value >> 3;
  }

  return 255;
}

byte getDebounce(int w) {
  multiplexReadPorts(w);
  _value2 = analogicValue[w] >> 3;

  if ( micros() - MUXPOTS[w]._timer >= MUXPOTS[w]._debounce)
  {
    if ( _value2 >= 80 )
    {
      MUXPOTS[w]._timer = micros();

      return _value2;
    }
  }

  return 255;
}

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
  for (int x = 0; x <= 1; x++) {
    int gType = MUXPOTS[x].getTypeP();

    if (MUXPOTS[x].getTypeP() == 1) {
      potMessage = getDebounce(x);

      if (potMessage != 255) {
        if (MUXPOTS[x].getCommand() == 0) { // NOTE
          Serial.println("PIEZZO " + (String)potMessage);
          noteOn(MUXPOTS[x].Pchannel, MUXPOTS[x].Pcontrol, 127);
          MidiUSB.flush();
        }
      }
    }

    if (MUXPOTS[x].getTypeP() == 0) {
      potMessage2 = getValue(x);

      if (potMessage2 != 255) {
        if (MUXPOTS[x].getCommand() == 1) { // CC
          controlChange(MUXPOTS[x].Pchannel, MUXPOTS[x].Pcontrol, potMessage2);
          MidiUSB.flush();

          Serial.println("POT " + (String)potMessage2);
        }
      }
    }

  }
}

//***********************************************************************
void multiplexReadPorts(int por) {
  //  for (int x = 0; x <= 1; x++) {
  for (int y = 0; y <= 3; y++) {
    PORTB |= ci[por][y];
  }

  analogicValue[por] = analogRead(A0);
  //  }

  // Set PORTS 8, 9, 10 and 11 to LOW
  PORTB &= ~(1 << 4);
  PORTB &= ~(1 << 5);
  PORTB &= ~(1 << 6);
  PORTB &= ~(1 << 7);
}
