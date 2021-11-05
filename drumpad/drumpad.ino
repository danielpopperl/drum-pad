//#include <MIDIUSB.h>
#include <Arduino.h>


//***********************************************************************
// Mapeamento dos pinos
#define SIG  A5   // pino analógico
#define S0   4
#define S1   5
#define S2   6
// o pino EN do mux vai no GND

int mix[3]={S2,S1,S0};

int valores_analogicos[8];

// Valores em binario 
byte ci[8][3]={
  {0,0,0},   // 0  em decimal
  {0,0,1},   // 1  em decimal
  {0,1,0},   // 2  em decimal
  {0,1,1},   // 3  em decimal
  {1,0,0},   // 4  em decimal
  {1,0,1},   // 5  em decimal
  {1,1,0},   // 6  em decimal
  {1,1,1},   // 7  em decimal
//  {1,0,0,0},   // 8  em decimal
//  {1,0,0,1},   // 9  em decimal
//  {1,0,1,0},   // 10 em decimal
//  {1,0,1,1},   // 11 em decimal  
//  {1,1,0,0},   // 12 em decimal
//  {1,1,0,1},   // 13 em decimal
//  {1,1,1,0},   // 14 em decimal
//  {1,1,1,1},   // 15 em decimal
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
//*************************************************************************
class Pot
{
  public:
    Pot(byte pin, byte command, byte control, byte channel, byte type);
    Pot(Mux mux, byte muxpin ,byte command, byte control, byte channel, byte type, int threshold);
    void muxUpdate();
    void newValue(byte command, byte value, byte channel);
    byte getValue();
    byte getDebounce(int i);
    byte getCommand();
    byte getTypeP();
    byte getMuxPin();
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
    int _threshold;
    byte _statusP;
    byte _timeP;
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
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  if (numPins > 8) pinMode(7, OUTPUT);
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

Pot::Pot(Mux mux, byte muxpin, byte command, byte control, byte channel, byte type, int threshold)
{
  _pin = mux.outpin;
  _numMuxPins = mux.numPins;
  _muxpin = muxpin;
  _control = control;
  _timeP = 0;
  _threshold = threshold;
  _statusP = 0b00000010;
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
//***SET THE NUMBER OF CONTROLS USED**************************
//************************************************************
//---How many buttons are connected directly to pins?---------
byte NUMBER_BUTTONS = 0;
//---How many potentiometers are connected directly to pins?--
byte NUMBER_POTS = 0;
//---How many buttons are connected to a multiplexer?---------
byte NUMBER_MUX_BUTTONS = 0;
//---How many potentiometers are connected to a multiplexer?--
const size_t NUMBER_MUX_POTS = 2;
//************************************************************


//***ANY MULTIPLEXERS? (74HC4067)************************************
//MUX address pins must be connected to Arduino Micro 4, 5, 6, 7
//*******************************************************************
//Mux NAME (OUTPUT PIN, , How Many Mux Pins?(8 or 16) , Is It Analog?);

//Mux M1(10, 16, false); //Digital multiplexer on Arduino pin 10
Mux M2(A5, 8, true); //Analog multiplexer on Arduino analog pin A5
//*******************************************************************


//***DEFINE POTENTIOMETERS CONNECTED TO MULTIPLEXER*******************
//Pot::Pot(Mux mux, byte muxpin, byte command, byte control/note, byte channel, byte type, byte threshold)
//** Command parameter 0=NOTE  1=CC **
//** Type parameter 0=Potentiometer 1=Piezzo **

Pot MUXPOTS[] = { 
  Pot(M2, 0, 0, 48, 1, 0, 0),
  Pot(M2, 1, 0, 48, 1, 1, 0),
};


//*******************************************************************



void setup() {
  Serial1.begin(9600);
//  Serial.begin(115200);
//  pinMode(LED_BUILTIN, OUTPUT);
  

  pinMode(11, OUTPUT);
  pinMode(S0,OUTPUT);
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);
}

void loop() {

  multiplex();
  for (int x = 0; x <= 7; x++){
     Serial1.print("Pino ");
     Serial1.print(x);
     Serial1.print(" = ");
     Serial1.println(valores_analogicos[x]);
   }
  Serial1.println("");
  delay(500);

//  Serial1.write("Ola");
//  digitalWrite(11, HIGH); // sets the digital pin 13 on
//  delay(500);            // waits for a second
//  digitalWrite(11, LOW);  // sets the digital pin 13 off
//  delay(500);            // waits for a second


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
byte Pot::getCommand(){
  return Pcommand;
}

byte Pot::getTypeP(){
  return Ptype;
}

byte Pot::getMuxPin(){
  return _muxpin;
}

void Pot::muxUpdate()
{
  byte temp = _muxpin;
  temp = temp << 2;
  if (_numMuxPins > 8) PORTD = PORTD & B11000011;
  else PORTD = PORTD & B11100011;
  //PORTD = PORTD & B11000011;
  PORTD = PORTD | temp;
}


void updateMuxPots() {
  byte potmessage;
  for (int i = 0; i < NUMBER_MUX_POTS; i = i + 1) {
    
    MUXPOTS[i].muxUpdate();
    
//    byte aa = MUXPOTS[i].getTypeP();
//    byte bb = MUXPOTS[i].getMuxPin();
//    Serial1.print("TYPE "); Serial1.println(aa);
//    Serial1.print("PORT "); Serial1.println(i);
//    Serial1.print("MUX PIN "); Serial1.println(bb);
//    Serial1.println(" ");
//    delay(1000);
    
    if (MUXPOTS[i].getTypeP() == 0) { 
      potmessage = MUXPOTS[i].getValue(); 
      Serial1.println("ty0");
      Serial1.println(potmessage);
      Serial1.println(" ");
    }
    if (MUXPOTS[i].getTypeP() == 1) { 
      potmessage = MUXPOTS[i].getDebounce(i); 
      Serial1.println("ty1");
      Serial1.println(potmessage);
      Serial1.println(" ");
    }

    //  Pot/Piezzo turn/hit
    if (potmessage != 255) {
      switch (MUXPOTS[i].getCommand()) {
        
        case 0: //Note
//          noteOn(MUXPOTS[i]->Pchannel, MUXPOTS[i]->Pcontrol, 127);
//          MidiUSB.flush();
          break;
        case 1: //CC
//          controlChange(MUXPOTS[i]->Pchannel, MUXPOTS[i]->Pcontrol, potmessage);
//          MidiUSB.flush();
          break;
      }
    }
  }
}

byte Pot::getValue()
{
  _value = analogRead(_pin); //0b01100000
  int tmp = (_oldValue - _value);

  if (tmp >= 8 || tmp <= -(8)) {
    _oldValue = _value >> 3; //0b00001100
    _oldValue = _oldValue << 3; //0b01100000
    return _value >> 3;
  }
  return 255;
}

byte Pot::getDebounce(int i)
{
  _value = valores_analogicos[i];
//  _value = analogRead(_pin);

  if(_value > 0)
  {
    // Check if debounce time has passed - If no, exit
    if (_timeP <  millis() - _threshold || _statusP == 0b00000010)
    {
      _timeP = millis();

      if (millis() - _threshold > _threshold) 
      {
        bitClear(_statusP, 1);
        bitSet(_statusP, 0);
      }
      return 1;
    }
  }
  return 255;
}



//***********************************************************************
//Função de leitura dos valores
void multiplex(){

  for(int x=0; x <= 7; x++){
   for (int y = 0; y <= 2; y++){
      digitalWrite (mix[y], ci[x][y]);   
    }
    valores_analogicos[x] = analogRead(SIG);
  }
  
}// fim do multiplex
