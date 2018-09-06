#include <SX2Arduino.h>

/*
     Test sketch for SX-bus interface for the ARduino
     All addresses are written with a value.
     In the background an ISR is running to send those values to the SX-bus.
     While sending the bus is read and put in a buffer

     Watch out: Adresses 104 to 111 are used. To avoid this set noCC2000 to false
*/

#include "Arduino.h"
#define DEBUG
// SX-bus interface
SX2Arduino SX;                 // Get access to the SX-bus

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 5000;           // interval at which to blink (milliseconds)
unsigned int misec = 0;
byte sxadr = 0;
ISR (INT0_vect)
{
  unsigned long currentmicros = micros();
  // if you want to understand this, see:
  // http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1239522239
  SX.isr();

  misec=micros()-currentmicros;
}

void setup()
{
  // put your setup code here, to run once:
  // Serial communication
  Serial.begin(230400);      // open the serial port
  // SX-bus communication
#ifdef DEBUG
  SX.init(&Serial);
#endif
  // Rising on INT1 triggers the interrupt routine sxisr (see above)
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
    //attachInterrupt(digitalPinToInterrupt(21), sxisr, RISING);
    EIMSK&=~(1<<INT0); //INTO off
    EICRA|=(1<<ISC01) | (1<<ISC00); // INT0 on rising edge
    EIMSK|=(1<<INT0);  //INT1 ON
  #else
    //attachInterrupt(0, sxisr, RISING);
  #endif

  delay(500);
}
uint8_t getSXbusval(int adr) {
  return SX.get(adr);
}

uint8_t getSXbuspower() {
  return SX.getPWR();
}

void setSXbusval(uint8_t adr, uint8_t val) {
  SX.set(adr, val);
}

void setSXbuspower(uint8_t onoff) {
  SX.setPWR(onoff);
}

void loop()
{
  unsigned long currentMillis = millis();
  byte frame;
  static uint16_t adrtmp,fkttmp=0;
  static uint8_t fsttmp,dirtmp,lighttmp=0;
  if (currentMillis - previousMillis >= interval)
  {
    frame = SX.regLoco(SX.calcSX2Par(130)<<2, 2);
    if (frame != 255)
    {
      while (SX.isSX2Set(frame));
      SX.setSX2Li(frame, true);
      Serial.println(F("****************************************************"));
      Serial.print (F("Lok is in Frame:"));
      Serial.println(frame);
      SX.set (10, 0);
      SX.set (100, 0xFF);
      SX.set (10|128, 0);
      SX.set (100|128, 0xFF);
      SX.setSX2Speed(frame, 30, 0);
      SX.setSX2Fkt(frame, 0);
      //while (SX.isSX2Set(frame));
      /*if ((SX.regPOM(130,2,1,99,0xFF)!=255))
        {
        Serial.println(F("POM Succesfull!"));
        }
      */
    }
    Serial.println(F("****************************************************"));
    Serial.print(F("SX ISR Laufzeit: "));
    Serial.print(misec);
    Serial.println(F(" Micro Sekunden"));
    for (int i = 0; i < 32; i++)
    {
      if (SX.checkSX2Frame(i))
      {

        Serial.println(F("****************************************************"));
        if (SX.returnSX2Adr(i) != 0xFFFF)
        {
          Serial.print(F("Adresse:"));
          Serial.print(SX.returnSX2Adr(i));
          switch (SX.returnSX2Format(i))
          {
            case (2):
              Serial.print(F("-->Motorola"));
              break;
            case (4):
              Serial.print(F("-->SX2 Betrieb"));
              break;
            case (1):
              Serial.print(F("-->DCC Kurz 28 Fst"));
              break;
            case (5):
              Serial.print(F("-->DCC Kurz 128 Fst"));
              break;
            case (3):
              Serial.print(F("-->DCC Lang 28 Fst"));
              break;
            case (7):
              Serial.print(F("-->DCC Lang 128 Fst"));
              break;

            default:
              Serial.print(F("Fehler"));
              break;
          }//Ende Switch
          Serial.println(" ");
          Serial.print(F("Licht:"));
          Serial.print(SX.returnSX2Li(i));
          Serial.print(F(" Richtung:"));
          Serial.print(SX.returnSX2Dir(i));
          Serial.print(F(" Speed:"));
          Serial.print(SX.returnSX2Fst(i)&127);
          Serial.print(F(" Funktion: "));
          Serial.print(SX.returnSX2Fkt(i), BIN);
          Serial.println(" ");
          Serial.print(F("PWR State SX Bus: "));
          Serial.println(SX.getPWR());
          Serial.print(F("SX Frame: "));
          Serial.println(i);
        }
      }
    }
    previousMillis = currentMillis;
  }
}
