/********VERSION 0.2******************************/
/********17.09.2018*******************************/
#include <SX2Arduino.h>
#if defined(ARDUINO_ARCH_AVR)
#include <avr/io.h>
#endif
#include "Arduino.h"
#define DEBUG
#define FWVERSION1    1
//#define FWVERSION2    6
#define FWVERSION2    13
#define DH          //D&H Mode for faster repsonse Time, may cause failures!
#define SX2LOCOS      32

#define COMPORTTIMEOUT 2000 //Comport Timout after 2000ms
#define RX_BUFFSIZE 5
#define BUFFSXCHAN 0
#define BUFFSXVAL 1

#define BUFFSX2CMD 0
#define BUFFSX2INDEX 1
#define BUFFSX2FST 2
#define BUFFSX2FORMAT 3

#define LIGHTBIT 0
#define FSTBIT   1

#if defined(ARDUINO_ARCH_SAM)
void serialEventUSB(void);
#endif
// SX-bus interface
SX2Arduino SX;                 // Get access to the SX-bus
struct _sx2_loco_values
{
  uint8_t frame;
  uint8_t pr;       //to store SX2 Präambel
  unsigned int adr;           //to store SX2 Adr
  bool dcc;
  bool dir;
  uint8_t cnt;
};
#if defined(ARDUINO_ARCH_AVR)
HardwareSerial* output;
#elif defined(ARDUINO_ARCH_SAM)
Serial_* output;
#endif
#ifdef DEBUG
HardwareSerial* debug;
#endif
uint8_t cmdRcvd;                     //To store Serial Command Recived
uint8_t rxBuffer[RX_BUFFSIZE];                 //to store serial data recived after command
//struct _sx2_loco_values sx2loco[SX2LOCOS];     //to store active locos with format
uint8_t zeformat;                //to store the current ze format
bool noserial = true;
bool locoActive[SX2LOCOS];
#if defined(ARDUINO_ARCH_AVR)
ISR (INT0_vect)
#elif defined(ARDUINO_ARCH_SAM)
void sxisr()
#endif
{
  volatile static uint8_t i = 0;
  SX.isr();
  if (i < SX2LOCOS)
  {
    i++;
  }
  else
  {
    i = 0;
  }
  if (locoActive[i] == true)
  {
    SX.isrholdLoco(i);
  }
}
void setup()
{
#if defined(ARDUINO_ARCH_AVR)
  output = &Serial;
#elif defined(ARDUINO_ARCH_SAM)
  output = &SerialUSB;
#ifdef DEBUG
  debug = &Serial2;
  debug->begin(19200);
#endif
#endif
  for (uint8_t i = 0; i < SX2LOCOS; i++)
  {
    locoActive[i] = false;
  }

  for ( uint8_t i = 0; i < RX_BUFFSIZE; i++)
  {
    rxBuffer[i] = 0xFF;
  }
#ifdef DEBUG
  SX.init(debug);
#else
  SX.init();
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
  //attachInterrupt(digitalPinToInterrupt(21), sxisr, RISING);
  EIMSK &= ~(1 << INT0); //INTO off
  EICRA |= (1 << ISC01) | (1 << ISC00); // INT0 on rising edge
  EIMSK |= (1 << INT0); //INT1 ON
#elif defined(ARDUINO_ARCH_SAM)
  attachInterrupt(digitalPinToInterrupt(42), sxisr, RISING);
#endif
  //delay(500);
  output->begin(230400);      // open the serial port

  //delay(500);
}
#if defined(ARDUINO_ARCH_SAM)
void serialEventUSB()
#else
void serialEvent()
#endif
{
  // Read all the data
  uint8_t databytes = 0;              //to defines datbytes to read vom buffer
  uint8_t serialreturn = 0xFF;           //to store date to return to pc
  uint16_t sx2fkt, sx2adress, sx2format = 0;
  noserial = false; //Serial Connection is active
  cmdRcvd = ( uint8_t) output->read();            //Read first incoming byte
  switch (cmdRcvd)
  {
    case (0x00):
      databytes = 2;
      break;
    case (0x01):
      databytes = 2;
      break;
    case (0x77):
      databytes = 2;
      break;
    case (0x78):
      databytes = 1;
      break;
    case (0x79):
      databytes = 4;
      break;
    case (0x7A):
      databytes = 5;
      break;
    case (0x7B):
      databytes = 5;
      break;
    case (0x81):
      databytes = 0;
      break;
    case (0x83):
      databytes = 4;
      break;
    default:
      databytes = 0;
      break;
  }
  if (cmdRcvd != 0xFF)
  {
    output->readBytes(rxBuffer, databytes);            //Read Data bytes into Buffer
#if defined(ARDUINO_ARCH_AVR)
    delayMicroseconds(600);                           //Wait a bit for next command
#endif
    if (output->available() != 0) //More recived then expected, reset serial!
    {
      cmdRcvd = 0xFF;
#ifdef DEBUG
      debug->print("COM Port Fail! Bytes still available in buffer:");
      debug->println(output->available());
#endif
    }

  }
  //break;
  //}// //End while
  switch (cmdRcvd)      //Decode command
  {
    case (0x00):
      if (rxBuffer[BUFFSXCHAN] == 0xFF)     //Set track Power on/off
      {
        if (rxBuffer[BUFFSXVAL] == 0)
        {
          SX.setPWR(0);
        }
        else
        {
          SX.setPWR(1);
        }
        output->write((uint8_t)0x00);            //Return 0x00 to pc
        break;
      }
      else if (rxBuffer[BUFFSXCHAN] == 0x7F)   //Get Track power
      {
        output->write((uint8_t) SX.getPWR());                      //Return Track Power to PC
        break;
      }
      else if (bitRead(rxBuffer[BUFFSXCHAN], 7))                                      //Write SX Channel if BIT 7 is set
      {
        bitClear(rxBuffer[BUFFSXCHAN], 7);                                            //Clear Bit 7
        output->write((uint8_t)SX.set(rxBuffer[BUFFSXCHAN], rxBuffer[BUFFSXVAL] ));         //Write on sx BUS
        break;
      }
      else if (rxBuffer[BUFFSXCHAN] < 112)                          //Return SX Channel to PC
      {
        output->write((uint8_t)SX.get(rxBuffer[BUFFSXCHAN]));
        break;
      }
      break;
    case (0x01):
      if (rxBuffer[BUFFSXCHAN] == 0xFF)     //Set track Power on/off
      {
        if (rxBuffer[BUFFSXVAL] == 0)
        {
          SX.setPWR(0);
        }
        else
        {
          SX.setPWR(1);
        }
        output->write((uint8_t)0x00);            //Return 0x00 to pc
        break;
      }
      else if (rxBuffer[BUFFSXCHAN] == 0x7F)   //Get Track power
      {
        output->write((uint8_t) SX.getPWR());                      //Return Track Power to PC
        break;
      }
      else if (bitRead(rxBuffer[BUFFSXCHAN], 7))                                      //Write SX Channel if BIT 7 is set
      {
        bitClear(rxBuffer[BUFFSXCHAN], 7);                                            //Clear Bit 7
        output->write((uint8_t)SX.set(rxBuffer[BUFFSXCHAN] | 128, rxBuffer[BUFFSXVAL] ));   //Write on sx 1 BUS
        break;
      }
      else if (rxBuffer[BUFFSXCHAN] < 112)                          //Return SX Channel to PC
      {
        output->write((uint8_t)SX.get(rxBuffer[BUFFSXCHAN] | 128));                         //Get sx 1 Bus
        break;
      }
      break;
    case (0x79):                                          //Handle SX2 or DCC Loco
      switch (rxBuffer[BUFFSX2CMD])
      {
        case (0x01):                                    //Log on loco on SX Bus
          sx2adress = 0;
          rxBuffer[2] &= ~3;                           //Delte first two bits
          sx2adress = ((rxBuffer[1] << 8 ) | rxBuffer[2]);   //Generate Adress
          switch (rxBuffer[BUFFSX2FORMAT])                        //Log on Loco with the correct format
          {
            case (0x04):                                          //Loco with SX2
              if (zeformat == 2 || zeformat == 4 || zeformat == 5 || zeformat == 11) //Check if Format is currently active on SX Bus
              {
#ifdef DEBUG
                debug->println(F("REG SX2 Adress"));
#endif
                sx2format = 2;
#ifdef DH
                serialreturn = SX.regLoco(sx2adress, sx2format, output);
#else
                serialreturn = SX.regLoco(sx2adress, sx2format);
#endif
                if (serialreturn != 0xFF)
                {
                  SX.holdLoco(serialreturn, sx2format, sx2adress);
                  locoActive[serialreturn] = true;
                }
                SX.setSX2Dccfst(serialreturn, false);             //Clear Bit
              }
              else
              {
                serialreturn = 0xFF;                      //Return Fault
              }
              break;
            case (0x91):                                               //Loco with DCC short Address 14FS
              if ((sx2adress >> 2) > 127)
              {
                serialreturn = 0xFF;//Return Fault
              }
              else if (zeformat == 4 || zeformat == 6 || zeformat == 11) //Check if Format is currently active on SX Bus
              {
                sx2format = 8;
#ifdef DH
                serialreturn = SX.regLoco(sx2adress, sx2format, output);
#else
                serialreturn = SX.regLoco(sx2adress, sx2format);
#endif
                if (serialreturn != 0xFF)
                {
                  SX.holdLoco(serialreturn, sx2format, sx2adress);
                  locoActive[serialreturn] = true;
                }
                SX.setSX2Dccfst(serialreturn, true);             //Set  Bit for F14DCC
              }
              else
              {
                serialreturn = 0xFF;                      //Return Fault
              }
              break;
            case (0x81):                                               //Loco with DCC short Address 28FS
              if ((sx2adress >> 2)  > 127)
              {
                serialreturn = 0xFF;//Return Fault
              }
              else if (zeformat == 4 || zeformat == 6 || zeformat == 11) //Check if Format is currently active on SX Bus
              {
                sx2format = 8;
#ifdef DH
                serialreturn = SX.regLoco(sx2adress, sx2format, output);
#else
                serialreturn = SX.regLoco(sx2adress, sx2format);
#endif
                if (serialreturn != 0xFF)
                {
                  SX.holdLoco(serialreturn, sx2format, sx2adress);
                  locoActive[serialreturn] = true;
                }
                SX.setSX2Dccfst(serialreturn, false);             //Clear Bit
              }
              else
              {
                serialreturn = 0xFF;                      //Return Fault
              }
              break;
            case (0x05):                                 //Loco with DCC short Address 127FS
              if ((sx2adress >> 2)  > 127)
              {
                serialreturn = 0xFF;//Return Fault
              }
              else if (zeformat == 4 || zeformat == 6 || zeformat == 11) //Check if Format is currently active on SX Bus
              {
                sx2format = 10;
#ifdef DH
                serialreturn = SX.regLoco(sx2adress, sx2format, output);
#else
                serialreturn = SX.regLoco(sx2adress, sx2format);
#endif
                if (serialreturn != 0xFF)
                {
                  SX.holdLoco(serialreturn, sx2format, sx2adress);
                  locoActive[serialreturn] = true;
                }
                SX.setSX2Dccfst(serialreturn, false);             //Clear Bit
              }
              else
              {
                serialreturn = 0xFF;                      //Return Fault
              }
              break;
            case (0x93):                                 //Loco with DCC long Address 14FS
              if (zeformat == 4 || zeformat == 6 || zeformat == 11) //Check if Format is currently active on SX Bus
              {
                sx2format = 12;
#ifdef DH
                serialreturn = SX.regLoco(sx2adress, sx2format, output);
#else
                serialreturn = SX.regLoco(sx2adress, sx2format);
#endif
                if (serialreturn != 0xFF)
                {
                  SX.holdLoco(serialreturn, sx2format, sx2adress);
                  locoActive[serialreturn] = true;
                }
                SX.setSX2Dccfst(serialreturn, true);             //Set  Bit for F14DCC
              }
              else
              {
                serialreturn = 0xFF;                      //Return Fault
              }
              break;
            case (0x83):                                 //Loco with DCC long Address 27FS

              if (zeformat == 4 || zeformat == 6 || zeformat == 11) //Check if Format is currently active on SX Bus
              {
                sx2format = 12;
#ifdef DH
                serialreturn = SX.regLoco(sx2adress, sx2format, output);
#else
                serialreturn = SX.regLoco(sx2adress, sx2format);
#endif
                if (serialreturn != 0xFF)
                {
                  SX.holdLoco(serialreturn, sx2format, sx2adress);
                  locoActive[serialreturn] = true;
                }
                SX.setSX2Dccfst(serialreturn, false);             //Clear Bit
              }
              else
              {
                serialreturn = 0xFF;                      //Return Fault
              }
              break;
            case (0x07):                                 //Loco with DCC long Address 127FS
              if (zeformat == 4 || zeformat == 6 || zeformat == 11) //Check if Format is currently active on SX Bus
              {
                sx2format = 14;
#ifdef DH
                serialreturn = SX.regLoco(sx2adress, sx2format, output);
#else
                serialreturn = SX.regLoco(sx2adress, sx2format);
#endif
                if (serialreturn != 0xFF)
                {
                  SX.holdLoco(serialreturn, sx2format, sx2adress);
                  locoActive[serialreturn] = true;
                }
                SX.setSX2Dccfst(serialreturn, false);             //Clear Bit
              }
              else
              {
                serialreturn = 0xFF;                      //Return Fault
              }
              break;
            default:
              serialreturn = 0xFF;                      //Return Fault
              break;
          }//End Switch rxBuffer[BUFFSX2FORMAT]
          if (serialreturn == 0xFF)//Check if fault
          {
            #ifndef DH
            output->write((uint8_t)serialreturn);     //Return Frame number
            #endif
            break;
          }
          else
          {
#ifndef DH
            output->write((uint8_t)serialreturn);     //Return Frame number
#endif
            break;
          }
        case (0x02):                               //Unregister LOCO from Interface, not form BUS!!
          if ((rxBuffer[BUFFSX2INDEX] < SX2LOCOS) && locoActive[rxBuffer[BUFFSX2INDEX]] == true) //Check if this frame was already registred before
          {
            if (SX.returnSX2POM(rxBuffer[BUFFSX2INDEX]) == true) //Unregister POM
            {
              SX.setSX2Li(rxBuffer[BUFFSX2INDEX], false);
              SX.setSX2Dccfst(rxBuffer[BUFFSX2INDEX], false);
              SX.setSX2Speed(rxBuffer[BUFFSX2INDEX], 0x00, 0);
              SX.setSX2Fkt(rxBuffer[BUFFSX2INDEX], 0x0000);
            }
            locoActive[rxBuffer[BUFFSX2INDEX]] = false;
#ifdef DEBUG
            debug->print(F("Loco unregistred:"));
            debug->println(rxBuffer[BUFFSX2INDEX]);
#endif
            serialreturn = 0;
          }
          else
          {
            serialreturn = 0xFF;
          }
          output->write((uint8_t)serialreturn);
          break;
        case (0x05):       //Change Light of registred loco
          if (rxBuffer[BUFFSX2INDEX] < SX2LOCOS && locoActive[rxBuffer[BUFFSX2INDEX]] == true)
          {

            if (rxBuffer[2] == 0x02)
            {
              serialreturn = SX.setSX2Li (rxBuffer[BUFFSX2INDEX], true); //Set Light
            }
            else
            {
              serialreturn = SX.setSX2Li (rxBuffer[BUFFSX2INDEX], false); //Set Light
            }
          }
          else
          {
            serialreturn = 0xFF;
          }
          output->write((uint8_t)serialreturn);
          break;
        case (0x13):       //Change speed and Direction of registred loco

          if (rxBuffer[BUFFSX2INDEX] < SX2LOCOS && locoActive[rxBuffer[BUFFSX2INDEX]] == true)
          {

            serialreturn = SX.setSX2Speed (rxBuffer[BUFFSX2INDEX], (rxBuffer[2] & ~128), bitRead(rxBuffer[2], 7)); //Set direction and Speed
          }
          else
          {
            serialreturn = 0xFF;
          }
          output->write((uint8_t)serialreturn);
          break;
        case (0x16):       //Change Functions of registred loco
          sx2fkt = ((rxBuffer[3] << 8 ) | rxBuffer[2]);   //Generate Adress
          if (rxBuffer[BUFFSX2INDEX] < SX2LOCOS && locoActive[rxBuffer[BUFFSX2INDEX]] == true)
          {
            serialreturn = SX.setSX2Fkt (rxBuffer[BUFFSX2INDEX], sx2fkt); //Set Funktions
          }
          else
          {
            serialreturn = 0xFF;
          }
          output->write((uint8_t)serialreturn);
          break;
        default:
          break;

      }//End Switch rxBufer[BUFFSX2CMD]
      break;
    case (0x78):
      switch (rxBuffer[BUFFSX2CMD])
      {
        case (0x03):
          for ( uint8_t i = 0; i < 112; i++)       //Return complete SX0 BUS!
          {
            output->write((uint8_t)SX.get(i));
          }
          output->write((uint8_t)SX.getPWR());
          for ( uint8_t i = 0; i < 112; i++)       //Return complete SX1 BUS!
          {
            output->write((uint8_t)SX.get(i | 128));
          }
          output->write((uint8_t)SX.getPWR());
          break;
        case (0xC0):
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Format SX Bus 0
          {
            output->write((uint8_t)SX.returnSX2Format(i));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            output->write((uint8_t)highByte(SX.returnSX2AdrLiDcc(i)));
          }
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            output->write((uint8_t)lowByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 0
          {
            output->write((uint8_t)SX.returnSX2Fst(i));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            output->write((uint8_t)lowByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            output->write((uint8_t)highByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          //Next Bus (SX1)
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Format SX Bus 0
          {
            output->write((uint8_t)SX.returnSX2Format(i));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            output->write((uint8_t)highByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            output->write((uint8_t)lowByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 0
          {
            output->write((uint8_t)SX.returnSX2Fst(i));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            output->write((uint8_t)lowByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            output->write((uint8_t)highByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          break;
        case (0xC3):
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Format SX Bus 0
          {
            output->write((uint8_t)SX.returnSX2Format(i));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            output->write((uint8_t)highByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            output->write((uint8_t)lowByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 0
          {
            output->write((uint8_t)SX.returnSX2Fst(i));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            output->write((uint8_t)lowByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fkt SX Bus 0
          {
            output->write((uint8_t)highByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fkt SX Bus 0
          }
          //Next Bus (SX1)
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Format SX Bus 1
          {
            output->write((uint8_t)SX.returnSX2Format(i));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 1
          {
            output->write((uint8_t)highByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 1
          {
            output->write((uint8_t)lowByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 1
          {
            output->write((uint8_t)SX.returnSX2Fst(i));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fst SX Bus 1
          {
            output->write((uint8_t)lowByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fkt SX Bus 1
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fkt SX Bus 1
          {
            output->write((uint8_t)highByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          //Read Sx1 Busses
          for ( uint8_t i = 0; i < 112; i++)       //Return complete SX0 BUS!
          {
            output->write((uint8_t)SX.get(i));
          }
          output->write((uint8_t)SX.getPWR());
          for ( uint8_t i = 0; i < 112; i++)       //Return complete SX1 BUS!
          {
            output->write((uint8_t)SX.get(i | 128));
          }
          output->write((uint8_t)SX.getPWR());
          break;
        default:
          break;
      }//End Switch rxBufer[BUFFSX2CMD]
      break;

    case (0x77):
      switch (rxBuffer[BUFFSX2CMD])
      {
        case (0x10):              //Return SX2 Bus SX 0
          output->write((uint8_t)0x00);       //Return SX2 Bus SX0
          output->write((uint8_t)0x80);
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Format SX Bus 0
          {
            output->write((uint8_t)SX.returnSX2Format(i));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            output->write((uint8_t)highByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            output->write((uint8_t)lowByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 0
          {
            output->write((uint8_t)SX.returnSX2Fst(i));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            output->write((uint8_t)lowByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            output->write((uint8_t)highByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions F´kt SX Bus 0
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fkt SX Bus 0
          {
            output->write((uint8_t)0x00); //Return Empty FKT (16-32)
            output->write((uint8_t)0x00); //Return Empty FKT (16-32)
          }
          break;
        case (0x20):
          output->write((uint8_t)0x00);       //Return SX2 Bus SX1
          output->write((uint8_t)0x80);
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Format SX Bus 1
          {
            output->write((uint8_t)SX.returnSX2Format(i));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 1
          {
            output->write((uint8_t)highByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 1
          {
            output->write((uint8_t)lowByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 1
          {
            output->write((uint8_t)SX.returnSX2Fst(i));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fst SX Bus 1
          {
            output->write((uint8_t)lowByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 1
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fkt SX Bus 1
          {
            output->write((uint8_t)highByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fkt SX Bus 1

          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fst SX Bus 1
          {
            output->write((uint8_t)0x00); //Return Empty FKT (16-32)
            output->write((uint8_t)0x00); //Return Empty FKT (16-32)
          }
          break;
        case (0x33):
          output->write((uint8_t)0x00);       //Return SX2 Bus SX1
          output->write((uint8_t)0x80);
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Format SX Bus 1
          {
            output->write((uint8_t)SX.returnSX2Format(i));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 1
          {
            output->write((uint8_t)highByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 1
          {
            output->write((uint8_t)lowByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 1
          {
            output->write((uint8_t)SX.returnSX2Fst(i));
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fst SX Bus 1
          {
            output->write((uint8_t)lowByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 1
          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fkt SX Bus 1
          {
            output->write((uint8_t)highByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fkt SX Bus 1

          }
          for ( uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fst SX Bus 1
          {
            output->write((uint8_t)0x00); //Return Empty FKT (16-32)
            output->write((uint8_t)0x00); //Return Empty FKT (16-32)
          }
          //Next Bus (SX0)
          output->write((uint8_t)0x00);       //Return SX2 Bus SX0
          output->write((uint8_t)0x80);
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Format SX Bus 0
          {
            output->write((uint8_t)SX.returnSX2Format(i));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            output->write((uint8_t)highByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            output->write((uint8_t)lowByte(SX.returnSX2AdrLiDcc(i)));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 0
          {
            output->write((uint8_t)SX.returnSX2Fst(i));
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            output->write((uint8_t)lowByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            output->write((uint8_t)highByte(SX.returnSX2Fkt(i))); //Return complete SX2 Bus additions F´kt SX Bus 0
          }
          for ( uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fkt SX Bus 0
          {
            output->write((uint8_t)0x00); //Return Empty FKT (16-32)
            output->write((uint8_t)0x00); //Return Empty FKT (16-32)
          }
          //Next Bus (SX1)
          output->write((uint8_t)0x00);       //Return SX1 Bus SX1
          output->write((uint8_t)0x71);
          for ( uint8_t i = 0; i < 112; i++)       //Return complete SX1 BUS!
          {
            output->write((uint8_t)SX.get(i | 128));
          }
          output->write((uint8_t)SX.getPWR());
          //Next Bus (SX0)
          output->write((uint8_t)0x00);       //Return SX1 Bus SX0
          output->write((uint8_t)0x71);
          for ( uint8_t i = 0; i < 112; i++)       //Return complete SX0 BUS!
          {
            output->write((uint8_t)SX.get(i));
          }
          output->write((uint8_t)SX.getPWR());
          break;
      }//End Switch rxBufer[BUFFSX2CMD]
      break;
    case (0x7A):
      serialreturn = SX.regPOM(rxBuffer[0], rxBuffer[1], 2, rxBuffer[2], rxBuffer[3], rxBuffer[4]);
      //sx2loco[serialreturn].pr = 3;
      output->write((uint8_t)serialreturn);
      break;
    case (0x81):                                         //Return Firmware Version
      output->write((uint8_t)0x61);
      delayMicroseconds(100) ;
      output->write((uint8_t)0x00);
      delayMicroseconds(100) ;
      output->write((uint8_t)0x01);
      delayMicroseconds(100) ;
      output->write((uint8_t)0x00);
      delayMicroseconds(100) ;
      output->write((uint8_t)FWVERSION2);
      delayMicroseconds(100) ;
      output->write((uint8_t)FWVERSION1);
      delayMicroseconds(100) ;
      output->write((uint8_t)0xE0);
      delayMicroseconds(100) ;
      break;
    case (0x83):
      if (rxBuffer[0] == 0xA0 && rxBuffer[1] == 0x00 && rxBuffer[2] == 0x00 && rxBuffer[3] == 0x00)
      {
        SX.set(106, 0x32);
        SX.set(105, 0x01);
        SX.set(104, 0x01);
        SX.set(106, 160);
        delay(100);
        SX.set(106, 0x00);
        SX.set(105, 0x00);
        SX.set(104, 0x00);
        output->write((uint8_t)0x00);
        output->write((uint8_t)0x00);
        output->write((uint8_t)0x00);
        break;
      }
      else
      {
        output->write((uint8_t)0xFF);
        output->write((uint8_t)0xFF);
        output->write((uint8_t)0xFF);
        break;
      }
    case (0xFF):
      while (output->available())
      {
        output->read();          //WRONG COMMAND RECIVED, CLEAR BUFFER
        delayMicroseconds(600); //Wait a bit for next command
      }
      break;
    default:
      break;
  }//Ende switch
} //End serial event
void loop()
{
  unsigned long currentMillis = millis();
  static unsigned int COMinterval = 0;
  static unsigned long previousMillis = 0;
#if defined(ARDUINO_ARCH_SAM)
  //while (!SerialUSB);
#endif
#if defined(ARDUINO_ARCH_SAM)
  if (output->available())
  {
    serialEventUSB();
  }
#endif
  // put your main code here, to run repeatedly:
  if (currentMillis - previousMillis >= COMinterval)
  {
    previousMillis = currentMillis;
    if (COMinterval == 0)       //startup
    {
      COMinterval = COMPORTTIMEOUT;
    }
    if (noserial == true)
    {
      for (uint8_t i = 0; i < SX2LOCOS; i++)
      {
        locoActive[i] = false;
      }
    }
    noserial = true;
  }
  zeformat = SX.get(110) & 15;
}
