#include <SX2Arduino.h>
#include <avr/io.h> 
#define BAUD 230400
#include <util/setbaud.h>
#include "Arduino.h"
#define FWVERSION1    1
#define FWVERSION2    6

#define SX2LOCOS      32

#define BUFFSXCHAN 0
#define BUFFSXVAL 1

#define BUFFSX2CMD 0
#define BUFFSX2INDEX 1
#define BUFFSX2FST 2
#define BUFFSX2FORMAT 3
// SX-bus interface
SX2Arduino SXbus;                 // Get access to the SX-bus

byte cmdRcvd;                     //To store Serial Command Recived
byte rxBuffer[5];                 //to store serial data recived after command
uint8_t sx2loco[SX2LOCOS];     //to store active locos with format
uint8_t zeformat;                //to store the current ze format
void sxisr(void)
{
  SXbus.isr();
}
void setup()
{
  for (uint8_t i = 0; i < SX2LOCOS; i++)
  {

    sx2loco[i] = 0;
  }
  Serial.begin(230400);      // open the serial port
  SXbus.init();
  attachInterrupt(0, sxisr, RISING);
  delay(500);

}
void serialEvent()
{
  // Read all the data
  uint8_t databytes;                //to defines datbytes to read vom buffer
  uint8_t serialreturn = 0xFF;           //to store date to return to pc
  uint16_t sx2fkt, sx2adress, sx2format = 0;
  while (Serial.available())
  {
    //Serial.print("Bytes available in buffer:");
    //Serial.println(Serial.available());
    cmdRcvd = (uint8_t) Serial.read();            //Read first incoming byte
    switch (cmdRcvd)
    {
      case (0x00):
        databytes = 2;
        break;
      case (0x01):
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
      case (0x83):
        databytes = 4;
        break;
      default:
        databytes = 0;
        break;
    }
    Serial.readBytes(rxBuffer, databytes);            //Read Data bytes into Buffer
    break;
  } //End while

  switch (cmdRcvd)      //Decode command
  {
    case (0x00):
      if (rxBuffer[BUFFSXCHAN] == 0xFF)     //Set track Power on/off
      {
        if (rxBuffer[BUFFSXVAL] == 0)
        {
          SXbus.setPWR(0);
        }
        else
        {
          SXbus.setPWR(1);
        }
        Serial.write(0x00);            //Return 0x00 to pc
        break;
      }
      else if (rxBuffer[BUFFSXCHAN] == 0x7F)   //Get Track power
      {
        Serial.write( SXbus.getPWR());                      //Return Track Power to PC
        break;
      }
      else if (bitRead(rxBuffer[BUFFSXCHAN], 7))                                      //Write SX Channel if BIT 7 is set
      {
        bitClear(rxBuffer[BUFFSXCHAN], 7);                                            //Clear Bit 7
        Serial.write (SXbus.set(rxBuffer[BUFFSXCHAN], rxBuffer[BUFFSXVAL] ));         //Write on sx BUS
        break;
      }
      else if (rxBuffer[BUFFSXCHAN] < 112)                          //Return SX Channel to PC
      {
        Serial.write(SXbus.get(rxBuffer[BUFFSXCHAN]));
        break;
      }
      break;
    case (0x01):
      if (rxBuffer[BUFFSXCHAN] == 0xFF)     //Set track Power on/off
      {
        if (rxBuffer[BUFFSXVAL] == 0)
        {
          SXbus.setPWR(0);
        }
        else
        {
          SXbus.setPWR(1);
        }
        Serial.write(0x00);            //Return 0x00 to pc
        break;
      }
      else if (rxBuffer[BUFFSXCHAN] == 0x7F)   //Get Track power
      {
        Serial.write( SXbus.getPWR());                      //Return Track Power to PC
        break;
      }
      else if (bitRead(rxBuffer[BUFFSXCHAN], 7))                                      //Write SX Channel if BIT 7 is set
      {
        bitClear(rxBuffer[BUFFSXCHAN], 7);                                            //Clear Bit 7
        Serial.write (SXbus.set(rxBuffer[BUFFSXCHAN] | 128, rxBuffer[BUFFSXVAL] ));   //Write on sx 1 BUS
        break;
      }
      else if (rxBuffer[BUFFSXCHAN] < 112)                          //Return SX Channel to PC
      {
        Serial.write(SXbus.get(rxBuffer[BUFFSXCHAN] | 128));                         //Get sx 1 Bus
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
                //Serial.print("SX2 Adress:");
                sx2format = 2;
                serialreturn = SXbus.regLoco(sx2adress, sx2format);
				if (serialreturn != 0xFF)
				{
				SXbus.setSX2Dccfst(serialreturn, false);             //Clear Bit
				}
              }
              else
              {
                serialreturn = 0xFF;                      //Return Fault
              }
              break;
            case (0x91):                                               //Loco with DCC short Address 14FS
              if ((sx2adress>>2) > 127)
              {
                serialreturn = 0xFF;//Return Fault
              }
              else if (zeformat == 4 || zeformat == 6 || zeformat == 11) //Check if Format is currently active on SX Bus
              {
                sx2format = 8;
                serialreturn = SXbus.regLoco(sx2adress, sx2format);
                if (serialreturn != 0xFF)
                {
                  SXbus.setSX2Dccfst(serialreturn, true);             //Set  Bit for F14DCC
                }
              }
              else
              {
                serialreturn = 0xFF;                      //Return Fault
              }
              break;
            case (0x81):                                               //Loco with DCC short Address 28FS
              if ((sx2adress>>2)  > 127)
              {
                serialreturn = 0xFF;//Return Fault
              }
              else if (zeformat == 4 || zeformat == 6 || zeformat == 11) //Check if Format is currently active on SX Bus
              {
                sx2format = 8;
                serialreturn = SXbus.regLoco(sx2adress, sx2format);
								if (serialreturn != 0xFF)
								{
									SXbus.setSX2Dccfst(serialreturn, false);             //Clear Bit
								}
              }
              else
              {
                serialreturn = 0xFF;                      //Return Fault
              }
              break;
            case (0x05):                                 //Loco with DCC short Address 127FS
              if ((sx2adress>>2)  > 127)
              {
                serialreturn = 0xFF;//Return Fault
              }
              else if (zeformat == 4 || zeformat == 6 || zeformat == 11) //Check if Format is currently active on SX Bus
              {
                sx2format = 10;
                serialreturn = SXbus.regLoco(sx2adress, sx2format);
								if (serialreturn != 0xFF)
								{
									SXbus.setSX2Dccfst(serialreturn, false);             //Clear Bit
								}
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
                serialreturn = SXbus.regLoco(sx2adress, sx2format);
                if (serialreturn != 0xFF)
                {
                  SXbus.setSX2Dccfst(serialreturn, true);             //Set  Bit for F14DCC
                }
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
                serialreturn = SXbus.regLoco(sx2adress, sx2format);
								if (serialreturn != 0xFF)
								{
									SXbus.setSX2Dccfst(serialreturn, false);             //Clear Bit
								}
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
                serialreturn = SXbus.regLoco(sx2adress, sx2format);
								if (serialreturn != 0xFF)
								{
									SXbus.setSX2Dccfst(serialreturn, false);             //Clear Bit
								}
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
            Serial.write(serialreturn);     //Return Frame number
            break;
          }
          else
          {
            sx2loco[serialreturn] = sx2format;
            Serial.write(serialreturn);     //Return Frame number
            break;
          }
        case (0x02):                               //Unregister LOCO from Interface, not form BUS!!
          if ((rxBuffer[BUFFSX2INDEX] < SX2LOCOS) && (sx2loco[rxBuffer[BUFFSX2INDEX]] != 0)) //Check if this frame was already registred before
          {
            sx2loco[rxBuffer[BUFFSX2INDEX]] = 0;
            serialreturn = 0;
          }
          else
          {
            serialreturn = 0xFF;
          }
          Serial.write (serialreturn);
          break;
        case (0x05):       //Change Light of registred loco
          if (rxBuffer[BUFFSX2INDEX] < SX2LOCOS && sx2loco[rxBuffer[BUFFSX2INDEX]] != 0)
          {
            if (rxBuffer[2]==0x02)
			{
				serialreturn = SXbus.setSX2Li (rxBuffer[BUFFSX2INDEX], true); //Set Light
			}
			else
			{
				serialreturn = SXbus.setSX2Li (rxBuffer[BUFFSX2INDEX], false); //Set Light
			}
          }
          else
          {
            serialreturn = 0xFF;
          }
          Serial.write (serialreturn);
          break;
        case (0x13):       //Change speed and Direction of registred loco
          if (rxBuffer[BUFFSX2INDEX] < SX2LOCOS && sx2loco[rxBuffer[BUFFSX2INDEX]] != 0)
          {
            serialreturn = SXbus.setSX2Speed (rxBuffer[BUFFSX2INDEX], (rxBuffer[2] & ~128), bitRead(rxBuffer[2], 7)); //Set direction and Speed
          }
          else
          {
            serialreturn = 0xFF;
          }
          Serial.write (serialreturn);
          break;
        case (0x16):       //Change Functions of registred loco
          sx2fkt = ((rxBuffer[3] << 8 ) | rxBuffer[2]);   //Generate Adress
          if (rxBuffer[BUFFSX2INDEX] < SX2LOCOS && sx2loco[rxBuffer[BUFFSX2INDEX]] != 0)
          {
            serialreturn = SXbus.setSX2Fkt (rxBuffer[BUFFSX2INDEX], sx2fkt); //Set direction and Speed
          }
          else
          {
            serialreturn = 0xFF;
          }
          Serial.write (serialreturn);
          break;
        default:
          break;

      }//End Switch rxBufer[BUFFSX2CMD]
      break;
    case (0x78):
      switch (rxBuffer[BUFFSX2CMD])
      {
        case (0x03):
          for (uint8_t i = 0; i < 112; i++)       //Return complete SX0 BUS!
          {
            Serial.write(SXbus.get(i));
          }
          Serial.write(SXbus.getPWR());
          for (uint8_t i = 0; i < 112; i++)       //Return complete SX1 BUS!
          {
            Serial.write(SXbus.get(i | 128));
          }
          Serial.write(SXbus.getPWR());
          break;
        case (0xC0):
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Format SX Bus 0
          {
            Serial.write (SXbus.returnSX2Format(i));
          }
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            Serial.write (highByte(SXbus.returnSX2AdrLiDcc(i)));
          }
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            Serial.write (lowByte(SXbus.returnSX2AdrLiDcc(i)));
          }
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 0
          {
            Serial.write (SXbus.returnSX2Fst(i));
          }
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            Serial.write (lowByte(SXbus.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            Serial.write (highByte(SXbus.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          //Next Bus (SX1)
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Format SX Bus 0
          {
            Serial.write (SXbus.returnSX2Format(i));
          }
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            Serial.write (highByte(SXbus.returnSX2AdrLiDcc(i)));
          }
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            Serial.write (lowByte(SXbus.returnSX2AdrLiDcc(i)));
          }
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 0
          {
            Serial.write (SXbus.returnSX2Fst(i));
          }
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            Serial.write (lowByte(SXbus.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            Serial.write (highByte(SXbus.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          break;
        case (0xC3):
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Format SX Bus 0
          {
            Serial.write (SXbus.returnSX2Format(i));
          }
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            Serial.write (highByte(SXbus.returnSX2AdrLiDcc(i)));
          }
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            Serial.write (lowByte(SXbus.returnSX2AdrLiDcc(i)));
          }
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 0
          {
            Serial.write (SXbus.returnSX2Fst(i));
          }
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            Serial.write (lowByte(SXbus.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          for (uint8_t i = 0; i < 16; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            Serial.write (highByte(SXbus.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          //Next Bus (SX1)
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Format SX Bus 0
          {
            Serial.write (SXbus.returnSX2Format(i));
          }
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            Serial.write (highByte(SXbus.returnSX2AdrLiDcc(i)));
          }
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adress SX Bus 0
          {
            Serial.write (lowByte(SXbus.returnSX2AdrLiDcc(i)));
          }
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Adr/Light/DCC Bit SX Bus 0
          {
            Serial.write (SXbus.returnSX2Fst(i));
          }
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            Serial.write (lowByte(SXbus.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          for (uint8_t i = 16; i < 32; i++)       //Return complete SX2 Bus additions Fst SX Bus 0
          {
            Serial.write (highByte(SXbus.returnSX2Fkt(i))); //Return complete SX2 Bus additions Fst SX Bus 0
          }
          //Read Sx1 Busses
          for (uint8_t i = 0; i < 112; i++)       //Return complete SX0 BUS!
          {
            Serial.write(SXbus.get(i));
          }
          Serial.write(SXbus.getPWR());
          for (uint8_t i = 0; i < 112; i++)       //Return complete SX1 BUS!
          {
            Serial.write(SXbus.get(i | 128));
          }
          Serial.write(SXbus.getPWR());
          break;
        default:
          break;
      }//End Switch rxBufer[BUFFSX2CMD]
      break;
    case (0x81):                                         //Return Firmware Version
      Serial.write(0x61);
      Serial.write(0x00);
      Serial.write(0x01);
      Serial.write(0x00);
      Serial.write(FWVERSION2);
      Serial.write(FWVERSION1);
      Serial.write(0xE0);
      break;
    default:
      break;
  }//Ende switch
} //End serial event
void loop()
{
  // put your main code here, to run repeatedly:
  zeformat = SXbus.get(110) & 15;
  for (uint8_t i = 0; i < SX2LOCOS; i++)       //loop for hold locos active on bus
  {
    if (sx2loco[i] != 0)           //CHeck if loco is active
    {
      SXbus.holdLoco(i, sx2loco[i]);        //Hold Loco active
    }
  }
}
