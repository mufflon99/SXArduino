/*
 * SX2Aduino.cpp
 *
 * Changed on: 14.09.2018
 * Version: 5.3
 * Changes: Added some methods for SX1 reading and writing
 *
 * Changed on: 14.09.2018
 * Version: 5.2
 * Changes: Now support Arduino Due
 *
 * Changed on: 12.09.2018
 * Version: 5.2
 * Changes: Now support reading SX BUS 0 and SX BUS 1, minor bug fixes
 *
 * Changed on: 22.08.2018
 * Version: 4.1
 * Changes: Read and write SX2 Bus addition should now woking correctly, registration of SX2 Locos optimized
 *
 * Version: 4.0 
 * Copyright: Michael Berger, main work done by Gerard van der Sel!
 *
 * Changed on: 12.08.2018
 * Version: 4.0
 * Changes: Read and write SX2 Bus addition, alpha state, need some more optmisations
 * 
 *
 *  Version:    3.1
 *  Copyright:  Gerard van der Sel
 *
 *  Changed on: 27.12.2015
 *  Version: 	3.1
 *  Changes: 	Added 3 and 4 pin interface.
 *
 *  Changed on: 19-12.2015
 *  Version: 	3.0
 *  Changes: 	Added some comment. Given its version number.
 *
 *  Changed on: 30.11.2015
 *  Version: 	0.5
 *  Changes: 	Reading and writing to multiple addresses in one cycle, resolved timing issues.
 *
 *  Changed on: 14.11.2015
 *  Version: 	0.4
 *  Changes: 	Reading and writing to multiple addresses in one cycle.
 *
 *  Changed on: 27.10.2015
 *  Version: 	0.3
 *  Changes: 	onWait() added to synchronise with the SXbus.
 *
 *  Changed on: 27.09.2015
 *  Version: 	0.2
 *  Changes: 	Minor changes 
 *
 *  Changed on: 10.07.2015
 *  Version: 	0.1
 *  Changes: 	Initial version
 *
 *  interface hardware needed !

 Interface SX-bus
 - SX T0 (Clock) must be connected to Pin 3 (IOL, INT1);
 - SX T1 must be connected to Pin 5 (IOL, T1);
 - SX D must be connected to Pin 6 (IOL, AIN0). 
 
 SX-bus interface (NEM 682)

 De clock lijn (T0) is verbonden met een interruptingang, zodat op
 de flanken van dit signaal een interrupt gegenereerd kan worden.
 Hierna kan data gelezen worden van T1 of data geschreven worden naar D.

 Clock:
  --    ----------------    ----------------    ----------------    ------
    |  |                |  |                |  |                |  |
     --                  --                  --                  -- 

 Data:
  -- ------------------- ------------------- ------------------- ---------
    X                   X                   X                   X
  -- ------------------- ------------------- ------------------- ---------

       ^                   ^                   ^                   ^
       P                   P                   P                   P

SX telegram (96 bits):
  0  0 0  1  S 1 A3 A2 1 A1 A0 1     Sync 'byte'
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1         7 data 'bytes'
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1   ieder 'byte' is de inhoud
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1         van een adres
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1

 0 = Logische 0
 1 = Logische 1
 S = Gleisspannung (0 = aus, 1= an)
 Ax = Telegramm Nummer 
 Dx = D0 t/m D7 Daten der zugehörigen SX Adresse, 7 pro Telegramm

 Verdeling adressen over de verschillende telegrammen:
 telegram  '0' : 111, 95, 79, 63, 47, 31, 15
 telegram  '1' : 110, 94, 78, 62, 46, 30, 14
 telegram  '2' : 109, 93, 77, 61, 45, 29, 13
 telegram  '3' : 108, 92, 76, 60, 44, 28, 12
 telegram  '4' : 107, 91, 75, 59, 43, 27, 11
 telegram  '5' : 106, 90, 74, 58, 42, 26, 10
 telegram  '6' : 105, 89, 73, 57, 41, 25,  9
 telegram  '7' : 104, 88, 72, 56, 40, 24,  8
 telegram  '8' : 103, 87, 71, 55, 39, 23,  7
 telegram  '9' : 102, 86, 70, 54, 38, 22,  6
 telegram '10' : 101, 85, 69, 53, 37, 21,  5
 telegram '11' : 100, 84, 68, 52, 36, 20,  4
 telegram '12' :  99, 83, 67, 51, 35, 19,  3
 telegram '13' :  98, 82, 66, 50, 34, 18,  2
 telegram '14' :  97, 81, 65, 49, 33, 17,  1
 telegram '15' :  96, 80, 64, 48, 32, 16,  0

This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 */
 
#include <Arduino.h> 
#if defined(ARDUINO_ARCH_AVR)
	#include <util/atomic.h>
#endif
#include "SX2Arduino.h"
/************ static variables common to all instances ***********************/
static uint8_t sxbusindex=0;						//Counter for SX index
static HardwareSerial* printer;			//shared by all Functions

/****************** end of static variables **********************************/
/************ static functions common to all instances ***********************/
// Local functions
uint8_t readT1(uint8_t nr) 
{
	#if defined(ARDUINO_ARCH_AVR)
		#ifdef _sxbus1
		if (nr==0)
		{
			return (SX0_T1_PINREG & _BV(SX0_T1_PORTPIN)) > 0;
		}
		else
		{
			return (SX1_T1_PINREG & _BV(SX1_T1_PORTPIN)) > 0;
		}
		#else 
			return (SX0_T1_PINREG & _BV(SX0_T1_PORTPIN)) > 0;
		#endif
	#elif defined(ARDUINO_ARCH_SAM)
		#ifdef _sxbus1
		if (nr==0)
		{
			return (PINPORT_READING->PIO_PDSR & SX0_T1_PORTPIN)>0;
		}
		else
		{
			return (PINPORT_READING->PIO_PDSR &SX1_T1_PORTPIN)>0;
		}
		#else 
			return (PINPORT_READING->PIO_PDSR & SX0_T1_PORTPIN) > 0;
		#endif
	#endif
}

void writeD(uint8_t nr,uint8_t val) 
{
	if (nr==0)
	{
		switch(val) 
		{
	#if defined(ARDUINO_ARCH_AVR)
		#ifndef _use4pin
			case 0:
				bitWrite(SX0_D_DDR, SX0_D_PORTPIN, HIGH);				      // Switch to low
				bitWrite(SX0_D_PORT, SX0_D_PORTPIN, LOW);
				break;
			case 1:	
				bitWrite(SX0_D_DDR, SX0_D_PORTPIN, HIGH);		              // Switch to high
				bitWrite(SX0_D_PORT, SX0_D_PORTPIN, HIGH);
				break;
			default:	
				bitWrite(SX0_D_DDR, SX0_D_PORTPIN, LOW);		                  // Switch to input (TRI_STATE)
				break;
		#else
			case 0:
				bitWrite(SX0_D_HIGH_PORT, SX0_D_HIGH_PORTPIN, HIGH);          // Switch to low
				bitWrite(SX0_D_LOW_PORT, SX0_D_LOW_PORTPIN, LOW);
				break;
			case 1:	
				bitWrite(SX0_D_LOW_PORT, SX0_D_LOW_PORTPIN, HIGH);            // Switch to high
				bitWrite(SX0_D_HIGH_PORT, SX0_D_HIGH_PORTPIN, LOW);
				break;
			default:	
				bitWrite(SX0_D_LOW_PORT, SX0_D_LOW_PORTPIN, HIGH);            // Switch off (TRI_STATE)
				bitWrite(SX0_D_HIGH_PORT, SX0_D_HIGH_PORTPIN, HIGH);
				break;
		#endif // _use4pin_
	#elif defined(ARDUINO_ARCH_SAM)
			case 0:
				PINPORT_WRITNG -> PIO_SODR = (1<<SX0_D_HIGH_PORTPIN);				//Switch to low
				PINPORT_WRITNG -> PIO_CODR = (1<<SX0_D_LOW_PORTPIN);
				break;
			case 1:	
				PINPORT_WRITNG -> PIO_CODR = (1<<SX0_D_HIGH_PORTPIN);				//Switch to high
				PINPORT_WRITNG -> PIO_SODR = (1<<SX0_D_LOW_PORTPIN);
				break;
			default:	
				PINPORT_WRITNG -> PIO_SODR = (1<<SX0_D_LOW_PORTPIN|1<<SX0_D_HIGH_PORTPIN);
				break;
		
	//PINPORT_WRITNG -> PIO_SODR = ; //Set
	//PINPORT_WRITNG -> PIO_CODR =; //Clear
	#endif
		}
	}
	#ifdef _sxbus1
	else 
	{
		switch(val) 
		{
		#if defined(ARDUINO_ARCH_AVR)
			#ifndef _use4pin
				case 0:
					bitWrite(SX1_D_DDR, SX1_D_PORTPIN, HIGH);				      // Switch to low
					bitWrite(SX1_D_PORT, SX1_D_PORTPIN, LOW);
					break;
				case 1:	
					bitWrite(SX1_D_DDR, SX1_D_PORTPIN, HIGH);		              // Switch to high
					bitWrite(SX1_D_PORT, SX1_D_PORTPIN, HIGH);
					break;
				default:	
					bitWrite(SX1_D_DDR, SX1_D_PORTPIN, LOW);		                  // Switch to input (TRI_STATE)
					break;
			#else
				case 0:
					bitWrite(SX1_D_HIGH_PORT, SX1_D_HIGH_PORTPIN, HIGH);          // Switch to low
					bitWrite(SX1_D_LOW_PORT, SX1_D_LOW_PORTPIN, LOW);
					break;
				case 1:	
					bitWrite(SX1_D_LOW_PORT, SX1_D_LOW_PORTPIN, HIGH);            // Switch to high
					bitWrite(SX1_D_HIGH_PORT, SX1_D_HIGH_PORTPIN, LOW);
					break;
				default:	
					bitWrite(SX1_D_LOW_PORT, SX1_D_LOW_PORTPIN, HIGH);            // Switch off (TRI_STATE)
					bitWrite(SX1_D_HIGH_PORT, SX1_D_HIGH_PORTPIN, HIGH);
					break;
			#endif // _use4pin
		#elif defined(ARDUINO_ARCH_SAM)
			case 0:
				PINPORT_WRITNG -> PIO_SODR = (1<<SX1_D_HIGH_PORTPIN);				//Switch to low
				PINPORT_WRITNG -> PIO_CODR = (1<<SX1_D_LOW_PORTPIN);
				break;
			case 1:	
				PINPORT_WRITNG -> PIO_CODR = (1<<SX1_D_HIGH_PORTPIN);				//Switch to high
				PINPORT_WRITNG -> PIO_SODR = (1<<SX1_D_LOW_PORTPIN);
				break;
			default:	
				PINPORT_WRITNG -> PIO_SODR = (1<<SX1_D_LOW_PORTPIN|1<<SX1_D_HIGH_PORTPIN);
				break;
		#endif
		
	//PINPORT_WRITNG -> PIO_SODR = ; //Set
	//PINPORT_WRITNG -> PIO_CODR =; //Clear
		}
	}
	#endif

}


/****************** end of static functions **********************************/

SX2Arduino::SX2Arduino() 
{

	printer=NULL;					//No Serial.Print for default
	pinMode(SX_T0, INPUT);           // SX-T0 is an input, no pull up
	if(sxbusindex<2)					//Check if Max Busindex is reached
		_sx_busNr=sxbusindex++;
	else
		_sx_busNr=sxbusindex;
	if (_sx_busNr==0)
		pinMode(SX_T0, INPUT);           // SX-T0 is an input, no pull up to simulate tri-state
}

uint8_t SX2Arduino::init(HardwareSerial* hwPrint) 
{
	printer = hwPrint; 	//operate on the adress of print
	return init();
}

uint8_t SX2Arduino::init()
{
	
     // initialize function
     // initialize pins and variables
	if (_sx_busNr==1)
		return (0xFF);					//Return fault if max busindex was reached
	if (printer!=NULL)
		{
			printer->println(F("SX Bus initaliszed"));
		}
		pinMode(SX0_T1, INPUT);           // SX-T1 is also an input, no pull up to simulate tri-state
		#ifndef _use4pin
			pinMode(SX0_D, INPUT);       // SX-D is also an input when not writing to allow other devices to write
		#else
			pinMode(SX0_D_LOW, OUTPUT);       // SX-LOW_D is output but set high to stop wrting low
			pinMode(SX0_D_HIGH, OUTPUT);      // SX-HIGH-D is output but set high to stop wrting high

			//bitWrite(SX0_D_LOW_PORT, SX0_D_LOW_PORTPIN, HIGH);            // Switch off (TRI_STATE)
			//bitWrite(SX0_D_HIGH_PORT, SX0_D_HIGH_PORTPIN, HIGH);
		#endif // _use4pin
		writeD(0,TRI_STATE);			// Switch off (TRI_STATE)
		#ifdef _sxbus1
			pinMode(SX1_T1, INPUT);           // SX-T1 is also an input, no pull up to simulate tri-state
			#ifndef _use4pin
				pinMode(SX1_D, INPUT);       // SX-D is also an input when not writing to allow other devices to write
			#else
				pinMode(SX1_D_LOW, OUTPUT);       // SX-LOW_D is output but set high to stop wrting low
				pinMode(SX1_D_HIGH, OUTPUT);      // SX-HIGH-D is output but set high to stop wrting high
				//bitWrite(SX1_D_LOW_PORT, SX1_D_LOW_PORTPIN, HIGH);            // Switch off (TRI_STATE)
				//bitWrite(SX1_D_HIGH_PORT, SX1_D_HIGH_PORTPIN, HIGH);
			#endif // _use4pin
		writeD(1,TRI_STATE);			// Switch off (TRI_STATE)
		#endif // _sxbus1
		initVar();
		return (0);
}

void SX2Arduino::initVar() 
{
	// start always with search for header
	_sx_state = SYNC;                                          // First look for SYNC pattern
	_sx_dataFrameCount = SX_DATACOUNT;                         // Read all dataframes
	_sx_sepCount = SX_SEPLEN;                                  // Distanse between two separators
	_sx_syncCount = SX_STOP;                                   // Check for SX_STOP bits of "0"
	_sx_numFrame = 0;                                          // Set frame 0
	_sx_index = 0;                                             // Set index 0
	// Powerbit  receive
	_sx_newPWR = 2;                                            // Don't write power bit
	// reset flags
	_sx_syncFlag = 0;                                          // Clear all flags (sync and write), at start no power
	for (int i = 0; i < SX_ADDRESS_NUMBER; i++)
		{
		if (i<SX2_FRAMES)										// reset sx2 received and send data to zero
			{
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif
				{
					_sx2bus[i].pr =0;		
					_sx2bus[i].adr = 0;		
					_sx2bus[i].fst = 0;
					_sx2bus[i].fkt = 0;
				}
			}
			_sxbus[i]=0;
		}
}
// interrupt service routine (AVR INT1)
// driven by RISING EDGES of the SX clock signal T0 (SX pin 1)
void SX2Arduino::isr() 
{
	/*
	if (printer!=NULL)
		{
			printer->print(F("ISR NR:"));
			printer->print(_sx_busNr);
			printer->println(F("Called"));
		}
	*/
	_sx0_bit = readT1(0);                                        // read pin SX 0 bus
	#ifdef _sxbus1
	_sx1_bit = readT1(1);                                        // read pin SX 1 bus
	#endif
	switch (_sx_state) 
	{
		// Find sync pattern "0001" to start reading and writing
		case SYNC:
			if (_sx0_bit == LOW) 
			{                              						// Sync bits "0"
				if (_sx_syncCount > 0) 
				{                       						// If more then 3
					_sx_syncCount--;
				}
			} 
			else 
			{
				if (_sx_syncCount == 0)							// High, read 3 bits?
				{                      
					_sx_state = PWR;                           // Setup for POWER bit
					_sx_sepCount = SX_SEPLEN - 1;              // Set _sx_sepCount and continue
					break;
				}
				_sx_syncCount = SX_STOP;                       // Error, setup for restart
				_sx_syncFlag =0;								//Reset all flag BITS
				#ifdef DEBUG
				if (printer!=NULL)
				{
					printer->print(F("SYNC Error Bus:"));
					printer->println(_sx_busNr);
				}		
				#endif
			}
			break;
		// Read (and write) the power bit.
		case PWR:
			_sx_sepCount--;
			if (_sx_sepCount == 0) {                           // Skip the separator
				writeD(_sx_busNr,TRI_STATE);		                       // Switch pin to input
				_sx_state = ADDR;                              // Setup for next state ADDR
				_sx_bitCount = SX_BYTELEN / 2;
				_sx_sepCount = SX_SEPLEN;
				_sx_numFrame = 0;
			} 
			else 
			{
				if (_sx_newPWR != 2) 							// Set power from me
					{                        
					writeD(_sx_busNr,_sx_newPWR);                        	// write newPWR
					_sx_newPWR = 2;                            	// Power set
					}   // end if _sx_newPWR 
				bitWrite(_sx_syncFlag,SXPWR,(_sx0_bit&1));		//Set PWR Bit in Flag bit
			}				
			break;
		// Read the address bits.
		case ADDR:  
			_sx_sepCount--;
			if (_sx_sepCount == 0) 
			{                           // Skip the separator
				_sx_sepCount = SX_SEPLEN;
			} 
			else 
			{
			_sx_numFrame = (_sx_numFrame * 2) + _sx0_bit;   // Read bit into framenumber
				if ((_sx_bitCount == 2) && (_sx_numFrame == 0)) 
				{
					bitSet(_sx_syncFlag,SX1SYNC);                              // Signal frame 0 for sync purposes
				}
			}
			_sx_bitCount--;
			if (_sx_bitCount == 0)
			{                          // Addres part is processed
				// Advance to the next state
				_sx_state = DATA;                              // Setup for DATA read
				_sx_bitCount = SX_BYTELEN;
				_sx_numFrame = (~_sx_numFrame)&15;				//Calculate current Frame, sended inverted on SX bus, stored in first nibble of _sx_numFrame
				_sx_index = _sx_numFrame + (16*(_sx_dataFrameCount-1));	//Calculate new index, started with _sx_numFrame+16*channel nr, started with channel nr 6
				// Check if we want to write and prepare it
				if (_sxbus[_sx_index] > 255)					//Write Bit set
				{
					#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
					{
						_sx0_write_data =lowByte (_sxbus[_sx_index]);     // Get data to write
					}
					bitSet(_sx_syncFlag,SX0ISRWRITEFLAG);       	// Write
				} else 
				{
					bitClear(_sx_syncFlag,SX0ISRWRITEFLAG);                            // No write
				}
				#ifdef _sxbus1
				if (_sxbus[_sx_index+112] > 255)					//Write Bit set
				{
				#if defined(ARDUINO_ARCH_AVR)
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
				#endif 
					{
						_sx1_write_data =lowByte (_sxbus[_sx_index+112]);     // Get data to write
					}
					bitSet(_sx_syncFlag,SX1ISRWRITEFLAG);       	// Write
				} else 
				{
					bitClear(_sx_syncFlag,SX1ISRWRITEFLAG);                            // No write
				}
				#endif
			}
			break;
		// Read (and write) the data bits
		case DATA: 
			_sx_sepCount--;
			if (_sx_sepCount == 0)  							// Skip the separator
			{             			
				writeD(0,TRI_STATE);                             // Switch pin to input
				#ifdef _sxbus1
				writeD(1,TRI_STATE);                             // Switch pin to input
				#endif
				_sx_sepCount = SX_SEPLEN;
			} 
			else 
			{
				if (bitRead(_sx_syncFlag,SX0ISRWRITEFLAG))   		// If we want to write
				{						                       
					writeD(0,bitRead(_sx0_write_data, 0));         // Write bit to bus
					_sx0_write_data = _sx0_write_data / 2;        // Prepare for next write
				}
				else
				{
					_sx0_read_data = (_sx0_read_data / 2);            // Prepare for reading data, same as >>2
					bitWrite(_sx0_read_data, 7, _sx0_bit);            // Insert the bit
				}
				#ifdef _sxbus1
				if (bitRead(_sx_syncFlag,SX1ISRWRITEFLAG))   		// If we want to write
				{						                       
					writeD(1,bitRead(_sx1_write_data, 0));         // Write bit to bus
					_sx1_write_data = _sx1_write_data / 2;        // Prepare for next write
				}
				else
				{
					_sx1_read_data = (_sx1_read_data / 2);            // Prepare for reading data, same as >>2
					bitWrite(_sx1_read_data, 7, _sx1_bit);            // Insert the bit
				}
				#endif
			}
			_sx_bitCount--; 
			if (_sx_bitCount == 0) 							// All bits done
			{                          
				// save read _data
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
				{
					if (_sxbus[_sx_index]>255)          			//Check if write flag is set
					{
						_sxbus[_sx_index] &= ~WRITE;          			//Clear Write Flag
					}
					else 
					{
					_sxbus[_sx_index] = _sx0_read_data;          	// Save read data in array
					}
				#ifdef _sxbus1
					if (_sxbus[_sx_index+112]>255)          			//Check if write flag is set
					{
						_sxbus[_sx_index+112] &= ~WRITE;          			//Clear Write Flag
					}
					else 
					{
					_sxbus[_sx_index+112] = _sx1_read_data;          	// Save read data in array
					}
				#endif
				}
				// Setup for next read/write
				_sx_bitCount = SX_BYTELEN;
				// Decrement dataFrameCount
				_sx_dataFrameCount--;
				// check, if we already reached the last DATA block - in this
				// case move on to the next SX-Datenpaket, i.e. look for SYNC or SX2 Bus 	
				if (_sx_dataFrameCount == 0) 
				{
					// Move on to check if SX2 is active
						_sx_dataFrameCount = SX_DATACOUNT;
						_sx_state = SX2Pr;
						_sx_bitCount = SX_BYTELEN / 2;
						_sx_syncCount=SX_STOP;
						_sx_sepCount=SX_SEPLEN;
						_sx0_read_data=0;
						#ifdef _sxbus1
						_sx1_read_data=0;
						#endif
						// Check if we want to write SX2 Praämbel and prepare it, but only when in Sync and Frame is reached
							if ( bitRead(_sx_syncFlag,SX2SYNC))
							{
								if (bitRead(_sx2bus[_sx_numFrame].pr,SX2WRITEPR))
								{
									#if defined(ARDUINO_ARCH_AVR)
									ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
									#endif 
									{
										_sx0_write_data = _sx2bus[_sx_numFrame].pr&~240;     		// Get data to write
									}
									bitSet(_sx_syncFlag,SX0ISRWRITEFLAG);                          // Write
									//#ifdef DEBUG
									//printer->println("Lösche Write Flag Praämbel");
									//#endif
								}
								else 
								{
								bitClear(_sx_syncFlag,SX0ISRWRITEFLAG);                            // No write
								}
								#ifdef _sxbus1
								if (bitRead(_sx2bus[_sx_numFrame+16].pr,SX2WRITEPR))
								{
									#if defined(ARDUINO_ARCH_AVR)
									ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
									#endif 
									{
										_sx1_write_data = _sx2bus[_sx_numFrame+16].pr&~240;     		// Get data to write
									}
									bitSet(_sx_syncFlag,SX1ISRWRITEFLAG);                          // Write
									//#ifdef DEBUG
									//printer->println("Lösche Write Flag Praämbel");
									//#endif
									//bitClear(_sx2bus[_sx_numFrame+16].pr,SX2WRITEPR);		//Delete Write Flag
								}
								else 
								{
								bitClear(_sx_syncFlag,SX1ISRWRITEFLAG);                            // No write
								}
								
								#endif
							}
							else 
							{
								bitClear(_sx_syncFlag,SX0ISRWRITEFLAG);                            // No write
								#ifdef _sxbus1
								bitClear(_sx_syncFlag,SX1ISRWRITEFLAG);                            // No write
								#endif
							}
				} 
				else
				{
					//Calculate new index
					_sx_index = _sx_numFrame + (16*(_sx_dataFrameCount-1));	//Calculate new index
					// Check if we want to write
					#if defined(ARDUINO_ARCH_AVR)
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
					#endif 
					{
						if (_sxbus[_sx_index] > 255)					//Write Bit set
						{
							_sx0_write_data =lowByte (_sxbus[_sx_index]);     // Get data to write
							bitSet(_sx_syncFlag,SX0ISRWRITEFLAG);       	// Write
						}
						else 
						{
							bitClear(_sx_syncFlag,SX0ISRWRITEFLAG);                            // No write
						}
						#ifdef _sxbus1
						if (_sxbus[_sx_index+112] > 255)					//Write Bit set
						{
							_sx1_write_data =lowByte (_sxbus[_sx_index+112]);     // Get data to write
							bitSet(_sx_syncFlag,SX1ISRWRITEFLAG);       	// Write
						} 
						else 
						{
							bitClear(_sx_syncFlag,SX1ISRWRITEFLAG);                            // No write
						}
						#endif
					}
				}
			}
			break;
		case (SX2Pr):
				_sx_sepCount--;
				if (_sx0_bit == LOW) 
				{                              						// Sync bits "0"
					if (_sx_syncCount > 0) 
					{                       						// If more then 3
						_sx_syncCount--;
					}
				}	
				if (_sx_sepCount == 0) 								// Skip the separator
				{                           
					writeD(0,TRI_STATE);                           	// Switch pin to input
					#ifdef _sxbus1
					writeD(1,TRI_STATE);                           	// Switch pin to input
					#endif
					if (_sx_syncCount==0) 		//3 mal 0 in Folge, keine Buserweiterung sondern SYNC!
					{
						_sx_state = SYNC;							//Setup for Sync
						bitClear(_sx_syncFlag,SX0ISRWRITEFLAG); 
						_sx2bus[_sx_numFrame].pr&=~15;				//Lösche SX2 Praämbel
						#ifdef _sxbus1
						bitClear(_sx_syncFlag,SX1ISRWRITEFLAG); 
						_sx2bus[_sx_numFrame+16].pr&=~15;				//Lösche SX2 Praämbel
						#endif
						bitClear(_sx_syncFlag,SX2SYNC);				//Lösche SX2 Syncflag, keine Buserweiterung aktiv!
						break;		
					}
					_sx_syncCount=SX_STOP;
					bitSet(_sx_syncFlag,SX2SYNC);					//Setze SX2 Syncflag, Buserweiterung aktiv!
					_sx_sepCount = SX_SEPLEN;
				} 
				else 
				{
					if (bitRead(_sx_syncFlag,SX0ISRWRITEFLAG))   // If we want to write
					{						                       
						writeD(0,bitRead(_sx0_write_data, 3));         // Write bit to bus
						_sx0_write_data = _sx0_write_data * 2;        // Prepare for next write
					}
					else
					{
						_sx0_read_data = (_sx0_read_data * 2);            // Prepare for reading data, same as >>2
						bitWrite(_sx0_read_data, 0, _sx0_bit);            // Insert the bit
					}
					#ifdef _sxbus1
					if (bitRead(_sx_syncFlag,SX1ISRWRITEFLAG))   // If we want to write
					{						                       
						writeD(1,bitRead(_sx1_write_data, 3));         // Write bit to bus
						_sx1_write_data = _sx1_write_data * 2;        // Prepare for next write
					}
					else
					{
						_sx1_read_data = (_sx1_read_data * 2);            // Prepare for reading data, same as >>2
						bitWrite(_sx1_read_data, 0, _sx1_bit);            // Insert the bit
					}
					#endif
				}
				_sx_bitCount--; 
				if (_sx_bitCount == 0) 							// All bits done
				{	
						if (bitRead(_sx_syncFlag,SX0ISRWRITEFLAG))
						{
							bitClear(_sx2bus[_sx_numFrame].pr,SX2WRITEPR);		//Delete Write Flag
						}
						else
						{
						_sx2bus[_sx_numFrame].pr&=~15;
						_sx2bus[_sx_numFrame].pr|=(_sx0_read_data&~240);
						}
						_sx0_read_data_16=0;
					// Check if we want to write SX2 Adr and prepare it, but only when in Sync and Frame is reached
					if (bitRead(_sx2bus[_sx_numFrame].pr,SX2WRITEADR))
						{
							#if defined(ARDUINO_ARCH_AVR)
							ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
							#endif 
							{
								_sx0_write_data_16 = _sx2bus[_sx_numFrame].adr;     	// Get data to write
							}
							bitSet(_sx_syncFlag,SX0ISRWRITEFLAG);                       // Write
							//#ifdef DEBUG
							//printer->println("Lösche Write Flag Praämbel");
							//#endif
							//bitClear(_sx2bus[_sx_numFrame].pr,SX2WRITEADR);		//Delete Write Flag for SX Adress
							
						} 
						else 
						{
							bitClear(_sx_syncFlag,SX0ISRWRITEFLAG);                            // No write
						}
					#ifdef _sxbus1
						if (bitRead(_sx_syncFlag,SX1ISRWRITEFLAG))
						{
							bitClear(_sx2bus[_sx_numFrame+16].pr,SX2WRITEPR);		//Delete Write Flag
						}
						else
						{
						_sx2bus[_sx_numFrame+16].pr&=~15;
						_sx2bus[_sx_numFrame+16].pr|=(_sx1_read_data&~240);
						}
						_sx1_read_data_16=0;
					// Check if we want to write SX2 Adr and prepare it, but only when in Sync and Frame is reached
					if (bitRead(_sx2bus[_sx_numFrame+16].pr,SX2WRITEADR))
						{
							#if defined(ARDUINO_ARCH_AVR)
							ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
							#endif 
							{
								_sx1_write_data_16 = _sx2bus[_sx_numFrame+16].adr;     	// Get data to write
							}
							bitSet(_sx_syncFlag,SX1ISRWRITEFLAG);                       // Write
							//#ifdef DEBUG
							//printer->println("Lösche Write Flag Praämbel");
							//#endif
							//bitClear(_sx2bussnd[_sx_numFrame].pr,SX2WRITEADR);		//Delete Write Flag for SX Adress
							
						} 
						else 
						{
							bitClear(_sx_syncFlag,SX1ISRWRITEFLAG);                            // No write
						}
					#endif
					_sx_state = SX2Adr;								//Setup for Adr
					_sx_sepCount=SX_SEPLEN;
					_sx_syncCount = SX_STOP;
					_sx_bitCount=SX_BYTELEN*2;						//24 Bits for Reading SX2 Adress and Light(14 Adress Bits+2 Light Bits+8 Space Bits)
				}
			break;
			//**************************************************************MOVE ON HERE*******************************!!!!!!!!!!!!!
		case (SX2Adr):
				_sx_sepCount--;
				if (_sx0_bit == LOW) 
				{                              						// Sync bits "0"
					if (_sx_syncCount > 0) 
					{                       						// If more then 3
						_sx_syncCount--;
					}
				}	
				if (_sx_sepCount == 0) 								// Skip the separator
				{                           
					writeD(0,TRI_STATE);                           	// Switch pin to input
					#ifdef _sxbus1
					writeD(1,TRI_STATE);                           	// Switch pin to input
					#endif
					if (_sx_syncCount==0) 							//3 mal 0 in Folge, keine weiteren Funktionen sondern SYNC!
					{
						_sx_state = SYNC;							//Setup for Sync
						bitClear(_sx_syncFlag,SX0ISRWRITEFLAG); 
						#ifdef _sxbus1
						bitClear(_sx_syncFlag,SX1ISRWRITEFLAG);
						#endif
						break;
					}
					_sx_syncCount=SX_STOP;
					_sx_sepCount = SX_SEPLEN;
				}
				else 
				{
					if (bitRead(_sx_syncFlag,SX0ISRWRITEFLAG))   						// If we want to write
					{						                       
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							writeD(0,bitRead(_sx0_write_data_16, 15));         // Write bit to bus, start with MSB
							_sx0_write_data_16 = _sx0_write_data_16 * 2;        // Prepare for next write
						}
					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							_sx0_read_data_16 = (_sx0_read_data_16*2);            // Prepare for reading data, same as >>2
							bitWrite(_sx0_read_data_16, 0, _sx0_bit);            // Insert the bit
						}
					}
					#ifdef _sxbus1
					if (bitRead(_sx_syncFlag,SX1ISRWRITEFLAG))   						// If we want to write
					{						                       
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							writeD(1,bitRead(_sx1_write_data_16, 15));         // Write bit to bus, start with MSB
							_sx1_write_data_16 = _sx1_write_data_16 * 2;        // Prepare for next write
						}

					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
						_sx1_read_data_16 = (_sx1_read_data_16*2);            // Prepare for reading data, same as >>2
						bitWrite(_sx1_read_data_16, 0, _sx1_bit);            // Insert the bit
						}
					}
					#endif
				}
				_sx_bitCount--; 
				if (_sx_bitCount == 0) 										// All bits done
				{
					if (bitRead(_sx_syncFlag,SX0ISRWRITEFLAG))   
					{
						bitClear(_sx2bus[_sx_numFrame].pr,SX2WRITEADR);		//Delete Write Flag for SX Adress	
					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							_sx2bus[_sx_numFrame].adr= (uint16_t) _sx0_read_data_16;        // Save read data in array
							_sx0_read_data=0;
						}
					}
					#ifdef _sxbus1
					if (bitRead(_sx_syncFlag,SX1ISRWRITEFLAG))   
					{
						bitClear(_sx2bus[_sx_numFrame+16].pr,SX2WRITEADR);		//Delete Write Flag for SX Adress	
					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							_sx2bus[_sx_numFrame+16].adr= (uint16_t) _sx1_read_data_16;        // Save read data in array
							_sx1_read_data=0;
						}
					}
					
					#endif
					_sx_bitCount = SX_BYTELEN;							//12 Bits for Reading SX2 Direction and Speed(8 Bits + 4 Space Bits)
					_sx_state = SX2Fst;									//Setup for Speed and Direction
					_sx_sepCount=SX_SEPLEN;
					// Check if we want to write SX2 FST and Direction and prepare it
					if (bitRead(_sx2bus[_sx_numFrame].pr,SX2WRITEFST))
						{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
							{
							_sx0_write_data = _sx2bus[_sx_numFrame].fst;     	// Get data to write
							}
							bitSet(_sx_syncFlag,SX0ISRWRITEFLAG);                         // Write
							//bitClear(_sx2bus[_sx_numFrame].pr,SX2WRITEFST);		//Delete Write Flag
							/*
							#ifdef DEBUG
							printer->println("Lösche Write Flag Speed");
							#endif
							*/
						}
					else 
					{
						bitClear(_sx_syncFlag,SX0ISRWRITEFLAG);                            // No write
					}
					#ifdef _sxbus1
					if (bitRead(_sx2bus[_sx_numFrame+16].pr,SX2WRITEFST))
						{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
							{
							_sx1_write_data = _sx2bus[_sx_numFrame+16].fst;     	// Get data to write
							}
							bitSet(_sx_syncFlag,SX1ISRWRITEFLAG);                         // Write
							//bitClear(_sx2bus[_sx_numFrame].pr,SX2WRITEFST);		//Delete Write Flag
							/*
							#ifdef DEBUG
							printer->println("Lösche Write Flag Speed");
							#endif
							*/
						}
					else 
					{
						bitClear(_sx_syncFlag,SX1ISRWRITEFLAG);                            // No write
					}
					#endif
				}
					/*
					#ifdef DEBUG
					printer->print("Adresse gelesen:");
					printer->print(_sx2bus[_sx_numFrame].adr,BIN);
					printer->print(" Frame:");
					printer->println(_sx_numFrame);
					#endif
					*/
			break;
		case (SX2Fst):
				_sx_sepCount--;
				if (_sx_sepCount == 0)  								// Skip the separator
				{                          
					writeD(0,TRI_STATE);                             		// Switch pin to input
					#ifdef _sxbus1
					writeD(1,TRI_STATE);                           	// Switch pin to input
					#endif
					_sx_sepCount = SX_SEPLEN;
				} 
				else 
				{
					if (bitRead(_sx_syncFlag,SX0ISRWRITEFLAG))   			// If we want to write
					{						                       
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							writeD(0,bitRead(_sx0_write_data, 0));         	// Write bit to bus
							_sx0_write_data = _sx0_write_data / 2;        	// Prepare for next write
						}
					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							_sx0_read_data = (_sx0_read_data/2);            	// Prepare for reading data, same as >>2
							bitWrite(_sx0_read_data, 7, _sx0_bit);            	// Insert the bit
						}
					}
					#ifdef _sxbus1
					if (bitRead(_sx_syncFlag,SX1ISRWRITEFLAG))   			// If we want to write
					{						                       
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							writeD(1,bitRead(_sx1_write_data, 0));         	// Write bit to bus
							_sx1_write_data = _sx1_write_data/ 2;        	// Prepare for next write
						}
					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							_sx1_read_data = (_sx1_read_data/2);            	// Prepare for reading data, same as >>2
							bitWrite(_sx1_read_data, 7, _sx1_bit);            	// Insert the bit
						}
					}
					#endif
				}
				_sx_bitCount--; 
				if (_sx_bitCount == 0) 									// All bits done
				{
					if (bitRead(_sx_syncFlag,SX0ISRWRITEFLAG))   			// If we want to write
					{
						bitClear(_sx2bus[_sx_numFrame].pr,SX2WRITEFST);		//Delete Write Flag
					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							_sx2bus[_sx_numFrame].fst= _sx0_read_data;       // Save read data in array
						}
					}
					_sx0_read_data_16=0;
					#ifdef _sxbus1
					if (bitRead(_sx_syncFlag,SX1ISRWRITEFLAG))   			// If we want to write
					{
						bitClear(_sx2bus[_sx_numFrame+16].pr,SX2WRITEFST);		//Delete Write Flag
					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							_sx2bus[_sx_numFrame+16].fst= (uint8_t)_sx1_read_data;       // Save read data in array

						}
					}
					_sx1_read_data_16=0;
					#endif
					_sx_bitCount = SX_BYTELEN*2;							//12 Bits for Reading SX2 Direction and Speed(8 Bits + 4 Space Bits)
					_sx_state = SX2Fkt;									//Setup for Funktion
					_sx_sepCount=SX_SEPLEN;								//Read F1-F16 (12*2 Bits);
					_sx_syncCount=SX_STOP;
					// Check if we want to write SX2 Finktions
					if (bitRead(_sx2bus[_sx_numFrame].pr,SX2WRITEFKT))
						{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
							{
							_sx0_write_data_16 = _sx2bus[_sx_numFrame].fkt;     	// Get data to write
							}
							bitSet(_sx_syncFlag,SX0ISRWRITEFLAG);                          // Write
							//#ifdef DEBUG
							//printer->println("Lösche Write Flag Praämbel");
							//#endif
							//bitClear(_sx2bussnd[_sx_numFrame].pr,SX2WRITEFKT);		//Delete Write Flag
						}
					else 
					{
						bitClear(_sx_syncFlag,SX0ISRWRITEFLAG);                            // No write

					}
					#ifdef _sxbus1
					if (bitRead(_sx2bus[_sx_numFrame+16].pr,SX2WRITEFKT))
						{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
							{
							_sx1_write_data_16 = _sx2bus[_sx_numFrame+16].fkt;     	// Get data to write
							}
							bitSet(_sx_syncFlag,SX1ISRWRITEFLAG);                          // Write
							//#ifdef DEBUG
							//printer->println("Lösche Write Flag Praämbel");
							//#endif
			
						}
					else 
					{
						bitClear(_sx_syncFlag,SX1ISRWRITEFLAG);                            // No write

					}
					#endif
				}
			break;
		case (SX2Fkt):
				_sx_sepCount--;
				if (_sx0_bit == LOW) 
				{                              						// Sync bits "0"
					if (_sx_syncCount > 0) 
					{                       						// If more then 3
						_sx_syncCount--;
					}
				}	
				if (_sx_sepCount == 0) 								// Skip the separator
				{                           
					writeD(0,TRI_STATE);                           	// Switch pin to input
					#ifdef _sxbus1
					writeD(1,TRI_STATE);                           	// Switch pin to input
					#endif
					if (_sx_syncCount==0) 							//3 mal 0 in Folge, keine weiteren Funktionen sondern SYNC!
					{
						_sx_state = SYNC;							//Setup for Sync
						bitClear(_sx_syncFlag,SX0ISRWRITEFLAG); 
						#ifdef _sxbus1
						bitClear(_sx_syncFlag,SX1ISRWRITEFLAG); 
						#endif
						#ifdef DEBUG
						if (printer!=NULL)
						{
							printer->println(F("SYNC In Function Bank Reading"));
						}
						#endif
						break;

					}
					_sx_syncCount=SX_STOP;
					_sx_sepCount = SX_SEPLEN;
				} 			
				else 
				{
					if (bitRead(_sx_syncFlag,SX0ISRWRITEFLAG))   		// If we want to write
					{						                       
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							writeD(0,bitRead(_sx0_write_data_16, 0));         // Write bit to bus
							_sx0_write_data_16 = _sx0_write_data_16 / 2;        // Prepare for next write
						}
					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							_sx0_read_data_16 = (_sx0_read_data_16 /2);            // Prepare for reading data, same as >>2
							bitWrite(_sx0_read_data_16, 15, _sx0_bit);           // Insert the bit
						}	
					}
					#ifdef _sxbus1
					if (bitRead(_sx_syncFlag,SX1ISRWRITEFLAG))   		// If we want to write
					{						                       
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							writeD(0,bitRead(_sx1_write_data_16, 0));         // Write bit to bus
							_sx1_write_data_16 = _sx1_write_data_16 / 2;        // Prepare for next write
						}
					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
							_sx1_read_data_16 = (_sx1_read_data_16 /2);            // Prepare for reading data, same as >>2
							bitWrite(_sx1_read_data_16, 15, _sx1_bit);           // Insert the bit
						}	
					}
					#endif
				}
				_sx_bitCount--; 
				if (_sx_bitCount == 0) 							// All bits done
				{                          
					if (bitRead(_sx_syncFlag,SX0ISRWRITEFLAG))   		// If we want to write
					{
						bitClear(_sx2bus[_sx_numFrame].pr,SX2WRITEFKT);		//Delete Write Flag
					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
						_sx2bus[_sx_numFrame].fkt = _sx0_read_data_16;         // Save read data in array
						}
					}
					_sx0_read_data_16=0; 
					_sx0_read_data=0; 
					#ifdef _sxbus1
					if (bitRead(_sx_syncFlag,SX1ISRWRITEFLAG))   		// If we want to write
					{
						bitClear(_sx2bus[_sx_numFrame+16].pr,SX2WRITEFKT);		//Delete Write Flag
					}
					else
					{
						#if defined(ARDUINO_ARCH_AVR)
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
						#endif 
						{
						_sx2bus[_sx_numFrame+16].fkt = _sx1_read_data_16;         // Save read data in array
						}
					}
					_sx1_read_data_16=0;
					_sx1_read_data=0; 
					#endif
					_sx_bitCount = SX_BYTELEN;
					// All SX2 Function read, move on to SYNC
					_sx_state = SYNC;
					_sx_syncCount=SX_STOP;
					bitClear(_sx_syncFlag,SX0ISRWRITEFLAG); 
					#ifdef _sxbus1
					bitClear(_sx_syncFlag,SX1ISRWRITEFLAG); 
					#endif
						/*
						#ifdef DEBUG
						printer->println("Complete SX2 Frame read!");
						#endif
						*/
				}
			break;
		default:
			writeD(0,TRI_STATE);                  			    // Switch pin to input
			bitClear(_sx_syncFlag,SX0ISRWRITEFLAG); 
			#ifdef _sxbus1
			writeD(1,TRI_STATE);                  			    // Switch pin to input
			bitClear(_sx_syncFlag,SX1ISRWRITEFLAG); 
			#endif		
			initVar();                                          // Start looking for SYNC
			break;
	}  //end switch/case _sx_state
}
// Convert Adress to SX2 Adr Par Stored in Array
uint16_t SX2Arduino::calcSX2Par(uint16_t adr) 				
{

	uint8_t par01,par02;
	uint16_t data =0 ;
	par01 = adr%100;
	par02=(adr-par01)/100;
	data=par01;
	data|=par02<<7;
	/*#ifdef DEBUG
	printer->print("Adresse Umgewandelt:");
	printer->println(data,BIN);
	#endif
	*/
	return data;                                  
}
// Convert Stored SX2 Adress Par to Real SX2 Adress
uint16_t SX2Arduino::calcSX2Adr(uint16_t adr) 				
{

	uint8_t par01,par02;
	uint16_t data =0 ;
	par01 = lowByte(adr); 		//Extrakt 10er und 1er Stelle	
	bitClear (par01,7);			//Lösche Bit 7, nicht relevant
	par02 = highByte(adr<<1);	//Extrakt 100er und 1000er Stelle, shift << 
	data = par02*100+par01;		//Addiere parameter zur SX2 Adresse
	/*
	#ifdef DEBUG
	printer->print("Adresse Umgewandelt:");
	printer->println(data,BIN);
	#endif
	*/
	return data;                                  
// functions 'accessing' the SX-bus
}
// Read data from the array, filled by the isr.
int SX2Arduino::get(uint8_t adr)
 {
     // returns the value of a SX address
	 #ifdef _sxbus1
	if (bitRead (adr,7))
	{
		if ((adr&~128) < SX_ADDRESS_NUMBER) 
		{
			bitClear(adr,7);
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
			{
				return (lowByte (_sxbus[adr+112]));  //Return SX bus 1
			}
		}
	}
	#endif
	if (adr < SX_ADDRESS_NUMBER)  
	{
		#if defined(ARDUINO_ARCH_AVR)
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
		#endif 
		{
			return (lowByte (_sxbus[adr]));
		}
	} 
	return (-1);                                              // Save value
}
int SX2Arduino::get(uint8_t adr,uint8_t bit)
 {
     // returns the value of a SX address
	 #ifdef _sxbus1
	if (bitRead (adr,7))
	{
		if ((adr&~128) < SX_ADDRESS_NUMBER) 
		{
			bitClear(adr,7);
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
			{
				return (bitRead(_sxbus[adr+112],bit));  //Return SX bus 1
			}
		}
	}
	#endif
	if (adr < SX_ADDRESS_NUMBER)  
	{
		#if defined(ARDUINO_ARCH_AVR)
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
		#endif 
		{
			return (bitRead(_sxbus[adr+112],bit));  //Return SX bus 1
		}
	} 
	return (-1);                                              // Save value
}
// Write data to the array, writing to the SX-bus is done by the isr.
// Check if invalid address.
uint8_t SX2Arduino::set(uint8_t adr, uint8_t dt) 
{
	#ifdef _sxbus1
	if (bitRead (adr,7))
	{
		
		if ((adr&~128) < SX_ADDRESS_NUMBER) 
		{	
			bitClear(adr,7);
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
			{
				_sxbus[adr+112] = (uint16_t) (dt|WRITE); //Set Data to write with write Flag set
			} 
			return 0;    // success
		}
	}
	#endif
	if (adr < SX_ADDRESS_NUMBER)  
	{
		#if defined(ARDUINO_ARCH_AVR)
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
		#endif 
		{
		_sxbus[adr] = (uint16_t) (dt|WRITE);		//Set Data to write with write Flag set
		}
		return 0;    // success
	}
	return 1;    // address out of range
}
//Set Specific Bit in SX Channel
uint8_t SX2Arduino::setBit(uint8_t adr, uint8_t bit,uint8_t bitVal) 
{
	uint8_t tmp;
	#ifdef _sxbus1
	if (bitRead (adr,7))
	{
		
		if ((adr&~128) < SX_ADDRESS_NUMBER) 
		{	
			bitClear(adr,7);
			tmp= (uint8_t) _sxbus[adr+112];				//Store old data
			bitWrite(tmp, bit,bitVal); 			//Set Bit
			if (tmp!=( uint8_t)_sxbus[adr+112])	//Check if Data was realy cahnged
			{
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
				{
				_sxbus[adr+112] = (uint16_t) (tmp|WRITE); //Set Data to write with write Flag set
				} 
			}
			return 0;    // success
		}
	}
	#endif
	if (adr < SX_ADDRESS_NUMBER)  
	{
		tmp= (uint8_t) _sxbus[adr];				//Store old data
		bitWrite(tmp, bit,bitVal); 			//Set Bit
		if (tmp!=( uint8_t)_sxbus[adr])	//Check if Data was realy cahnged
		{
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
			{
				_sxbus[adr] = (uint16_t) (tmp|WRITE); //Set Data to write with write Flag set
			} 
		}
		return 0;    // success
	}
	return 1;    // address out of range
}
uint8_t SX2Arduino::setBitmask(uint8_t adr, uint8_t dt,uint8_t mask) //Only write specific Bits in Bitmask
{
	uint8_t tmp;
	#ifdef _sxbus1
	if (bitRead (adr,7))
	{
		
		if ((adr&~128) < SX_ADDRESS_NUMBER) 
		{	
			bitClear(adr,7);
			tmp= (uint8_t) _sxbus[adr+112];				//Store old data
			tmp&=~mask; //Clear Bit Mask
			tmp |= dt; //Set Data 
			if (tmp!=( uint8_t)_sxbus[adr+112])	//Check if Data was realy cahnged
			{
				#if defined(ARDUINO_ARCH_AVR)
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
				#endif 
				{
					_sxbus[adr+112] = (uint16_t) (tmp|WRITE); //Set Data to write with write Flag set
				} 
			}
			return 0;    // success
		}
	}
	#endif
	if (adr < SX_ADDRESS_NUMBER)  
	{
		tmp= (uint8_t) _sxbus[adr];				//Store old data
		tmp&=~mask; //Clear Bit Mask
		tmp |= dt; //Set Data 
		if (tmp!=( uint8_t)_sxbus[adr])	//Check if Data was realy cahnged
		{
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
			{
				_sxbus[adr] = (uint16_t) (tmp|WRITE); //Set Data to write with write Flag set
			} 
		}
		return 0;    // success
	}
	return 1;    // address out of range
}
// Checks if the isr has written the data to the SX-bus
uint8_t SX2Arduino::isSet(uint8_t adr)
 {
	#ifdef _sxbus1
	if (bitRead(adr,7))
	{
		if ((adr&~128) < SX_ADDRESS_NUMBER)  
		{
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
			{
				bitClear(adr,7);
				if (_sxbus[adr+112]>255) 
				{
					return 2;   // not written
				}
				else 
				{
					return 0;   // written
				}
			}
		}
	}
	#endif
	if (adr < SX_ADDRESS_NUMBER)  
	{
		#if defined(ARDUINO_ARCH_AVR)
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
		#endif 
		{
			if (_sxbus[adr]>255) 
			{
				return 2;   // not written
			}
			else 
			{
				return 0;   // written
			}
		}
	}
	return 1;    // address out of range
}
// Checks if the isr has written the SX2 data to the SX-bus
uint8_t SX2Arduino::isSX2Set(uint8_t frame)
{
	if (frame<SX2_FRAMES)
	{
		return ((_sx2bus[frame].pr&~15)>>4);	//return status flags
	}
	else 
	{
		return 0xFF;   // fault
	}
}
// Write Praämbel to the SX2 Frame to send, writing to the SX-bus is done by the isr.
uint8_t SX2Arduino::setSX2Pr(uint8_t frame, uint8_t dt) 
{
	//if (isSX2Set(frame)==1)		//Check if writing SX2 is currently active, or SX2 not in SYNC!
	//{
		//return 2;	
	//}
	if (dt < 16 && frame < SX2_FRAMES)  			// Check if invalid address or frame.
	{
		_sx2bus[frame].pr &=~15; 				//Clear the first 4 Bits
		_sx2bus[frame].pr |= (dt&15);			//Load new first 4 
		bitSet(_sx2bus[frame].pr,SX2WRITEPR);	//Set Write Flag
		return 0;    // success
	}
	return 1;    // address out of range
}
// Write Adr to the SX2 Frame to send, writing to the SX-bus is done by the isr.
uint8_t SX2Arduino::setSX2Adr(uint8_t frame, uint16_t adr,uint8_t format) 
{
	//if ((isSX2Set(frame)==2) || (isSX2Set(frame)==1))		//Check if writing SX2 is currently active, or SX2 not in SYNC!
	//{
		//return 2;					
	//}
	if ((frame < SX2_FRAMES) && (format<16))  		// Check if invalid address or frame.
	{
		#if defined(ARDUINO_ARCH_AVR)
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
		#endif 
			{
			_sx2bus[frame].pr 	=0; 				//Clear Präambel with write flags
			_sx2bus[frame].pr |= (format&15);		//Load new first 4 Bits
			_sx2bus[frame].adr &=~ 0xFFFF;			//Clear all BITS
			_sx2bus[frame].adr |= adr;			//Load new adress from BIT 15 to BIT2 (BIT1 for Light BIT 0 for DCC FST)
			}
			bitSet(_sx2bus[frame].pr,SX2WRITEPR);	//Set Write Flag Präambel
			bitSet(_sx2bus[frame].pr,SX2WRITEADR);	//Set Write Flag Adr
			/*
			#ifdef DEBUG
			printer->print(F("Schreibe Adresse:"));
			printer->println(adr,BIN);
			#endif
			*/
			return 0;    // success
	
	}
	return 1;    // address out of range
}
// Write Light to the SX2 Frame to send, writing to the SX-bus is done by the isr.
uint8_t SX2Arduino::setSX2Li(uint8_t frame, bool dt) 
{
	//if ((_sx2bus[frame].pr&~240)==0)							//Frame not active!
	//{
		//return 0xFF;
	//}
	if ((dt < 2) && (frame < SX2_FRAMES)) 		// Check if invalid address or frame.
	{
		#if defined(ARDUINO_ARCH_AVR)
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
		#endif 
		{
			if (dt==true)
				{
				bitSet(_sx2bus[frame].adr,1);
				}
				else
				{
				bitClear(_sx2bus[frame].adr,1);
				}
		}
		bitSet(_sx2bus[frame].pr,SX2WRITEADR);	//Set Write Flag
		#ifdef DEBUG
		if (printer!=NULL)
		{
			printer->println(F("Schreibe Licht"));
		}
		#endif
		return 0;    // success
	}
	return 1;    // address out of range
}
uint8_t SX2Arduino::setSX2Dccfst(uint8_t frame, bool dt) 
{
	//if ((_sx2bus[frame].pr&~240)==0)							//Frame not active!
	//{
		//return 0xFF;
	//}
	if ((dt < 2) && (frame < SX2_FRAMES)) 		// Check if invalid address or frame.
	{
		#if defined(ARDUINO_ARCH_AVR)
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
		#endif 
			{
				if (dt==true)
				{
					bitSet(_sx2bus[frame].adr,0);
				}
				else
				{
					bitClear(_sx2bus[frame].adr,0);
				}
			}
			bitSet(_sx2bus[frame].pr,SX2WRITEADR);	//Set Write Flag
			#ifdef DEBUG

		if (printer!=NULL)
		{
			printer->println(F("Schreibe DCC F14 BIT"));
		}
		#endif
		return 0;    // success
	}
	return 1;    // address out of range
}
// Write Speed to the SX2 Frame to send, writing to the SX-bus is done by the isr.
uint8_t SX2Arduino::setSX2Speed(uint8_t frame, uint8_t speed, bool dir) 
{
		/*#ifdef DEBUG
		printer->print(F("Speed Funktion aufgerufen:-->frame:"));
		printer->print(frame);
		printer->print(F(" Präambel Bus Send:"));
		printer->println(_sx2bussnd[frame].pr,BIN);
		#endif
		*/
	//if ((_sx2bus[frame].pr&~240)==0)							//Frame not active!
	//{
		//return 0xFF;
	//}
	if ((speed < 128) && (frame < SX2_FRAMES) && (dir < 2))  	// Check if invalid address or frame.
	{
		_sx2bus[frame].fst  =	0;					//Clear all BITS
		_sx2bus[frame].fst |= speed;					//Load Speed
		_sx2bus[frame].fst |= (dir<<7);				//Load Direction
		bitSet(_sx2bus[frame].pr,SX2WRITEFST);		//Set Write Flag
		#ifdef DEBUG
		if (printer!=NULL)
		{
			printer->print(F("Schreibe Speed:"));
			printer->println(_sx2bus[frame].fst);
		}
		#endif
		return 0;    // success
	}
	return 1;    // address out of range
}
// Write Fuction to the SX2 Frame to send, writing to the SX-bus is done by the isr.
uint8_t SX2Arduino::setSX2Fkt(uint8_t frame, uint16_t dt) 
{
	//if ((_sx2bus[frame].pr&~240)==0)							//Frame not active!
	//{
		//return 0xFF;
	//}
	if (frame < SX2_FRAMES)  		// Check if invalid frame.
	{

		_sx2bus[frame].fkt	= dt;
		bitSet(_sx2bus[frame].pr,SX2WRITEFKT);		//Set Write Flag
		#ifdef DEBUG
		if (printer!=NULL)
		{
			printer->println(F("Schreibe Funktionen"));
		}
		#endif
		return 0;    // success
	}
	return 1;    // address out of range
}

// Read POWER status from the SX-bus
bool SX2Arduino::getPWR() 
{
	if (bitRead(_sx_syncFlag,SXPWR))
	{
		return true;		//Return pwr set
	}
	else 
	{
		return false;		//Return no pwr set
	}
}
uint8_t SX2Arduino::getZEState()
{
	if (bitRead(_sxbus[109],6))
		return (0x20);			//Service Mode Active
	else if (bitRead(_sx_syncFlag,SXPWR))
		return (0x00);			//Normal Mode
	else 
		return (0x02);			//Track Voltage off
}

// Write POWER status to the SX-bus and control a connected central.
void SX2Arduino::setPWR(bool val) 
{
	if (val == 0 || val == 1)
	{
		_sx_newPWR = val;
	}
}

// Every time frame 0 is passed sync bit is set by isr.
uint8_t SX2Arduino::inSync()
{
	if (bitRead (_sx_syncFlag,SX1SYNC)) 
	{
		bitClear(_sx_syncFlag,SX1SYNC);             // reset sync bits to check for next pass
		return 1;                  					// report frame 0 found
	}
	return 0;
}
uint8_t SX2Arduino::inSX2Sync()
{
	if (bitRead (_sx_syncFlag,SX2SYNC)&& bitRead (_sx_syncFlag,SX1SYNC)) 
	{				
		return 1;                  					// report sx2 sync
	}
	return 0;
}
//Check if SX2Frame used
uint8_t  SX2Arduino::checkSX2Frame (uint8_t frame)
{
if (frame >= SX2_FRAMES)
	return (0xFF); //return Fehler
else if ((_sx2bus[frame].pr&15) == 0)
	return false;
else
	return true;
}
//Search empty SX2 frame
uint8_t SX2Arduino::searchSX2EmptyFrame (void)
{
for (uint8_t i=0;i<SX2_FRAMES;i++)
	{
	if ((_sx2bus[i].pr&15)==0) 	//Check if Frame is used
		return i;					//if frame empty, return frame nbr
	}
return (0xFF);						//no empty frame found	
}
// Return Decoder POM of given Frame
bool SX2Arduino::returnSX2POM(uint8_t frame)
{
	if (frame<SX2_FRAMES)
	{
		if (bitRead((_sx2bus[frame].pr&15),0))		//POM Mode
		{
			return (true);			//Prog mode
		}
	}
	return (false);
}
// Return Decoder Format of given Frame
uint8_t SX2Arduino::returnSX2Format(uint8_t frame)
{
		/*
		#ifdef DEBUG
		if (printer!=NULL)
		{
			printer->println(F(" "));
			printer->print(F("Loco Format-->SX2 Präambel:"));
			printer->println(_sx2bus[frame].pr,BIN);
		}
		#endif
		*/
	if (frame<SX2_FRAMES)
	{
		switch (_sx2bus[frame].pr&15)
		{
			case (0):
				return(0x00);
			case (1):
				return (0x02);
			case (2):
				return(0x04);
			case (8):
				return(0x01);
			case (10):
				return(0x05);
			case(12):
				return (0x03);
			case(14):
				return (0x07);
			default:
				return (0x00);
		}
	}
	return (0xFF);								//Return Fault
}


//Reutn Adress of given Frame 
uint16_t SX2Arduino::returnSX2Adr(uint8_t frame)
{
if ((_sx2bus[frame].pr&15)!=0 && frame<SX2_FRAMES) 						//Check if Frame is used
  {	
		return (_sx2bus[frame].adr&0xFFFC);			//return  Adress stored in Frame,no convertion needed
  }
return 0x0000	;									//return nothing
}
uint16_t SX2Arduino::returnSX2AdrLiDcc(uint8_t frame)				//Returnung complete value store, without converstation
{
if 	((_sx2bus[frame].pr&15)!=0 && frame<SX2_FRAMES)
	{
	return (_sx2bus[frame].adr);
	}
return 0;
}
// Return Light Status of given Frame
uint8_t SX2Arduino::returnSX2Li(uint8_t frame)
{
if ((_sx2bus[frame].pr&15)!=0&& frame<SX2_FRAMES) 						//Check if Frame is used
		return ((_sx2bus[frame].adr&2)>>1);			//Return Light
	else 
		return (0x00);									//return Fehler
}
uint8_t SX2Arduino::returnSX2Dccfst(uint8_t frame)
{
if ((_sx2bus[frame].pr&15)!=0&& frame<SX2_FRAMES) 						//Check if Frame is used
		return ((_sx2bus[frame].adr&1));			//Return DCC fst
	else 
		return (0x00);									//return nothing
}
// Return Loco Speed of given Frame
uint8_t SX2Arduino::returnSX2Fst(uint8_t frame)
{
if ((_sx2bus[frame].pr&15)!=0 &&frame<SX2_FRAMES) 						//Check if Frame is used
		return (_sx2bus[frame].fst);			//Return Speed with Direction Bit set
	else 
		return (0x00);								//return nothing
}
// Return Loco Direction of given Frame
uint8_t SX2Arduino::returnSX2Dir(uint8_t frame)
{
if ((_sx2bus[frame].pr&15)!=0&& frame<SX2_FRAMES) 						//Check if Frame is used
		return ((_sx2bus[frame].fst&128)>>7);	//Return Direction
	else 
		return (0x00);								//return nothing
}
// Return Funkion Bank of given Frame and Funktion Bank Number
uint16_t SX2Arduino::returnSX2Fkt(uint8_t frame)
{
if ((_sx2bus[frame].pr&15)!=0 && frame<SX2_FRAMES) 						//Check if Frame is used
		return (_sx2bus[frame].fkt);		//Funktion Status 
	else 
		return (0x00);							//return nothing
}
// Function to Register new loco on SX Bus
uint8_t SX2Arduino::regLoco (uint16_t adr, uint8_t format)
{
	unsigned long startMillis = millis();							
	uint8_t state=0;
	uint8_t frame=checkLoco (adr, format);
	adr=adr&0xFFFC;
	//regflag=true;
	if (format>15)
	{
		return (0xFF);
	}
	if (frame<SX2_FRAMES)							//Loco already registred
	{
		return frame;
	}
	else if (frame==0xFF)							//Loco was never registred, seach empty frame
	{
		frame=searchSX2EmptyFrame ();				//Search new empty frame
		if (frame==255)								//No empty frame found
		{
			return (0xFF);
		}
	}
	else if (bitRead(frame,6))
	{
		bitClear(frame,6);
		#ifdef DEBUG
		if (printer!=NULL)
		{
			printer->print(F("Loco Registred before, new "));
		}
		#endif
	}
	while (1)											//Loop to register adress
		{
			switch (state)
			{
				case (0):
					#ifdef DEBUG
					if (printer!=NULL)
					{
						printer->print(F("Register SX2 Loco--->frame:"));
						printer->println(frame);
						printer->print(F(" Adr:"));
						printer->println(adr,BIN);
					}
					#endif
					if (setSX2Adr(frame,adr,format)==0)//Set the Adress and Präambel on empty frame
						{
						state=1;							//Switch to next state
						}
					break;
				case (1):
					#ifdef DEBUG
					if (printer!=NULL)
					{
						printer->print(F("Wait till Präambel is set, ms:"));
						printer->println((millis()-startMillis));
					}
					#endif	
					if (isSX2Set(frame)==0)				//Wait till SX2 Adress is set
						{
						state=2;
						}
					break;
				case (2):
					#ifdef DEBUG
					if (printer!=NULL)
					{
						printer->print(F("Wait till Präambel is new read, ms:"));
						printer->println((millis()-startMillis));
					}
					#endif
					#if defined(ARDUINO_ARCH_AVR)
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
					#endif 
					{
						_sx2bus[frame].adr = 0;					//Reset recived adress to zero
					}
					state=3;
					break;
				case (3):
					
					#ifdef DEBUG
					if (printer!=NULL)
					{
						printer->print(F("Wait till Adress is new read, ms:"));
						printer->println((millis()-startMillis));
					}
					#endif	
					if (!(_sx2bus[frame].adr == 0))					//Wait till Adress is new read from bus
					{
						state=4;
					}
					break;
				case (4):
					if (((_sx2bus[frame].adr&0xFFFC)) == adr) //Loco Registred, set default values
					{
						setSX2Adr(frame,adr,format);
						//setSX2Li(frame,0);
						//setSX2Speed(frame,0,0);
						//setSX2Fkt(frame,0);
						#ifdef DEBUG
						if (printer!=NULL)
						{
							printer->print(F("Loco Registred,ms:"));
							printer->println((millis()-startMillis));
						}
						#endif
						return (frame);										//return registred frame	
					}
					else 
					{
						#ifdef DEBUG
						if (printer!=NULL)
						{
							printer->println(F("New Try to register Loco-->"));
						}
						#endif
						state=0;
						frame=searchSX2EmptyFrame ();					//Search new empty frame
						if (frame==255)								//No empty frame found
						{
						return (0xFF);
						}						
						//setSX2Adr(frame,adr,format);				//Set the Adress and Präambel on empty frame	
					}						
					break;
				default:
					break;
			}	//end switch state
			if ((millis()-startMillis)>=500)							//End loco registration after 2000 ms		
			{
				#ifdef DEBUG
				if (printer!=NULL)
				{
					printer->println(F("Register LOCO failed"));
				}
				#endif 
				bitClear (_sx2bus[frame].pr,SX2WRITEPR);		//Set no writing
				bitClear (_sx2bus[frame].pr,SX2WRITEADR);	//Set no writing
				#if defined(ARDUINO_ARCH_AVR)
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
				#endif 
				{
					_sx2bus[frame].pr &= ~15;				//Reset Präambel on recive frames
					_sx2bus[frame].adr = 0;				//Reset Adr on recive frames
				}
				return 0xFF;							//return fault 
			}
			
		}
	return (0xFF);
}
// Function to Register new loco on SX Bus, with serial return
#if defined(ARDUINO_ARCH_AVR)
uint8_t SX2Arduino::regLoco (uint16_t adr, uint8_t format,HardwareSerial* hwPrint)
#elif defined(ARDUINO_ARCH_SAM)
uint8_t SX2Arduino::regLoco (uint16_t adr, uint8_t format,Serial_* hwPrint)
#endif
{
	unsigned long startMillis = millis();							
	uint8_t state=0;
	uint8_t frame=checkLoco (adr, format);
	adr=adr&0xFFFC;
	if (format>15)
	{
		hwPrint->write(0xFF);
		return (0xFF);	
	}
	if (frame<SX2_FRAMES)							//Loco already registred
	{
		hwPrint->write(frame);
		return frame;
	}
	else if (frame==0xFF)							//Loco was never registred, seach empty frame
	{
		frame=searchSX2EmptyFrame ();				//Search new empty frame
		if (frame==255)								//No empty frame found
		{
			hwPrint->write(0xFF);
			return (0xFF);
		}
	}
	else if (bitRead(frame,6))
	{
		bitClear(frame,6);
		#ifdef DEBUG
		if (printer!=NULL)
		{
			printer->print(F("Loco Registred before, new "));
		}
		#endif
	}
	while (1)											//Loop to register adress
		{
			switch (state)
			{
				case (0):
					#ifdef DEBUG
					if (printer!=NULL)
					{
						printer->print(F("Register SX2 Loco--->frame:"));
						printer->println(frame);
						printer->print(F(" Adr:"));
						printer->println(adr,BIN);
					}
					#endif
					if (setSX2Adr(frame,adr,format)==0)//Set the Adress and Präambel on empty frame
						{
						state=1;							//Switch to next state
						}
					break;
				case (1):
					#ifdef DEBUG
					if (printer!=NULL)
					{
						printer->print(F("Wait till Präambel is set, ms:"));
						printer->println((millis()-startMillis));
					}
					#endif	
					if (isSX2Set(frame)==0)				//Wait till SX2 Adress is set
						{
						state=2;
						hwPrint->write(frame);
						}
					break;
				case (2):
					#ifdef DEBUG
					if (printer!=NULL)
					{
						printer->print(F("Wait till Präambel is new read, ms:"));
						printer->println((millis()-startMillis));
					}
					#endif
					#if defined(ARDUINO_ARCH_AVR)
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
					#endif 
					{
						_sx2bus[frame].adr = 0;					//Reset recived adress to zero
					}
					state=3;
					break;
				case (3):
					
					#ifdef DEBUG
					if (printer!=NULL)
					{
						printer->print(F("Wait till Adress is new read, ms:"));
						printer->println((millis()-startMillis));
					}
					#endif	
					if (!(_sx2bus[frame].adr == 0))					//Wait till Adress is new read from bus
					{
						state=4;
					}
					break;
				case (4):
					if (((_sx2bus[frame].adr&0xFFFC)) == adr) //Loco Registred, set default values
					{
						#ifdef DEBUG
						if (printer!=NULL)
						{
							printer->print(F("Loco Registred,ms:"));
							printer->println((millis()-startMillis));
						}
						#endif
						setSX2Adr(frame,adr,format);
						//setSX2Li(frame,0);
						//setSX2Speed(frame,0,0);
						//setSX2Fkt(frame,0);
						return (frame);										//return registred frame	
					}
					else 
					{
						#ifdef DEBUG
						if (printer!=NULL)
						{
							printer->println(F("New Try to register Loco-->"));
						}
						#endif
						state=0;
						//frame=searchSX2EmptyFrame ();					//Search new empty frame
						//if (frame==255)								//No empty frame found
						//{
							//regflag=false;
							//return (0xFF);
						//}						
						//setSX2Adr(frame,adr,format);				//Set the Adress and Präambel on empty frame	
					}						
					break;
				default:
					break;
			}	//end switch state
			if ((millis()-startMillis)>=2000)							//End loco registration after 1000 ms		
			{
				#ifdef DEBUG
				if (printer!=NULL)
				{
					printer->println(F("Register LOCO failed"));
				}
				#endif 
				bitClear (_sx2bus[frame].pr,SX2WRITEPR);		//Set no writing
				bitClear (_sx2bus[frame].pr,SX2WRITEADR);	//Set no writing
				#if defined(ARDUINO_ARCH_AVR)
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
				#endif 
				{
					_sx2bus[frame].pr &= ~15;				//Reset Präambel on recive frames
					_sx2bus[frame].adr = 0;				//Reset Adr on recive frames
				}
				return 0xFF;							//return fault, registration takes to long
			}
			
		}
	return (0xFF);
}
// Function to Check SX2BUS if loco already registred
uint8_t SX2Arduino::checkLoco (uint16_t adr,uint8_t format)
{
	adr=adr&0xFFFC;
	if (format>15)
	{
		return (0xFF);	
	}
	for (uint8_t i=0;i<SX2_FRAMES;i++)
	{
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
		{
		if ((adr ==((_sx2bus[i].adr&0xFFFC))) && (format== (_sx2bus[i].pr &15)))	
			{
			return i;																	//Return Frame Number if Adress Match and Präambel
			}
		}
	}
	for (uint8_t i=0;i<SX2_FRAMES;i++)
	{
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
		{
			if (adr ==((_sx2bus[i].adr&0xFFFC)))
			{
			bitSet(i,6);																//Frame used before, return before used number											
			return i;																	//Return Frame Number if Adress Match, with BIT 6 set	
			}
		}
	}
	return (0xFF); 																		//return no Adress found
	
}
// Function to hold LOCO on SX2 BUS
uint8_t SX2Arduino::holdLoco (uint8_t frame,uint8_t format,uint16_t adr)
{
	adr=adr&0xFFFC;
	if (format > 15)		//Check given values
		{
		return 0xFF;								//return fault
		}
	if (frame<SX2_FRAMES)		//Loco already registred
	{
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
		{
			_sx2bus[frame].pr 	&=~15; 				//Clear Präambel without write flags
			_sx2bus[frame].pr |= (format&15);		//Load new first 4 Bits
			_sx2bus[frame].adr &=~ 0xFFFC;			//Clear all BITS
			_sx2bus[frame].adr |= adr;			//Load new adress from BIT 15 to BIT2 (BIT1 for Light BIT 0 for DCC FST)
		}
		bitSet(_sx2bus[frame].pr,SX2WRITEPR);	//Set Write Flag Präambel
		bitSet(_sx2bus[frame].pr,SX2WRITEADR);	//Set Write Flag Adr
		return 0;    // success
}
return 0xFF;   
}
uint8_t SX2Arduino::holdLoco (uint8_t frame,uint8_t format)
{
	if (format > 15)		//Check given values
		{
		return 0xFF;								//return fault
		}
	if (frame<SX2_FRAMES)		//Loco already registred
	{
		if (setSX2Pr(frame,format)==0)
		{
			
			return 0;    // success
		}
		else
		{
			return 0xFF; 
		}	
	}
return 0xFF;   
}
//jold loco, to call in isr, no checks!!
void SX2Arduino::isrholdLoco (uint8_t frame)
{
	if (frame<SX2_FRAMES)		//Set Write flag for register loco
	{
		bitSet(_sx2bus[frame].pr,SX2WRITEPR);	//Set Write Flag Präambel
		//bitSet(_sx2bus[frame].pr,SX2WRITEADR);	//Set Write Flag Adr
	}
}
// Set SX2 POM
uint8_t SX2Arduino::regPOM( uint8_t adr_high,uint8_t adr_low, uint8_t format, uint8_t par100, uint8_t par1, uint8_t dat) 
{
	unsigned long startMillis = millis();
	uint8_t tmpformat=format;
	uint8_t tmpfst = 0;
	uint16_t tmpfkt = 0;
	uint8_t state = 0;
	uint8_t frame=0;
	uint16_t adr=0;
	for (uint8_t i=0;i<7;i++)
	{
		if (bitRead(adr_low,i))	
		{
			bitSet(adr,i+2);	
		}
		else
		{
			bitClear(adr,i+2);	
		}
		if (bitRead(adr_high,i))	
		{
			bitSet(adr,i+9);	
		}
		else
		{
			bitClear(adr,i+9);	
		}
	}
	//uint16_t adr= (((adr_high&~128) << 10 ) | (adr_low&~128)<<2);   //Generate Adress
	bitSet(tmpformat,0);								//Set PROG bit
	frame=checkLoco (adr, tmpformat);
	if (tmpformat > 15 || par100>15 || par1>99)		//Check given values
	{
		return 0xFF;								//return fault
	}
	if ((frame==0xFF) || (bitRead(frame,5)))							//Loco was never registred, seach empty frame
	{
		frame=searchSX2EmptyFrame ();				//Search new empty frame
		if (frame==255)								//No empty frame found
		{
			return (0xFF);
		}
	}
	else if (frame<SX2_FRAMES)									//Loco already registred
	{
		return frame;
	}
	par100&=15;																											//Only first four bits used
	par1&=127;																											//Only first seven bits used
	for (uint8_t i=0;i<4;i++)
	{
		bitWrite (tmpfst,3+i,bitRead(par100,3-i));
	}
	bitWrite (tmpfst,7,(bitRead(par1,6)));	
	for (uint8_t i=0;i<7;i++)
	{
		bitWrite (tmpfkt,i,bitRead(par1,6-i));
	}
	tmpfkt|=dat<<6;
	#ifdef DEBUG
	if (printer!=NULL)
	{
		printer->println(F("------>SX2 POM<------"));
		printer->print(F("Set Fst with bits:"));
		printer->print(tmpfst,BIN);
		printer->print(F(" Set Fkt with bits"));
		printer->println(tmpfkt,BIN);
	}
	#endif
	while (1)											//Loop to register adress
	{
		switch (state)
		{
			case (0):
				#ifdef DEBUG
				if (printer!=NULL)
				{
					printer->print(F("Register SX2 Loco--->frame:"));
					printer->print(frame);
					printer->print(F(" Adr:"));
					printer->println(adr,BIN);
				}
				#endif
				#if defined(ARDUINO_ARCH_AVR)
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
				#endif 
				{
					_sx2bus[frame].pr &=~15; 				//Clear the first 4 Bits
					_sx2bus[frame].pr |= (tmpformat&15);		//Load new first 4 Bits
					_sx2bus[frame].adr = adr;		//Load new adress from BIT 15 to BIT2 (BIT1 for Light BIT 0 for DCC FST)
					_sx2bus[frame].fst = tmpfst;				//Load fst values to send
					_sx2bus[frame].fkt = tmpfkt;				//Load Function values to send
					bitSet(_sx2bus[frame].pr,SX2WRITEFST);	//Set Write Flag Fst
					bitSet(_sx2bus[frame].pr,SX2WRITEFKT);	//Set Write Flag Fkt
					bitSet(_sx2bus[frame].pr,SX2WRITEPR);	//Set Write Flag Präambel
					bitSet(_sx2bus[frame].pr,SX2WRITEADR);	//Set Write Flag Adr
				}
				state=1;									//Switch to next state
				break;
			case (1):
				#ifdef DEBUG
				if (printer!=NULL)
				{
				printer->print(F("Wait till Präambel is set, ms:"));
				printer->println((millis()-startMillis));
				}
				#endif	
				if (isSX2Set(frame)==0)				//Wait till SX2 Adress is set
					{
					state=2;
					}
				break;
			case (2):
				#ifdef DEBUG
				if (printer!=NULL)
				{
				printer->print(F("Wait till Präambel is new read, ms:"));
				printer->println((millis()-startMillis));
				}
				#endif
				#if defined(ARDUINO_ARCH_AVR)
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
				#endif 
				{
				_sx2bus[frame].adr = 0;					//Reset recived adress to zero
				}
				state=3;
				break;
			case (3):
				#ifdef DEBUG
				if (printer!=NULL)
				{
				printer->print(F("Wait till Adress is new read, ms:"));
				printer->println((millis()-startMillis));
				}
				#endif	
				if (!(_sx2bus[frame].adr == 0))					//Wait till Adress is new read from bus
				{
					state=4;
				}
				break;
			case (4):
				if ((_sx2bus[frame].adr) == adr) //POM Registred, 
				{
					#if defined(ARDUINO_ARCH_AVR)
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
					#endif 
					{
						_sx2bus[frame].fst = tmpfst;				//Load fst values to send
						_sx2bus[frame].fkt = tmpfkt;				//Load Function values to send
						bitSet(_sx2bus[frame].pr,SX2WRITEFST);	//Set Write Flag Fst
						bitSet(_sx2bus[frame].pr,SX2WRITEFKT);	//Set Write Flag Fkt
					}
					#ifdef DEBUG
					if (printer!=NULL)
					{
						printer->print(F("POM Registred, ms:"));
						printer->println((millis()-startMillis));
					}
					#endif
					return (frame);										//return registred frame	
				}
				else 
				{
					#ifdef DEBUG
					if (printer!=NULL)
					{
						printer->println(F("New Try to register POM-->"));
					}
					#endif
					state=0;
					/*
					frame=searchSX2EmptyFrame ();					//Search new empty frame
					if (frame==255)								//No empty frame found
					{
					regflag=false;
					return (0xFF);
					}
					*/
				}						
				break;
			default:
				break;
		}	//end switch state
		if ((millis()-startMillis)>=100)							//End loco registration after 2000 ms		
		{
			#ifdef DEBUG
			if (printer!=NULL)
			{
				printer->println(F("Register POM failed"));
			}
			#endif 
			#if defined(ARDUINO_ARCH_AVR)
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
			#endif 
			{
				bitClear (_sx2bus[frame].pr,SX2WRITEPR);		//Set no writing
				bitClear (_sx2bus[frame].pr,SX2WRITEADR);	//Set no writing
				bitClear(_sx2bus[frame].pr,SX2WRITEFST);	//Set Write Flag Fst
				bitClear(_sx2bus[frame].pr,SX2WRITEFKT);	//Set Write Flag Fkt
				_sx2bus[frame].pr &=~15; 				//Clear the first 4 Bits
				_sx2bus[frame].adr = 0;				//Reset Adr on recive frames
			}
			return 0xFF;							//return fault 
		}
			
	}
	return (0xFF);
}
