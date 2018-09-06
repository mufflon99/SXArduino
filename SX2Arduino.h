/*
 * SXArduino.h
 *
 * Changed on: 06.09.2018
 * Version: 5.1
 * Changes: Now support reading SX BUS 0 and SX BUS 1
 * 
 * Version: 4.1 
 * Copyright: Michael Berger, main work done by Gerard van der Sel!
 *	
 * Changed on: 22.08.2018
 * Version: 4.1
 * Changes: Read and write SX2 Bus addition should now woking correctly, registration of SX2 Locos optimized
 *
 * Changed on: 14.08.2018
 * Version: 4.0
 * Changes: Read and write SX2 Bus addition, alpha state, need some more optmisations
 * 
 *  Version:    3.1
 *  Copyright:  Gerard van der Sel
 *
 *  Changed on: 27.12.2015
 *  Version: 	3.1
 *  Changes: 	Seperated read and write. Write on falling edge, read on rising edge (ISR on CHANGE).
 *
 *  Changed on: 19.12.2015
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
 *  interface hardware needed ! see 

 Read SX Signal - SX Clock must be connected to Pin3 = INT1 and
 SX Data must be connected to Pin 5. Both are connected through a resistor off 22 kilo ohm.
 Pin 6 can be connected via a 100 ohm resistor to the write line 
 of the SX bus
 
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

#ifndef SX2Arduino_H_
#define SX2Arduino_H_

//#define DEBUG                                //For Debugging over HardwareSerial
// define arduino pins, ports and bits
// depends on the hardware used.
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#ifdef DEBUG
	#warning "-->DEBUG MODE<--"
#endif

#define _use4pin                             // Switch for 3 or 4 portpin interface (set to 4 portpin)
#define _sxbus1								//Reading and writing both SX Busses (0 and 1)

#if defined (__AVR_ATmega328P__) ||  defined (__AVR_ATmega48P__) || defined (__AVR_ATmega88P__)  || defined (__AVR_ATmega168P__) // default settings for Arduino Nano 3.0 5V 16MHz
	#define SX_T0			2                    // Clock
	#define SX_T0_PORTPIN 	PIND2 				 // SX_T0
	#define SX_T0_PINREG	PIND
	#define SX_T0_DDR		DDRD
	#ifndef _use4pin
		#define SX0_T1			4                    	//SX0 Data read arduino port
		#define SX0_T1_PORTPIN 	PIND4                  // SX0_T1
		#define SX0_T1_PINREG	PIND
		#define SX0_T1_DDR		DDRD

		#define SX0_D			7                     //SX0 Data write arduino port
		#define SX0_D_PORTPIN	PORTD7			       // SX0_D
		#define SX0_D_PORT		PORTD
		#define SX0_D_DDR		DDRD

		#ifdef _sxbus1
			#define SX1_T1			8                    	//SX1 Data read arduino port
			#define SX1_T1_PORTPIN 	PINB0                  // SX1_T1
			#define SX1_T1_PINREG	PINB
			#define SX1_T1_DDR		DDRB

			#define SX1_D			10                     //SX1 Data write arduino port
			#define SX1_D_PORTPIN	PORTB2			       // SX1_D
			#define SX1_D_PORT		PORTB
			#define SX1_D_DDR		DDRB
		#endif
	#else
		#define SX0_T1			4                    	//SX0 Data read arduino port
		#define SX0_T1_PORTPIN 	PIND4                  // SX0_T1
		#define SX0_T1_PINREG	PIND
		#define SX0_T1_DDR		DDRD	
	
		#define SX0_D_HIGH			7                      // SX0 Data high write arduino port
		#define SX0_D_HIGH_PORTPIN 	PORTD7 			   // SX0_D_HIGH
		#define SX0_D_HIGH_PORT	P	ORTD
		#define SX0_D_HIGH_DDR		DDRD
		
		#define SX0_D_LOW		3                     // SX0 Data low write arduino port
		#define SX0_D_LOW_PORTPIN 	PORTD3			       // SX0_D_LOW
		#define SX0_D_LOW_PORT		PORTD
		#define SX0_D_LOW_PINREG	PORTD
		#define SX0_D_LOW_DDR		DDRD
		
		#ifdef _sxbus1
			#define SX1_T1				8                  		// SX1 Data read arduino port
			#define SX1_T1_PORTPIN 		PINB0                  // SX1_T1
			#define SX1_T1_PINREG		PINB
			#define SX1_T1_DDR			DDRB	
		
			#define SX1_D_HIGH			10                     // SX1 Data high write arduino port
			#define SX1_D_HIGH_PORTPIN 	PORTB2 			   // SX1_D_HIGH
			#define SX1_D_HIGH_PORT		PORTB
			#define SX1_D_HIGH_DDR		DDRB
		
			#define SX1_D_LOW			9                    // SX1 Data low write arduino port
			#define SX1_D_LOW_PORTPIN 	PORTB1			  // SX1_D_LOW
			#define SX1_D_LOW_PORT		PORTB
			#define SX1_D_LOW_DDR		DDRB
		#endif
		
	#endif                                         // _use4pin
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
	#define SX_T0			21                    // Clock
	#define SX_T0_PORTPIN 	PIND0 				 // SX_T0
	#define SX_T0_PINREG	PIND
	#define SX_T0_DDR		DDRD
	#ifndef _use4pin
		#define SX0_T1			30                    	//SX0 Data read arduino port
		#define SX0_T1_PORTPIN 	PINC7                  // SX0_T1
		#define SX0_T1_PINREG	PINC
		#define SX0_T1_DDR		DDRC	

		#define SX0_D			32                     //SX0 Data write arduino port
		#define SX0_D_PORTPIN	PORTC5			       // SX0_D
		#define SX0_D_PORT		PORTC
		#define SX0_D_DDR		DDRC

		#ifdef _sxbus1
			#define SX1_T1			31                  		// SX1 Data read arduino port
			#define SX1_T1_PORTPIN 	PINC6                  // SX1_T1
			#define SX1_T1_PINREG	PINC
			#define SX1_T1_DDR		DDRC

			#define SX1_D			33                     //SX1 Data write arduino port
			#define SX1_D_PORTPIN	PORTC4			       // SX1_D
			#define SX1_D_PORT		PORTC
			#define SX1_D_DDR		DDRC
		#endif
	#else
		#define SX0_T1			30                    	//SX0 Data read arduino port
		#define SX0_T1_PORTPIN 	PINC7                  // SX0_T1
		#define SX0_T1_PINREG	PINC
		#define SX0_T1_DDR		DDRC	
	
		#define SX0_D_LOW			32                     // SX0 Data low write arduino port
		#define SX0_D_LOW_PORTPIN 	PORTC5			       // SX0_D_LOW
		#define SX0_D_LOW_PORT		PORTC
		#define SX0_D_LOW_DDR		DDRC
		
		#define SX0_D_HIGH			34                      // SX0 Data high write arduino port
		#define SX0_D_HIGH_PORTPIN 	PORTC3 			   // SX0_D_HIGH
		#define SX0_D_HIGH_PORT		PORTC
		#define SX0_D_HIGH_DDR		DDRC
		
		#ifdef _sxbus1	
			#define SX1_T1			31                  		// SX1 Data read arduino port
			#define SX1_T1_PORTPIN 	PINC6                  // SX1_T1
			#define SX1_T1_PINREG	PINC
			#define SX1_T1_DDR		DDRC	
			
			#define SX1_D_LOW			33                   // SX1 Data low write arduino port
			#define SX1_D_LOW_PORTPIN 	PORTC4			  // SX1_D_LOW
			#define SX1_D_LOW_PORT		PORTC
			#define SX1_D_LOW_PINREG	PORTC
			#define SX1_D_LOW_DDR		DDRC
				
			#define SX1_D_HIGH			35                     // SX1 Data high write arduino port
			#define SX1_D_HIGH_PORTPIN 	PORTC2 			   // SX1_D_HIGH
			#define SX1_D_HIGH_PORT		PORTC
			#define SX1_D_HIGH_DDR	D	DRC
		#endif
		
	#endif                                        // _use4pin
#else
  #error "--> CPU settings not supported in SX2Arduino.h <--"
#endif

#define TRI_STATE 3
//defines for status flag, stored in _sx_syncFlag
#define SX1SYNC 0								//Flag Bit for SX1 Sync
#define SX2SYNC 1								//Flag Bit for SX2 Sync
#define SXPWR 2									//Flag Bit for SX PWR 
#define SX0ISRWRITEFLAG 3						//Flag Bit for SX0 Writing in ISR
#ifdef _sxbus1
	#define SX1ISRWRITEFLAG 4						//Flag Bit for SX1 Writing in ISR		
#endif					
//defines for SX2 Write flags, stored in high nibble of _sxbussnd[frame].pr
#define SX2WRITEPR 	4							//Flag Bit for SX2 Write, stored in high nibble of _sxbussnd[frame].pr
#define SX2WRITEADR	5							//Flag Bit for SX2 Write, stored in high nibble of _sxbussnd[frame].pr
#define SX2WRITEFST	6							//Flag Bit for SX2 Write, stored in high nibble of _sxbussnd[frame].pr
#define SX2WRITEFKT	7							//Flag Bit for SX2 Write, stored in high nibble of _sxbussnd[frame].pr
// defines for state machine
#define DATA	0                              // For performance DATA first
#define SYNC	1                              // (Gives fastest code)
#define PWR     2
#define ADDR    3
#define SX2Pr	4								
#define SX2Adr	5
#define SX2Fst	6
#define SX2Fkt	7
// defines for Selectrix constants
#define SX_STOP         3                      	// 3 "0" for SYNC
#define SX2_PRCOUNT		6 						// 6 bytes for SX2 Praämbel	
#define SX_DATACOUNT    7                      	// 7 dataframes in 1 SYNC Channel
#define SX_SEPLEN       3                      	// 3 bit in a separated part
#define SX_BYTELEN     12                     	// 12 bits for one byte
#define SX2_FKTCHANELS	  2

#ifdef _sxbus1
	#define SX_ADDRESS_NUMBER 112*2                 	// number SX channels
	#define SX2_FRAMES		  16*2					//number of SX2 Frames
#else 
	#define SX_ADDRESS_NUMBER 112                  	// number SX channels
	#define SX2_FRAMES		  16					//number of SX2 Frames
#endif
// defines for SX Bus Writing
#define NO_WRITE 256                           	// No data to write
#define WRITE	256								//Data to write


struct _sx2_values
{
	uint8_t pr;				//to store SX2 Präambel
	uint16_t adr;						//to store SX2 Adr
	uint8_t li;							//to store SX2 Light
	uint8_t fst;						//to store SX2 Speed 
	uint16_t fkt;						//to store SX2 Funktionen
};
class SX2Arduino 
{	
public:
	SX2Arduino();
	uint8_t init(HardwareSerial* hwPrint);
	uint8_t init(void);	
	int get(uint8_t);
	uint8_t set(uint8_t, uint8_t);
	uint8_t isSet(uint8_t);
	uint8_t isSX2Set(uint8_t);
	uint8_t setSX2Li(uint8_t , bool);
	uint8_t setSX2Dccfst(uint8_t, bool );
	uint8_t setSX2Speed(uint8_t , uint8_t, bool);
	uint8_t setSX2Fkt(uint8_t , uint16_t);
    bool getPWR(void);
	void setPWR(bool);
	void isr(void);
	uint8_t inSync(void);
	uint8_t inSX2Sync(void);
	uint8_t checkSX2Frame (uint8_t);
	uint8_t searchSX2EmptyFrame (void);
	uint8_t returnSX2Format(uint8_t);
	uint16_t returnSX2AdrLiDcc(uint8_t);
	uint16_t returnSX2Adr(uint8_t);
	uint8_t returnSX2Li(uint8_t);
	uint8_t returnSX2Dccfst(uint8_t);
	uint8_t returnSX2Fst(uint8_t);
	uint8_t returnSX2Dir(uint8_t);
	uint16_t returnSX2Fkt(uint8_t);
	uint8_t regLoco (uint16_t , uint8_t );
	uint8_t regLoco (uint16_t , uint8_t ,HardwareSerial* hwPrint);
	uint8_t checkLoco(uint16_t,uint8_t);
	uint8_t holdLoco (uint8_t,uint8_t);
	uint8_t regPOM(uint8_t,uint8_t,uint8_t,  uint8_t , uint8_t , uint8_t);
	uint16_t calcSX2Adr(uint16_t);
	uint16_t calcSX2Par(uint16_t); 		

private:
	void initVar();
	uint8_t setSX2Pr(uint8_t, uint8_t) ;
	uint8_t setSX2Adr(uint8_t, uint16_t,uint8_t); 
	volatile uint8_t _sx_busNr;						//to store Bus nummer
	volatile uint8_t _sx_numFrame;         			// number frame
	volatile uint8_t _sx_dataFrameCount;            // frame counting
	volatile uint8_t _sx_state;
	volatile uint8_t _sx_sepCount;                  	// bit counting (seperator)
	volatile uint8_t _sx_bitCount;                 	// bit counting (byte)
	volatile uint8_t _sx_syncCount;					//bit counting for SYNC
	volatile uint8_t _sx_index;                     // current index in the array
	volatile uint8_t _sx_syncFlag;         // to store SYNC and WRITE flags

	volatile uint8_t _sx0_read_data;                 // read data
	volatile uint16_t _sx0_read_data_16;				//read data for sx2
	volatile uint8_t _sx0_write_data;  			   // data to write
	volatile uint16_t _sx0_write_data_16;  			   // data to write sx2
	volatile uint8_t _sx0_bit;                       // value data bit (T1)
	//volatile uint8_t _sx0_sd;                        // value data bit (D)

	#ifdef _sxbus1
	volatile uint8_t _sx1_read_data;                 // read data
	volatile uint16_t _sx1_read_data_16;				//read data for sx2
	volatile uint8_t _sx1_write_data;  			   // data to write
	volatile uint16_t _sx1_write_data_16;  			   // data to write sx2
	volatile uint8_t _sx1_bit;                       // value data bit (T1)
	//volatile uint8_t _sx1_sd;                       // value data bit (D)
	#endif
	//Arrays for SX DATA
	volatile uint16_t _sxbus[SX_ADDRESS_NUMBER]; 				// to store the received SX data
	volatile struct _sx2_values _sx2bus[SX2_FRAMES]; 		//to store the recived SX2 data Frames, shared by all intnstances
	volatile uint8_t _sx_newPWR;                   				// command POWER on track
	//Array for writing SX DATA
	//volatile uint16_t _sxbussnd[SX_ADDRESS_NUMBER]; 				// to store the SX1 data to send
	//volatile struct _sx2_values _sx2bussnd[SX2_FRAMES]; //to store the SX2 data frame to send, shared by all intnstances
	//shared by all Functions

	
	/* SX Timing
	 1   Bit             50 us
	 1   Kanal          600 us (= 12 Bit)
	 1   Grundrahmen    ca. 4,8 ms
	 1   Gesamtrahmen   ca.  80 ms (= 16 Grundrahmen)  
	 0  0  0  1  S   1  A3  A2  1  A1  A0  1 == sync frame of 12 bits
	 */
};

#endif /* SX2Arduino_H_ */
