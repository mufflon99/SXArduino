/*
 * SXArduino.h
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

#ifndef SXArduino_H_
#define SXArduino_H_

#include <Arduino.h>

// define arduino pins, ports and bits
// depends on the hardware used.
// #define _use4pin                              // Switch for 3 or 4 portpin interface (set to 4 portpin)

#define SX_T0			3                      // Clock
#define SX_T0_PORTPIN 	PIND3  				   // SX_T0
#define SX_T0_PINREG	PIND
#define SX_T0_DDR		DDRD

#define SX_T1			5                      // Data read
#define SX_T1_PORTPIN 	PIND5                  // SX_T1
#define SX_T1_PINREG	PIND
#define SX_T1_DDR		DDRD

#ifndef _use4pin

#define SX_D			6                      // Data write
#define SX_D_PORTPIN	PORTD6			       // SX_D
#define SX_D_PORT		PORTD
#define SX_D_DDR		DDRD

#else

#define SX_D_HIGH		6                      // Data high write
#define SX_D_HIGH_PORTPIN PORTD6  			   // SX_D_HIGH
#define SX_D_HIGH_PORT	PORTD
#define SX_D_HIGH_DDR	DDRD

#define SX_D_LOW		7                      // Data low write
#define SX_D_LOW_PORTPIN PORTD7			       // SX_D_LOW
#define SX_D_LOW_PORT	PORTD
#define SX_D_LOW_PINREG	PORTD
#define SX_D_LOW_DDR	DDRD

#endif                                         // _use4pin

#define TRI_STATE 3

// defines for state machine
#define DATA	0                              // For performance DATA first
#define SYNC	1                              // (Gives fastest code)
#define PWR     2
#define ADDR    3

// defines for Selectrix constants
#define SX_STOP         3                      // 3 "0" bits achter elkaar
#define SX_DATACOUNT    7                      // 7 dataframes in 1 SYNC Channel
#define SX_SEPLEN       3                      // 3 bit in a separated part
#define SX_BYTELEN     12                      // 12 bits for one byte

#define SX_ADDRESS_NUMBER 112                  // number SX channels

#define NO_WRITE 256                           // No data to write

class SXArduino {
public:
	SXArduino();
	void init(void);	
	int get(uint8_t);
	uint8_t set(uint8_t, uint8_t);
	uint8_t isSet(uint8_t);
    uint8_t getPWR(void);
	void setPWR(uint8_t);
	void isr(void);
	uint8_t inSync(void);
	
private:
	void initVar();
	uint8_t calcIndex(uint8_t adr);

	uint8_t _sx_numFrame;                  // number frame
	uint8_t _sx_dataFrameCount;            // frame counting
	uint8_t _sx_state;
	uint8_t _sx_sepCount;                  // bit counting (seperator)
	uint8_t _sx_byteCount;                 // bit counting (byte)
	
	uint8_t _sx_PWR;                       // current state of POWER on track
	uint8_t _sx_newPWR;                    // command POWER on track

	uint8_t _sx_read_data;                 // read data
    uint8_t _sx_write_data;  			   // data to write
	uint8_t _sx_index;                     // current index in the array
	uint8_t _sx_writing;    			   // active during the actual writing
	
	uint8_t _sx_bit;                       // value data bit (T1)
	uint8_t _sx_sd;                        // value data bit (D)
	uint8_t _sx_sync;                      // set if Frame 0 is processed
	
	uint8_t _sxbusrcev[SX_ADDRESS_NUMBER]; // to store the received SX data
	uint16_t _sxbussnd[SX_ADDRESS_NUMBER]; // to store the SX data to send

	/* SX Timing
	 1   Bit             50 us
	 1   Kanal          600 us (= 12 Bit)
	 1   Grundrahmen    ca. 4,8 ms
	 1   Gesamtrahmen   ca.  80 ms (= 16 Grundrahmen)  
	 0  0  0  1  S   1  A3  A2  1  A1  A0  1 == sync frame of 12 bits
	 */
};

#endif /* SXArduino_H_ */
