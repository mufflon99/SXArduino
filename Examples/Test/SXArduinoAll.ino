/*
 *   Test sketch for SX-bus interface for the ARduino
 *   All addresses are written with a value.
 *   In the background an ISR is running to send those values to the SX-bus.
 *   While sending the bus is read and put in a buffer
 *    
 *   Watch out: Adresses 104 to 111 are used. To avoid this set noCC2000 to false
 */

#include "SXArduino.h"
#include "Arduino.h"

// SX-bus interface
SXArduino SXbus;                 // Get access to the SX-bus
uint8_t valrcv;
uint8_t valsnd;
int waitSX;
int waitLussen;
int waitloop;
int burst;                      // pauze
int adr;
bool noCC2000;

void sxisr(void) {
    // if you want to understand this, see:
    // http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1239522239   
    SXbus.isr();
} 

void setup() {
    // put your setup code here, to run once:
    // Serial communication
    Serial.begin(230400);      // open the serial port

    // SX-bus communication
    SXbus.init();
    // CHANGE on INT1 triggers the interrupt routine sxisr (see above)
    attachInterrupt(1, sxisr, RISING); 

    // Init prog
    waitSX = 0;
    waitLussen = 0;
    valsnd = 0;
    waitloop = 0;
    noCC2000 = true;   // false 0 ... 103, true 0 ... 111
    burst = 5;         // 5 slagen pauze (= 5 * 77 = 385 msec)
}

uint8_t getSXbusval(int adr) {
    return SXbus.get(adr);
}

uint8_t getSXbuspower() {
    return SXbus.getPWR();
}

void setSXbusval(uint8_t adr, uint8_t val) {
    SXbus.set(adr, val);
}

void setSXbuspower(uint8_t onoff) {
    SXbus.setPWR(onoff);
}

void loop() {
    // put your main code here, to run repeatedly:
    if (SXbus.inSync() == 1) {                  //  Iedere 76,8 msec true
        // fill bytes conform SX messages
        for (int j = 0; j < 16; j++) {     // 16 frames
            Serial.print("Data: ");
            for (int i = 0; i < 7; i++) {  //  7 databytes per frame
                adr = ((6 - i) << 4) + (15 - j);
                if ((adr < 104) || (noCC2000)) {       // Exclude the addresses used by the CC2000
                    if (waitLussen == 0) {
                        Serial.print(valsnd);
                        Serial.print("=");
                        setSXbusval(adr, valsnd);
                        valsnd++;
                    }
                    valrcv = getSXbusval(adr);
                    Serial.print(valrcv);
                    if (i < 6) {
                        Serial.print(", ");
                    }
                }
            }
            Serial.println();
      }
        if (burst < waitLussen) {
            waitLussen = 0;
        } else {
            waitLussen++;
        }
        Serial.print("Power: ");           // 7 tekens
        if (getSXbuspower() == 1) {
            Serial.println("On");
        } else {
            Serial.println("Off");         // max 3 tekens
        }

        waitSX++;
        if (waitSX > 5) {   // 5 x 80 ms = 400 ms wachttijd
            waitSX = 0;
            Serial.print("Switch power ");          
            if (getSXbuspower() == 1) {
                setSXbuspower(0);
                Serial.println("off");          
            } else {
                setSXbuspower(1);          
                Serial.println("on");
            }
        }
        waitloop = 0;
    } else {
        if (waitloop == 0) {
            Serial.println("Waiting.... ");                   // 15 tekens (incl CR+LF)
        }
        waitloop++;                                            // max 63 tekens totaal => 32 msec bij 19200 baud
        }
}
