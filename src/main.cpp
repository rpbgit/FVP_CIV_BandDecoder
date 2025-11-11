#include <Arduino.h>

// a debug widget to stop/hold further execution of the code until we get sent something from the host
// it will also print out the line of source code it is being executed in.
void _stall(){  // DO NOT USE THIS FUNCTION DIRECTLY, USE THE stall() MACRO INSTEAD`
    while(Serial.read() != -1){;}
    while(!Serial.available()){;}   // spin here waiting for something from the pc to be sent.
    delay(100);
    while(Serial.read() != -1){;}  // gobble up any chars in the input buffer, then go on
}
    
// add this macro to any line of code you want to stop at for debugging, hit any key to continue execution
// example:  int x = 5; stall();
#define stall() Serial.print("Stall @ line #");Serial.println(__LINE__);_stall();
    
/*
7-Oct-2025 Wa9fvP   v1.1  removed unused #defines.  Deleted commeted stuff at the battom . 
14-Oct-2025 ZV      v1.1.1 added  version info and printout, allow 7300 and 705 default addresses.
15-Oct-2025 ZV      v1.1.2 fix BCD check in state machine to properly validate BCD upper nibble. SM clarified comments.
16-Oct-2025 ZV      v1.2 added opportunistic resync on preamble byte mid-frame.
11-Nov-2025 ZV      v2.0 added CI-V frequency query sender to prompt radio to send freq if no messages seen for a while.
*/
// REMEMBER TO UPDATE VERSION NUMBER !!!! 
#define VERSION     "2.0" // software version

//=====[ Settings ]===========================================================================================
#define CIVBAUD 9600  // [baud] Serial port CIV in/out baudrate  IC-705
//#define CIVBAUD        19200  // [baud] Serial port CIV in/out baudrate

constexpr uint8_t CIV_ADDR_705 = 0xA4;  // CIV input HEX Icom address (0x is prefix) 0xA4 = IC-705
constexpr uint8_t CIV_ADDR_7300 = 0x94;  // CIV input HEX Icom address (0x is prefix) 0x94 = IC-705
constexpr uint8_t CIV_PREAMBLE_BYTE =0xFE;  // CIV preamble byte - each frame starts with 0xFE 0xFE
constexpr uint8_t CIV_FRAME_END_BYTE = 0xFD;  // CIV frame end byte - each frame ends with 0xFD
constexpr uint8_t CIV_CONTROLLER_ADDR = 0xE0; // Arduino/Nano Every default "controller" address
constexpr uint8_t CIV_QUERY_FREQ_CMD = 0x03; // CIV command to query frequency
constexpr unsigned long MESSAGE_TIMEOUT_MS = 30 * 1000UL;     // 30s without a complete message triggers a query
constexpr unsigned long INITIAL_QUERY_DELAY_MS = 1 * 1000UL;  // after boot, if we've never decoded anything, send a query after 1second
#define CIV_ADDRESSES_MATCH(b) (((b)==CIV_ADDR_705 || (b)== CIV_ADDR_7300)) // macro to check if address is valid

// a small inline function to improve type safety and avoid double evaluation while keeping the same semantics as a macro.
static inline bool CIV_IS_VALID_BCD_u8(uint8_t b) {
    return ((b >> 4) <= 9) && ((b & 0x0F) <= 9);
}

//=====[ End Settings ]========================================================================================
// the icom CIV state machine function prototype
bool icomSM2(uint8_t b, unsigned long * freq);  // prototype for fwd ref
// forward declaration for CI-V frequency query sender so it can be used from loop()
void civSendFreqQuery();
    
void setup() {

    Serial.begin(115200);    // Serial monitor
    delay(100);
    Serial1.begin(CIVBAUD);   // Serial1 for CIV D0-D1
    Serial1.setTimeout(10);
    delay(2000); // allow enough time on VSCode/PIO for the serial monitor to connect/startup after build and upload.

    //=====[ Set pin mode ]===================================================================================
    // using direct port manipulation for fast operation and atomic (all bits in byte)
    //PORTF.DIR = 0x0F; // A3-0 output, A7-4 input, D17, D16, D15, D14 are our BAND outputs, 1 = output
    VPORTD.DIR = 0x0F; // A3-0 output, A7-4 input, D17, D16, D15, D14 are our BAND outputs, 1 = output

    // PORTx.OUT → write all pins on a port
    // PORTx.OUTSET → set selected pins HIGH
    // PORTx.OUTCLR → clear selected pins LOW
    // PORTx.OUTTGL → toggle selected pins   

    Serial.print(F("\nCIV Band decoder started"));
    Serial.print(F(" Version: ")); Serial.print(VERSION);
    Serial.print(F(" CIV Baud: ")); Serial.println(CIVBAUD);
    Serial.print(F("Compiled on: ")); Serial.print(__DATE__); Serial.print(F(" ")); Serial.println(__TIME__);

}
    
void loop() {
    byte incomingCIVByte = 0;
    unsigned long freq = 0;
    int BAND = -1;  // band number 0-13, -1 is bogus band
    static int msgCount = 0;
    // Track timing related to complete CI-V message decodes and query retries
    static unsigned long lastCompleteMessageMillis = 0;   // last time a full/valid CI-V message was decoded
    static unsigned long lastQueryMillis = 0;             // last time we actively sent a CI-V frequency query
    
    // If we have not decoded a complete message recently, send a CI-V frequency query to prompt a response.
    // Behavior:
    //  - On startup (no decodes yet), send one query after INITIAL_QUERY_DELAY_MS to kick things off.
    //  - During normal operation, if no complete decode has happened in the last MESSAGE_TIMEOUT_MS,
    //    send a query at most once per MESSAGE_TIMEOUT_MS (throttled by lastQueryMillis) to avoid flooding.
    unsigned long now = millis();
    if (lastCompleteMessageMillis == 0) {
        // Never decoded a full message yet; opportunistically query after a short delay
        if (now - lastQueryMillis >= INITIAL_QUERY_DELAY_MS) {
            civSendFreqQuery();
            lastQueryMillis = now;
            Serial.println(F("Sent initial CI-V frequency query (no complete messages yet)"));
        }
    } else {
        // Have decoded before; check inactivity window and throttle queries
        if ((now - lastCompleteMessageMillis) >= MESSAGE_TIMEOUT_MS &&
            (now - lastQueryMillis) >= MESSAGE_TIMEOUT_MS) {
            civSendFreqQuery();
            lastQueryMillis = now;
            Serial.println(F("Sent CI-V frequency query due to 30s inactivity"));
        }
    }
 //while(1){
 //    Serial.print("message count:");
 //    Serial.print(msgCount, HEX);
 //    Serial.println();
 //    delay(1000);
 //}
    if (Serial1.available() > 0) {
        incomingCIVByte = Serial1.read();
Serial.print(incomingCIVByte, HEX); Serial.print(" ");
        // feed each byte into the state machine
        if (icomSM2(incomingCIVByte, &freq)) {  // if we were successful in decoding a full message with valid freq.
            // valid frequency received from Icom CIV
            // Update the timestamp for the last time a complete/valid message was decoded
            lastCompleteMessageMillis = now;
            msgCount++;
            //printf("Msg # %d, Freq: %ld Hz, ", msgCount, freq);
            Serial.print(" --> msg #");Serial.print(msgCount, DEC);
            Serial.print(" Freq ");Serial.print(freq, DEC);
            //Serial.println("") ;   
            
            //=====[ Frequency (Hz) to Band rules ]======================================
            //       you can expand rules up to 14 Bands
            if      (freq >= 5300000 && freq <= 5450000) { BAND = 0; }      // 60m  ???  5300000 to 5450000 should work?
            else if (freq >= 1800000 && freq <= 2000000) { BAND = 1; }      //  160m
            else if (freq >= 3500000 && freq <= 4000000) { BAND = 2; }      //  80m
            else if (freq >= 7000000 && freq <= 7300000) { BAND = 3; }      //  40m
            else if (freq >= 10100000 && freq <= 10150000) { BAND = 4; }    //  30m
            else if (freq >= 14000000 && freq <= 14350000) { BAND = 5; }    //  20m
            else if (freq >= 18068000 && freq <= 18168000) { BAND = 6; }    //  17m
            else if (freq >= 21000000 && freq <= 21450000) { BAND = 7; }    //  15m
            else if (freq >= 24890000 && freq <= 24990000) { BAND = 8; }    //  12m
            else if (freq >= 28000000 && freq <= 29700000) { BAND = 9; }    //  10m
            else if (freq >= 50000000 && freq <= 52000000) { BAND = 10; }   //   6m
            else if (freq >= 144000000 && freq <= 148000000) { BAND = 11; } //  2m
            else if (freq >= 420000000 && freq <= 450000000) { BAND = 12; } // 432
            else { BAND = -1; }                                             // bogus band
            //===========================================================================
            //printf("Band number: %d\n", BAND);
            Serial.print(" BAND "); Serial.print(BAND, DEC);
            Serial.println("") ;
            // map band number into this lookup table to get the binary bits to output to the BAND GPIO bits.
            static byte bandtoBandBitMapTable[] = {
                0b0000,         // 60m kinda an anomaly
                0b0001,         // 160m
                0b0010,         // 80m  
                0b0011,         // 40m
                0b0100,         // 30m
                0b0101,         // 20m
                0b0110,         // 17m
                0b0111,         // 15m
                0b1000,         // 12m
                0b1001,         // 10m
                0b1010,		    // 6m
                0b1011,         // 2m
                0b1100          // 432
            };
            //  --------------------------------------------------------------------
                //  Set the output bits to the band data bits
                //  Assuming using PORTF for the band outputs, and the band bits are on the low nibble
                //  Make sure the PORTF pins are set as outputs in setup()
                //  Example: if BAND = 5 (20m), then bandtoBandBitMapTable[5] = 0b0101
                //           This will set PORTF pins as:  F0=1, F1=0, F2=1, F3=0 (20m selected)
                //  Note: This operation preserves the upper nibble of PORTF and only modifies the lower nibble
                //  Example operation:
                //      Current PORTF.out = 0b10110000 (upper nibble is preserved)
                //      BAND = 5 (20m) -> bandtoBandBitMapTable[5] = 0b0101
                //      New PORTF.out = (0b10110000 & 0b11110000) | 0b0101
                //                    = 0b10110101
                //  --------------------------------------------------------------------
            // Safety check to ensure BAND is within valid range
            // only do this if BAND is valid
            if (BAND >= 0 && BAND < (int)(sizeof(bandtoBandBitMapTable))) {
                //PORTF.OUT = (PORTF.OUT & ~0x0F) | (bandtoBandBitMapTable[BAND] & 0x0F); // set BAND bits, preserve upper nibble
                VPORTD.OUT = (VPORTD.OUT & ~0x0F) | (bandtoBandBitMapTable[BAND] & 0x0F); // set BAND bits, preserve upper nibble
            }
            BAND = -1;  //
        } // end if icomSM2
    // else no data available on Serial1, keep feeding the pig
    }
    // go do other stuff here if needed.
}

bool icomSM2(byte b, unsigned long * freq) {      // state machine

    // This filter solves read from 0x00 0x05 0x03 commands and 00 E0 F1 adress used by software
    static byte rcvBuff[32] = {0}; // buffer to keep the incoming freq data in until a full message is received
    static int state = 1;  // state machine
    
    /* Opportunistic resync on preamble byte anywhere mid-frame.
    // encountering 0xFE while parsing TO/FROM/command/data likely indicates we lost
    // sync (dropped byte, noise, or concatenated frames).

    This implements a mid-stream recovery mechanism for the CI-V parser. In the CI-V protocol, frames start 
    with a two-byte preamble 0xFE 0xFE. Seeing a 0xFE while already past the preamble parsing states (state > 2) 
    strongly suggests the parser lost synchronization—perhaps due to a dropped byte, line noise, or two frames 
    being back-to-back without a gap.  When that happens, the code opportunistically resynchronizes by setting 
    state = 2 (the state that expects the second 0xFE), priming rcvBuff[0] with 0xFE as the first preamble byte, 
    and returning false to signal that no complete message was produced from this byte. This approach is conservative: 
    it won’t advance unless the next byte is another 0xFE, minimizing false resyncs. It improves robustness without 
    requiring a full buffer reset, since subsequent states will overwrite the needed positions anyway.
    */
    if (b == CIV_PREAMBLE_BYTE && state > 2) {
        state = 2;
        rcvBuff[0] = CIV_PREAMBLE_BYTE;
        return false;
    }

    switch (state) {
    // PREAMBLE 0xFE 0xFE
    case 1: if (b == CIV_PREAMBLE_BYTE) { state = 2; rcvBuff[0] = b; }; break;
    case 2: if (b == CIV_PREAMBLE_BYTE) { state = 3; rcvBuff[1] = b; } else { state = 1; }; break;

    // TO ADDRESS BYTE  (accept only expected destinations)
    // addresses that use different software 00-trx, e0-pc-ale, winlinkRMS, f1-winlink trimode
    case 3: if (b == 0x00 || b == 0xE0 || b == 0xF1) { state = 4; rcvBuff[2] = b; }
          else if ( CIV_ADDRESSES_MATCH(b) ) { state = 6; rcvBuff[2] = b; }
          else { state = 1; }; break;      
          
    // FROM ADDRESS BYTE     
    case 4: if ( CIV_ADDRESSES_MATCH(b) ) { state = 5; rcvBuff[3] = b; } else { state = 1; }; break;
    
    // Command filtering
    case 5: if (b == 0x00 || b == 0x03) { state = 8; rcvBuff[4] = b; } else { state = 1; }; break;
    case 6: if (b == 0x00 || b == 0xE0 || b == 0xF1) { state = 7; rcvBuff[3] = b; } else { state = 1; }; break;
    case 7: if (b == 0x00 || b == 0x05) { state = 8; rcvBuff[4] = b; }  else { state = 1; }; break;

    // FREQUENCY BYTES
    // next five bytes are frequency data, must ensure only valid packed BCD data (each nibble <= 0-9), or toss the frame
    // this is the most efficient way to check for valid BCD i could think of
    case 8:  if (CIV_IS_VALID_BCD_u8(b)) { state = 9;  rcvBuff[5] = b; } else { state = 1; }; break;
    case 9:  if (CIV_IS_VALID_BCD_u8(b)) { state = 10; rcvBuff[6] = b; } else { state = 1; }; break;
    case 10: if (CIV_IS_VALID_BCD_u8(b)) { state = 11; rcvBuff[7] = b; } else { state = 1; }; break;
    case 11: if (CIV_IS_VALID_BCD_u8(b)) { state = 12; rcvBuff[8] = b; } else { state = 1; }; break;
    case 12: if (CIV_IS_VALID_BCD_u8(b)) { state = 13; rcvBuff[9] = b; } else { state = 1; }; break;

    // FRAME END BYTE       
    case 13: if (b == CIV_FRAME_END_BYTE ) { state = 1; rcvBuff[10] = b; } 
        else { state = 1; rcvBuff[10] = 0; }; 
        break; // 0xFD is frame end byte
    
    default: state = 1; break;
    } // end switch

	// Check if we have received a full message (indicated by 0xFD at the end of message)
    // If so, decode the frequency from the received BCD bytes
    if (rcvBuff[10] == CIV_FRAME_END_BYTE) { // full message received
        *freq = 0;
        // Decode the frequency from the received BCD bytes, bytes 5 to 9 in rcvBuff in reverse order (LSB first)
        for (int j = 9; j >= 5; j--) {
            // Each byte contains two BCD digits: high nibble and low nibble, read from right to left - see ICOM manual
            uint8_t high = (uint8_t)(rcvBuff[j] >> 4);
            uint8_t low  = (uint8_t)(rcvBuff[j] & 0x0F);
            *freq = *freq * 100UL + (unsigned long)high * 10UL + (unsigned long)low;
        }
        memset(rcvBuff, 0, sizeof(rcvBuff));
        return true;
    }else 
		return false; // valid Message not complete yet, keep feeding the pig until a full message is received
}


// Sends a CI-V frequency query to an Icom IC-7300 over Serial1.
// Uses correct CI-V preamble, controller address, radio address, and terminator.
void civSendFreqQuery()
{
    // Properly framed CI-V message: FE FE E0 94 03 FD
    static const uint8_t msg[] = {
        CIV_PREAMBLE_BYTE, CIV_PREAMBLE_BYTE,
        CIV_ADDR_7300,  // to address
        CIV_CONTROLLER_ADDR, // from address
        CIV_QUERY_FREQ_CMD, //0x00,   no subcmd for frequency query
        CIV_FRAME_END_BYTE
    };

    Serial1.write(msg, sizeof(msg));
}
