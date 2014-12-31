// LxTx.h
//
// LightwaveRF 434MHz tx interface for Arduino
// 
// Author: Bob Tidey (robert@tideys.net)
//Choose environment to compile for. Only one should be defined
//For SparkCore the SparkIntervalTimer Library code needs to be present
//For Due the DueTimer library code needs to be present
//#define SPARK_CORE 1
//#define DUE 1
#define AVR328 1

//Choose whether to include EEPROM support, comment or set to 0 to disable, 1 use with library support, 2 use with native support
#define EEPROM_EN 1

//Include basic library header and set default TX pin
#ifdef SPARK_CORE
#include "application.h"
#define TX_PIN_DEFAULT D3
#elif DUE
#include <Arduino.h>
#define TX_PIN_DEFAULT 3
#else
#include <Arduino.h>
#define TX_PIN_DEFAULT 3
#endif

//Include EEPROM if required to include storing device paramters in EEPROM
#if EEPROM_EN == 1
#include <EEPROM.h>
#endif
//define default EEPROMaddr to location to store message addr
#define EEPROM_ADDR_DEFAULT 0

class LwTx {

public:
    //Sets up basic parameters must be called at least once
    void setup(int pin, byte repeats, byte invert, int period);

    //Allows changing basic tick counts from their defaults
    void setTickCounts( byte lowCount, byte highCount, byte trailCount, byte gapCount);

    //Allws multiplying the gap period for creating very large gaps
    void setGapMultiplier(byte gapMultiplier);

    // determines whether incoming data or should be translated from nibble data
    void setTranslate(boolean txtranslate);

    //Checks whether tx is free to accept a new message
    boolean free(void);

    //Basic send of new 10 char message, not normally needed if setaddr and cmd are used.
    void send(byte* msg);

    //Sets up 5 char address which will be used to form messages for cmd
    void setAddr(byte* addr);

    //Send Command
    void cmd(byte command, byte parameter, byte room, byte device);

    //Set base address for EEPROM storage
    void setEEPROMaddr(int addr);

    //Genralised timer routines go here
    //Sets up timer and the callback to the interrupt service routine
    void timerSetup(void (LwTx::*isrCallback)(), int period);

    //Allows changing basic tick counts from their defaults
    void timerStart(void);

    //Allws multiplying the gap period for creating very large gaps
    void timerStop(void);

    void isrTXtimer(void);

private:
    int EEPROMaddr = EEPROM_ADDR_DEFAULT;

    byte tx_nibble[16] = {0xF6,0xEE,0xED,0xEB,0xDE,0xDD,0xDB,0xBE,0xBD,0xBB,0xB7,0x7E,0x7D,0x7B,0x77,0x6F};

    #ifdef TX_PIN_DEFAULT
    int tx_pin = TX_PIN_DEFAULT;
    #else
    int tx_pin = 3;
    #endif

    static const byte tx_msglen = 10; // the expected length of the message

    //Transmit mode constants and variables
    byte tx_repeats = 12; // Number of repeats of message sent
    byte txon = 1;
    byte txoff = 0;
    volatile boolean tx_msg_active = false; //set true to activate message sending
    boolean tx_translate = true; // Set false to send raw data

    byte tx_buf[tx_msglen]; // the message buffer during reception
    byte tx_repeat = 0; //counter for repeats
    volatile byte tx_state = 0;
    volatile byte tx_toggle_count = 3;
    uint16_t tx_gap_repeat = 0;  //unsigned int

    // These set the pulse durations in ticks
    volatile byte tx_low_count = 7; // total number of ticks in a low (980 uSec)
    volatile byte tx_high_count = 4; // total number of ticks in a high (560 uSec)
    volatile byte tx_trail_count = 2; //tick count to set line low (280 uSec)
    // Use with low repeat counts
    byte tx_gap_count = 72; // Inter-message gap count (10.8 msec)
    //Gap multiplier byte is used to multiply gap if longer periods are needed for experimentation
    //If gap is 255 (35msec) then this to give a max of 9 seconds
    //Used with low repeat counts to find if device times out
    byte tx_gap_multiplier = 0; //Gap extension byte 

    static const byte tx_state_idle = 0;
    static const byte tx_state_msgstart = 1;
    static const byte tx_state_bytestart = 2;
    static const byte tx_state_sendbyte = 3;
    static const byte tx_state_msgend = 4;
    static const byte tx_state_gapstart = 5;
    static const byte tx_state_gapend = 6;

    volatile byte tx_bit_mask = 0; // bit mask in current byte
    volatile byte tx_num_bytes = 0; // number of bytes sent 

};
