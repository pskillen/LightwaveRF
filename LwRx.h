// LwRx.h
//
// LightwaveRF 434MHz receiver for Arduino
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

//Include basic library header and set RX pin logic
#ifdef SPARK_CORE
#include "application.h"
#elif DUE
#include <Arduino.h>
#else
#include <Arduino.h>
// define which pins can be used for rx interrupts, leave undefined for all pins no translation
//328
#define PIN_NUMBERS 2,3 
//LEONARDO
//#define PIN_NUMBERS 3,2,0,1,7
//MEGA2560
//#define PIN_NUMBERS 2,3,21,20,19,18
#endif

//Include EEPROM if required to include storing device paramters in EEPROM
#if EEPROM_EN == 1
#include <../EEPROM/EEPROM.h>
#endif
//define default EEPROMaddr to location to store message addr
#define EEPROM_ADDR_DEFAULT 0


#define rx_stat_high_ave 0
#define rx_stat_high_max 1
#define rx_stat_high_min 2
#define rx_stat_low0_ave 3
#define rx_stat_low0_max 4
#define rx_stat_low0_min 5
#define rx_stat_low1_ave 6
#define rx_stat_low1_max 7
#define rx_stat_low1_min 8
#define rx_stat_count 9

static const byte rx_nibble[] = {0xF6,0xEE,0xED,0xEB,0xDE,0xDD,0xDB,0xBE,0xBD,0xBB,0xB7,0x7E,0x7D,0x7B,0x77,0x6F};
static const uint16_t lwrx_statsdflt[rx_stat_count] = {5000,0,5000,20000,0,2500,4000,0,500};

//sets maximum number of pairings which can be held
#define rx_maxpairs 10


class LwRx {

public:
    //Setup must be called once, set up pin used to receive data
    void setup(int pin);

    //Set translate to determine whether translating from nibbles to bytes in message
    //Translate off only applies to 10char message returns
    void setTranslate(boolean translate);

    // Check to see whether message available
    boolean message(void);

    //Get a message, len controls format (2 cmd+param, 4 cmd+param+room+device),10 full message
    boolean getMessage(byte* buf, byte len);

    //Setup repeat filter
    void setFilter(byte repeats, byte timeout);

    //Add pair, if no pairing set then all messages are received, returns number of pairs
    byte addPair(byte* pairdata);

    // Get pair data into buffer  for the pairnumber. Returns current paircount
    // Use pairnumber 255 to just get current paircount
    byte getPair(byte* pairdata, byte pairnumber);

    //Make a pair from next message received within timeout 100mSec
    //This call returns immediately whilst message checking continues
    void makePair(byte timeout);

    //Set pair mode controls
    void setPairMode(boolean pairEnforce, boolean pairBaseOnly);

    //Returns time from last packet received in msec
    // Can be used to determine if Rx may be still receiving repeats
    unsigned long packetInterval(void);

    void clearPairing(void);

    //Return stats on pulse timings
    boolean getStats(unsigned int* stats);

    //Enable collection of stats on pulse timings
    void setStatsEnable(boolean rx_stats_enable);

    //Set base address for EEPROM storage
    void setEEPROMaddr(int addr);

    //internal support functions
private:
    static const byte rx_cmd_off     = 0xF6; // raw 0
    static const byte rx_cmd_on      = 0xEE; // raw 1
    static const byte rx_cmd_mood    = 0xED; // raw 2
    static const byte rx_par0_alloff = 0x7D; // param 192-255 all off (12 in msb)
    static const byte rx_dev_15      = 0x6F; // device 15

    int rx_pin = 2;
    int EEPROMaddr = EEPROM_ADDR_DEFAULT;
    static const byte rx_msglen = 10; // expected length of rx message

    //Receive mode constants and variables
    byte rx_msg[rx_msglen]; // raw message received
    byte rx_buf[rx_msglen]; // message buffer during reception

    unsigned long rx_prev; // time of previous interrupt in microseconds

    volatile boolean rx_msgcomplete = false; //set high when message available
    boolean rx_translate = true; // Set false to get raw data

    byte rx_state = 0;
    static const byte rx_state_idle = 0;
    static const byte rx_state_msgstartfound = 1;
    static const byte rx_state_bytestartfound = 2;
    static const byte rx_state_getbyte = 3;

    byte rx_num_bits = 0; // number of bits in the current byte
    byte rx_num_bytes = 0; // number of bytes received 

    //Pairing data
    byte rx_paircount = 0;
    byte rx_pairs[rx_maxpairs][8];
    byte rx_pairtimeout = 0; // 100msec units
    //set false to responds to all messages if no pairs set up
    boolean rx_pairEnforce = false;
    //set false to use Address, Room and Device in pairs, true just the Address part
    boolean rx_pairBaseOnly = false;

    // Repeat filters
    byte rx_repeats = 2; //msg must be repeated at least this number of times
    byte rx_repeatcount = 0;
    byte rx_timeout = 20; //reset repeat window after this in 100mSecs
    unsigned long rx_prevpkttime = 0; //last packet time in milliseconds
    unsigned long rx_pairstarttime = 0; //last msg time in milliseconds

    // Gather stats for pulse widths (ave is x 16)
    uint16_t lwrx_stats[rx_stat_count];
    boolean lwrx_stats_enable = true;

public:
    // this needs to be public, as it must be called from main sketch
    void rx_process_bits(void);

private:
    boolean rx_reportMessage(void);
    int16_t rx_findNibble(byte data);
    void rx_addPairFromMsg(void);
    void rx_pairCommit(void);
    void rx_removePair(byte *buf);
    int16_t rx_checkPairs(byte *buf, boolean allDevices);
    void restoreEEPROMPairing(void);
    int getIntNo(int pin);
};
