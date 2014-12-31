// LwTx.cpp
//
// LightwaveRF 434MHz tx interface for Arduino
// 
// Author: Bob Tidey (robert@tideys.net)

#include "LwTx.h"


/**
  Set translate mode
**/
void LwTx::setTranslate(boolean txtranslate)
{
    tx_translate = txtranslate;
}

void LwTx::isrTXtimer() {
   //Set low after toggle count interrupts
   tx_toggle_count--;
   if (tx_toggle_count == tx_trail_count) {
      digitalWrite(tx_pin, txoff);
   } else if (tx_toggle_count == 0) {
     tx_toggle_count = tx_high_count; //default high pulse duration
     switch (tx_state) {
       case tx_state_idle:
         if(tx_msg_active) {
           tx_repeat = 0;
           tx_state = tx_state_msgstart;
         }
         break;
       case tx_state_msgstart:
         digitalWrite(tx_pin, txon);
         tx_num_bytes = 0;
         tx_state = tx_state_bytestart;
         break;
       case tx_state_bytestart:
         digitalWrite(tx_pin, txon);
         tx_bit_mask = 0x80;
         tx_state = tx_state_sendbyte;
         break;
       case tx_state_sendbyte:
         if(tx_buf[tx_num_bytes] & tx_bit_mask) {
           digitalWrite(tx_pin, txon);
         } else {
           // toggle count for the 0 pulse
           tx_toggle_count = tx_low_count;
         }
         tx_bit_mask >>=1;
         if(tx_bit_mask == 0) {
           tx_num_bytes++;
           if(tx_num_bytes >= tx_msglen) {
             tx_state = tx_state_msgend;
           } else {
             tx_state = tx_state_bytestart;
           }
         }
         break;
       case tx_state_msgend:
         digitalWrite(tx_pin, txon);
         tx_state = tx_state_gapstart;
         tx_gap_repeat = tx_gap_multiplier;
         break;
       case tx_state_gapstart:
         tx_toggle_count = tx_gap_count;
         if (tx_gap_repeat == 0) {
            tx_state = tx_state_gapend;
         } else {
            tx_gap_repeat--;
         }
         break;
       case tx_state_gapend:
         tx_repeat++;
         if(tx_repeat >= tx_repeats) {
           //disable timer nterrupt
           LwTx::timerStop();
           tx_msg_active = false;
           tx_state = tx_state_idle;
         } else {
            tx_state = tx_state_msgstart;
         }
         break;
     }
   }
}

/**
  Check for send free
**/
boolean LwTx::free() {
  return !tx_msg_active;
}

/**
  Send a LightwaveRF message (10 nibbles in bytes)
**/
void LwTx::send(byte *msg) {
  if (tx_translate) {
    for (byte i=0; i < tx_msglen; i++) {
      tx_buf[i] = tx_nibble[msg[i] & 0xF];
    }
  } else {
    memcpy(tx_buf, msg, tx_msglen);
  }
  LwTx::timerStart();
  tx_msg_active = true;
}

/**
  Set 5 char address for future messages
**/
void LwTx::setAddr(byte *addr) {
   for (byte i=0; i < 5; i++) {
      tx_buf[i+4] = tx_nibble[addr[i] & 0xF];
#if EEPROM_EN
      EEPROM.write(EEPROMaddr + i, tx_buf[i+4]);
#endif
   }
}

/**
  Send a LightwaveRF message (10 nibbles in bytes)
**/
void LwTx::cmd(byte command, byte parameter, byte room, byte device) {
  //enable timer 2 interrupts
  tx_buf[0] = tx_nibble[parameter >> 4];
  tx_buf[1] = tx_nibble[parameter  & 0xF];
  tx_buf[2] = tx_nibble[device  & 0xF];
  tx_buf[3] = tx_nibble[command  & 0xF];
  tx_buf[9] = tx_nibble[room  & 0xF];
  LwTx::timerStart();
  tx_msg_active = true;
}

/**
  Set things up to transmit LightWaveRF 434Mhz messages
**/
void LwTx::setup(int pin, byte repeats, byte invert, int period) {
#if EEPROM_EN
  for(int i=0; i<5; i++) {
    tx_buf[i+4] = EEPROM.read(EEPROMaddr+i);
  }
#endif
  if(pin !=0) {
    tx_pin = pin;
  }
  pinMode(tx_pin,OUTPUT);
  digitalWrite(tx_pin, txoff);
  
  if(repeats > 0 && repeats < 40) {
   tx_repeats = repeats;
  }
  if(invert != 0) {
   txon = 0;
   txoff = 1;
  } else {
   txon = 1;
   txoff = 0;
  }
  
  int period1;
  if (period > 32 && period < 1000) {
    period1 = period; 
  } else {
    //default 140 uSec
    period1 = 140;
  }
  LwTx::timerSetup(&LwTx::isrTXtimer, period1);
}

void LwTx::setTickCounts( byte lowCount, byte highCount, byte trailCount, byte gapCount) {
  tx_low_count = lowCount;
  tx_high_count = highCount;
  tx_trail_count = trailCount;
  tx_gap_count = gapCount;
}

void LwTx::setGapMultiplier(byte gapMultiplier) {
   tx_gap_multiplier = gapMultiplier;
}

/**
  Set EEPROMAddr
**/
extern void LwTx::setEEPROMaddr(int addr) {
   EEPROMaddr = addr;
}

// There are 3 timer support routines. Variants of these may be placed here to support different environments
void (LwTx::*isrRoutine) ();
#if defined(SPARK_CORE)
//#include "SparkIntervalTimer.h"
//reference for library when imported in Spark IDE
#include "SparkIntervalTimer/SparkIntervalTimer.h"
IntervalTimer txmtTimer;

extern void LwTx::timerSetup(void (LwTx::*isrCallback)(), int period) {
  isrRoutine = isrCallback;
  noInterrupts();
  txmtTimer.begin(isrRoutine, period, uSec);  //set IntervalTimer interrupt at period uSec (default 140)
  txmtTimer.interrupt_SIT(INT_DISABLE); // initialised as off, first message starts it
  interrupts();
}
extern void LwTx::timerStart() {
  txmtTimer.interrupt_SIT(INT_ENABLE);
}
extern void LwTx::timerStop() {
  txmtTimer.interrupt_SIT(INT_DISABLE);
}

#elif defined(DUE)
#include "DueTimer.h"
DueTimer txmtTimer = DueTimer::DueTimer(0);
boolean dueDefined = false;
extern void LwTx::timerSetup(void (LwTx::*isrCallback)(), int period) {
  if (!dueDefined) {
    txmtTimer = DueTimer::getAvailable();
    dueDefined = true;
  }
  isrRoutine = isrCallback;
  noInterrupts();
  txmtTimer.attachInterrupt(isrCallback);
  txmtTimer.setPeriod(period);
  interrupts();
}
extern void LwTx::timerStart() {
  txmtTimer.start();
}

extern void LwTx::timerStop() {
  txmtTimer.stop();
}
#else
//Default case is Arduino Mega328 which uses the TIMER2
extern void LwTx::timerSetup(void (LwTx::*isrCallback)(), int period) {
  isrRoutine = isrCallback; // unused here as callback is direct
  byte clock = (period / 4) - 1;;
  cli();//stop interrupts
  //set timer2 interrupt at  clock uSec (default 140)
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for clock uSec
  OCR2A = clock;// = 16MHz Prescale to 4 uSec * (counter+1)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS11 bit for 64 prescaler
  TCCR2B |= (1 << CS22);   
  // disable timer compare interrupt
  TIMSK2 &= ~(1 << OCIE2A);
  sei();//enable interrupts
}

extern void LwTx::timerStart() {
   //enable timer 2 interrupts
  TIMSK2 |= (1 << OCIE2A);
}

extern void LwTx::timerStop() {
  //disable timer 2 interrupt
  TIMSK2 &= ~(1 << OCIE2A);
}

#endif
