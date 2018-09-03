/*
 * hc12_mb_listener
 * Modbus listener, for diagnostic purposes
 */

#define HC12_BAUD 1200          // Baudrate 
#define HC12_CHANNEL "001"      // 001 to 127, use spacing of 5 channels 
#define HC12_POWER 8            // TX power 1-8 = -1/2/5/8/11/14/17/20dBm
#define HC12_MODE 3             // 1-4 = FU1/FU2/FU3/FU4  FU3 is the factory default

#include <SoftwareSerial.h>

#include "debug.h"
#define DEBUG_LEVEL_DEFAULT L_INFO;
debugLevels currentDebugLevel;

#define HC12_SET_DELAY 300      // Timeout for HC12 set mode
#define HC12_SET_RX_TIMEOUT 500 // Timeout when receiving data in set mode

const byte HC12_rx_pin = 4;
const byte HC12_tx_pin = 5;
const byte HC12_set_pin = 6;

unsigned int _t15;
unsigned int _t35;
byte *frame;

#define CONSOLE_BAUD 9600

SoftwareSerial HC12( HC12_tx_pin, HC12_rx_pin);

char HC12ByteIn;
String HC12ReadBuffer = "";               

int loopCount = 0;

char* functionStr[]={"--", "Read Coil", "Read DI", "Read HR", "Read Input Reg", "Write Coil", "Write Holding"};

void setup() {
  delay(1000);
  Serial.begin(CONSOLE_BAUD);
  currentDebugLevel = DEBUG_LEVEL_DEFAULT;
  
  HC12ReadBuffer.reserve(64);
  pinMode(HC12_set_pin, OUTPUT);
  hc12_set_mode(false);
  HC12.begin(HC12_BAUD);
  // Calculate Modbus timeouts
  if (HC12_BAUD > 19200) {
        _t15 = 750;
        _t35 = 1750;
    } else {
        _t15 = 15000000/HC12_BAUD; // 1T * 1.5 = T1.5
        _t35 = 35000000/HC12_BAUD; // 1T * 3.5 = T3.5
    }
  
  debug( L_INFO, "\n-- HC12 Modbus listener --\n");
  hc12_show_config();

  debug( L_INFO, "\nWaiting for Modbus frame .....\n");
}

void loop() { 

  // any data arrived -> process modbus frame
  if (HC12.available()) {
    read_frame();
  }
}

// read and process one modbus frame
void read_frame() {
  int len = 0;

  while (HC12.available() > len) {
    len = HC12.available();
    delayMicroseconds(_t15);
  }

  if (len == 0) return;

  byte i;
  frame = (byte*) malloc(len);
  for (i=0 ; i < len ; i++) frame[i] = HC12.read();

  process_frame(len);

  free(frame);
  len = 0;
}

// print frame data
void process_frame(int len) {
  //first byte of frame = address
  //int address = frame[0];

  debug (L_INFO, "%8lu: #%d %s, raw:", millis(), frame[0], functionStr[frame[1]]);
  for (int i = 0; i < len; i++) {
    debug (L_INFO, " %02X", frame[i]);
  }
  debug (L_INFO, "\n");
}

// print debug output on console interface
void debug(debugLevels level, const char *sFmt, ...)
{
  if (level > currentDebugLevel) return;  // bypass if level is not high enough
  char acTmp[128];       // place holder for sprintf output
  va_list args;          // args variable to hold the list of parameters
  va_start(args, sFmt);  // mandatory call to initilase args 

  vsprintf(acTmp, sFmt, args);
  Serial.print(acTmp);
  // mandatory tidy up
  va_end(args);
  return;
}

void hc12_show_config() {
  debug( L_INFO, "HC12 configuration:\n");
  hc12_set_mode(true);
  // retrieve HC12 version 
  HC12.print("AT+RX\n");
  
  for (int i=1; i<=4; i++) {
    if ( hc12_rx_line( HC12_SET_RX_TIMEOUT ) ) {
      debug( L_INFO, HC12ReadBuffer.c_str() );
    }
  }
  hc12_set_mode(false);
  return;
}

bool hc12_rx(int timeout) {
  int rxCount = 0;
  HC12ReadBuffer.remove(0);   // clear string
  // wait for reply from hc12
  long finishtime = millis() + timeout;
  while (millis() < finishtime) {
    if (HC12.available()) {
      rxCount++;
      HC12ByteIn = HC12.read();
      HC12ReadBuffer += char(HC12ByteIn);
    }
  }
  if (rxCount < 1) return false;
  return true;
}

bool hc12_rx_line(int timeout) {
  // receive one line
  HC12ReadBuffer.remove(0);   // clear string
  long finishtime = millis() + timeout;
  while (millis() < finishtime) {
    if (HC12.available()) {
      HC12ByteIn = HC12.read();
      HC12ReadBuffer += char(HC12ByteIn);
      if (HC12ByteIn == '\n') return true;
    }
  }
  debug(L_ERROR, "hc12_rx_line() Timeout\n");
  return false;
}

void hc12_set_mode(bool newMode) {
  if (newMode) {
    digitalWrite(HC12_set_pin, LOW);
    delay(HC12_SET_DELAY);
  } else {
    digitalWrite(HC12_set_pin, HIGH);
    delay(HC12_SET_DELAY);
  }
}



