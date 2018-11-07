/*
 * hc12_setup 
 * Configure HC12 wireless module settings
 * The new setting are specified in the 4 lines below
 * This program will carry out the following steps:
 * - auto detect the current HC12 baudrate 
 * - print the current HC12 configuration to the console
 * - transfer the new settings to the HC12
 * - print the new HC12 configuration to the console
 * - continually send beacons via the HC12
 */
//---New Settings--------------------------------------------------------------------
#define HC12_BAUD 1200          // Baudrate 
#define HC12_CHANNEL 1          // 001 to 127, use spacing of 5 channels 

#define HC12_POWER 8            // TX power 1-8 = -1/2/5/8/11/14/17/20dBm
#define HC12_MODE 3             // 1-4 = FU1/FU2/FU3/FU4  FU3 is the factory default
//-----------------------------------------------------------------------------------


#include <SoftwareSerial.h>

#include "debug.h"
#define DEBUG_LEVEL_DEFAULT L_INFO;
debugLevels currentDebugLevel;

#define HC12_SET_DELAY 300      // Timeout for HC12 set mode
#define HC12_SET_RX_TIMEOUT 1000 // Timeout when receiving data in set mode

const byte HC12_rx_pin = 4;
const byte HC12_tx_pin = 5;
const byte HC12_set_pin = 6;

#define CONSOLE_BAUD 9600

long baudrates[] = { 1200, 2400, 4800, 9600, 38400, 57600, 115200, 0 };
long hc12_active_baud = 9600;

SoftwareSerial HC12( HC12_tx_pin, HC12_rx_pin);

char HC12ByteIn;
String HC12ReadBuffer = "";               

int loopCount = 0;

void setup() {
  Serial.begin(CONSOLE_BAUD);
  Serial.println("Starting");
  currentDebugLevel = DEBUG_LEVEL_DEFAULT;
  
  HC12ReadBuffer.reserve(256);
  pinMode(HC12_set_pin, OUTPUT);
  hc12_set_mode(false);
  
  if (!hc12_auto_baud()) {
    debug(L_ERROR, "Unable to detect HC12\n");
  } else {
    debug(L_INFO, "Detected Baudrate: %u\n", hc12_active_baud);
    debug( L_INFO, "\nCurrent HC12 configuration:\n");
    hc12_show_config();
  }
 
  hc12_setup(); // configure HC12 with new settings

  debug( L_INFO, "\nNew HC12 configuration:\n");
  hc12_show_config();
}

void loop() {
  sendBeacon();  
  delay(3000);

  if (HC12.available()) {
    HC12ByteIn = HC12.read();
    Serial.print(HC12ByteIn);
  }

}

void sendBeacon() {
  HC12.print("HC12 Beacon #");
  HC12.print(loopCount++);
  HC12.print("\n"); 
  debug(L_INFO, "Beacon...\n");
}

// print debug output on console interface
void debug(debugLevels level, char *sFmt, ...)
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

// automatically detect HC12 baudrate
bool hc12_auto_baud() {

  int baud_index = 0;
  long baudrate = baudrates[baud_index];
  
  hc12_set_mode(true);        // place HC12 into set mode

  while (baudrate != 0) {
    HC12.begin(baudrate);
    debug(L_INFO, "Checking Baud: %u\n", baudrate);
    delay(20);
    HC12.print("AT\n");
    if ( !hc12_rx_line( HC12_SET_RX_TIMEOUT ) ) goto nextBaud;
    if ( hc12_check_set_response() )  {
      //debug(L_INFO, "Detected Baud: %u\n", baudrate);
      hc12_set_mode(false);   // return HC12 to transparent state
      hc12_active_baud = baudrate;
      return true;
    }
nextBaud:
    baud_index++;
    baudrate = baudrates[baud_index];
  }
  debug(L_ERROR, "Unable to detect Baudrate\n");
  return false;
}

void hc12_show_config() {
  hc12_set_mode(true);
  HC12.print("AT+RX\n");
  
  for (int i=1; i<=4; i++) {
    if ( hc12_rx_line( HC12_SET_RX_TIMEOUT ) ) {
      debug( L_INFO, HC12ReadBuffer.c_str() );
    }
  }
  hc12_set_mode(false);
  return;
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

bool hc12_setup() {
  bool ret_stat = false;

  Serial.print("\nChanging HC12 configuration:\n");
  hc12_set_mode(true);

  // check if HC12 is responding
  hc12_cmd("AT");

  // retrieve HC12 version (doesn't work on 1200 Baud)
  if (hc12_active_baud > 1200) {
    hc12_cmd("AT+V");
  }

  // set channel number
  hc12_cmd("AT+C%03d", HC12_CHANNEL);
  
  // set baudrate for transparent mode
  hc12_cmd("AT+B%d", HC12_BAUD);
  
  // set power level
  hc12_cmd("AT+P%d", HC12_POWER);

  // set mode
  hc12_cmd("AT+FU%d", HC12_MODE);
  
  ret_stat = true;

failed:
  // return HC12 to transparent mode
  hc12_set_mode(false);
  HC12.begin(HC12_BAUD);
  
  if (!ret_stat)
    debug(L_ERROR, "hc12_setup failed\n");
  return ret_stat;
}

bool hc12_cmd(char *sFmt, ...)
{
  int retry_count = 0;
  bool success = false;
  char cmdStr[128];       // place holder for sprintf output
  va_list args;          // args variable to hold the list of parameters
  va_start(args, sFmt);  // mandatory call to initilase args 

  vsprintf(cmdStr, sFmt, args);
  va_end(args);

  while (retry_count < 3) {
    HC12.print(cmdStr);
    HC12.print("\n");
    if (retry_count) {
      debug( L_INFO, "%s (retry %d)\n", cmdStr , retry_count);
      hc12_set_mode(true);
    } else {
      debug( L_INFO, "%s\n", cmdStr);
    }
    if ( hc12_rx_line( HC12_SET_RX_TIMEOUT ) ) {
      debug( L_INFO, "%s", HC12ReadBuffer.c_str());
      success = hc12_check_set_response();
      if (success) break;      
    }
    retry_count++;
  } // while
  if (!success) {
    debug( L_INFO, "%s failed\n", cmdStr);
  }
  return success;
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

//
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

bool hc12_check_set_response() {
  // check if we received "OK" from HC12
  if ( (HC12ReadBuffer[0] != 'O') || (HC12ReadBuffer[1] != 'K') ) {
    debug(L_ERROR, HC12ReadBuffer.c_str());
    return false;
  }
  return true;
}

