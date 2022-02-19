/*
 * Home Automation
 * Station
 * Shack 12V Power Control
 * 
 * Target: Arduino Nano
 * 
 * Modbus on HC12 wireless
 * 8 x Digital Output
 * 1 x DS18B20 temperature sensor (onboard)
 * 
 * HC12 Wiring :
 * Nano - HC12
 *  5V - VCC
 * GND - GND
 *  D4 - RXD 
 *  D5 - TXD
 *  D6 - SET (not used)
 *  
 * DS18B20 Wiring:
 * 4.7k pullup resistor between DS18B20 Data (yellow) and +5V (red)
 * The data line of all connected DS18B20 sensors is wired pin D2
 * 
 * Builtin LED
 * D13 - Indicates Modbus activity
 * 
 * Modbus Coil Registers
 * 0 = D7  - Power Outlet 1+2
 * 1 = D8  - Power Outlet 3+4
 * 2 = D9  - Power Outlet 5
 * 3 = D10 - Power Outlet 6
 * 4 = D11 - Power Outlet 7
 * 5 = D12 - Power Outlet 8
 * 6 = D16 - Power Outlet 9
 * 7 = D17 - Power Outlet 10
 * 
 * Note: using SDA and SCA as there are already pins on the PCB
 * 
 * Modbus Input Registers:
 * 0 = D3  - Not Used
 *
 * 
 * Modbus Holding Registers:
 * 100 = DS18B20 Temp sensor 1 (on board)
 * 
 */

const byte VERSION_MAJOR = 1;
const byte VERSION_MINOR = 0;
const byte VERSION_RELEASE = 0;

// Comment out definitions below to disable sensors 
#define DS18B20       //DS18B20 Temperature Sensors

#include "Modbus.h"
#include "ModbusSerial.h"
#include "EEPROM.h"

// libraries:
#ifdef DS18B20
#include <OneWire.h> 
#include <DallasTemperature.h>
#endif

#define MODBUS_DEFAULT_ADDRESS 247     // Modbus Slave default address

#define DEFAULT_REMOTE_LO_SPD 1       // Default speeds are deployed upon 
#define DEFAULT_REMOTE_HI_SPD 5       // change of control from Local to Remote

const int HR_DS18B20_BASE_ADDR = 100;    // Holding Register base address for Temperatures
const int HR_DS18B20_OFFSET[] = { 0, -200 };  // Offset to correct temp reading of crappy sensors
const int COIL_BASE_ADDR = 0;       // Coil address for modbus outputs (coils)
const uint8_t COIL_PINS[] = { 7, 8, 9, 10, 11, 12, 16, 17 };   //Output pins for modbus coils
const bool COIL_DEFAULTS[] = { false, false, false };  //Default state for modbus coil pins
const int INPUT_BASE_ADDR = 0;      // Input address for modbus inputs
const uint8_t INPUT_PINS[] = { 3 };   //Input pins for modbus inputs
const int HR_ANALOG_BASE_ADDR = 110;     // Holding Register base address for Analog Inputs
//const uint8_t ANANLOG_PINS[] = { A7 };    // Input pins for Analogs

// these relate to the Modbus COIL_PINS array
#define COIL_POWER1 0
#define COIL_POWER2 1

const long MODBUS_BAUD = 1200;      // Baudrate for Modbus comms
const unsigned long MODBUS_TASK_DELAY = 200;  // run modbus task every X ms
const int TEMP_UPDATE = 40;         // process temp update every X * MODBUS_TASK_DELAY

const byte HC12RxdPin = 4;          // RX Pin on HC12
const byte HC12TxdPin = 5;          // TX Pin on HC12
const byte HC12_set_pin = 6;          // SET Pin on HC12

const uint8_t ONE_WIRE_BUS = 2;     // D2 is used for one wire bus

const long LED_ON_TIME = 500;       // [ms] LED on modbus activity

const char CMD_MODE_CHAR = '+';     // chars to enter command mode
const byte CMD_MODE_CNT = 3;        // number of characters to enter command mode

#define LED_ON HIGH
#define LED_OFF LOW

SoftwareSerial HC12(HC12TxdPin,HC12RxdPin); // Software serial for HC12 module
ModbusSerial mb;                          

#ifdef DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);
#endif

uint8_t num_temp_sensors = 1;       // number of connected DS18B20 temperature sensors
uint8_t modbus_address = 0;         // RTU slave address
int loop_count = 0;
unsigned long LED_off_time = 0;
byte cmd_mode_cnt = CMD_MODE_CNT;

void setup() {
  unsigned int i;
  int value;
  // start serial port for debug
  Serial.begin(9600);     // for debug output only

  Serial.print("\nHA_ShackPower12V_Control V");
  Serial.print(VERSION_MAJOR);
  Serial.print(".");
  Serial.print(VERSION_MINOR);
  Serial.print(".");
  Serial.println(VERSION_RELEASE);

  getModbusAddress();     // get modbus address from EEPROM
  
  // modbus config
  mb.config (&HC12, MODBUS_BAUD);  
  mb.setSlaveId (modbus_address);

  Serial.print("\nModbus Slave #");
  Serial.println(modbus_address);

  Serial.println("\nInitializing Sensors:");
  
  // LED setup
  pinMode(LED_BUILTIN, OUTPUT);      
  digitalWrite(LED_BUILTIN, LED_OFF);

  // Set HC12 set mode off
  pinMode(HC12_set_pin, OUTPUT);
  digitalWrite(HC12_set_pin, HIGH);
  
// Digital Input config
  for (i=0; i<sizeof(INPUT_PINS); i++) {
    pinMode(INPUT_PINS[i], INPUT);
    // set to pulldown
    digitalWrite(INPUT_PINS[i], HIGH);
    mb.addIsts(INPUT_BASE_ADDR + i);
  }
  
  // Digital Output config
  for (i=0; i<sizeof(COIL_PINS); i++) {
    pinMode(COIL_PINS[i], OUTPUT);
    mb.addCoil(COIL_BASE_ADDR + i, COIL_DEFAULTS[i]);
  }
  // set digital outputs (Modbus Coils)
  for (i=0; i<sizeof(COIL_PINS); i++) {
    digitalWrite(COIL_PINS[i], mb.Coil(COIL_BASE_ADDR + i) );
  } 
  
#ifdef DS18B20
  // DS18B20 temp sensors
  tempSensors.begin();
  num_temp_sensors = tempSensors.getDeviceCount();
  if (num_temp_sensors < 1) {
    mb.addHreg (HR_DS18B20_BASE_ADDR, 0);
  } else {
    for(int i=0; i<num_temp_sensors; i++) {
      mb.addHreg (HR_DS18B20_BASE_ADDR + i, 0);
    }
  }
  Serial.print("DS18B20 Temperature Sensor(s): ");
  Serial.println(num_temp_sensors);
  
#endif
  
  // Advertise command mode
  Serial.println("\n+++ to enter command mode");
}

void getModbusAddress() {
  //Serial.println("EEPROM:");
  byte addr = EEPROM.read(0);
  // check for invalid address range
  if ( (addr > 247) || (addr < 1) ) {
    modbus_address = MODBUS_DEFAULT_ADDRESS;
  } else {
    modbus_address = addr;
  }
}

bool setModbusAddress(byte addr) {
  // check for invalid address range
  if ( (addr > 247) || (addr < 1) ) {
    return false; 
  } else {
    EEPROM.write(0, addr);
  }
  return true;
}

void readTemps() {
  int i, regValue;
  float temp;
#ifdef DS18B20
  if (num_temp_sensors > 0) {
    tempSensors.requestTemperatures();
    for(i=0; i<num_temp_sensors; i++) {
      temp = tempSensors.getTempCByIndex(i);
      regValue = (int) (temp *100.0);
      regValue += HR_DS18B20_OFFSET[i];
      mb.Hreg(HR_DS18B20_BASE_ADDR + i, regValue);
      Serial.print("DS18B20 Temperature[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.print(mb.Hreg(HR_DS18B20_BASE_ADDR + i));
      Serial.print("\n");
    }
  }
#endif
}

/*
void readAnalogs() {
  int rawValue;
  rawValue = analogRead(ANANLOG_PINS[0]);
  mb.Hreg(HR_ANALOG_BASE_ADDR, rawValue);
  //Serial.print("A7: ");
  //Serial.print(rawValue);
  //Serial.print("\n");
}
*/

int read_value() {
  char inStr[20];
  char inChar;
  int index = 0;
  unsigned long timeout = millis() + 5000;
  while (millis() < timeout) {
    if (Serial.available()) {
      inChar = Serial.read();

      if ((inChar < '0') || (inChar > '9')) {   // detect end of input (e.g. CR or LF
         while(Serial.available()) Serial.read();   // clear buffer
         inStr[index++] = 0;            /// end of string
         return(atoi(inStr));
      } else {
        inStr[index++] = inChar;
      }
    }
  }
  return -1;
}

void command_loop() {
  char inChar;
  bool endLoop = false;
  int newAddr;
  unsigned long timeout = millis() + 30000;
  Serial.println("\nCommand Mode\nAvailable Commands:");
  Serial.println("Mxxx = set modbus address to xxx");
  Serial.println("X    = exit command mode");
  while(!endLoop) {
    if(Serial.available()) {
      //unsigned long timeout = millis() + 30000; // restart timeout
      inChar = Serial.read();
      if (inChar < ' ') continue; // discard control characters
      switch(inChar) {
        case 'm':
        case 'M':
          newAddr = read_value();
          if (setModbusAddress(newAddr)) {
            Serial.print("powercycle device to activate new modbus address ");
            Serial.println(newAddr);
          } else {
            Serial.println("Error: Invalid modbus address.");
          }
          break;
        case 'x':
        case 'X':
          endLoop = true;
          break;
        default:
          Serial.print("Unknown command ");
          Serial.println(inChar);
      }
    } else {
      if (millis() > timeout) {
        endLoop = true;
        Serial.print("\nTimeout - ");
      }
    }
    
  }
  Serial.println("Resuming Modbus mode");
}

void processSerialInput() {
  //byte in;
  
  while(Serial.available()) {
    if (Serial.read() == CMD_MODE_CHAR) {
      cmd_mode_cnt-- ;
    } else {
      cmd_mode_cnt = CMD_MODE_CNT;
    }
    if (cmd_mode_cnt < 1) {
      while(Serial.available()) Serial.read();  // discard buffer
      command_loop();
    }
  }
}


void loop() {

  unsigned int i;
  // Modbus task is processed frequently
  if (mb.task()) {    // returns true on modbus activity
    //Serial.print("Modbus activity\n");
    digitalWrite(LED_BUILTIN, LED_ON);
    LED_off_time = millis() + LED_ON_TIME;
    // write digital outputs (Modbus Coils)
    for (i=0; i<sizeof(COIL_PINS); i++) {
      digitalWrite(COIL_PINS[i], mb.Coil(COIL_BASE_ADDR + i) );
    } 
  }
  loop_count++;
  delay(MODBUS_TASK_DELAY);

  // Read inputs and write value to modbus registers
  for (i=0; i<sizeof(INPUT_PINS); i++) {
    mb.Ists(INPUT_BASE_ADDR + i, digitalRead(INPUT_PINS[i]) );
  } 
    
  // Temperature update is processed less frequently
  if (loop_count >= TEMP_UPDATE) {
    readTemps();
    loop_count = 0;
  }
  
  // LED off control
  if (LED_off_time != 0) {
    if (millis() > LED_off_time) {
      digitalWrite(LED_BUILTIN, LED_OFF);
      LED_off_time = 0;
    }
  }

  if (Serial.available()) {
    processSerialInput();
  }
}
