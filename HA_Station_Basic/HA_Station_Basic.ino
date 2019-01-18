/*
 * Home Automation
 * Station
 * Basic
 * 
 * Target: Arduino Nano
 * 
 * Modbus on HC12 wireless
 * 3 x Digital Input
 * 3 x Digital Output
 * 3 x DS18B20 temperature sensor(s)
 * 3 x MCP9808 temperature sensor(s)
 * 1 x Si7021 Temp & Humidity Sensor
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
 * Digital Outputs
 * D10 - Modbus Coils 0
 * D11 - Modbus Coils 1
 * D12 - Modbus Coils 2
 * 
 * Digital Inputs
 * D7 - Modbus Input 0
 * D8 - Modbus Input 1
 * D9 - Modbus Input 2
 * 
 * Modbus Coil Registers
 * 0 = D10
 * 1 = D11
 * 2 = D12
 * 
 * Modbus Input Registers:
 * 0 = D7
 * 1 = D8
 * 2 = D9
 *
 * Modbus Holding Registers:
 * 100 = First DS18B20 Temp sensor
 * 101 = Second DS18B20 Temp sensor
 * 102 = Third DS18B20 Temp sensor
 * 105 = First MCP9808 Temp snesor
 * 106 = Second MCP9808 Temp sensor
 * 107 = Third MCP9808 Temp sensor
 * 110 = Si7021 Temperature
 * 111 = Si7021 Humidity
 * 
 */

const byte VERSION_MAJOR = 1;
const byte VERSION_MINOR = 0;
const byte VERSION_RELEASE = 0;

// Comment out definitions below to disable sensors 
#define DS18B20       //DS18B20 Temperature Sensors
#define SI7021        //Si7021 Temperature & Humidity
#define MCP9808_NUM 3 //Maximum number of MCP9808 Temp sensors (<= 8)

#include "Modbus.h"
#include "ModbusSerial.h"
#include "EEPROM.h"

// libraries:
#ifdef DS18B20
#include <OneWire.h> 
#include <DallasTemperature.h>
#endif

#define MODBUS_DEFAULT_ADDRESS 247     // Modbus Slave default address

const int HR_MCP9808_BASE_ADDR = 105;    // Holding Register base address
const int HR_SI7021_BASE_ADDR = 110;     // Holding Register base address
const int HR_DS18B20_BASE_ADDR = 100;    // Holding Register base address
const int COIL_BASE_ADDR = 0;       // Coil address for modbus outputs (coils)
const uint8_t COIL_PINS[] = { 10, 11, 12 };   //Output pins for modbus coils
const bool COIL_DEFAULTS[] = { false, false, false };  //Default state for modbus coil pins
const int INPUT_BASE_ADDR = 0;      // Input address for modbus inputs
const uint8_t INPUT_PINS[] = { 7, 8, 9 };   //Input pins for modbus inputs

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

#ifdef SI7021
#include "Si7021.h"
Si7021 th_sensor = Si7021();
#endif

#ifdef MCP9808_NUM
#include "MCP9808.h"
//MCP9808 t_sensor = MCP9808();
MCP9808 *tmp_sensor[MCP9808_NUM];
#endif

SoftwareSerial HC12(HC12TxdPin,HC12RxdPin); // Software serial for HC12 module
ModbusSerial mb;                          

#ifdef DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);
#endif

uint8_t num_temp_sensors = 0;       // number of connected DS18B20 temperature sensors
uint8_t modbus_address = 0;         // RTU slave address
bool th_sensor_present = false;     // Si7021 Temp Hum sensor present
int loop_count = 0;
unsigned long LED_off_time = 0;
byte cmd_mode_cnt = CMD_MODE_CNT;


void setup() {
  int i;
  // start serial port for debug
  Serial.begin(9600);     // for debug output only

  Serial.print("\nHA_Station_Basic V");
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
  
#ifdef SI7021
  Serial.print("Si7021 Temp/Hum sensor ");
  if (!th_sensor.begin()) {
    th_sensor_present = false;
    Serial.println("not found");
  } else {
    th_sensor_present = true;
    Serial.println("found");
    mb.addHreg (HR_SI7021_BASE_ADDR, 0);
    mb.addHreg (HR_SI7021_BASE_ADDR + 1, 0);
  }
#endif

#ifdef MCP9808_NUM
  for(i=MCP9808_NUM-1; i >= 0 ;i--) {
    tmp_sensor[i] = new MCP9808();
    Serial.print("MCP9808 @ 0x");
    Serial.print(MCP9808_I2CADDR_DEFAULT + i, HEX);
    if (! tmp_sensor[i]->begin(MCP9808_I2CADDR_DEFAULT + i) ) {
      Serial.println(" not found");
      delete tmp_sensor[i];      // invalidate MCP9808 object to indicate sensor is not present
      tmp_sensor[i] = NULL;
    } else {
      Serial.println(" found");
    }
  }
#endif
  
  // LED setup
  pinMode(LED_BUILTIN, OUTPUT);      
  digitalWrite(LED_BUILTIN, LED_OFF);

  // Set HC12 set mode off
  pinMode(HC12_set_pin, OUTPUT);
  digitalWrite(HC12_set_pin, HIGH);
  


  // Digital Output config
  for (i=0; i<sizeof(COIL_PINS); i++) {
    pinMode(COIL_PINS[i], OUTPUT);
    mb.addCoil(COIL_BASE_ADDR + i, COIL_DEFAULTS[i]);
  }

  // Digital Input config
  for (i=0; i<sizeof(INPUT_PINS); i++) {
    pinMode(INPUT_PINS[i], INPUT);
    mb.addIsts(INPUT_BASE_ADDR + i);
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
      mb.Hreg(HR_DS18B20_BASE_ADDR + i, regValue);
      Serial.print("DS18B20 Temperature[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.print(regValue);
      Serial.print("\n");
    }
  }
#endif
  
#ifdef SI7021
  if (th_sensor_present) {
    regValue = (int) (th_sensor.readTemperature() * 100.0);
    mb.Hreg(HR_SI7021_BASE_ADDR, regValue);
    Serial.print("Si7021 Temperature: "); Serial.print(regValue);
    regValue = (int) (th_sensor.readHumidity() * 100.0);
    mb.Hreg(HR_SI7021_BASE_ADDR + 1, regValue);
    Serial.print("\tHumidity: "); Serial.println(regValue);
  }
#endif

#ifdef MCP9808_NUM
  for(i=0; i < MCP9808_NUM ;i++) {
    if (tmp_sensor[i]) {
      regValue = (int) (tmp_sensor[i]->readTempC() * 100.0);
      mb.Hreg(HR_MCP9808_BASE_ADDR + i, regValue);
      Serial.print("MCP9808 Temperature[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.print(regValue);
      Serial.print("\n");
    }
  }
#endif  
 
}

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
      unsigned long timeout = millis() + 30000; // restart timeout
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
  byte in;
  
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
  
  // Modbus task is processed frequently
  if (mb.task()) {    // returns true on modbus activity
    //Serial.print("Modbus activity\n");
    digitalWrite(LED_BUILTIN, LED_ON);
    LED_off_time = millis() + LED_ON_TIME;
    // write digital outputs (Modbus Coils)
    for (int i=0; i<sizeof(COIL_PINS); i++) {
      digitalWrite(COIL_PINS[i], mb.Coil(COIL_BASE_ADDR + i) );
    } 
  }
  loop_count++;
  delay(MODBUS_TASK_DELAY);

  // Read inputs and write value to modbus registers
  for (int i=0; i<sizeof(INPUT_PINS); i++) {
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
