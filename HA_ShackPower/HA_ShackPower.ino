/*
 * Home Automation
 * Station
 * Shack Power Control
 * 
 * Target: Arduino Nano
 * 
 * Modbus on HC12 wireless
 * 5 x Digital Input
 * 5 x Digital Output
 * 2 x DS18B20 temperature sensor
 * 1 x Analog Input potentiometer
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
 * 0 = D16 - VSD Run signal
 * 1 = D17 - VSD B0 (speed)
 * 2 = D15 - VSD B1 (speed)
 * 3 = D14 - VSD B2 (spped)
 * 4 = D3  - Water Pump
 * 
 * Modbus Input Registers:
 * 0 = D7  - Pump Run Command
 * 1 = D8  - Aircon Hi Speed command
 * 2 = D9  - Local Control Start PB
 * 3 = D10 - Local Control Stop PB (Normally Closed)
 * 4 = D11 - Local Control Selected (Hi = Local, Lo = Remote)
 * 
 * Modbus Holding Registers:
 * 100 = DS18B20 Temp sensor 1 (environment)
 * 101 = DS18B20 Temp sensor 2 (on board)
 * 
 * 110 = A7 - Analog Input Potentiometer (0-764) max 3.3V
 * 
 * 112 = Speed Setpoint to drive (0-7)
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
const uint8_t COIL_PINS[] = { 16, 17, 15, 14, 3 };   //Output pins for modbus coils
const bool COIL_DEFAULTS[] = { true, true, true, true, false };  //Default state for modbus coil pins
const int INPUT_BASE_ADDR = 0;      // Input address for modbus inputs
const uint8_t INPUT_PINS[] = { 7, 8, 9, 10, 11 };   //Input pins for modbus inputs
const int HR_ANALOG_BASE_ADDR = 110;     // Holding Register base address for Analog Inputs
const uint8_t ANANLOG_PINS[] = { A7 };    // Input pins for Analogs
const int HR_SPEED_SP_ADDR = 112;       // Holding Register for VSD speed setpoint (1-7)
//const uint8_t HR_SPEED_SP_DEFAULT = 7;  // Default value for speed SP (used after cold start in remote mode)

// these relate to the Modbus COIL_PINS array
#define COIL_VSD_START_ADDR 0
#define COIL_VSD_B0_ADDR 1
#define COIL_VSD_B1_ADDR 2
#define COIL_VSD_B2_ADDR 3
#define COIL_PUMP_ADDR 4

const long MODBUS_BAUD = 1200;      // Baudrate for Modbus comms
const unsigned long MODBUS_TASK_DELAY = 200;  // run modbus task every X ms
const int TEMP_UPDATE = 40;         // process temp update every X * MODBUS_TASK_DELAY

const byte HC12RxdPin = 4;          // RX Pin on HC12
const byte HC12TxdPin = 5;          // TX Pin on HC12
const byte HC12_set_pin = 6;          // SET Pin on HC12

const uint8_t ONE_WIRE_BUS = 2;     // D2 is used for one wire bus

const byte CTRL_PUMP_CMD_PIN = 7;       // D7 remote "pump on" command
const byte CTRL_HI_SPD_CMD_PIN = 8;     // D8 remote "hi speed" command
const byte CTRL_START_CMD_PIN = 9;      // D9 local Start PB
const byte CTRL_STOP_CMD_PIN = 10;       // D10 local Stop PB (NC, activated when low)
const byte CTRL_LOCAL_CMD_PIN = 11;     // D11 high = Local Control override activated

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

uint8_t num_temp_sensors = 2;       // number of connected DS18B20 temperature sensors
uint8_t modbus_address = 0;         // RTU slave address
int loop_count = 0;
unsigned long LED_off_time = 0;
byte cmd_mode_cnt = CMD_MODE_CNT;
bool local_ctrl_active = false;
bool local_ctrl_active_shadow = false;
bool hi_speed_cmd = false;
bool hi_speed_cmd_shadow = false;
bool pump_cmd = false;
bool pump_cmd_shadow = false;

void setup() {
  unsigned int i;
  int value;
  // start serial port for debug
  Serial.begin(9600);     // for debug output only

  Serial.print("\nHA_SHackPower_Control V");
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
  local_ctrl_active = !digitalRead(CTRL_LOCAL_CMD_PIN);
  
  if (local_ctrl_active) {
    Serial.println("Local control");
  } else {
    Serial.println("Remote control");
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

  // Analog Input
  value = analogRead(ANANLOG_PINS[0]);
  mb.addHreg (HR_ANALOG_BASE_ADDR, value);

  // Speed setpoint
  if (!digitalRead(CTRL_HI_SPD_CMD_PIN)) {
    mb.addHreg (HR_SPEED_SP_ADDR, DEFAULT_REMOTE_HI_SPD);
  } else {
    mb.addHreg (HR_SPEED_SP_ADDR, DEFAULT_REMOTE_LO_SPD);
  }

  // Pump 
  mb.Coil(COIL_PUMP_ADDR, !digitalRead(CTRL_PUMP_CMD_PIN));
  if (mb.Coil(COIL_PUMP_ADDR)) {
    Serial.println("Pump is ON");
  } else {
    Serial.println("Pump is OFF");
  }
  
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

void readAnalogs() {
  int rawValue;
  rawValue = analogRead(ANANLOG_PINS[0]);
  mb.Hreg(HR_ANALOG_BASE_ADDR, rawValue);
  //Serial.print("A7: ");
  //Serial.print(rawValue);
  //Serial.print("\n");
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

// called when local control is activated by selectror switch on panel
// allows on/off control from panel
void processControlLocal() {
  int newSpeedSetpoint = 0;
  int rawAnalog = mb.Hreg(HR_ANALOG_BASE_ADDR);

  // in local mode the speed reference comes from the potentiometer 
  // determine speed setpoint (1-7) from analog input 
  if (rawAnalog < 105) newSpeedSetpoint = 1;
  if ((rawAnalog > 115) && (rawAnalog < 215)) newSpeedSetpoint = 2;
  if ((rawAnalog > 225) && (rawAnalog < 325)) newSpeedSetpoint = 3;
  if ((rawAnalog > 335) && (rawAnalog < 435)) newSpeedSetpoint = 4;
  if ((rawAnalog > 445) && (rawAnalog < 545)) newSpeedSetpoint = 5;
  if ((rawAnalog > 555) && (rawAnalog < 655)) newSpeedSetpoint = 6;
  if (rawAnalog > 665) newSpeedSetpoint = 7;

  int old_sp = mb.Hreg(HR_SPEED_SP_ADDR);
  // if pot is between ranges we have a value of 0
  if (newSpeedSetpoint != 0) {
    // check if the speed setpoint has changed   
    // modify setpoint if it has changed
    if (old_sp != newSpeedSetpoint) {
      mb.Hreg(HR_SPEED_SP_ADDR, newSpeedSetpoint);    // update MB register with new speed SP
      Serial.print("New Speed SP = ");
      Serial.print(newSpeedSetpoint);
      Serial.print("\n");   
    }
  }
  // Start button activated
  if (digitalRead(CTRL_START_CMD_PIN) == false) {
    if (!mb.Coil(COIL_VSD_START_ADDR)) Serial.print("Started\n");
    mb.Coil(COIL_VSD_START_ADDR, true);
    mb.Coil(COIL_PUMP_ADDR, true);
  }

  // Stop button activate
  if (digitalRead(CTRL_STOP_CMD_PIN) == true) {
    if (mb.Coil(COIL_VSD_START_ADDR)) Serial.print("Stopped\n");
    mb.Coil(COIL_VSD_START_ADDR, false);
    mb.Coil(COIL_PUMP_ADDR, false); 
  }
}

void processControlRemote() {
  hi_speed_cmd = !digitalRead(CTRL_HI_SPD_CMD_PIN);
  // look for change in remote speed selection (switch hi or low)
  if (hi_speed_cmd != hi_speed_cmd_shadow) {
    hi_speed_cmd_shadow = hi_speed_cmd;
    // set default speed
    if (hi_speed_cmd) {
      mb.Hreg(HR_SPEED_SP_ADDR, DEFAULT_REMOTE_HI_SPD);
    } else {
      mb.Hreg(HR_SPEED_SP_ADDR, DEFAULT_REMOTE_LO_SPD);
    }
  }

  pump_cmd = !digitalRead(CTRL_PUMP_CMD_PIN);
  if (pump_cmd != pump_cmd_shadow) {
    pump_cmd_shadow = pump_cmd;
    mb.Coil(COIL_PUMP_ADDR, pump_cmd);
  }
}

void processControl() {
  local_ctrl_active = !digitalRead(CTRL_LOCAL_CMD_PIN);

  // detect change in local/remote mode
  if (local_ctrl_active != local_ctrl_active_shadow) {
    if (local_ctrl_active) {
      Serial.print("Local Control Activated\n");
      mb.Coil(COIL_PUMP_ADDR, true);
    } else {
      Serial.print("Local Control Deactivated\n");
      // set VSD ON
      mb.Coil(COIL_VSD_START_ADDR, true);
      // pump depends on command signal
      mb.Coil(COIL_PUMP_ADDR, !digitalRead(CTRL_PUMP_CMD_PIN));
      // fan speed
      if (!digitalRead(CTRL_HI_SPD_CMD_PIN)) {
        mb.Hreg(HR_SPEED_SP_ADDR, DEFAULT_REMOTE_HI_SPD);    // update MB register with new speed SP
      } else {
        mb.Hreg(HR_SPEED_SP_ADDR, DEFAULT_REMOTE_LO_SPD);    // update MB register with new speed SP
      }
    }
  } 
  
  if (local_ctrl_active) {
    processControlLocal();
  } else {
    processControlRemote();
  }

  // speed setpoint to VSD
  int spd = mb.Hreg(HR_SPEED_SP_ADDR);
  mb.Coil(COIL_VSD_B0_ADDR, spd & 0x01);
  mb.Coil(COIL_VSD_B1_ADDR, spd & 0x02);
  mb.Coil(COIL_VSD_B2_ADDR, spd & 0x04);
  
  local_ctrl_active_shadow = local_ctrl_active;
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
  readAnalogs();
    
  // Temperature update is processed less frequently
  if (loop_count >= TEMP_UPDATE) {
    readTemps();
    loop_count = 0;
  }

  // aircon control functions
  processControl();
  
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
