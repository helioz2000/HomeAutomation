/*
 * Home Automation
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

// Comment out definitions below for sensors whcih are not fitted
//#define DS18B20       //DS18B20 Temperature Sensors
//#define SI7021        //Si7021 Temperature & Humidity
//#define MCP9808_NUM 1 //Number of MCP9808 Temp sensors

#include "Modbus.h"
#include "ModbusSerial.h"

// libraries:
#ifdef DS18B20
#include <OneWire.h> 
#include <DallasTemperature.h>
#endif

const byte MODBUS_ADDRESS = 12;     // Modbus Slave address - adjust for each module

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

#define LED_ON HIGH
#define LED_OFF LOW

#ifdef SI7021
#include "Si7021.h"
Si7021 th_sensor = Si7021();
#endif

#ifdef MCP9808_NUM
#include "MCP9808.h"
MCP9808 t_sensor = MCP9808();
MCP9808 tmp_sensor[MCP9808_NUM];
#endif

SoftwareSerial HC12(HC12TxdPin,HC12RxdPin); // Software serial for HC12 module
ModbusSerial mb;                          

#ifdef DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);
#endif

uint8_t num_temp_sensors = 0;       // number of connected temperature sensors
int loop_count = 0;
unsigned long LED_off_time = 0;

void setup() {
  int i;
  // start serial port for debug
  Serial.begin(9600);     // for debug output only
  
#ifdef SI7021
  if (!th_sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
  } else {
    mb.addHreg (HR_SI7021_BASE_ADDR, 0);
    mb.addHreg (HR_SI7021_BASE_ADDR + 1, 0);
  }
#endif

#ifdef MCP9808_NUM
  for(i=0; i < MCP9808_NUM ;i++) {
    tmp_sensor[i] = MCP9808();
    if (! tmp_sensor[i].begin(MCP9808_I2CADDR_DEFAULT + i) ) {
      Serial.print("Couldn't find MCP9808 #");
      Serial.println(i);
    }
  }
#endif
  
  // LED setup
  pinMode(LED_BUILTIN, OUTPUT);      
  digitalWrite(LED_BUILTIN, LED_OFF);

  // Set HC12 set mode off
  pinMode(HC12_set_pin, OUTPUT);
  digitalWrite(HC12_set_pin, HIGH);
  
  Serial.print("\nModbus Slave #");
  Serial.print(MODBUS_ADDRESS);
  //Serial.println(" - Temperature Sensor\n");

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
  
  // modbus config
  mb.config (&HC12, MODBUS_BAUD);  
  mb.setSlaveId (MODBUS_ADDRESS);

#ifdef DS18B20
  // temperature sensors
  tempSensors.begin();
  num_temp_sensors = tempSensors.getDeviceCount();
  for(int i=0; i<num_temp_sensors; i++) {
    mb.addHreg (HR_DS18B20_BASE_ADDR + i, 0);
  }
  if (num_temp_sensors < 1) {
    Serial.println("Error - No Temperature Sensor found!\n");
    mb.addHreg (HR_DS18B20_BASE_ADDR, 0);
  }
#endif
  
}

void readTemps() {
  int i, regValue;
  float temp;
#ifdef DS18B20
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
#endif
  
#ifdef SI7021
  regValue = (int) (th_sensor.readTemperature() * 100.0);
  mb.Hreg(HR_SI7021_BASE_ADDR, regValue);
  Serial.print("Si7021 Temperature: "); Serial.print(regValue);
  regValue = (int) (th_sensor.readHumidity() * 100.0);
  mb.Hreg(HR_SI7021_BASE_ADDR + 1, regValue);
  Serial.print("\tHumidity: "); Serial.println(regValue);
#endif

#ifdef MCP9808_NUM
  for(i=0; i < MCP9808_NUM ;i++) {
    regValue = (int) (tmp_sensor[i].readTempC() * 100.0);
    mb.Hreg(HR_MCP9808_BASE_ADDR + i, regValue);
    Serial.print("MCP9808 Temperature[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(regValue);
    Serial.print("\n");
  }
#endif  
 
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
}
