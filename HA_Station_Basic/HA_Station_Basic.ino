/*
 * Home Automation
 * Modbus on HC12 wireless
 * 3 x Digital Input
 * 3 x Digital Output
 * DS18B20 temperature sensor(s)
 * Si7021 Temp & Humidity Sensor
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
 * 110 = Si7021 Temperature
 * 111 = Si7021 Humidity
 * 
 */

#include "Modbus.h"
#include "ModbusSerial.h"
#include "Si7021.h"

// libraries:
#include <OneWire.h> 
#include <DallasTemperature.h>

const int HR_SI7021_BASE_ADDR = 110;     // Holding Register base address
const int HR_DS18B20_BASE_ADDR = 100;    // Holding Register base address
const int COIL_BASE_ADDR = 0;       // Coil address for modbus outputs (coils)
const uint8_t COIL_PINS[] = { 10, 11, 12 };   //Output pins for modbus coils
const bool COIL_DEFAULTS[] = { false, false, false };  //Default state for modbus coil pins
const int INPUT_BASE_ADDR = 0;      // Input address for modbus inputs
const uint8_t INPUT_PINS[] = { 7, 8, 9 };   //Input pins for modbus inputs
const long MODBUS_BAUD = 1200;      // Baudrate for Modbus comms
const byte MODBUS_ADDRESS = 10;     // Modbus Slave address

const unsigned long MODBUS_TASK_DELAY = 200;  // run modbus task every X ms
const int TEMP_UPDATE = 40;         // process temp update every X * MODBUS_TASK_DELAY

const byte HC12RxdPin = 4;          // RX Pin on HC12
const byte HC12TxdPin = 5;          // TX Pin on HC12
const byte HC12_set_pin = 6;          // SET Pin on HC12

const uint8_t ONE_WIRE_BUS = 2;     // D2 is used for one wire bus

const long LED_ON_TIME = 500;       // [ms] LED on modbus activity

#define LED_ON HIGH
#define LED_OFF LOW

#define SI7021    //Si7021 Temperature & Humidity

#ifdef SI7021
Si7021 th_sensor = Si7021();
#endif
SoftwareSerial HC12(HC12TxdPin,HC12RxdPin); // Software serial for HC12 module
ModbusSerial mb;                          

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);

uint8_t num_temp_sensors = 0;       // number of connected temperature sensors
int loop_count = 0;
unsigned long LED_off_time = 0;

void setup() {
  int i;
  // start serial port 
  Serial.begin(9600);     // for debug output only
  
#ifdef SI7021
  if (!th_sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
  }
#endif
  
  // LED setup
  pinMode(LED_BUILTIN, OUTPUT);      
  digitalWrite(LED_BUILTIN, LED_OFF);

  // Set HC12 set mode off
  pinMode(HC12_set_pin, OUTPUT);
  digitalWrite(HC12_set_pin, HIGH);
  
  
  Serial.print("Modbus Slave #");
  Serial.print(MODBUS_ADDRESS);
  Serial.println(" - Temperature Sensor\n");

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
  mb.addHreg (HR_SI7021_BASE_ADDR, 0);
  mb.addHreg (HR_SI7021_BASE_ADDR + 1, 0);
}

void readTemps() {
  int i, regValue;
  float temp;
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
#ifdef SI7021
  regValue = (int) (th_sensor.readTemperature() * 100.0);
  mb.Hreg(HR_SI7021_BASE_ADDR, regValue);
  Serial.print("Si7021 Temperature: "); Serial.print(regValue);
  regValue = (int) (th_sensor.readHumidity() * 100.0);
  mb.Hreg(HR_SI7021_BASE_ADDR + 1, regValue);
  Serial.print("\tHumidity: "); Serial.println(regValue);
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
