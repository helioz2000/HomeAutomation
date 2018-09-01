/*
 * Home Automation
 * Modbus on HC12 wireless
 * DS18B20 temperature sensor(s)
 * 
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
 * D13
 * 
 * 
 */

#include "Modbus.h"
#include "ModbusSerial.h"

// libraries:
#include <OneWire.h> 
#include <DallasTemperature.h>

const int HR_TEMP_BASE = 100;       // Holding Register base address for temperatures
const int COIL_BASE_ADDR = 0;       // Coil address for local outputs (coils)
const uint8_t COIL_PINS[] = { 10, 11, 12 };   //Output pins for modbus coils
const bool COIL_DEFAULTS[] = { false, false, false };     //Default state for modbus coil pins
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

SoftwareSerial HC12(HC12TxdPin,HC12RxdPin); // Software serial for HC12 module
ModbusSerial mb;                          

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);

uint8_t num_temp_sensors = 0;       // number of connected temperature sensors
int loop_count = 0;
unsigned long LED_off_time = 0;

void setup() {
  // LED setup
  pinMode(LED_BUILTIN, OUTPUT);      
  digitalWrite(LED_BUILTIN, LED_OFF);

  // Set HC12 set mode off
  pinMode(HC12_set_pin, OUTPUT);
  digitalWrite(HC12_set_pin, HIGH);
  
  // start serial port 
  Serial.begin(9600);     // for debug output only
  Serial.print("Modbus Slave #");
  Serial.print(MODBUS_ADDRESS);
  Serial.println(" - Temperature Sensor\n");

  // Digital Output config
  for (int i=0; i<sizeof(COIL_PINS); i++) {
    pinMode(COIL_PINS[i], OUTPUT);
    mb.addCoil(COIL_BASE_ADDR + i, COIL_DEFAULTS[i]);
  }
  Serial.print("\n");

  // modbus config
  mb.config (&HC12, MODBUS_BAUD);  
  mb.setSlaveId (MODBUS_ADDRESS);

  // temperature sensors
  tempSensors.begin();
  num_temp_sensors = tempSensors.getDeviceCount();
  for(int i=0; i<num_temp_sensors; i++) {
    mb.addHreg (HR_TEMP_BASE + i, 0);
  }
  if (num_temp_sensors < 1) {
    Serial.println("Error - No Temperature Sensor found!\n");
    mb.addHreg (HR_TEMP_BASE, 0);
  }
}

void readTemps() {
  int i, regValue;
  float temp;
  tempSensors.requestTemperatures();
  for(i=0; i<num_temp_sensors; i++) {
    temp = tempSensors.getTempCByIndex(i);
    regValue = (int) (temp *100.0);
    mb.Hreg(HR_TEMP_BASE, regValue);
    Serial.print("Temperature");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(regValue);
    Serial.print("\n");
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
