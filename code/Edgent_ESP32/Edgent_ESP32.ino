/*************************************************************
  Blynk is a platform with iOS and Android apps to control
  ESP32, Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build mobile and web interfaces for any
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: https://www.blynk.io
    Sketch generator:           https://examples.blynk.cc
    Blynk community:            https://community.blynk.cc
    Follow us:                  https://www.fb.com/blynkapp
                                https://twitter.com/blynk_app

  Blynk library is licensed under MIT license
 *************************************************************
  Blynk.Edgent implements:
  - Blynk.Inject - Dynamic WiFi credentials provisioning
  - Blynk.Air    - Over The Air firmware updates
  - Device state indication using a physical LED
  - Credentials reset using a physical Button
 *************************************************************/

/* Fill in information from your Blynk Template here */
/* Read more: https://bit.ly/BlynkInject */

#define BLYNK_TEMPLATE_ID "TMPL2DtaWZOlt"
#define BLYNK_TEMPLATE_NAME "inverterCopy"
#define BLYNK_FIRMWARE_VERSION "0.1.2"
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define BLYNK_PRINT Serial
#include <PZEM004Tv30.h>
//#define BLYNK_DEBUG

#define APP_DEBUG



#include "BlynkEdgent.h"
#define ONE_WIRE_BUS 16
#define TEMPERATURE_PRECISION 9
#define COOLING_PIN 17
#define VPIN_batTemp V7
#define VPIN_setpoint V8
#define VPIN_hysteresis V9
#define VPIN_coolerbtn V10
#define VPIN_bCal V11
#define EEPROM_SIZE 512
#define SETPOINT_ADDR 0
#define HYSTERESIS_ADDR 10
int count = 0;

float R1 = 10000.0;
float R2 = 824.0;
int countW=0;


// Global Variables

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 3000;
bool coolerState = LOW;
float setpoint, hysteresis, currentTemp, bCal;
#if defined(ESP32)
PZEM004Tv30 pzem(Serial2, 26, 25);
#else
PZEM004Tv30 pzem(Serial2);
#endif

// Blynk Callbacks
BLYNK_WRITE(VPIN_setpoint) {
  setpoint = param.asFloat();
  EEPROM.put(SETPOINT_ADDR, setpoint);
  EEPROM.commit();
}

BLYNK_WRITE(VPIN_hysteresis) {
  hysteresis = param.asFloat();
  EEPROM.put(HYSTERESIS_ADDR, hysteresis);
  EEPROM.commit();
}

BLYNK_WRITE(VPIN_bCal) {
  bCal = param.asFloat();
  EEPROM.put(20, bCal);
  EEPROM.commit();
}
// Function to handle temperature-based cooling
void handleCooling() {
  sensors.requestTemperatures();
  float tempC1 = sensors.getTempC(insideThermometer);

  if (tempC1 > setpoint + hysteresis) {
    coolerState = HIGH;
  } else {
    coolerState = LOW;
  }
  digitalWrite(COOLING_PIN, coolerState);
  Blynk.virtualWrite(VPIN_coolerbtn, coolerState);
}

// Function to handle ADC readings and averaging
float getAverageADCReading() {
  const unsigned int samples = 50;
  float sum = 0.0;

  for (unsigned int y = 0; y < samples; y++) {
    float batteryVoltage = (analogRead(35) * 3.3) / 4095;
    batteryVoltage = batteryVoltage * ((R1 + bCal) / bCal);
    sum += batteryVoltage;
  }
  return sum / samples;
}



void setup() {
  Serial.begin(115200);
  delay(100);
  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(9600);
  pinMode(COOLING_PIN, OUTPUT);
  pinMode(4, INPUT_PULLUP);
if(digitalRead(4)==LOW){
  enterResetConfig();
}
  digitalWrite(COOLING_PIN, LOW);

 //enterResetConfig();
  // wm.resetSettings();
  sensors.begin();
  if (!sensors.getAddress(insideThermometer, 0)) {
    Serial.println("Unable to find address for Device 0");
  }


  EEPROM.get(SETPOINT_ADDR, setpoint);
  EEPROM.get(HYSTERESIS_ADDR, hysteresis);
  EEPROM.get(20, bCal);

  BlynkEdgent.begin();
}

void loop() {
  BlynkEdgent.run();
if (digitalRead(4) ==LOW){
  countW++;
}
if (countW>5){
  enterResetConfig();
}
  handleCooling();
  float dcVoltage = getAverageADCReading();
  Serial.println(dcVoltage);

  // Read the data from the sensor
  float acVoltage = pzem.voltage();
  float current = pzem.current();
  float power = pzem.power();
  float energy = pzem.energy();
  float frequency = pzem.frequency();
  float pf = pzem.pf();

  // Check if the data is valid
  if (isnan(acVoltage)) {
    acVoltage = 0;
  } else if (isnan(current)) {
    current = 0;
  } else if (isnan(power)) {
    power = 0;
  } else if (isnan(energy)) {
    energy = 0;
  } else if (isnan(frequency)) {
    frequency = 0;
  } else if (isnan(pf)) {
    pf = 0;
  } else {

    // Print the values to the Serial console
    Serial.print("Voltage: ");
    Serial.print(acVoltage);
    Serial.println("V");
    Serial.print("Current: ");
    Serial.print(current);
    Serial.println("A");
    Serial.print("Power: ");
    Serial.print(power);
    Serial.println("W");
    Serial.print("Energy: ");
    Serial.print(energy, 3);
    Serial.println("kWh");
    Serial.print("Frequency: ");
    Serial.print(frequency, 1);
    Serial.println("Hz");
    Serial.print("PF: ");
    Serial.println(pf);
  }

  Serial.println(sensors.getTempC(insideThermometer));
  

  Blynk.virtualWrite(V0, dcVoltage);
  Blynk.virtualWrite(V1, acVoltage);
  Blynk.virtualWrite(V2, current);
  Blynk.virtualWrite(V3, power);
  Blynk.virtualWrite(V4, energy);
  Blynk.virtualWrite(V5, frequency);
  Blynk.virtualWrite(V6, pf);
  Blynk.virtualWrite(VPIN_batTemp, sensors.getTempC(insideThermometer));
  Blynk.syncVirtual(VPIN_setpoint);
  Blynk.syncVirtual(VPIN_hysteresis);
  Blynk.syncVirtual(VPIN_bCal);
}
