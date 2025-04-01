#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "LoRaWan_APP.h"

// BME280 pins
#define SDA_PIN 42
#define SCL_PIN 41

TwoWire sensorI2CBus = TwoWire(1);
Adafruit_BME280 bme;
bool bmeFound = false;

// OTAA parameters
uint8_t devEui[] = { "devEui"};
uint8_t appEui[] = { "appEui" };
uint8_t appKey[] = { "appKey" };

/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_A;
uint32_t appTxDutyCycle = 60000;  // 1 minute
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = true;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

float temperature = 0.0;
float pressure = 0.0;
float humidity = 0.0;

static void prepareTxFrame(uint8_t port) {
  int temp = (int)((temperature * 100)) + 5000;
  int pres = (int)(pressure * 10);
  byte hum = (int)(humidity * 2);
  
  appDataSize = 8;
  appData[0] = temp >> 8;
  appData[1] = temp & 0xFF;
  appData[2] = pres >> 8;
  appData[3] = pres & 0xFF;
  appData[4] = hum;
  appData[5] = 0;  // wind speed (not used)
  appData[6] = 0;  // wind speed (not used)
  appData[7] = 0;  // sunlight (not used)
}

void setup() {
  Serial.begin(115200);
  
  sensorI2CBus.begin(SDA_PIN, SCL_PIN, 400000);
  
  if (bme.begin(0x76, &sensorI2CBus) || bme.begin(0x77, &sensorI2CBus)) {
    Serial.println("BME280 sensor found!");
    bmeFound = true;
  } else {
    Serial.println("Could not find BME280 sensor!");
  }
  
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
}

void readSensorData() {
  if (bmeFound) {
    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    humidity = bme.readHumidity();
    
    Serial.printf("Temp: %.1f°C, Pressure: %.1f hPa, Humidity: %.1f%%\n", 
                 temperature, pressure, humidity);
  }
}

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3);
      deviceState = DEVICE_STATE_JOIN;
      break;

    case DEVICE_STATE_JOIN:
      LoRaWAN.join();
      break;

    case DEVICE_STATE_SEND:
      readSensorData();
      prepareTxFrame(appPort);
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;

    case DEVICE_STATE_CYCLE:
      txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;

    case DEVICE_STATE_SLEEP:
      LoRaWAN.sleep(loraWanClass);
      break;

    default:
      deviceState = DEVICE_STATE_INIT;
      break;
  }
}