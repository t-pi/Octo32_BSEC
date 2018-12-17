/*
 * Sketch: octo32_ttn_bsec01
 * Basic sketch for Octopus32 by G. Burger, 2018
 * Initial code
 * 2018-12 G. Burger: Blinks Neopixels, scans I2C bus and connects to TTN network
 * Changelog
 * 2018-12b t-pi: Bosch Sensortec BME680 + BSEC
 * 2018-12a t-pi: Neopixels disabled, as LoRa was not working correctly
 */
#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "bsec.h"

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);

// Create an object of the class Bsec
Bsec iaqSensor;

String output; // Serial.println - buffer

// Adafruit_NeoPixel pixels = Adafruit_NeoPixel(2,21,NEO_GRBW + NEO_KHZ800);
// --> 2018-12 not working with LoRa

// LoraWAN Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
// https://github.com/matthijskooijman/arduino-lmic
// -------- LoRa PinMapping FeatherWing Octopus
const lmic_pinmap lmic_pins = {  
  .nss = 14,                            // Connected to pin D
  .rxtx = LMIC_UNUSED_PIN,             // For placeholder only, Do not connected on RFM92/RFM95
  .rst = LMIC_UNUSED_PIN,              // Needed on RFM92/RFM95? (probably not) D0/GPIO16 
  .dio = {
    33, 33, LMIC_UNUSED_PIN         }
};

#include "access_keys.h"
// include file for The Things Network keys
//static const u1_t PROGMEM DEVEUI[8]= { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }; // lsb
//void os_getDevEui (u1_t* buf) { 
//  memcpy_P(buf, DEVEUI, 8);
//}
//
//static const u1_t PROGMEM APPEUI[8]={ 0x00,0x00,0x00,0x00,0x00,0x00,0xB3,0x70 }; // lsb
//void os_getArtEui (u1_t* buf) { 
//  memcpy_P(buf, APPEUI, 8);
//}
//
//static const u1_t PROGMEM APPKEY[16]={ 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }; // MSB !!
//void os_getDevKey (u1_t* buf) {  
//  memcpy_P(buf, APPKEY, 16);
//};
//
volatile int LoRaWAN_Tx_Ready      = 0; // Merker für ACK 

int LoRaWAN_Rx_Payload = 0 ;
// -------- LoRa Event 
void onEvent (ev_t ev) { 
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    // Disable link check validation (automatically enabled
    // during join, but not supported by TTN at this time).
    LMIC_setLinkCheckMode(0);
    break;
  case EV_RFU1:
    Serial.println(F("EV_RFU1"));
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen) {
      Serial.println(F("Received "));
      Serial.println(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
      LoRaWAN_Rx_Payload = 0; 
      for (int i = 0;i<LMIC.dataLen;i++) { 
        Serial.println(LMIC.frame[i+ LMIC.dataBeg],HEX);
        LoRaWAN_Rx_Payload = 256*LoRaWAN_Rx_Payload+LMIC.frame[i+ LMIC.dataBeg];
      }
    }
    LoRaWAN_Tx_Ready = 1;
    // Schedule next transmission
    //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  default:
    Serial.println(F("Unknown event"));
    break;
  }
}


///////////////////////////////
//////// SETUP
///////////////////////////////
void setup(){

  Serial.begin(115200);
  int i = 0;
  for (i=0;i<10;i++)
    errLeds(); // Test-Blink
  
//////////////////////// NeoPixel
// --> 2018-12 interferes with LoRa, disabled
//  pixels.begin();//-------------- Initialisierung Neopixel
//  delay(1);
//  pixels.show();
//  pixels.setPixelColor(0,0,0,0); // alle aus
//  pixels.setPixelColor(1,0,0,0);
//  pixels.show();                 // und anzeigen

  ///////////////////// LoraWAN 
  os_init();             // LMIC LoraWAN
  LMIC_reset();          // Reset the MAC state 
  LMIC.txpow = 27;       // Maximum TX power 
  LMIC.datarate=DR_SF12; // Long Range
  LMIC.rps = updr2rps(LMIC.datarate);

  //////////////////// BME680 - BSEC
  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  iaqSensor.setTemperatureOffset(5.0); // set temperature correction for BSEC algorithm. Positive value --> lower temperature
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);

}

unsigned long last_time = millis();

///////////////////////////////
//////// LOOP
///////////////////////////////
void loop() {

  //////////////////// BME680 - BSEC
  unsigned long time_trigger = millis();
  if (iaqSensor.run()) { // If new data is available
    output = String(time_trigger);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance);
    output += ", " + String(iaqSensor.iaqEstimate);
    output += ", " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.co2Equivalent);
    output += ", " + String(iaqSensor.breathVocEquivalent);
    Serial.println(output);
  } else {
    checkIaqSensorStatus();
  }

  uint16_t t_value, p_value, h_value, i_value, is_value, ac_value;

  //////////////////// LoRaWAN TTN
  if (time_trigger>last_time+60000)
  { //Block------------------------------ sende Data to TTN  
    // getting sensor values
    p_value = iaqSensor.pressure / 100;
    i_value = iaqSensor.iaqEstimate * 10;
    ac_value = iaqSensor.iaqAccuracy;
    t_value = iaqSensor.temperature*100 + 27315;
    h_value = iaqSensor.humidity*100;
    is_value = iaqSensor.staticIaq * 10;
    
    last_time = time_trigger;
    int port = 10;
    static uint8_t mydata[12];
    mydata[0] = t_value&0xFF; // lower byte
    mydata[1] = t_value>>8; // higher byte
    mydata[2] = h_value&0xFF; // lower byte
    mydata[3] = h_value>>8; // higher byte
    mydata[4] = p_value&0xFF; // lower byte
    mydata[5] = p_value>>8; // higher byte
    mydata[6] = i_value&0xFF; // lower byte
    mydata[7] = i_value>>8; // higher byte
    mydata[8] = is_value&0xFF; // lower byte
    mydata[9] = is_value>>8; // higher byte
    mydata[10] = ac_value&0xFF; // lower byte
    mydata[11] = ac_value>>8; // higher byte
    // Check if there is not a current TX/RX job running
    //if (LMIC.opmode & OP_TXRXPEND) {
    if (LMIC.opmode & (1 << 7)) { 
      Serial.println(F("OP_TXRXPEND, not sending"));
    } 
    else {
      // Prepare upstream data transmission at the next possible time.
      LoRaWAN_Tx_Ready = 0;                                 // Merker für ACK
      LMIC_setTxData2(port, mydata, sizeof(mydata)-1, 0);   // Sende         
      Serial.println(F("Packet queued"));
      while(LoRaWAN_Tx_Ready==0) {
        yield();
        os_runloop_once();
      };  // Warte bis gesendet
    }
  } // Blockende
}


///////////////////////////////////////////
///////// Helper function definitions
///////////////////////////////////////////
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}
