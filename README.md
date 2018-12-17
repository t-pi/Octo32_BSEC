# Octo32_BSEC
Octopus32 sketch (Arduino) measuring temp, humidity, pressure and air quality with Bosch Sensortec BSEC library

Octopus32 Board with ESP32/BME680 runs Bosch Sensortec BSEC for AQI and pushes data to The Things Network via LoRaWAN.

Octopus Board, s. here: https://www.tindie.com/products/FabLab/iot-octopus-badge-for-iot-evaluation/#, Octopus32 is the newest version (as of 12-2018) using a ESP32 chip and including a LoRa radio on the backside.

The BSEC library is (c) by Robert Bosch GmbH / Bosch Sensortec GmbH and is available here: https://www.bosch-sensortec.com/bst/products/all_products/bsec

Tested with BSEC_1.4.7.1_Generic_Release_20180907

More informations on The Things Network: https://www.thethingsnetwork.org/

Getting your devices into TTN: https://www.thethingsnetwork.org/docs/devices/

# Installation

- Arduino 1.8.8 portable installation

- Follow BoschSensortec's instructions: https://github.com/BoschSensortec/BSEC-Arduino-library#3-replace-the-arduino-builder

- arduino-builder has to be swapped despite the newer Arduino version

- String to change in ...\Arduino188p\portable\packages\esp32\hardware\esp32\1.0.0\platform.txt:

~~~~
## Combine gc-sections, archives, and objects
recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" {compiler.c.elf.flags} -o "{build.path}/{build.project_name}.elf" -Wl,--start-group {object_files} "{build.path}/arduino.ar" {compiler.c.elf.libs} {compiler.c.elf.extra_flags} -Wl,--end-group "-L{build.path}"
~~~~

- No modification of linker needed for ESP32

- Add the keys from your TTN app (TTN console) to the sketch / in a suitable access_keys.h. An example file is provided.

# Payload function for TTN console
~~~~
function Decoder(bytes, port) {

var temp = ((bytes[1] << 8) + bytes[0]);
var hum = ((bytes[3] << 8) + bytes[2]);
var press = ((bytes[5] << 8) + bytes[4]);
var iaq = ((bytes[7] << 8) + bytes[6]);
var iaq_stat = ((bytes[9] << 8) + bytes[8]);
var accuracy = ((bytes[11] << 8) + bytes[10]);


return {
temp: (temp - 27315)/100.0,
hum: hum/100,
press: press,
iaq: iaq/10.0,
iaq_stat: iaq_stat/10.0,
accuracy: accuracy
};

}
~~~~

# Output
* temp: temperature in degree Celsius, an offset of 5°C for self-heating is set and can be changed in the code
* hum: relative humidty in percent
* press: ambient air pressure in mbar
* iaq_stat: indoor air quality index, typically in the range between 25 (best air of last 4 days) and 250 (worst air of 4 days), can peak up to 500 for very bad air and get to 0 for very fresh air.
* iaq: indoor air quality index, typically in a comparable range starting from 25 for fresh air, but not scaled to 250 for best air.
* accuracy: accuracy of air quality index. 0 for little variations of IAQ so far, increases up to 3 with more dynamic in the measured signal (i.e. breathing on the sensor and venting the room after a while should help increase accuracy. The sensor needs also some time for warm-up after restart)

# Notes
* No storage of state so far, if node is restarted, baseline of BSEC IAQ restarts from scratch.
* No threading yet, so every time a packet is sent the sensor idles longer than the 3 seconds the BSEC algorithm would need.
