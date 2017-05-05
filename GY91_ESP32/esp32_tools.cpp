#include "esp32_tools.h"
#include <Wire.h>

_EXTERN_ATTRIB void esp32_get_ms(unsigned long *count) {
	*count = millis();
}


_EXTERN_ATTRIB int esp32_i2c_write(uint8_t address, uint8_t subAddress, uint8_t length, uint8_t const *data) {
	Wire.beginTransmission(address);
	Wire.write(subAddress); // send address
	for (uint8_t i = 0; i < length; i++) {
		Wire.write((uint8_t)data[i]);
	}
	Wire.endTransmission();
	return 0;
}

_EXTERN_ATTRIB int esp32_i2c_read(uint8_t address, uint8_t subAddress, uint8_t length, uint8_t *data) {

	uint16_t timeout = 3000;	 // Default
	int8_t count = 0;
	uint32_t t1 = millis();

	Wire.beginTransmission(address);
	Wire.write(subAddress);
	Wire.endTransmission(true);
	Wire.requestFrom(address, (size_t)length);
	while (Wire.available()) {
		data[count++] = Wire.read();
	}
	return 0;
}