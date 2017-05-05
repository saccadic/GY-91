#ifndef __ESP32TOOLS
#define __ESP32TOOLS

#include "Arduino.h"

#ifdef __cplusplus
#define _EXTERN_ATTRIB extern "C"
#else
#define _EXTERN_ATTRIB
#endif

#ifdef SERIAL_OUTPUT_DEVICE

extern HardwareSerial Serial;
#define log_i       SERIAL_OUTPUT_DEVICE.println
#define log_e		SERIAL_OUTPUT_DEVICE.println

#else // No Debug Output

#define log_i(...) do { } while (0)
#define log_e(...) do { } while (0)

#endif

#define __no_operation() __asm__("nop\n\t") // emit AVR no-op 

#ifdef __cplusplus
#define _EXTERN_ATTRIB extern "C"
#else
#define _EXTERN_ATTRIB
#endif

#define get_ms 		esp32_get_ms
#define delay_ms    delay

_EXTERN_ATTRIB void esp32_get_ms(unsigned long *count);
_EXTERN_ATTRIB int esp32_i2c_write(uint8_t address, uint8_t subAddress, uint8_t length, uint8_t const *data);
_EXTERN_ATTRIB int esp32_i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);

#endif