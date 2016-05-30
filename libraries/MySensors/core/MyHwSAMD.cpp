/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "MyHwSAMD.h"


/*
int8_t pinIntTrigger = 0;
void wakeUp()	 //place to send the interrupts
{
	pinIntTrigger = 1;
}
void wakeUp2()	 //place to send the second interrupts
{
	pinIntTrigger = 2;
}

// Watchdog Timer interrupt service routine. This routine is required
// to allow automatic WDIF and WDIE bit clearance in hardware.
ISR (WDT_vect)
{
	// WDIE & WDIF is cleared in hardware upon entering this ISR
	wdt_disable();
}
*/



void i2c_eeprom_write_byte(unsigned int eeaddress, byte data ) {
  int rdata = data;
  Wire.beginTransmission(I2C_EEP_ADDRESS);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
}

byte i2c_eeprom_read_byte(unsigned int eeaddress ) {
  byte rdata = 0xFF;
  Wire.beginTransmission(I2C_EEP_ADDRESS);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(I2C_EEP_ADDRESS,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

void hwReadConfigBlock(void* buf, void* adr, size_t length)
{
  uint8_t* dst = static_cast<uint8_t*>(buf);
  int offs = reinterpret_cast<int>(adr);
  while (length-- > 0)
  {
    *dst++ = i2c_eeprom_read_byte(offs++); 
  }
}

void hwWriteConfigBlock(void* buf, void* adr, size_t length)
{
  uint8_t* src = static_cast<uint8_t*>(buf);
  int offs = reinterpret_cast<int>(adr);
  while (length-- > 0)
  {
     i2c_eeprom_write_byte(offs++, *src++);
  }
}

uint8_t hwReadConfig(int adr)
{
  uint8_t value;
  hwReadConfigBlock(&value, reinterpret_cast<void*>(adr), 1);
  return value;
}

void hwWriteConfig(int adr, uint8_t value)
{
  uint8_t curr = hwReadConfig(adr);
  if (curr != value)
  {
    hwWriteConfigBlock(&value, reinterpret_cast<void*>(adr), 1);
  }
}

void hwInit() {
  MY_SERIALDEVICE.begin(MY_BAUD_RATE);
  #ifndef MY_GATEWAY_W5100
  while (!MY_SERIALDEVICE) {}
  #endif
  Wire.begin();
}

void hwWatchdogReset() {
  // TODO: Not supported!
}

void hwReboot() {
 // TODO: Not supported!
}

int8_t hwSleep(unsigned long ms) {
  // TODO: Not supported!
  (void)ms;
  return -2;
}

int8_t hwSleep(uint8_t interrupt, uint8_t mode, unsigned long ms) {
  // TODO: Not supported!
  (void)interrupt;
  (void)mode;
  (void)ms;
  return -2;
}

int8_t hwSleep(uint8_t interrupt1, uint8_t mode1, uint8_t interrupt2, uint8_t mode2, unsigned long ms) {
  // TODO: Not supported!
  (void)interrupt1;
  (void)mode1;
  (void)interrupt2;
  (void)mode2;
  (void)ms;
  return -2;
}

uint16_t hwCPUVoltage() {
	// TODO: Not supported!
	return 0;
}
 
uint16_t hwCPUFrequency() {
	// TODO: Not supported!
	return 0;
}
 
uint16_t hwFreeMem() {
	// TODO: Not supported!
	return 0;
}

#ifdef MY_DEBUG
void hwDebugPrint(const char *fmt, ... ) {
  if (MY_SERIALDEVICE) {
	char fmtBuffer[300];
	#ifdef MY_GATEWAY_FEATURE
		// prepend debug message to be handled correctly by controller (C_INTERNAL, I_LOG_MESSAGE)
		snprintf(fmtBuffer, 299, PSTR("0;255;%d;0;%d;"), C_INTERNAL, I_LOG_MESSAGE);
		MY_SERIALDEVICE.print(fmtBuffer);
	#endif
	va_list args;
	va_start (args, fmt );
	va_end (args);
	#ifdef MY_GATEWAY_FEATURE
		// Truncate message if this is gateway node
		vsnprintf(fmtBuffer, 60, fmt, args);
		fmtBuffer[59] = '\n';
		fmtBuffer[60] = '\0';
	#else
		vsnprintf(fmtBuffer, 299, fmt, args);
	#endif
	va_end (args);
	MY_SERIALDEVICE.print(fmtBuffer);
//	MY_SERIALDEVICE.flush();

	//MY_SERIALDEVICE.write(freeRam());
    }
}
#endif
