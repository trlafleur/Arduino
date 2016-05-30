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

#include "MyConfig.h"
#include "MyTransport.h"
#include <stdint.h>
#include "drivers/RFM69/RFM69.h"
#include "drivers/RFM69/RFM69_ATC.h"


//#ifdef MY_RFM69_Enable_ATC
//RFM69_ATC 	_radio(MY_RF69_SPI_CS, MY_RF69_IRQ_PIN, MY_RFM69HW, MY_RF69_IRQ_NUM);
//#else
RFM69 		_radio(MY_RF69_SPI_CS, MY_RF69_IRQ_PIN, MY_RFM69HW, MY_RF69_IRQ_NUM);
//#endif

uint8_t _address;




/* A function to print a buffer in hex format
 *
 *  hexdump (buffer, length, 16);
 */
#include <stdio.h>
#include <stdlib.h>

 void hexdump(unsigned char *buffer, unsigned long index, unsigned long width)
 {
    unsigned long i;
    for (i=0; i<index; i++)
    {
      debug1(PSTR("%02x "), buffer[i]);       //printf("%02x ",buffer[i]);
    }
    
    for (unsigned long spacer=index;spacer<width;spacer++)
      debug1(PSTR("  "));                    //printf("  ");
      debug1(PSTR(": "));                    //printf(": ");
      for (i=0; i<index; i++)
      {
        if (buffer[i] < 32) debug1(PSTR("."));   //printf(".");
        else debug1(PSTR("%c"), buffer[i]);      //printf("%c",buffer[i]);
      }
    debug1(PSTR("\n"));                      //printf("\n");
 }





bool transportInit() 
{
	// Start up the radio library (_address will be set later by the MySensors library)
	if (_radio.initialize(MY_RFM69_FREQUENCY, _address, MY_RFM69_NETWORKID)) 
		{
		#ifdef MY_RFM69_ENABLE_ENCRYPTION
			uint8_t _psk[16];
			hwReadConfigBlock((void*)_psk, (void*)EEPROM_RF_ENCRYPTION_AES_KEY_ADDRESS, 16);
			_radio.encrypt((const char*)_psk);
			memset(_psk, 0, 16); // Make sure it is purged from memory when set
		#endif
		return true;
		}
	return false;
}

void transportSetAddress(uint8_t address) 
{
	_address = address;
	_radio.setAddress(address);
}

uint8_t transportGetAddress() 
{
	return _address;
}

bool transportSend(uint8_t to, const void* data, uint8_t len) 
{
	
	debug1(PSTR("\n *** Hex dump from transportSend \n"));
	debug1(PSTR(" *** TO: %d Len: %d \n"), to, len);
	hexdump ((uint8_t*)data, (unsigned long) len, 16);
	debug1(PSTR("\n"));
	
	
	
	return _radio.sendWithRetry(to,data,len,5);
}

bool transportAvailable(uint8_t *to) 
{
	if (_radio.TARGETID == BROADCAST_ADDRESS)
		*to = BROADCAST_ADDRESS;
	else
		*to = _address;
	return _radio.receiveDone();
}

uint8_t transportReceive(void* data) 
{
	memcpy(data,(const void *)_radio.DATA, _radio.DATALEN);
	
    debug1(PSTR("\n *** Hex dump from transportReceive \n"));
    debug1(PSTR(" *** From: %d Len: %d \n"), _radio.SENDERID, _radio.DATALEN);
	hexdump ( (unsigned char*) & _radio.DATA , (unsigned long)  _radio.DATALEN, 16);
	//hexdump ((unsigned char*) data , (unsigned long) sizeof (data), 16);
	debug1(PSTR("\n"));	
	
	
	// Send ack back if this message wasn't a broadcast
	if (_radio.TARGETID != RF69_BROADCAST_ADDR)
		_radio.ACKRequested();
    _radio.sendACK();
    
	return _radio.DATALEN;
}	

void transportPowerDown() 
{
	_radio.sleep();
}
