/**
 *
 *	The is the development version of a test transport for the RadioHead 
 *  series of radio drivers. Tested with RH_RF69, RH_RF95 radios
 *	For use with MySensor 2.0
 *
 *	It requires some minor changes to the RadioHead drivers:
 *		To removed the 4 byte header sent and received by the RH drivers
 *		
 *		An #define noHeader was added in the "driver".h file
 *
 *		// This is use for compatable with LowPowerLab and MySensor drivers
 *		// It removed the 4 byte header used in the RH drivers
 *		#define noHeader
 *
 *
 *		For 100% register compatable with the MySensor RFM_69 driver, we need to make a
 *		change in the RH_RF69.cpp MODEM_CONFIG_TABLE for the FSK_Rb55555Fd50 to set CONFIG_NOWHITE 
 *
 * //   { CONFIG_FSK,  0x02, 0x40, 0x03, 0x33, 0x42, 0x42, CONFIG_WHITE}, // FSK_Rb55555Fd50 
 *  	{ CONFIG_FSK,  0x02, 0x40, 0x03, 0x33, 0x42, 0xf4, CONFIG_NOWHITE}, // FSK_Rb55555Fd50
 *
 *		A note:
 *		The RH driver require setting the frequency in MHz and the LowPowerLab requires it in Hz
 *
 *		Items NOT yet Tested:
 *			Encryption
 *			sleep
 *			repeater function
 *			Tested ONLY with Moteino and MoteinoMega
 *
 *  31 May 2016		TRL		First working version with RH_RF69
 *
 *
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

uint8_t _address = MY_NODE_ID;		// this is my node's address

// This define's the network ID word for this network
static const uint8_t _NID[] = {0x2d, (const uint8_t) MY_RFM69_NETWORKID};

// This define's the radio class
#if	defined MY_RADIO_RH_RF69
#include "RH_RF69.h"
RH_RF69					_radio(MY_RF69_SPI_CS, MY_RF69_IRQ_PIN);
#elif  defined MY_RADIO_RH_RF95
#include "RH_RF95.h"
RH_RF95					_radio(MY_RF69_SPI_CS, MY_RF69_IRQ_PIN);
#else
#error Radio is not defined in MyTransportRadioHead.cpp
#endif		


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


/* ********************************************************** */
bool transportInit() 
{
	if (_radio.init())
	{

//		#ifdef MY_RFM69_ENABLE_ENCRYPTION
//			uint8_t _psk[16];
//			hwReadConfigBlock((void*)_psk, (void*)EEPROM_RF_ENCRYPTION_AES_KEY_ADDRESS, 16);
//			_radio.encrypt((const char*)_psk);
//			memset(_psk, 0, 16); // Make sure it is purged from memory when set
//		#endif

#if 	defined MY_RADIO_RH_RF69		
		if (!_radio.setFrequency(915.0))						debug(PSTR(" ** setFrequency failed **\n") );
		if (!_radio.setModemConfig( _radio.FSK_Rb55555Fd50 ))	debug(PSTR(" ** setModemConfig failed **\n") );
			
		_radio.setEncryptionKey(NULL);		// no for now
		_radio.setSyncWords(_NID,2);		// this sets the unique network ID
		_radio.setTxPower(20);				// 14 to 20 is the range for the RFM69HW (HCW) 
		_radio.setPreambleLength (3);		// for compatable with  LowPowerLab stack
		_address = MY_NODE_ID;				// this is my node's address		
		_radio.spiWrite(RH_RF69_REG_39_NODEADRS, MY_NODE_ID);
		
#elif	defined MY_RADIO_RH_RF95		
		if (!_radio.setFrequency(915.0))							debug(PSTR(" ** setFrequency failed **\n") );
		if (!_radio.setModemConfig( _radio.Bw125Cr45Sf128 ))		debug(PSTR(" ** setModemConfig failed **\n") );
	
		_radio.setTxPower(13);				// default
		_radio.setPreambleLength (8);		// default
		_address = MY_NODE_ID;				// this is my node's address		
		//_radio.spiWrite(RH_RF69_REG_39_NODEADRS, MY_NODE_ID);

#endif		

		return true;
	}

	return false;
		
}


/* ********************************************************** */
// This set this nodes address
void transportSetAddress(uint8_t address) 
{
	_address = address;

#if defined MY_RADIO_RH_RF69
	_radio.spiWrite(RH_RF69_REG_39_NODEADRS, address);
#elif defined MY_RADIO_RH_RF95
#endif
}


/* ********************************************************** */
uint8_t transportGetAddress() 
{
	return _address;
}


/* ********************************************************** */
bool transportSend(uint8_t to, const void* data, uint8_t len) 
{
	if (len > _radio.maxMessageLength() ) len = (uint8_t) _radio.maxMessageLength();
		
		debug1(PSTR("\n *** Hex dump from transportSend \n"));
		debug1(PSTR(" *** TO: %d Len: %d \n"), to, len);
		hexdump ((uint8_t*) data, (unsigned long) len, 16);
		debug1(PSTR("\n"));

	return _radio.send( (uint8_t*)data, (uint8_t) len);		
}


/* ********************************************************** */
bool transportAvailable(uint8_t *to) 
{
//	if (driver.headerTo() == 0xff)
//	//if (manager.headerTo() == 0xff)
//		*to =  0xff;
//		
////	if (manager.headerTo == RH_BROADCAST_ADDRESS)
////		*to =  RH_BROADCAST_ADDRESS;
//		
//	else
		*to = _address;
		
	//Serial.println("\n *** RH_TransportAvailable \n");
		
	return	_radio.available();	// return true if we have a message, there for transport is available
	//return	manager.available();	// return true if we have a message, there for transport is available
}


/* ********************************************************** */
uint8_t transportReceive(uint8_t* data) 
{
	uint8_t len = _radio.maxMessageLength();	// need a place to hold the length of the data message
	uint8_t from;
	
	_radio.recv(data, &len);		// this return the correct length of the message from the radio

    debug1(PSTR("\n *** Hex dump from transportReceive \n"));
    debug1(PSTR(" *** From: %d Len: %d \n"), from, len);
	hexdump ((unsigned char*) data , len, 16);
	debug1(PSTR("\n"));
	
//    // Send ack back if this message wasn't a broadcast
//	if (_radio.TARGETID != RF69_BROADCAST_ADDR)
//		_radio.ACKRequested();
//    _radio.sendACK();
    
	return len;
}	


/* ********************************************************** */
void transportPowerDown() 
{
	//_radio.sleep();
}
