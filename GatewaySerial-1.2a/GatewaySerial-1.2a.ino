  
  
//#include <radio_config_Si4460.h>
//#include <RadioHead.h>
//#include <RH_ASK.h>
//#include <RH_CC110.h>
//#include <RH_MRF89.h>
//#include <RH_NRF24.h>
//#include <RH_NRF51.h>
//#include <RH_NRF905.h>
//#include <RH_RF22.h>
//#include <RH_RF24.h>

//#include <RH_RF69.h>
//#include <RH_RF95.h>
//#include <RH_Serial.h>
//#include <RH_TCP.h>
//#include <RHCRC.h>

//#include <RHDatagram.h>
//#include <RHReliableDatagram.h>
//#include <RHGenericDriver.h>

//#include <RHGenericSPI.h>
//#include <RHHardwareSPI.h>
//#include <RHMesh.h>
//#include <RHNRFSPIDriver.h>

//#include <RHRouter.h>
//#include <RHSoftwareSPI.h>
//#include <RHSPIDriver.h>
//#include <RHTcpProtocol.h>

#define RH_HAVE_SERIAL    <------------------------------

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
 *
 *******************************
 *
 * DESCRIPTION
 * The ArduinoGateway prints data received from sensors on the serial link. 
 * The gateway accepts input on seral which will be sent out on radio network.
 *
 * The GW code is designed for Arduino Nano 328p / 16MHz
 *
 * Wire connections (OPTIONAL):
 * - Inclusion button should be connected between digital pin 3 and GND  
 * - RX/TX/ERR leds need to be connected between +5V (anode) and digital pin 6/5/4 with resistor 270-330R in a series
 *
 * LEDs (OPTIONAL):
 * - To use the feature, uncomment MY_LEDS_BLINKING_FEATURE in MyConfig.h
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error 
 * 
 * 
 * 
 * 
 * * CHANGE LOG:
 *
 *  DATE         REV  DESCRIPTION
 *  -----------  ---  ----------------------------------------------------------
 *  06-May-2016  1.2a TRL - Change of Network ID, added MoteionMega, RFM69-ATC
 *  
 *
 *  Notes:  1)  Tested with Arduino 1.6.9
 *          2)  Testing using MotinoMega with RFM69HW
 *    
 *
 *
 *
 *
 */


/* ************************************************************************************** */
// These items need to be prior to #include <MySensor.h>  below

/*  Enable debug prints to serial monitor on port 0 */
#define MY_DEBUG            // used by MySensor
#define MY_DEBUG1           // used in this program, level 1 debug
//#define MY_DEBUG2           // used in this program, level 2 debug

#define SKETCHNAME "Serial Gateway"
#define SKETCHVERSION "1.2b"

// Enable and select radio type attached

//#define MY_RADIO_RH_RF69
#define MY_RADIO_RH_RF95

//#define MY_RADIO_RFM69
//#define MY_RFM69_Enable_ATC
#define MY_IS_RFM69HW               true
#define MY_RFM69_FREQUENCY          RF69_915MHZ     // this set frequency band, not real operating frequency
#define MY_RFM69_Real_FREQUENCY     915000000       // this set the real operation frequency, NOTE: RH_RF69 driver requires freq in Mhz, not Hz
#define CSMA_LIMIT                  -60             // Set the RSSI noise limit for CSMA

#ifdef __AVR_ATmega1284P__      // use for Moteino Mega Note: LED on Mega are 1 = on, 0 = off
// MoteinoMEGA
#define MY_RF69_SPI_CS            4
#define MY_RF69_IRQ_PIN           2
#define MY_RF69_IRQ_NUM           2
#define MY_DEFAULT_TX_LED_PIN     15   // the PCB, on board LED
#define MY_DEFAULT_ERR_LED_PIN    7
#define MY_DEFAULT_RX_LED_PIN     6
#else

// Moteino 
#define MY_RF69_SPI_CS            10
#define MY_RF69_IRQ_PIN           2
#define MY_RF69_IRQ_NUM           0
#define MY_DEFAULT_TX_LED_PIN     9   // 4 on sensor 2a PCB as LED3 (9 == on Moteino)
#define MY_DEFAULT_ERR_LED_PIN    6   // 6 on sensor 2a PCB as Err
#define MY_DEFAULT_RX_LED_PIN     7   // 7 on sensor 2a PCB as Rx
#endif

// Enable serial gateway
#define MY_GATEWAY_SERIAL
#define MY_RFM69_NETWORKID        200 // Network ID, unique to each network, production = 100, test = 200 <----------------------


// Flash leds on rx/tx/err
#define MY_LEDS_BLINKING_FEATURE
// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD     300

// Inverses the behavior of leds
#define MY_WITH_LEDS_BLINKING_INVERSE

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
#define MY_INCLUSION_BUTTON_FEATURE

// Inverses behavior of inclusion button (if using external pullup)
//#define MY_INCLUSION_BUTTON_EXTERNAL_PULLUP

// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION      60 
// Digital pin used for inclusion mode button
#define MY_INCLUSION_MODE_BUTTON_PIN    3 


/* ************************************************************************************** */
/* These are use for local debug of code, hwDebugPrint is defined in MyHwATMega328.cpp */
#ifdef MY_DEBUG1
#define debug1(x,...) hwDebugPrint(x, ##__VA_ARGS__)
#else
#define debug1(x,...)
#endif

#ifdef MY_DEBUG2
#define debug2(x,...) hwDebugPrint(x, ##__VA_ARGS__)
#else
#define debug2(x,...)
#endif

/* ************************************************************************************** */
#include <SPI.h>
#include <MySensor.h>  


unsigned long currentTime = 0;
unsigned long lastSend = 0;
unsigned long SEND_FREQUENCY = 4000;           // Minimum time between send (in milliseconds). We don't want to spam the gateway.

/* **************************************************************************** */
/* **************************************************************************** */
/* **************************************************************************** */
void setup() 
{ 
  // Setup locally attached sensors

#ifdef __AVR_ATmega1284P__      // use for Moteino Mega Note: LED on Mega are 1 = on, 0 = off
  debug(PSTR(" ** Hello from the Serial Gateway on a MoteinoMega **\n") );
  #else                         // we must have a Moteino
  debug(PSTR(" ** Hello from the Serial Gateway on a Moteino **\n") );
#endif

  const char compile_file[]  = __FILE__ ;
  debug(PSTR(" %s %s\n"), SKETCHNAME, SKETCHVERSION);
  debug(PSTR(" %s \n"), compile_file);
  const char compile_date[]  = __DATE__ ", " __TIME__;
  debug(PSTR(" %s \n\n"), compile_date);
  debug(PSTR(" Network ID: %u\n\n"), MY_RFM69_NETWORKID);

/*
 * This set the real operating frequency of the radio
 * Frequency is set in Hz
 */
 //   _radio.setFrequency(MY_RFM69_Real_FREQUENCY);
    
#ifdef MY_RFM69_Enable_ATC
    _radio.enableAutoPower(-70);
#endif

#ifdef  MY_RADIO_RFM69
debug(PSTR(" ** Printing Registers RFM69 **\n") ); 
#define REGISTER_DETAIL
  _radio.readAllRegs();
#endif

#if defined (MY_RADIO_RH_RF69) || defined (MY_RADIO_RH_RF95)
debug(PSTR(" ** Printing Registers RH_RF69/95 **\n") );
#define RH_HAVE_SERIAL
  _radio.printRegisters();
#endif

}

/* **************************************************************************** */
/* **************************************************************************** */
/* **************************************************************************** */
void presentation() 
{
 // Present locally attached sensors 
}

/* **************************************************************************** */
/* **************************************************************************** */
/* **************************************************************************** */
void loop() 
{ 

 currentTime = millis();                 // get the current time

if (currentTime - lastSend > SEND_FREQUENCY)
  {
    lastSend = currentTime;

  //debug(PSTR("\n *** In Loop \n\n"));

//debug(PSTR("\n *** RSSI: %d  MY TX Pwr: %d\n\n"), _radio.getAckRSSI(), _radio._transmitLevel);


#ifdef MY_RFM69_Enable_ATC
    debug(PSTR("\n *** RSSI: %d  MY TX Pwr: %d\n\n"), _radio.getAckRSSI(), _radio._transmitLevel);
#endif
  }


  // Send locally attached sensor data here 
}


/* ************** The End ****************** */


