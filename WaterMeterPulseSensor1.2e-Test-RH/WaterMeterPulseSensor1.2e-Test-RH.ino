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
//#include <RHReliableDatagram.h>
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

/*  
 *  Water flow and pressure meter
 *  
 *  
 *  Water flow from a Dwyer 1gal/pulse meter
 *  We measure the period of the pulse to determine the short term flow rate
 *    We also count the pulses to obtain the total gallons used
 *    We filter the input to debounce the reed switch in the meter the best we can, with a cutoff of ~80 gal/min
 *   
 *  Water from a pressure transducer, 0 to 5v
 *    Input fron sensor is scaled by .6577, 5.0v * .6577 = 3.288v
 *    (10.2k and a 19.6k resistor, flitered with a .1uf cap)
 *   
 *  Output: 0.5V – 4.5V linear voltage output. 0 psi outputs 0.5V, 50 psi outputs 2.5V, 100 psi outputs 4.5V 
 *    0   psi = 0.33v after scalling 5.0v to 3.3v
 *    50  psi = 1.65v
 *    100 psi = 2.97v
 *    3.3v/1024 = .0032266 volt per bit
 *    
 *    TX Led was on D3, now use for pulse-in
 *
 *
 * CHANGE LOG:
 *
 *  DATE         REV  DESCRIPTION
 *  -----------  ---  ----------------------------------------------------------
 *  06-May-2016 1.2c  TRL - Change of Network ID
 *  09-May-2016 1.2d  TRL - added ACK's to send for testing RFM69_ATC driver
 *                          added real frequency set
 *  
 *
 *  Notes:  1)  Tested with Arduino 1.6.9
 *          2)  Testing using Moteino with RFM69HW
 *    
 *
 *
 *
 */


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
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * Version 1.1 - GizMoCuz
 * 
 * DESCRIPTION
 * Use this sensor to measure volume and flow of your house watermeter.
 * You need to set the correct pulsefactor of your meter (pulses per gal).
 * The sensor starts by fetching current volume reading from gateway (VAR 1).
 * Reports both volume and flow back to gateway.
 *
 * Unfortunately millis() won't increment when the Arduino is in 
 * sleepmode. So we cannot make this sensor sleep if we also want  
 * to calculate/report flow.
 * http://www.mysensors.org/build/pulse_water
 */



/* ************************************************************************************** */
// These items need to be prior to #include <MySensor.h>  below

/*  Enable debug prints to serial monitor on port 0 */
#define MY_DEBUG            // used by MySensor
#define MY_DEBUG1           // used in this program, level 1 debug
#define MY_DEBUG2           // used in this program, level 2 debug

#define SKETCHNAME "Water and Pressure Meter"
#define SKETCHVERSION "1.2d"

// Enable and select radio type attached

//#define MY_RADIO_RH_RF69
#define MY_RADIO_RH_RF95

//#define MY_RADIO_RFM69

//#define MY_RFM69_Enable_ATC
#define MY_IS_RFM69HW               true
#define MY_RFM69_FREQUENCY          RF69_915MHZ    // this set frequency band, not the real operating frequency
#define MY_RFM69_Real_FREQUENCY     915000000      // this set the real operation frequency

#define MY_LEDS_BLINKING_FEATURE
#define MY_WITH_LEDS_BLINKING_INVERSE

#ifdef __AVR_ATmega1284P__        // use for Moteino Mega Note: LED on Mega are 1 = on, 0 = off
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

// Enabled repeater feature for this node
//#define MY_REPEATER_FEATURE

#define MY_RFM69_NETWORKID    200               // Network ID, unique to each network, production = 100, test = 200 <----------------------
#define MY_NODE_ID            10                 // My Sensor Node ID real = 6, 7 = test  <----------------------
#define MY_PARENT_NODE_ID     0                 // GW ID
#define CHILD_ID              1                 // Id of my Water sensor child
#define CHILD_ID2             2                 // Id of my 2nd sensor child

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

#define DIGITAL_INPUT_SENSOR 3                    // The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)
#define SENSOR_INTERRUPT DIGITAL_INPUT_SENSOR-2   // Usually the interrupt = pin -2 (on uno/nano anyway)

#define PressPin      A0                          // Pressure sensor is on analog input A0
#define PULSE_FACTOR  1                           // Nummber of pulse per gal of your meter (One rotation/gal)
#define SLEEP_MODE    false                       // flow value can only be reported when sleep mode is false.
#define MAX_FLOW      60                          // Max flow (gal/min) value to report. This filters outliers.


/* ******************************************************************* */

unsigned long SEND_FREQUENCY = 4000;              // Minimum time between send (in milliseconds). We don't want to spam the gateway.
unsigned long FLOW_FREQUENCY = 180000;            // time to refresh flow to gateway

MyMessage flowMsg         (CHILD_ID,V_FLOW);      // 34
MyMessage volumeMsg       (CHILD_ID,V_VOLUME);    // 35
MyMessage lastCounterMsg  (CHILD_ID,V_VAR1);      // 24
MyMessage VAR2Msg         (CHILD_ID,V_VAR2);      // 25
MyMessage VAR3Msg         (CHILD_ID,V_VAR3);      // 26
MyMessage VAR4Msg         (CHILD_ID,V_VAR4);      // 27
MyMessage pressureMsg     (CHILD_ID,V_PRESSURE);  // 4

MyMessage TextMsg         (CHILD_ID,V_TEXT);      // 47

double ppg = ((double) PULSE_FACTOR) ;            // Pulses per gal 

// Uses in interrupt routine
volatile unsigned long pulseCount = 0;            // counting of water meter pulses   
volatile unsigned long lastBlink = 0;             // time in us of last pulse
volatile unsigned long newBlink = 0;              // time in us of current pulse   
volatile unsigned long interval = 0;              // delta time in us from last pulse to current pulse
volatile double flow = 0;                         // curent flow rate 

boolean pcReceived = false;

unsigned long oldPulseCount = 0;

double oldflow = 0;
double volume = 0;                     
double oldvolume = 0;
int FLOWmsb = 0;                   // Flow MSB
int FLOWr = 0;                     // Flow remainder

unsigned long currentTime = 0;
unsigned long lastSend = 0;
unsigned long lastPulse = 0;

int pressure = 0;                 // Current value from ATD
float PSI = 0;                    // Current PSI
float PSI_CAL = 2.0;              // Calibration of sensor
int PSImsb = 0;                   // PSI MSB
int PSIr = 0;                     // PSI remainder


/* A function to print a buffer in hex format
 *
 *  hexdump (buffer, length, 16);
 */
//#include <stdio.h>
//#include <stdlib.h>
//// debug(PSTR(" %s \n"), compile_file);
// void hexdump(unsigned char *buffer, unsigned long index, unsigned long width)
// {
//    unsigned long i;
//    for (i=0; i<index; i++)
//    {
//      debug(PSTR("%02x "), buffer[i]);       //printf("%02x ",buffer[i]);
//    }
//    
//    for (unsigned long spacer=index;spacer<width;spacer++)
//      debug(PSTR("  "));                    //printf("  ");
//      debug(PSTR(": "));                    //printf(": ");
//      for (i=0; i<index; i++)
//      {
//        if (buffer[i] < 32) debug(PSTR("."));   //printf(".");
//        else debug(PSTR("%c"), buffer[i]);      //printf("%c",buffer[i]);
//      }
//    debug(PSTR("\n"));                      //printf("\n");
// }

 
/* **************************************************************************** */
/* **************************************************************************** */
/* **************************************************************************** */
void setup()  
{  
  // initialize serial communication at 115200bits per second:
  Serial.begin(115200);

#ifdef __AVR_ATmega1284P__      // use for Moteino Mega Note: LED on Mega are 1 = on, 0 = off
  debug(PSTR(" ** Hello from the Water Meter on a MoteinoMega **\n") );
  #else                         // we must have a Moteino
  debug(PSTR(" ** Hello from the Water Meter on a Moteino **\n") );
#endif

/*
 * This set the real operating frequency of the radio
 * Frequency is set in Hz
 */
 //   _radio.setFrequency(MY_RFM69_Real_FREQUENCY);

/*
 * This set the minimum RSSI for the RFM69_ATC mode
 */
#ifdef MY_RFM69_Enable_ATC
    _radio.enableAutoPower(-70);        // set minimum receive RSSI for this node
#endif

  const char compile_file[]  = __FILE__ ;
  debug(PSTR(" %s %s\n"), SKETCHNAME, SKETCHVERSION);
  debug(PSTR(" %s \n"), compile_file);
  const char compile_date[]  = __DATE__ ", " __TIME__;
  debug(PSTR(" %s \n\n"), compile_date);
  debug(PSTR(" Network ID: %u  Node ID: %u\n\n"), MY_RFM69_NETWORKID, MY_NODE_ID);

  // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion) 
  pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);
  
  // reference to use (DEFAULT(3.3v), INTERNAL, INTERNAL1V1, INTERNAL2V56, or EXTERNAL). 
  analogReference(DEFAULT);

  pulseCount = oldPulseCount = 0;

  // Fetch last known pulse count value from gw
  request(CHILD_ID, V_VAR1, true);

  lastSend = lastPulse = millis();      // set current time

  attachInterrupt(SENSOR_INTERRUPT, onPulse, FALLING);

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
  
} // end setup()

/* **************************************************************************** */
/* **************************************************************************** */
/* **************************************************************************** */
void presentation()  
{
 // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCHNAME, SKETCHVERSION, true);
  wait(250);

  // Register this device as Waterflow sensor
  present(CHILD_ID, S_WATER, "Water Flow", true);       // S_WATER = 21 
  wait(250);

}

/* **************************************************************************** */
/* **************************************************************************** */
/* **************************************************************************** */
void loop()     
{ 

    _process();
    
    currentTime = millis();                 // get the current time
  
 // Only send values at a maximum frequency 
  if (currentTime - lastSend > SEND_FREQUENCY)
  {
    lastSend = currentTime;
    
//    if (!pcReceived) 
//    {
//      //Last Pulsecount not yet received from controller, request it again
//      request(CHILD_ID, V_VAR1, true);
//      Serial.println("Last Pulsecount not yet received from controller");
//      return;
//    }


//    send(TextMsg.set("123456789A123456789B123456789C123456789D"), true);                   // Send long text message to gateway


/* ***************** Flow Rate ***************** */
    if (flow != oldflow) 
    {
      oldflow = flow;

      flow = ((60000000.0 / (double) interval) ) / ppg;

      if (flow < 0) flow = 0;

      // Check that we dont get unresonable large flow value. 
      // could happen when long wraps or false interrupt triggered
      if (flow < ((unsigned long) MAX_FLOW)) 
      {

        FLOWmsb = flow * 100;                               // we donot have float printing in debug print
        FLOWr = PSImsb % 100;                               // so we will breakup float to integer parts
        debug1(PSTR("gal/min: %0d.%02d \n"), FLOWmsb/100, FLOWr);

        send(flowMsg.set(flow, 2), true);                   // Send flow value to gateway
        wait(200);
        
      }  // end if (flow < ((unsigned long) MAX_FLOW)) 
    }  // end if (flow != oldflow) 


    // If no Pulse count received in 2min, update gateway flow to zero
    if(currentTime - lastPulse > FLOW_FREQUENCY)
    {
      flow = 0;
      debug1(PSTR("***Sending zero flow rate\n"));
 
      send(flowMsg.set(flow, 2), true);                   // Send flow value to gateway
      wait(200);
      
    } // end if(currentTime - lastPulse > FLOW_FREQUENCY)


/* ***************** Pluse Count ***************** */
    // Pulse count has changed
    if (pulseCount != oldPulseCount)
    {
      oldPulseCount = pulseCount;

      debug1(PSTR("Pulse Count:  %u\n"), (unsigned int) pulseCount );
      
//      send(lastCounterMsg.set(pulseCount));         // Send  pulse count value to gw in VAR1
      wait(200);


      double volume = ((double) pulseCount / ((double) PULSE_FACTOR));     
      if (volume != oldvolume)
      {
        oldvolume = volume;

        debug1(PSTR("Volume (gal): %u\n"), (unsigned int) volume );


        send(volumeMsg.set (volume,0), true);            // Send total volume to gateway
        wait(200);
      } // end f (volume != oldvolume)

#ifdef MY_RFM69_Enable_ATC
    debug(PSTR("\n *** RSSI: %d  MY TX Pwr: %d\n\n"), _radio.getAckRSSI(), _radio._transmitLevel);
#endif

     }  // end  if (pulseCount != oldPulseCount)



/* ***************** Analog Read ***************** */
/* We will read the analog input from the presure transducer 
 *  and convert it from an analog voltage to a pressure in PSI
 * 
 *  Output: 0.5V – 4.5V linear voltage output. 0 psi outputs 0.5V, 50 psi outputs 2.5V, 100 psi outputs 4.5V 
 *  0   psi = .33v after scalling 5.0v to 3.3v
 *  50  psi = 1.65v
 *  100 psi = 2.97v
 *
 *  3.3v/1024 = .0032266 volt per bit
 */

    pressure  = analogRead    (PressPin) ;        // this is a junk read to clear ATD
    wait(25);
 
    pressure  = analogRead    (PressPin) ;

    if (pressure < 106) pressure = 106;         // this is minimum of .5v
    PSI = (pressure - 106 ) * .1246;            // where did we get this?? was .119904
    PSI = PSI + PSI_CAL;                        // calibration adjustment
    
    PSImsb = PSI * 100;
    PSIr = PSImsb % 100;
    debug1(PSTR("PSI:  %0u.%02u\n"), PSImsb/100, PSIr);

    send(pressureMsg.set(PSI, 2), true);         // Send water pressure to gateway
    wait(200);
    
  }   // end of if (currentTime - lastSend > SEND_FREQUENCY)

}   // end of loop



/****************** Message Receive Loop ***************** */
/* This the message receive loop, here we look for messages address to us
 *  
 */
void receive(const MyMessage &message) 
{
   //debug1(PSTR("Received message from gw\n"));
   
// Make sure its for our child ID
  if (message.sensor == CHILD_ID )
  {
    if  (message.type==V_VAR1) 
      {
        unsigned long gwPulseCount=message.getULong();
        pulseCount += gwPulseCount;
        flow=oldflow=0;
        debug1(PSTR("Received last pulse count from gw: %u\n"), pulseCount);
        pcReceived = true;
      }
    
     if ( message.type==V_VAR2) 
      {
        pulseCount = message.getULong();
        flow=oldflow=0;
        debug1(PSTR("Received V_VAR2 message from gw: %u\n"), pulseCount );
      }
    
     if ( message.type==V_VAR3) 
      {
        FLOW_FREQUENCY = message.getULong();
        debug1(PSTR("Received V_VAR3 message from gw: %u\n"), FLOW_FREQUENCY);

     if ( message.type==V_VAR4) 
      {
  
        PSI_CAL = message.getULong();
        debug1(PSTR("Received V_VAR4 message from gw: %u\n"), PSI_CAL);
      }
      }
    }  // end if (message.sensor == CHILD_ID )

 // Check for any messages for a 2nd child ID
  if (message.sensor == CHILD_ID2 )
    {
      
    }
}

/* ***************** Interrupt Service ***************** */
/* As this is an interrupt service, we need to keep the code here as short as possable 
 * we count the pulses from the meter
 */
void onPulse()     
{
    newBlink = micros();              // get the current time in microseconds
    interval = newBlink-lastBlink;    // get the delta time in us from last interrupt
                                      // we will use this delta to compute flow rate
    lastBlink = newBlink;             // setup for next pulse
  
    pulseCount++;                     // we also want to count the pulses from meter
    lastPulse = millis();             // use to check for zero flow rate
}

/* ***************** The End ***************** */


