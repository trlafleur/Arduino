//
// Author: Maarten Westenberg
// Version: 1.0.0
// Date: 2016-03-25
//
// This file contains a number of compile-time settings that can be set on (=1) or off (=0)
// The disadvantage of compile time is minor compared to the memory gain of not having
// too much code compiled and loaded on your ESP8266.
//
// See http://pubsubclient.knolleary.net/api.html for API description of MQTT
// ----------------------------------------------------------------------------------------

// WiFi definitions
#define _SSID "lafleur"
#define _PASS "sonysonysony1"

// MQTT definitions
#define _TTNSERVER "router.us.thethings.network"  //"40.114.249.243"   //"croft.thethings.girovito.nl"
#define _MQTTSERVER "192.94.167.32"

// Gateway Ident definitions
#define _DESCRIPTION "ESP Test Gateway"
#define _EMAIL "tom@lafleur.us"
#define _PLATFORM "ESP8266"
#define _LAT 32.984709
#define _LON -117.178966
#define _ALT 100

#define STATISTICS 1		// Gather statistics on sensor and Wifi status
#define DEBUG 1					// Initial value of debug var. Can be hanged using the admin webserver
								        // For operational use, set initial DEBUG vaulue 0
								
// Definitions for the admin webserver
#define A_SERVER 1				  // Define local WebServer only if this define is set
#define SERVERPORT 8080			// local webserver port

#define A_MAXBUFSIZE 192		// Must be larger than 128, but small enough to work
#define _BAUDRATE 115200		// Works for debug messages to serial momitor (if attached).

// ntp
#define NTP_TIMESERVER "north-america.pool.ntp.org"	// Country and region specific
#define NTP_INTERVAL	3600	// How often do we want time NTP synchronization
#define NTP_TIMEZONES	-8		// How far is our Timezone from UTC (excl daylight saving/summer time)
#define SECS_PER_HOUR	3600
