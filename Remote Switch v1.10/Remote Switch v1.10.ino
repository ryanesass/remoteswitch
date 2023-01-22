/*-----------------------------/START OF CODE/--------------------------------------------------------------------//
  Remote computer switch
  This code runs a ESP8266 that handles switching on and off a computer. It is set to stay
  awake for only a minute unless it receives commands or is commanded to stay awake. 
  It can turn on and off a computer, as well as perform basic checks on the computer status
  and update the user about the boar operation and connection detals. It writes data on an 
  as needed basis to the RTC memory bank depending on any situations that have changed. 

  It connects to an MQTT server and only handles QoS 1 data packages when connected. It will
  notify the MQTT client when it connects, and when it is soon to be heading to sleep. It 
  will always send feedback commands when it receives commands, including unknown commands. 

  This is an ongoing project and will be updated as need be. 
  
  V1.0 - initial run, works and tested to full functionality
  V1.1 - updated the display to show the computer status when connecting, as well as did 
  some code cleanup. Also updated some display output changes to the countdown code to 
  display minutes instead of seconds only. 
  V1.2 - updated to sanatizePayload message commands, handle blank commands, provide status reports
  on RTC memory flags, user can now toggle publishing UTC or local time, and now user can 
  set specified awake times (in min), as well as specified hours for sleeping. For specified 
  sleeping, user must CONFIRM the sleep by calling sleepy5, and board will then sleep for
  input hours. Any other command (except countdown) will cancel out the order. There is a 
  maximum limit to sleeping. 
  V1.3 - added persistence messaging. Now the board will clear an incoming persistent message
  during initial startup. This is assuming it is receiving this command for itself. It also 
  clears it's own last persistent message it publishes to (usually the last sleep message). 
  Finally, it will publish a persistent message about the status of the computer. 
  V1.4 - revamped how handling the status recording of the computer is handled. it is now more stable
  and accurate. Also made changes to persistent sleep command - board will keep sleeping regardless 
  of resets. Now, it will also wait for computer status changes before going to sleep (both forced 
  and timeout) though the forced sleep can be overridden to sleep by commanding 2x when waiting. 
  V1.5 - fixed a bug with caffen8 command, updated RTC to updated correctly, and added software
  restart and hardware restart commands. Also, doubled the amount of values in state (to 20) to 
  handle the correction occurring during restart
  V1.6 - updated a few minor print typos and sequencing of printed information. Also, updated entire TZ
  list to include a sample from all 54 active time zones (as of Jan 2023). Finally, added a secondary
  TZ list to print out the TZ offset from UTC. A star (*) next to the offset indicates that TZ observes
  DST as well. 
  V1.7 - added hibernate function and changed travel sleep to have a 31 day window. Travel sleep and 
  hibernation both sleep for an extended time period, with travel sleep ending at a given time. Hibernate
  resumes long sleeping indefinitely. Both can be ended by caffenating or the Awake Commands. Also, minor 
  changes to memory handling. Number of output changes. 
  V1.8 - pulling out bugs in code, fixing the handling between the different functions and cleaning up 
  the leftover commented out code no longer needed. Also, downsized the handling of numbers being sent
  via MQTT to be a for loop instead of a nested if statement
  V1.9 - added commands to update all sleep length durations as well as printing out sleep durations. Also
  added last will and testiment call for connection as well as doing a graceful disconnect. Finally, added
  a channel and function to show when the device is expected to reconnect to the server. 
  V1.10 - fixed bug in night time sleep where it was setting to day time sleep, and another bug in 
  checkStayAsleep() where it tries to publish when it hasn't connected.

  Last Update: 16 Jan 2023 11:49PM PST -- Ryan Sass
  */

/////////////////////////Declarations////////////////////////////////////////////////////////////////////
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <TZ.h>

#define MSG_BUFFER_SIZE (110)
#define PORT (1883)
#define HEARTBEAT (5 * (60000))  // value in minutes
#define SIXTY_SECONDS 60000
#define TWENTY_SECONDS 20000
#define SLEEP_MODIFIER 6e7
#define WakeMode RFMode
#define MAC_SIZE 6
#define RESTART_PIN_D0 16                //D0
#define PIN_POWER_LIGHT_OUT_D1 5         //D1
#define STAY_ASLEEP_SW 4                 //D2
#define PIN_POWER_LIGHT_IN_D5 14         //D5
#define PIN_POWER_BUTTON_D6 12           //D6
#define PIN_RESET_BUTTON_D7 13           //D7
#define BUTTON_PRESS_TIME 350            //ms
#define DEFAULT_DAY_SLEEP_TIME 9         // minutes
#define DEFAULT_NIGHT_SLEEP_TIME 29      //minutes
#define STAY_ASLEEP_TIME 60              // minutes
#define STAY_AWAKE 120                   //minutes
#define MAX_TRAVEL_SLEEP_TIME 744        //hours (31 days) -- careful extending this, at 744hrs, this is ~2.67B micro seconds. It is handled by an unsigned long, which maxes out at ~4.294B...
#define MAX_ESP_SLEEP 180                //TODO DETERMINE BIG THIS CAN GET, now trying 3.5 hrs, up to 180 mins? //minutes, used for hibernation and travel time, as well as keep asleep time.
#define DEFAULT_TRAVEL_SLEEP 120         //minutes
#define DEFAULT_HIBERNATE_SLEEP 180      //minutes
#define MAX_TZ 54                        //maximum number of TZ's listed
#define DEFAULT_AWAKE_TIME_AFTER_CALL 3  //time to stay awake (min) after a normal command call
#define COFFEE_RESET 8                   //hour in which to reset the coffee flag
#define COFFEE_SLEEP 8                   //minutes
#define STATUS_LOOPS 10                  // used for size and loop length for state[payloadIndex], which records the state of the output
#define ON '+'
#define OFF '-'
#define SLEEP '/'
#define AUTO 'A'
#ifndef WAKE_RF_DEFAULT
#define WAKE_RF_DEFAULT RF_DEFAULT
#define WAKE_RFCAL RF_CAL
#define WAKE_NO_RFCAL RF_NO_CAL
#define WAKE_RF_DISABLED RF_DISABLED
#endif
#ifndef STASSID
#define STASSID "" //MUST UPDATE THIS PRIOR TO RUNNING, USE LOCAL WIFI SSID
#define STAPSK "" //MUST UPDATE THIS PRIOR TO RUNNING, USE LOCAL WIFI PASSWORD
#endif
#define COMMAND_1 "Status"             //Status Check light
#define COMMAND_2 "ON"                 // turn on computer
#define COMMAND_3 "OFF"                // turn off computer
#define COMMAND_4 "Reset"              // press reset button
#define COMMAND_5 "Force Off"          //hold off button 10s
#define COMMAND_6 "pressPwr"           // press power button
#define COMMAND_7 "releasePwr"         // release power button
#define COMMAND_8 "Light On"           // turn case led on
#define COMMAND_9 "Light Off"          // turn case led off
#define COMMAND_10 "Light Auto"        // auto case led handling
#define COMMAND_11 "Board LED On"      // turn ESP8266 LED on
#define COMMAND_12 "Board LED Off"     // turn ESP8266 LED off
#define COMMAND_13 "Sleepy5"           // force board to sleep
#define COMMAND_14 "Awaken63"          // keep board awake, 2 hrs or forced sleep
#define COMMAND_15 "Time Zone:"        // change local time zone
#define COMMAND_16 "Time Check"        //display current time (local on serial, and UTC on MQTT), as well as show current time zone code selected
#define COMMAND_17 "Heart Beat"        // force a heart beat
#define COMMAND_18 "Board Status"      //update on board status
#define COMMAND_19 "Wifi Status"       //update on wifi status
#define COMMAND_20 "Caffen8"           //stay awake during the night
#define COMMAND_21 "Countdown"         //display current countdown
#define COMMAND_22 "Travel:"           //set a travel sleeping time
#define COMMAND_23 "Awake:"            //set a time to stay awake for
#define COMMAND_24 "LocUTC"            //set to toggle UTC or Local time on publish
#define COMMAND_25 ""                  //when receiving empty message
#define COMMAND_26 "RTC Status"        // reports the RTC status and all state of flags
#define COMMAND_27 "Board Restart"     // restarts the board via ESP.restart()
#define COMMAND_28 "Board RST Button"  //restarts the board via the RST input (assumed DO0 is connected)
#define COMMAND_29 "Hibernate"         //sets the board to continually long sleep
#define COMMAND_30 "Day Sleep:"        // sets day sleep time
#define COMMAND_31 "Night Sleep:"      //sets the night sleep time
#define COMMAND_32 "Travel Sleep:"     // sets the travel sleep time
#define COMMAND_33 "Hibernate Sleep:"  // sets the hibernate sleep time
#define COMMAND_34 "Coffee Sleep:"     //sets the coffee sleep time
#define COMMAND_35 "Time Status"       //displays the time status of the board
#define COMMAND_36 "Sleep Default"     //sets the sleep values to defaults
//#define COMMAND_37 "Locaition:" //sets the current location of the board TODO: FLESH OUT LOCATION COMMANDS
//REMEMBER THERE ARE 3 PLACES TO ADD NEW COMMANDS: messageConversion(), processCommand(), and commandCode()
const char *subscribeTopic = "remote/controller/subscribe";           //command input channel
const char *publishTopic = "remote/controller/publish";           //command output channel
const char *publishTopicStatus = "remote/controller/Status";       //computer status reporting channel
const char *publishTopicAwake = "remote/controller/Awake";      //stay awake time channel
const char *publishTopicSleep = "remote/controller/Sleep";      //travel sleep time channel
const char *publishTopicBoardState = "remote/controller/BoardState";     //board state channel
const char *publishTopicNextAwake = "remote/controller/NextAwake";  //calculating and displaying the next time to expect connection
//const char* server1 = "pool.ntp.org";
//const char* server2 = "time.nist.gov";
const char *ssid = STASSID;
const char *password = STAPSK;
const char *mqttServerAddress = "91.121.93.94";  // test.mosquitto.org
//const char* mqttServerAddress = "2001:41d0:1:925e::1"; //test.mosquitto.org ipv6 address
//const char* mqttServerAddress = "broker.mqtt-dashboard.com"; //original

typedef struct {
  uint32_t crc32;            //4 bytes
  uint8_t channel;           //1 byte, 5 in total
  uint8_t ap_mac[MAC_SIZE];  //MAC_SIZE (6) bytes, 11 in total
  uint8_t timeZone;          //1 byte, 12 in total
  uint8_t lightMode;         //1 byte, 13 in total
  uint8_t coffee;            //1 byte, 14 in total
  uint8_t local;             //1 byte, 15 total
  uint8_t hibernate;         //1 byte, 16 total
  uint8_t daySleep;          //1 byte, 17 total
  uint8_t nightSleep;        //1 byte, 18 total
  uint8_t travelSleep;       //1 byte, 19 total
  uint8_t hibernateSleep;    //1 byte, 20 total
  uint8_t coffeeSleep;       //1 byte, 21 total
  uint8_t padding;           //1 byte, 22 in total
} rtcStore;
rtcStore rtcData, rtcTemp;

IPAddress ip(192, 168, 0, 252);  //pick an IP outside the DHCP range
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(75, 75, 75, 75);

char currentStatus = OFF,
     lightStatus = AUTO,
     repStatus = OFF,
     globalMessage[MSG_BUFFER_SIZE],
     globalMessageAlternate[MSG_BUFFER_SIZE],
     globalMessageUTC[MSG_BUFFER_SIZE],
     globalMessageAwake[MSG_BUFFER_SIZE];

unsigned long powerButtonReleaseTime = 0,
              lastOn = 0,
              lastOff = 0,
              lastStatChange = 0,
              lastMessage = 0,
              awakeStartTime = 0,
              originalStartTime = 0,
              timeAwake = 60000;  //1 min awake default

unsigned int travelTimeStopTime = 0;  //travel time stop point
int state[STATUS_LOOPS];
int SLEEPTIME = 9,    //9 min sleep default
  sleepTime = 9,      //default sleep time in minutes, will be converted to micro seconds (us)
  timeZone = 0,       //defaults to 0, which is set to PST/PDT
  heartBeatCount = 0,        //publishing code
  sleepCountdownCode = 1,      //sleep countdown code
  mqttReconnectAttempts = 0,  //mqtt reconnect attempts
  globalPayload = 0,           //message information payload
  payloadIndex = 0,              //index
  publishState = 0;       //states published to channel. -2 == hibernation, -1 == travel sleeping 0 == not published, 1 == caffenated, 2 == awake, 3 == normal state
bool printSpace = false,
     forceOff = false,
     rtcValid = false,
     ledState = false,
     dayTime = true,
     wifiMemUpdate = false,
     timeZoneMemUpdate = false,
     lightUpdate = false,
     coffeeMemUpdate = false,
     localTZMemUpdate = false,
     hibernateMemUpdate = false,
     timeMemUpdate = false,
     noTime = false,
     noPublish = false,
     commandCall = false,
     stayAwake = false,
     firstLoop = true,
     coffee = false,
     local = false,
     hibernate = false;

//DEBUG FLAGS //
///normal settings for normal operation///
/*/bool outputSerial = false;
    #define GETTIME true
    #define SLEEPALLOW true//*/
///full debug settings////
/*/bool outputSerial = true;
    #define GETTIME false
    #define SLEEPALLOW false//*/
///custom debug settings////
/**/ bool outputSerial = true;
#define GETTIME true
#define SLEEPALLOW true  //*/

WiFiClient espClient;
PubSubClient client(espClient);

char *currentTime(bool local = false, int minutes = 0);
void publishTravelTimeStopTime(char *display, bool empty = false);
void publish(char *display, int heartbeatCount = -1);
void print(String toPrint = "");
void println(String toPrint = "");
void sleepMode(int sleepMode, int duration = 120);
void computerStatusPrint(int print = 0);
void reportCommandClear(int ignore = 0);

/////////////////////////RTC FUNCTIONS///////////////////////////////////////////////////////////////////
void checkRTC() {
  rtcValid = false;
  if (ESP.rtcUserMemoryRead(0, (uint32_t *)&rtcData, sizeof(rtcData))) {
    //calculate the  CRC of what was just read from RTC memory, but skip the first 4 bytes as that's the checksum
    delay(1);
    uint32_t crc = calculateCRC32(((uint8_t *)&rtcData) + 4, sizeof(rtcData) - 4);
    if (crc == rtcData.crc32) {
      rtcValid = true;
    }
  }
}

void rtcMemUpdate() {
  bool crcUpdate = false;
  if (wifiMemUpdate) {  //wifi information update
    rtcTemp.channel = rtcData.channel;
    rtcData.channel = WiFi.channel();
    if (rtcData.channel == rtcTemp.channel) wifiMemUpdate = false;
    for (int count = 0; count < MAC_SIZE; count++) rtcTemp.ap_mac[count] = rtcData.ap_mac[count];
    memcpy(rtcData.ap_mac, WiFi.BSSID(), 6);  //copy 6 bytes of BSSID (AP's MAC address)
    for (int count = 0; count < MAC_SIZE; count++)
      if (rtcData.ap_mac[count] == rtcTemp.ap_mac[count]) wifiMemUpdate = false;
  }
  if (timeZoneMemUpdate) {
    if (rtcData.timeZone != timeZone) rtcData.timeZone = timeZone;
    else { timeZoneMemUpdate = false; }
  }
  if (lightUpdate) {
    int lightMode = 2;
    if (lightStatus == OFF) lightMode = 0;
    if (lightStatus == ON) lightMode = 1;
    if (rtcData.lightMode != lightMode) rtcData.lightMode = lightMode;
    else { lightUpdate = false; }
  }
  if (coffeeMemUpdate) {
    if (rtcData.coffee = boolToInt(coffee)) rtcData.coffee = boolToInt(coffee);
    else { coffeeMemUpdate = false; }
  }
  if (localTZMemUpdate) {
    if (rtcData.local != boolToInt(local)) rtcData.local = boolToInt(local);
    else { localTZMemUpdate = false; }
  }
  if (hibernateMemUpdate) {
    if (rtcData.hibernate != boolToInt(hibernate)) rtcData.hibernate = boolToInt(hibernate);
    else { hibernateMemUpdate = false; }
  }
  rtcTemp.crc32 = rtcData.crc32;
  rtcData.crc32 = calculateCRC32(((uint8_t *)&rtcData) + 4, sizeof(rtcData) - 4);
  if (rtcData.crc32 != rtcTemp.crc32) crcUpdate = true;
  if (checkRTCUpdate() || crcUpdate) {
    ESP.rtcUserMemoryWrite(0, (uint32_t *)&rtcData, sizeof(rtcData));
    delay(100);
    publish("RTC Updated");
  }
  wifiMemUpdate = false;
  timeZoneMemUpdate = false;
  lightUpdate = false;
  coffeeMemUpdate = false;
  localTZMemUpdate = false;
  timeMemUpdate = false;
  return;
}

bool checkRTCUpdate() {
  return (wifiMemUpdate || timeZoneMemUpdate || lightUpdate || coffeeMemUpdate || localTZMemUpdate || hibernateMemUpdate || timeMemUpdate);
}

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t count = 0x80; count > 0; count >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & count) bit = !bit;
      crc <<= 1;
      if (bit) crc ^= 0x04c11db7;
    }
  }
  return crc;
}

/////////////////////////CONFIG FUNCTIONS ///////////////////////////////////////////////////////////////
void configure() {
  if (outputSerial) {
    Serial.begin(115200);
    Serial.setTimeout(2000);
  }
  delay(50);
  println();
  if (rtcValid) println("rtcValid!");
  else println("rtcInValid!");  //
  if (rtcValid) {
    timeZone = rtcData.timeZone;  //reestablishing the local time zone
    switch (rtcData.lightMode) {  //reestablish led mode
      case 0:
        lightStatus = OFF;
        break;
      case 1:
        lightStatus = ON;
        break;
      default:
        lightStatus = AUTO;
        break;
    }
    coffee = intToBool(rtcData.coffee);  //reestablish if caffenated
    local = intToBool(rtcData.local);    //reestablish if displaying local time
    hibernate = intToBool(rtcData.hibernate);
  }  // reestablish if hibernating
  else
    setSleepDefault();
  if (SLEEPALLOW) sleepMode(1);
  else {
    sleepMode(0);
    publish("Debug Sleep Unallowed!");
  }
  pinMode(PIN_POWER_LIGHT_OUT_D1, OUTPUT);
  pinMode(PIN_POWER_LIGHT_IN_D5, INPUT_PULLUP);
  delay(50);
  for (int count = 0; count < STATUS_LOOPS; count++) state[count] = 0;  //goosing the state array
  for (int count = 0; count < STATUS_LOOPS; count++) statusChange();
  for (int count = 0; count < STATUS_LOOPS; count++) {
    delay(random(110));
    statusChange();
  }
  for (int count = 0; count < STATUS_LOOPS; count++) {
    delay(random(150));
    statusChange();
  }
  currentStatus = determineState();
  repStatus = currentStatus;
  //attachInterrupt(digitalPinToInterrupt(PIN_POWER_LIGHT_IN_D5), statusChange, CHANGE);
  pinMode(BUILTIN_LED, OUTPUT);  // Initialize the BUILTIN_LED pin as an output
  led_OFF();
}  // default turn LED off

void turnIOPinsOn() {
  pinMode(PIN_POWER_BUTTON_D6, OUTPUT);
  pinMode(PIN_RESET_BUTTON_D7, OUTPUT);
}

int boolToInt(bool input) {
  if (input) return 1;
  else return 0;
}

bool intToBool(int input) {
  if (input == 0) return false;
  else return true;
}

String boolToString(bool input) {
  if (input) return "TRUE";
  else return "FALSE";
}

int asciiToInt(int input) {
  switch (input) {
    case 48:
      return 0;
    case 49:
      return 1;
    case 50:
      return 2;
    case 51:
      return 3;
    case 52:
      return 4;
    case 53:
      return 5;
    case 54:
      return 6;
    case 55:
      return 7;
    case 56:
      return 8;
    case 57:
      return 9;
    default:
      return -1;
  }
}

void restart() {
  ESP.restart();
  publish("ESP.restart() Called...?????");
}

void rstButton() {
  pinMode(RESTART_PIN_D0, OUTPUT);
  digitalWrite(RESTART_PIN_D0, LOW);  //must have DO0 connected to RST pin.
  publish("Restart Pin Disconnected!");
}

/////////////////////////CONNECT FUNCTIONS///////////////////////////////////////////////////////////////
void startWiFi() {
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(10);
  WiFi.forceSleepWake();
  delay(10);
  //Disable the WiFi persistence. The ESP8266 will not load and save WiFi settings unnecessarily in the flash memory.
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, dns, gateway, subnet);
  println();
  print("Connecting to ");
  print(ssid);
  if (rtcValid) WiFi.begin(ssid, password, rtcData.channel, rtcData.ap_mac, true);
  else WiFi.begin(ssid, password);
  unsigned long loopStart = millis();
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    retries++;
    delay(100);
    print(".");
    if (retries >= 100 && retries <= 104) {
      //after 20s quick connect is not functioning reset WiFi and try regular connection
      println("Resetting WiFi...");
      WiFi.disconnect();
      delay(100);
      WiFi.forceSleepBegin();
      delay(100);
      WiFi.forceSleepWake();
      delay(50);
      WiFi.begin(ssid, password);
      retries = 105;
    }
    if (retries >= 600) {
      //giving up after 60 seconds and going back to sleep
      println("");
      println("Connection Timeout. Restarting.");
      WiFi.disconnect(true);
      delay(100);
      WiFi.mode(WIFI_OFF);
      delay(50);
      noPublish = true;
      restart();
      break;
    }
    if ((millis() - loopStart) > SIXTY_SECONDS) {
      restart();
      break;
    }
  }
  if (retries > 0) println();
  print("Conntected to WiFi: ");
  println(ssid);
  println("IP Address: ");
  if (outputSerial) Serial.println(WiFi.localIP());
  if (!rtcValid) wifiMemUpdate = true;
}  //setupCompleteBlink(); ///commenting out to save LED life

void setupMQTT() {
  client.setServer(mqttServerAddress, PORT);
  client.setCallback(callback);
}

void reconnect() {
  unsigned long loopStart = millis();
  while (!client.connected()) {  // Loop until reconnection
    mqttReconnectAttempts++;
    print("Attempting MQTT connection on ");
    print(mqttServerAddress);
    print("...");
    String clientId = "ESP8266Client-";  // Create a random client ID
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), publishTopicNextAwake, 0, true, "From MQTT server: Device disconnected unexpectedly!")) {  // Attempt to connect
      println("connected!");
      //client.subscribe(publishTopic);
      //delay(75); // code for clearing out last printed message. i'm removing it for now as I think it's unnecessary to clear it.
      //client.unsubscribe(publishTopic);
      snprintf(globalMessage, MSG_BUFFER_SIZE, "Connection Successful! at (TZ#: %d) %s %s", timeZone, UTCoffset(timeZone), currentTime(true));
      client.publish(publishTopic, globalMessage, true);  // Once connected, publish an announcement...
      client.subscribe(subscribeTopic);           // ... and resubscribe
      if (firstLoop) {                        // perform things during the first go of the main loop
        //client.publish(publishTopic,"", true); //moved this clearing to above, subscribe and unsubscribe, and handle any errant messages in callback function
        client.subscribe(publishTopicAwake);
        client.subscribe(publishTopicSleep);
        originalStartTime = millis();
        computerStatusPrint();  //print status of computer
        if (!rtcValid) {
          publish("RTC Invalid");
          if (checkRTCUpdate()) rtcMemUpdate();
        }
        if (hibernate) {
          publish("Hibernate Mode Engaged");
          sleepMode(7);
          if (publishState != -2) publishBoardState();
          publishState = -2;
        }
        delay(10);
        firstLoop = false;
      }
    } else {
      print("failed, rc=");
      print(String(client.state()));
      println(" try again in 5 seconds");
      mqttConnFailBlink();
    }  //the blink command takes 5000 ms
    if ((millis() - loopStart) > SIXTY_SECONDS) startDeepSleep(1);
  }
  printSp();
  awakeStartTime += (millis() - loopStart);
}

unsigned long heartbeat() {
  if (firstLoop) {
    println("Heartbeat 0");
    println();
  } else publish("", heartBeatCount++);
  println();
  heartbeatBlink();
  return millis();
}

/////////////////////////TIME FUNCTIONS//////////////////////////////////////////////////////////////////
void setLocalTimeZone(int zone) {
  switch (zone) {  //remember to update UTCoffset() function and MAX_TZ definition when any changes are made here
    case 0:
      //the global servers: "pool.ntp.org", "time.nist.gov"
      configTime(TZ_America_Los_Angeles, "pool.ntp.org", "time.nist.gov");
      break;
    case 1:
      configTime(TZ_America_Phoenix, "pool.ntp.org", "time.nist.gov");
      break;
    case 2:
      configTime(TZ_America_Denver, "pool.ntp.org", "time.nist.gov");
      break;
    case 3:
      configTime(TZ_America_Chicago, "pool.ntp.org", "time.nist.gov");
      break;
    case 4:
      configTime(TZ_America_New_York, "pool.ntp.org", "time.nist.gov");
      break;
    case 5:
      configTime(TZ_America_Anchorage, "pool.ntp.org", "time.nist.gov");
      break;
    case 6:
      configTime(TZ_Pacific_Honolulu, "pool.ntp.org", "time.nist.gov");
      break;
    case 7:
      configTime(TZ_Etc_GMTp12, "pool.ntp.org", "time.nist.gov");
      break;
    case 8:
      configTime(TZ_Pacific_Midway, "pool.ntp.org", "time.nist.gov");
      break;
    case 9:
      configTime(TZ_Pacific_Gambier, "pool.ntp.org", "time.nist.gov");
      break;
    case 10:
      configTime(TZ_Pacific_Pitcairn, "pool.ntp.org", "time.nist.gov");
      break;
    case 11:
      configTime(TZ_America_Edmonton, "pool.ntp.org", "time.nist.gov");
      break;
    case 12:
      configTime(TZ_America_Mexico_City, "pool.ntp.org", "time.nist.gov");
      break;
    case 13:
      configTime(TZ_America_Cayman, "pool.ntp.org", "time.nist.gov");
      break;
    case 14:
      configTime(TZ_America_Santiago, "pool.ntp.org", "time.nist.gov");
      break;
    case 15:
      configTime(TZ_America_Puerto_Rico, "pool.ntp.org", "time.nist.gov");
      break;
    case 16:
      configTime(TZ_America_Sao_Paulo, "pool.ntp.org", "time.nist.gov");
      break;
    case 17:
      configTime(TZ_Atlantic_Cape_Verde, "pool.ntp.org", "time.nist.gov");
      break;
    case 18:
      configTime(TZ_Europe_London, "pool.ntp.org", "time.nist.gov");
      break;
    case 19:
      configTime(TZ_Atlantic_Reykjavik, "pool.ntp.org", "time.nist.gov");
      break;
    case 20:
      configTime(TZ_Europe_Paris, "pool.ntp.org", "time.nist.gov");
      break;
    case 21:
      configTime(TZ_Africa_Casablanca, "pool.ntp.org", "time.nist.gov");
      break;
    case 22:
      configTime(TZ_Europe_Kiev, "pool.ntp.org", "time.nist.gov");
      break;
    case 23:
      configTime(TZ_Africa_Cairo, "pool.ntp.org", "time.nist.gov");
      break;
    case 24:
      configTime(TZ_Europe_Moscow, "pool.ntp.org", "time.nist.gov");
      break;
    case 25:
      configTime(TZ_Asia_Dubai, "pool.ntp.org", "time.nist.gov");
      break;
    case 26:
      configTime(TZ_Asia_Karachi, "pool.ntp.org", "time.nist.gov");
      break;
    case 27:
      configTime(TZ_Asia_Kolkata, "pool.ntp.org", "time.nist.gov");
      break;
    case 28:
      configTime(TZ_Asia_Bangkok, "pool.ntp.org", "time.nist.gov");
      break;
    case 29:
      configTime(TZ_Asia_Shanghai, "pool.ntp.org", "time.nist.gov");
      break;
    case 30:
      configTime(TZ_Asia_Tokyo, "pool.ntp.org", "time.nist.gov");
      break;
    case 31:
      configTime(TZ_Australia_Sydney, "pool.ntp.org", "time.nist.gov");
      break;
    case 32:
      configTime(TZ_Pacific_Guam, "pool.ntp.org", "time.nist.gov");
      break;
    case 33:
      configTime(TZ_Pacific_Norfolk, "pool.ntp.org", "time.nist.gov");
      break;
    case 34:
      configTime(TZ_Pacific_Kosrae, "pool.ntp.org", "time.nist.gov");
      break;
    case 35:
      configTime(TZ_Pacific_Auckland, "pool.ntp.org", "time.nist.gov");
      break;
    case 36:
      configTime(TZ_Pacific_Wake, "pool.ntp.org", "time.nist.gov");
      break;
    case 37:
      configTime(TZ_Australia_Broken_Hill, "pool.ntp.org", "time.nist.gov");
      break;
    case 38:
      configTime(TZ_Australia_Darwin, "pool.ntp.org", "time.nist.gov");
      break;
    case 39:
      configTime(TZ_Pacific_Fakaofo, "pool.ntp.org", "time.nist.gov");
      break;
    case 40:
      configTime(TZ_Pacific_Kiritimati, "pool.ntp.org", "time.nist.gov");
      break;
    case 41:
      configTime(TZ_Asia_Kathmandu, "pool.ntp.org", "time.nist.gov");
      break;
    case 42:
      configTime(TZ_America_St_Johns, "pool.ntp.org", "time.nist.gov");
      break;
    case 43:
      configTime(TZ_Atlantic_South_Georgia, "pool.ntp.org", "time.nist.gov");
      break;
    case 44:
      configTime(TZ_Asia_Dhaka, "pool.ntp.org", "time.nist.gov");
      break;
    case 45:
      configTime(TZ_Pacific_Marquesas, "pool.ntp.org", "time.nist.gov");
      break;
    case 46:
      configTime(TZ_Asia_Tehran, "pool.ntp.org", "time.nist.gov");
      break;
    case 47:
      configTime(TZ_Asia_Kabul, "pool.ntp.org", "time.nist.gov");
      break;
    case 48:
      configTime(TZ_Indian_Cocos, "pool.ntp.org", "time.nist.gov");
      break;
    case 49:
      configTime(TZ_Australia_Eucla, "pool.ntp.org", "time.nist.gov");
      break;
    case 50:
      configTime(TZ_Australia_Lord_Howe, "pool.ntp.org", "time.nist.gov");
      break;
    case 51:
      configTime(TZ_Pacific_Chatham, "pool.ntp.org", "time.nist.gov");
      break;
    case 52:
      configTime(TZ_America_Adak, "pool.ntp.org", "time.nist.gov");
      break;
    case 53:
      configTime(TZ_America_Nuuk, "pool.ntp.org", "time.nist.gov");
      break;
    case 54:
      configTime(TZ_Atlantic_Azores, "pool.ntp.org", "time.nist.gov");
      break;
    default:
      configTime(TZ_Etc_Universal, "pool.ntp.org", "time.nist.gov");
      timeZone = 66;
      break;
  }
}

String UTCoffset(int zone) {
  switch (zone) {  //remember to update setLocalTimeZone() function and MAX_TZ definition when any changes are made here
    case 0:        //America Los Angeles
      return "<UTC-8*>";
    case 1:  //America Phoenix
      return "<UTC-7>";
    case 2:  //America Denver
      return "<UTC-7*>";
    case 3:  //America Chicago
      return "<UTC-6*>";
    case 4:  //America New York
      return "<UTC-5*>";
    case 5:  //America Anchorage
      return "<UTC-9*>";
    case 6:  //America Honolulu
      return "<UTC-10>";
    case 7:  //America Internaltion Date Line West (TZ ETC GMT p12)
      return "<UTC-12>";
    case 8:  //Pacific Midway
      return "<UTC-11>";
    case 9:  //Pacific Gambier (Alaska)
      return "<UTC-9>";
    case 10:  //Pacific Pitcarin
      return "<UTC-8>";
    case 11:  //America Edmonton
      return "<UTC-7>";
    case 12:  //America Mexico City
      return "<UTC-6>";
    case 13:  //America Cayman
      return "<UTC-5>";
    case 14:  //America Santiago
      return "<UTC-4*>";
    case 15:  //America Puerto Rico
      return "<UTC-4>";
    case 16:  //America Sao Paulo
      return "<UTC-3>";
    case 17:  //Atlantic Cape Verde
      return "<UTC-1>";
    case 18:  //Europe London
      return "<UTC*>";
    case 19:  //Atlantic Reykjavik
      return "<UTC>";
    case 20:  //Europe Paris
      return "<UTC+1*>";
    case 21:  //Africa Casablanca
      return "<UTC+1>";
    case 22:  //Europe Kiev
      return "<UTC+2*>";
    case 23:  //Africa Cairo
      return "<UTC+2>";
    case 24:  //Europe Moscow
      return "<UTC+3>";
    case 25:  //Asia Dubai
      return "<UTC+4>";
    case 26:  //Asia Karachi
      return "<UTC+5>";
    case 27:  //Asia Kolkata
      return "<UTC+5:30>";
    case 28:  //Asia Bangkok
      return "<UTC+7>";
    case 29:  //Asia Shanghai
      return "<UTC+8>";
    case 30:  //Asia Tokyo
      return "<UTC+9>";
    case 31:  //Australia Sydney
      return "<UTC+10*>";
    case 32:  //Pacific Guam
      return "<UTC+10>";
    case 33:  //Pacific Norfolk Island
      return "<UTC+11*>";
    case 34:  //Pacific Kosrae
      return "<UTC+11>";
    case 35:  //Pacific Auckland
      return "<UTC+12*>";
    case 36:  //Pacific Wake Islands
      return "<UTC+12>";
    case 37:  //Australia Broken Hill
      return "<UTC+9:30*>";
    case 38:  //Australia Darwin
      return "<UTC+9:30>";
    case 39:  //Pacific Fakaofo
      return "<UTC+13>";
    case 40:  //Pacific Kiritimati
      return "<UTC+14>";
    case 41:  //Asia Kathmandu (Nepal)
      return "<UTC+5:45>";
    case 42:  //America St Johns
      return "<UTC-3:30*>";
    case 43:  //Atlantic South Georgia
      return "<UTC-2>";
    case 44:  //Asia Dhaka
      return "<UTC+6>";
    case 45:  //Pacific Marquesas
      return "<UTC-9:30>";
    case 46:  //Asia Tehran
      return "<UTC+3:30>";
    case 47:  //Asia Kabul
      return "<UTC+4:30>";
    case 48:  //Indian Cocos
      return "<UTC+6:30>";
    case 49:  //Australia Eucla
      return "<UTC+8:45>";
    case 50:  //Australia Lord Howe
      return "<UTC+10:30*>";
    case 51:  //Pacific Chatham
      return "<UTC+12:45*>";
    case 52:  //America Adak
      return "<UTC-10*>";
    case 53:  //America Nuuk
      return "<UTC-3*>";
    case 54:  //Atlantic Azores
      return "<UTC-1*>";
    default:  //TZ Etc Universal
      return "<UTC>";
  }
}
void setDateTime() {
  if (GETTIME) {
    setLocalTimeZone(timeZone);
    print("Waiting for NTP time sync: ");
    unsigned long loopStart = millis();
    //configTime(0,0,"pool.ntp.org");
    time_t now = time(nullptr);
    while (now < 1000) {
      //led_ON(); delay(50);
      if (outputSerial) print("*");
      now = time(nullptr);
      delay(1000);
      //led_OFF(); delay(50);
      if ((millis() - loopStart) > SIXTY_SECONDS) {
        noTime = true;
        break;
      }
    }
    struct tm timeinfo;
    if (outputSerial) {
      Serial.println();
      if (noTime) Serial.print("Server Timeout - No Time Synced!\n");
      else {
        Serial.printf("Current Universal Time: UTC %s", currentTime());
        localtime_r(&now, &timeinfo);
        Serial.printf("Current Local Time: %s %s", tzname[0], currentTime(true));
        Serial.println();
      }
    }
  } else noTime = true;
}

void determineDayNight() {
  if (SLEEPALLOW && GETTIME) {
    struct tm timeinfo;
    time_t now = time(nullptr);
    localtime_r(&now, &timeinfo);
    if ((timeinfo.tm_hour >= 23) || (timeinfo.tm_hour < COFFEE_RESET)) {
      dayTime = false;
      if ((timeinfo.tm_hour >= 2) && (timeinfo.tm_hour <= 5)) sleepMode(4);  //sleep for 60 min
      else sleepMode(2);                                                     //sleep for 30 min
      return;
    }
    if ((timeinfo.tm_hour == COFFEE_RESET) && coffee) {
      coffee = false;
      coffeeMemUpdate = true;
    }
  }
}

char *currentTime(bool local, int minutes) {
  if (noTime) return "Unknown Time!\n";
  unsigned long loopStart = millis();
  time_t now = time(nullptr), newTime;
  while (now < 1000) {
    delay(50);
    print("-");
    now = time(nullptr);
    if ((millis() - loopStart) > (SIXTY_SECONDS / 10)) {
      break;
    }
  }
  struct tm timeinfo;
  size_t seconds = minutes * 60;
  newTime = now + seconds;
  memset(&timeinfo, '\0', sizeof(struct tm));
  if (local) localtime_r(&newTime, &timeinfo);
  else gmtime_r(&newTime, &timeinfo);
  return asctime(&timeinfo);
}

/////////////////////////MQTT FUNCTIONS//////////////////////////////////////////////////////////////////
void callback(char *topic, byte *payload, unsigned int length) {
  String payloadString, payloadClean, localMessage;
  for (int count = 0; count < length; count++) payloadString += (String(payload[count]));
  if (strcmp(topic, publishTopicAwake) == 0) {  //if time awake stored
    if (millis() - originalStartTime <= 2550)
      if (!noTime) processCmdAwake(payloadString, length);
    client.unsubscribe(publishTopicAwake);
    return;
  }
  if (strcmp(topic, publishTopicSleep) == 0) {  //if travel time sleep stored
    if (millis() - originalStartTime <= 2550)
      if (!noTime && !coffee) processCmdTravelSleep(payloadString, length);
    client.unsubscribe(publishTopicSleep);
    return;
  }
  payloadClean = sanatizePayload(payloadString, length);
  length = payloadLength(payloadClean, length);
  payloadString = messageConversion(payloadClean, length);
  String truncatedMessage = payloadString.substring(0, length);
  payloadClean = (char *)topic;
  if (payloadString.equalsIgnoreCase(F(COMMAND_15)) || payloadString.equalsIgnoreCase(F(COMMAND_22)) || payloadString.equalsIgnoreCase(F(COMMAND_23))) {
    timeZone = globalPayload;
    localMessage = "Message from [" + payloadClean + "]: " + truncatedMessage + " " + String(globalPayload) + " at " + tzname[0] + " " + currentTime(true);
  } else localMessage = "Message from [" + payloadClean + "]: " + truncatedMessage + " at " + tzname[0] + " " + currentTime(true);
  print((char *)localMessage.c_str());
  acknowledgeCmdBlink();
  processCommand(payloadString);
}

int characterIndex(int value) {
  if (32 <= value && value <= 99) return 2;
  else return 3;
}

int payloadLength(String payload, unsigned int length) {
  int payloadIndex = 0, nextIndex, refind, result, spaceCount = 0;
  for (int index = 0; index < length; index++) {
    result = (String(payload[payloadIndex]) + String(payload[payloadIndex + 1])).toInt();
    if (result == 32) {
      if (((String(payload[payloadIndex + 2]) + String(payload[payloadIndex + 3])).toInt() == 32) || (length - index) <= 1) {
        spaceCount++;
        refind = index + 1;
        nextIndex = payloadIndex + 2;
        //count extra spaces in front of initial space
        while (refind < length) {
          result = (String(payload[nextIndex]) + String(payload[nextIndex + 1])).toInt();
          if (result != 32) break;
          else spaceCount++;
          nextIndex += 2;
          refind++;
        }
        return (length - spaceCount);
      }
    }  //if the tail spaces
    payloadIndex += characterIndex((String(payload[payloadIndex]) + String(payload[payloadIndex + 1])).toInt());
    spaceCount = 0;
  }
  return (length - spaceCount);
}

String sanatizePayload(String payload, unsigned int length) {
  int payloadIndex = 0, nextIndex, nextPayloadIndex, refind, result, extraDebugSpace = 0, spaceCount = 0, numberFromPayload = 0;
  bool save = true;
  char charOne, charTwo;
  for (int index = 0; index < length; index++) {
    result = (String(payload[payloadIndex]) + String(payload[payloadIndex + 1])).toInt();
    if (result == 32) {
      if (save) {
        charOne = payload[payloadIndex];
        charTwo = payload[payloadIndex + 1];
        save = false;
      }
      spaceCount++;
      refind = index + 1;
      nextIndex = payloadIndex + 2;
      //count extra spaces in front of initial space
      while (refind < length) {
        result = (String(payload[nextIndex]) + String(payload[nextIndex + 1])).toInt();
        if (result != 32) break;
        else spaceCount++;
        nextIndex += 2;
        refind++;
      }
      nextPayloadIndex = payloadIndex;
      if (index == 0) extraDebugSpace = 0;
      else {
        extraDebugSpace = 1;
        nextPayloadIndex += 2;
      }
      spaceCount -= extraDebugSpace;
      if ((refind < length) && (spaceCount > 0)) {  //if not the tail spaces
        for (refind; refind < length; refind++) {
          payload[nextPayloadIndex] = payload[nextIndex];
          payload[nextPayloadIndex + 1] = payload[nextIndex + 1];
          numberFromPayload = characterIndex((String(payload[nextIndex]) + String(payload[nextIndex + 1])).toInt());
          if (numberFromPayload == 3) payload[nextPayloadIndex + 2] = payload[nextIndex + 2];
          nextPayloadIndex += numberFromPayload;
          nextIndex += numberFromPayload;
        }
        for (refind = length - spaceCount; refind < length; refind++) {
          payload[nextPayloadIndex] = charOne;
          payload[nextPayloadIndex + 1] = charTwo;
          nextPayloadIndex += 2;
        }
      } else if (refind >= length) return payload;
    }
    payloadIndex += characterIndex((String(payload[payloadIndex]) + String(payload[payloadIndex + 1])).toInt());
    spaceCount = 0;
  }
  return payload;
}

String messageConversion(String message, unsigned int length) {
  String payloadString, localTimeZone;
  int messageIndex = 0, nextIndex = 0, indexRef = 0, result, payloadTimeZone = -1;
  for (int index = 0; index < length; index++) {
    if (payloadString.equalsIgnoreCase(F(COMMAND_15)) || payloadString.equalsIgnoreCase(F(COMMAND_22)) || payloadString.equalsIgnoreCase(F(COMMAND_23)) || payloadString.equalsIgnoreCase(F(COMMAND_30)) || payloadString.equalsIgnoreCase(F(COMMAND_31)) || payloadString.equalsIgnoreCase(F(COMMAND_32)) || payloadString.equalsIgnoreCase(F(COMMAND_33)) || payloadString.equalsIgnoreCase(F(COMMAND_34))) {
      int digits = 2;
      if (payloadString.equalsIgnoreCase(F(COMMAND_22)) || payloadString.equalsIgnoreCase(F(COMMAND_23))) digits = 3;
      result = (String(message[messageIndex]) + String(message[messageIndex + 1])).toInt();
      nextIndex = messageIndex;
      indexRef = index;
      if (result == 32) {
        nextIndex += 2;
        indexRef++;
      }
      for (int q = 0; q <= digits; q++) {  //process digits
        if (indexRef < length) {
          result = (String(message[nextIndex]) + String(message[nextIndex + 1])).toInt();
          if (48 <= result && result <= 57) {
            localTimeZone += char(result);
            nextIndex += 2;
            indexRef++;
          } else break;
        } else {
          if (q != 0) payloadTimeZone = localTimeZone.toInt();
          break;
        }
      }
      //check for more digits
      if (indexRef < length) {
        if ((String(message[nextIndex]) + String(message[nextIndex + 1])).toInt() == 32) {
          indexRef++;
          nextIndex += 2;
          if (indexRef < length)
            if ((String(message[nextIndex]) + String(message[nextIndex + 1])).toInt() == 32) payloadTimeZone = localTimeZone.toInt();
            else payloadTimeZone = localTimeZone.toInt();
        }
      }
      globalPayload = payloadTimeZone;
      if (payloadString.equalsIgnoreCase(F(COMMAND_15))) {
        if ((0 <= payloadTimeZone) && (payloadTimeZone <= MAX_TZ)) return COMMAND_15;
      }
      if (payloadString.equalsIgnoreCase(F(COMMAND_22))) return COMMAND_22;
      if (payloadString.equalsIgnoreCase(F(COMMAND_23))) return COMMAND_23;
      if (payloadString.equalsIgnoreCase(F(COMMAND_30))) return COMMAND_30;
      if (payloadString.equalsIgnoreCase(F(COMMAND_31))) return COMMAND_31;
      if (payloadString.equalsIgnoreCase(F(COMMAND_32))) return COMMAND_32;
      if (payloadString.equalsIgnoreCase(F(COMMAND_33))) return COMMAND_33;
      if (payloadString.equalsIgnoreCase(F(COMMAND_34))) return COMMAND_34;
    }
    nextIndex = characterIndex((String(message[messageIndex]) + String(message[messageIndex + 1])).toInt());
    if (nextIndex == 2) result = (String(message[messageIndex]) + String(message[messageIndex + 1])).toInt();
    else result = (String(message[messageIndex]) + String(message[messageIndex + 1]) + String(message[messageIndex + 2])).toInt();
    messageIndex += nextIndex;
    payloadString += char(result);
  }
  if (payloadString.equalsIgnoreCase(F(COMMAND_1))) return COMMAND_1;  //COMMAND LIST HERE
  if (payloadString.equalsIgnoreCase(F(COMMAND_2))) return COMMAND_2;
  if (payloadString.equalsIgnoreCase(F(COMMAND_3))) return COMMAND_3;
  if (payloadString.equalsIgnoreCase(F(COMMAND_4))) return COMMAND_4;
  if (payloadString.equalsIgnoreCase(F(COMMAND_5))) return COMMAND_5;
  if (payloadString.equalsIgnoreCase(F(COMMAND_6))) return COMMAND_6;
  if (payloadString.equalsIgnoreCase(F(COMMAND_7))) return COMMAND_7;
  if (payloadString.equalsIgnoreCase(F(COMMAND_8))) return COMMAND_8;
  if (payloadString.equalsIgnoreCase(F(COMMAND_9))) return COMMAND_9;
  if (payloadString.equalsIgnoreCase(F(COMMAND_10))) return COMMAND_10;
  if (payloadString.equalsIgnoreCase(F(COMMAND_11))) return COMMAND_11;
  if (payloadString.equalsIgnoreCase(F(COMMAND_12))) return COMMAND_12;
  if (payloadString.equalsIgnoreCase(F(COMMAND_13))) return COMMAND_13;
  if (payloadString.equalsIgnoreCase(F(COMMAND_14))) return COMMAND_14;
  //if(payloadString.equalsIgnoreCase(F(COMMAND_15))) return COMMAND_15;
  if (payloadString.equalsIgnoreCase(F(COMMAND_16))) return COMMAND_16;
  if (payloadString.equalsIgnoreCase(F(COMMAND_17))) return COMMAND_17;
  if (payloadString.equalsIgnoreCase(F(COMMAND_18))) return COMMAND_18;
  if (payloadString.equalsIgnoreCase(F(COMMAND_19))) return COMMAND_19;
  if (payloadString.equalsIgnoreCase(F(COMMAND_20))) return COMMAND_20;
  if (payloadString.equalsIgnoreCase(F(COMMAND_21))) return COMMAND_21;
  //if(payloadString.equalsIgnoreCase(F(COMMAND_22))) return COMMAND_22;
  //if(payloadString.equalsIgnoreCase(F(COMMAND_23))) return COMMAND_23;
  if (payloadString.equalsIgnoreCase(F(COMMAND_24))) return COMMAND_24;
  if (payloadString.equalsIgnoreCase(F(COMMAND_25))) return COMMAND_25;
  if (payloadString.equalsIgnoreCase(F(COMMAND_26))) return COMMAND_26;
  if (payloadString.equalsIgnoreCase(F(COMMAND_27))) return COMMAND_27;
  if (payloadString.equalsIgnoreCase(F(COMMAND_28))) return COMMAND_28;
  if (payloadString.equalsIgnoreCase(F(COMMAND_29))) return COMMAND_29;
  //if(payloadString.equalsIgnoreCase(F(COMMAND_30))) return COMMAND_30;
  //if(payloadString.equalsIgnoreCase(F(COMMAND_31))) return COMMAND_31;
  //if(payloadString.equalsIgnoreCase(F(COMMAND_32))) return COMMAND_32;
  //if(payloadString.equalsIgnoreCase(F(COMMAND_33))) return COMMAND_33;
  //if(payloadString.equalsIgnoreCase(F(COMMAND_34))) return COMMAND_34;
  if (payloadString.equalsIgnoreCase(F(COMMAND_35))) return COMMAND_35;
  if (payloadString.equalsIgnoreCase(F(COMMAND_36))) return COMMAND_36;
  return payloadString;
}

void processCommand(String command) {
  bool extendSleepTime = true;
  printSpace = true;
  String truncatedMessage = command.substring(0, 18);
  if (truncatedMessage == "") truncatedMessage = "[Blank]";
  String commandMessage = "Received command " + truncatedMessage;
  publish((char *)commandMessage.c_str());
  int commandCodeResult = commandCode(command);
  if ((commandCodeResult > 0) || (commandCodeResult <= 13)) turnIOPinsOn();
  switch (commandCodeResult) {
    case 1:  // Status
      computerStatusPrint();
      statusPrint();
      break;
    case 2:  //On
      commandCall = true;
      if (currentStatus != ON) {
        publish("Powering On...");
        pressPowerButton(BUTTON_PRESS_TIME);
      } else publish("Machine already on");
      break;
    case 3:  //Off or Sleep
      commandCall = true;
      if (currentStatus == ON) {
        //publish("Power Off...");
        publish("Sleeping machine...");
        pressPowerButton(BUTTON_PRESS_TIME);
      } else publish("Machine already off");
      break;
    case 4:  //Reset
      {
        unsigned long releaseTime = millis() + 500;
        commandCall = true;
        digitalWrite(PIN_RESET_BUTTON_D7, HIGH);
        while (millis() < releaseTime) {}
        digitalWrite(PIN_RESET_BUTTON_D7, LOW);
        delay(10);
        digitalWrite(PIN_RESET_BUTTON_D7, HIGH);
        delay(10);
        digitalWrite(PIN_RESET_BUTTON_D7, LOW);
        publish("Reset Button Pressed");
        break;
      }
    case 5:  //Force Off
      commandCall = true;
      if (currentStatus != OFF) {
        publish("Forcing off...");
        forceOff = true;
        pressPowerButton(10000);
      } else publish("Power is already OFF!");
      break;
    case 6:  //pressPwr
      commandCall = true;
      digitalWrite(PIN_POWER_BUTTON_D6, HIGH);
      publish("Power Button Pressed");
      break;
    case 7:  //releasePwr
      digitalWrite(PIN_POWER_BUTTON_D6, LOW);
      publish("Power Button Released");
      break;
    case 8:  //Light On
      setLightStatus(ON);
      break;
    case 9:  //Light Off
      setLightStatus(OFF);
      break;
    case 10:  //Light Auto
      setLightStatus(AUTO);
      break;
    case 11:     //Board LED ON
      led_ON();  // Turn the LED on (Note that LOW is the voltage level
      publish("BOARD LED ON");
      break;
    case 12:      //Board LED OFF
      led_OFF();  // Turn the LED off by making the voltage HIGH
      publish("BOARD LED OFF");
      break;
    case 13:  //Sleepy5
      if (commandCall) {
        commandCall = false;
        publish("Waiting for status change, try again");
      } else {
        if (coffee) {
          coffee = false;
          coffeeMemUpdate = true;
          publish("Decaffenated!");
        }
        if (sleepCountdownCode == -1) {  //confirming call for travel sleep
          publish("Travel Sleep Confirmed!");
          if (hibernate) stopHibernate();  //but already set to hibernate...
          client.publish(publishTopicAwake, "", true);
          travelTimeStopTime = globalPayload;
          publishState = -1;
          publishSleep(travelTimeStopTime);
          startDeepSleep(globalPayload);
        } else if (sleepCountdownCode == -2) {  //confirming call for hibernate
          publish("Confirmed! Hibernation Started!");
          if (travelTimeStopTime > 0) travelTimeStopTime = 0;
          publishState = -2;
          if (!hibernate) {
            hibernate = true;
            hibernateMemUpdate = true;
          }
          startDeepSleep(MAX_ESP_SLEEP);
        } else startDeepSleep(1);
      }
      break;
    case 14:  //Awaken63
      stayAwake = true;
      sleepMode(0, STAY_AWAKE);
      publish("Auto Sleep OFF");
      publishTime(STAY_AWAKE);
      if (hibernate || sleepCountdownCode == -2) stopHibernate();
      if (travelTimeStopTime > 0 || sleepCountdownCode == -1) stopTravelSleep();
      if (!publishState) publishBoardState();
      publishState = 2;
      extendSleepTime = false;
      break;
    case 15:  //Time Zone: n
      commandMessage = "Time Zone changed to #" + String(timeZone);
      timeZoneMemUpdate = true;
      setLocalTimeZone(timeZone);
      publish((char *)commandMessage.c_str());
      break;
    case 16:  //Time Check
      commandMessage = "Current Time Zone #" + String(timeZone) + " (" + tzname[0] + ") " + UTCoffset(timeZone);
      publish(COMMAND_16);
      publish((char *)commandMessage.c_str());
      break;
    case 17:  //heartbeat command
      heartbeat();
      break;
    case 18:  //board status
      {
        unsigned long deltaMilliSeconds = (timeAwake - (millis() - awakeStartTime));
        sleepMode(3);
        boardStatus(deltaMilliSeconds);
        extendSleepTime = false;
        break;
      }
    case 19:  //wifi Status
      wifiStatus();
      break;
    case 20:  //Caffen8
      caffenate();
      break;
    case 21:  //countdown
      {
        int secs = ((timeAwake - (millis() - awakeStartTime))) / 1000;
        if (secs > 60) {
          secs /= 60;
          if (secs > 1) commandMessage = String("About ") + String(secs) + String(" mins till board sleeps");
          else commandMessage = String("A little over ") + String(secs) + String(" min till board sleeps");
        } else commandMessage = String(secs) + String("s till board sleeps");
        publish((char *)commandMessage.c_str());
        extendSleepTime = false;
        break;
      }
    case 22:  //Travel: n
      if (globalPayload <= 0) {
        publish("Invalid Travel Duration!");
        break;
      }
      if (!coffee) {
        if (globalPayload >= MAX_TRAVEL_SLEEP_TIME) globalPayload = MAX_TRAVEL_SLEEP_TIME;
        commandMessage = "Confirm Travel Sleep for ";
        if (globalPayload > 24) commandMessage += String(globalPayload / 24) + " days, " + String(globalPayload % 24) + " hours";
        else commandMessage += String(globalPayload) + " hours";
        publish((char *)commandMessage.c_str());
        globalPayload *= 60;
        sleepMode(5);
      } else publish("Can't Travel Sleep while Caffeinated!");
      extendSleepTime = false;
      break;
    case 23:  //Awake: n
      if (globalPayload <= 0) {
        publish("Invalid Awake Duration!");
        break;
      }
      stayAwake = true;
      sleepMode(0, globalPayload);
      commandMessage = "Staying awake for " + String(globalPayload) + " minutes";
      publish((char *)commandMessage.c_str());
      publishTime(globalPayload);
      if (hibernate || sleepCountdownCode == -2) stopHibernate();
      if (travelTimeStopTime > 0 || sleepCountdownCode == -1) stopTravelSleep();
      if (!publishState) publishBoardState();
      publishState = 2;
      extendSleepTime = false;
      break;
    case 24:  //LocUTC
      if (local == false) {
        local = true;
        publish("Publishing Local Time");
        if (rtcData.local != local) localTZMemUpdate = true;
        else localTZMemUpdate = false;
      } else {
        local = false;
        publish("Publishing UTC Time");
        if (rtcData.local != local) localTZMemUpdate = true;
        else localTZMemUpdate = false;
      }
      break;
    case 25:  //[empty command]
      //publish("Blank Command");
      extendSleepTime = false;
      break;
    case 26:  //RTC Status
      rtcStatus();
      break;
    case 27:  //Board Restart
      publish("Restarting Board");
      delay(1000);
      restart();
      break;
    case 28:  //Board RST Button
      publish("Restarting Board via DO0 and RST");
      delay(1000);
      rstButton();
      publish("DO0 and RST disconnected!");
      break;
    case 29:  //Hibernate
      if (coffee) publish("Can't Hibernate while Caffeinated!");
      else {
        if (!hibernate && sleepCountdownCode != -2) {
          publish("Confirm Hibernate");
          sleepMode(6);
        } else publish("Already Set to Hibernate");
      }
      extendSleepTime = false;
      break;
    case 30:  // Day Sleep: n
      if (globalPayload <= 0) {
        publish("Invalid Day Sleep Duration!");
        break;
      }
      if (3 > globalPayload || globalPayload > 15) {
        publish("Pick between (3-15) mins for Day Sleep");
        break;
      }
      if (rtcData.daySleep == globalPayload) publish("Sleep Time Already Set!");
      else {
        String localMessage = String("Day Sleep Changed to: ") + globalPayload + String(" minutes");
        publish((char *)localMessage.c_str());
        localMessage = String("Coffee Sleep Changed to: ") + String(globalPayload - 1) + String(" minutes");
        publish((char *)localMessage.c_str());
        rtcData.daySleep = globalPayload;
        rtcData.coffeeSleep = globalPayload - 1;
        timeMemUpdate = true;
      }
      break;
    case 31:  // Night Sleep: n
      if (globalPayload <= 0) {
        publish("Invalid Night Sleep Duration!");
        break;
      }
      if (15 > globalPayload || globalPayload > 60) {
        publish("Pick between (15-60) mins for Night Sleep");
        break;
      }
      if (rtcData.nightSleep == globalPayload) publish("Sleep Time Already Set!");
      else {
        String localMessage = String("Night Sleep Changed to: ") + globalPayload + String(" minutes");
        publish((char *)localMessage.c_str());
        rtcData.nightSleep = globalPayload;
        timeMemUpdate = true;
      }
      break;
    case 32:  // Travel Sleep: n
      if (globalPayload <= 0) {
        publish("Invalid Travel Sleep Duration!");
        break;
      }
      if (1 > globalPayload || globalPayload > 3) {
        publish("Pick between (1-3) hours for Travel Sleep");
        break;
      }
      if (rtcData.travelSleep == (globalPayload * 60)) publish("Sleep Time Already Set!");
      else {
        String localMessage = String("Travel Sleep Changed to: ") + globalPayload + String(" hours");
        publish((char *)localMessage.c_str());
        rtcData.travelSleep = (globalPayload * 60);
        timeMemUpdate = true;
      }
      break;
    case 33:  // Hibernate Sleep: n
      if (globalPayload <= 0) {
        publish("Invalid Hibernate Sleep Duration!");
        break;
      }
      if (1 > globalPayload || globalPayload > 3) {
        publish("Pick between (1-3) hours for Hibernate Sleep");
        break;
      }
      if (rtcData.hibernateSleep == (globalPayload * 60)) publish("Sleep Time Already Set!");
      else {
        String localMessage = String("Hibernate Sleep Changed to: ") + globalPayload + String(" hours");
        publish((char *)localMessage.c_str());
        rtcData.hibernateSleep = (globalPayload * 60);
        timeMemUpdate = true;
      }
      break;
    case 34:  // Coffee Sleep: n
      if (globalPayload <= 0) {
        publish("Invalid Coffee Sleep Duration!");
        break;
      }
      if (2 > globalPayload || globalPayload > 14) {
        String localMessage = "Pick between (2-14) mins for Coffee Sleep";
        publish((char *)localMessage.c_str());
        break;
      }
      if (rtcData.coffeeSleep == globalPayload) publish("Sleep Time Already Set!");
      else {
        String localMessage = String("Coffee Sleep Changed to: ") + globalPayload + String(" minutes");
        publish((char *)localMessage.c_str());
        localMessage = String("Day Sleep Changed to: ") + String(globalPayload + 1) + String(" minutes");
        publish((char *)localMessage.c_str());
        rtcData.coffeeSleep = globalPayload;
        rtcData.daySleep = globalPayload + 1;
        timeMemUpdate = true;
      }
      break;
    case 35:  // Time Status Print
      timePrint();
      break;
    case 36:
      {  //Sleep defaults
        bool rtcUpdate = false;
        if (rtcData.daySleep != DEFAULT_DAY_SLEEP_TIME) rtcUpdate = true;
        if (rtcData.nightSleep != DEFAULT_NIGHT_SLEEP_TIME) rtcUpdate = true;
        if (rtcData.coffeeSleep != COFFEE_SLEEP) rtcUpdate = true;
        if (rtcData.travelSleep != DEFAULT_TRAVEL_SLEEP) rtcUpdate = true;
        if (rtcData.hibernateSleep != DEFAULT_HIBERNATE_SLEEP) rtcUpdate = true;
        if (rtcUpdate) {
          setSleepDefault();
          timeMemUpdate = true;
        }
        timePrint();
        publish("Sleep defaults set!");
        break;
      }
    default:  //unknown command
      publish("Unknown Command");
      extendSleepTime = false;
      break;
  }
  if (extendSleepTime) sleepMode(3);
  if (millis() - originalStartTime <= 1500) {
    client.unsubscribe(subscribeTopic);
    client.publish(subscribeTopic, "", true);
    delay(10);
    client.subscribe(subscribeTopic);
  }
}

int commandCode(String message) {
  if (message == COMMAND_1) return 1;
  if (message == COMMAND_2) return 2;
  if (message == COMMAND_3) return 3;
  if (message == COMMAND_4) return 4;
  if (message == COMMAND_5) return 5;
  if (message == COMMAND_6) return 6;
  if (message == COMMAND_7) return 7;
  if (message == COMMAND_8) return 8;
  if (message == COMMAND_9) return 9;
  if (message == COMMAND_10) return 10;
  if (message == COMMAND_11) return 11;
  if (message == COMMAND_12) return 12;
  if (message == COMMAND_13) return 13;
  if (message == COMMAND_14) return 14;
  if (message == COMMAND_15) return 15;
  if (message == COMMAND_16) return 16;
  if (message == COMMAND_17) return 17;
  if (message == COMMAND_18) return 18;
  if (message == COMMAND_19) return 19;
  if (message == COMMAND_20) return 20;
  if (message == COMMAND_21) return 21;
  if (message == COMMAND_22) return 22;
  if (message == COMMAND_23) return 23;
  if (message == COMMAND_24) return 24;
  if (message == COMMAND_25) return 25;
  if (message == COMMAND_26) return 26;
  if (message == COMMAND_27) return 27;
  if (message == COMMAND_28) return 28;
  if (message == COMMAND_29) return 29;
  if (message == COMMAND_30) return 30;
  if (message == COMMAND_31) return 31;
  if (message == COMMAND_32) return 32;
  if (message == COMMAND_33) return 33;
  if (message == COMMAND_34) return 34;
  if (message == COMMAND_35) return 35;
  if (message == COMMAND_36) return 36;
  return 0;
}

void clearAwakeSleepChannels() {
  client.unsubscribe(publishTopicAwake);
  delay(50);
  client.unsubscribe(publishTopicSleep);
  if (stayAwake) {
    //client.publish(publishTopicAwake, "", true);
    client.publish(publishTopicSleep, "", true);
    return;
  }
}

void processCmdAwake(String payload, unsigned int length) {
  clearAwakeSleepChannels();
  if (stayAwake) return;
  unsigned long long awakeTill = 0, sizeTimeMessage;
  time_t now = time(nullptr);
  int individualNumber, payloadIndex = 0;
  if (length <= 10) {
    for (int count = 0; count < length; count++) {
      individualNumber = asciiToInt((String(payload[payloadIndex]) + String(payload[payloadIndex + 1])).toInt());
      if (individualNumber >= 0) {
        sizeTimeMessage = 1;
        for (int countInner = 0; countInner < (length - count - 1); countInner++) sizeTimeMessage *= 10;
        sizeTimeMessage *= individualNumber;
        awakeTill += sizeTimeMessage;
      } else {
        publish("Invalid Data, Awake Channel");
        client.publish(publishTopicAwake, "", true);
        return;
      }
      payloadIndex += 2;
    }
    long duration = (awakeTill - now) / 60;
    if (1 <= duration && duration <= STAY_AWAKE) {  //resuming sleep
      sleepMode(0, duration);
      payload = "Resume staying awake for " + String(duration) + " minutes";
      snprintf(globalMessage, MSG_BUFFER_SIZE, "%s", payload);
      publish((char *)payload.c_str());
      //client.publish(publishTopicAwake, "", true);
      if (publishState != 2) publishBoardState();
      publishState = 2;
      return;
    } else publish("Invalid Data, Awake Channel");
  } else publish("Invalid Data, Awake Channel");
  if (rtcValid) client.publish(publishTopicAwake, "", true);
  return;
}

void processCmdTravelSleep(String payload, unsigned int length) {
  clearAwakeSleepChannels();
  if (stayAwake) return;
  unsigned long long sleepTill = 0, sizeTimeMessage;
  time_t now = time(nullptr);
  int individualNumber, payloadIndex = 0;
  String partialMessage;
  if (length <= 10) {
    for (int count = 0; count < length; count++) {
      individualNumber = asciiToInt((String(payload[payloadIndex]) + String(payload[payloadIndex + 1])).toInt());
      if (individualNumber >= 0) {
        sizeTimeMessage = 1;
        for (int countInner = 0; countInner < (length - count - 1); countInner++) sizeTimeMessage *= 10;
        sizeTimeMessage *= individualNumber;
        sleepTill += sizeTimeMessage;
      } else {
        publish("Invalid Data, Sleep Channel");
        clearSleepChannel();
        return;
      }
      payloadIndex += 2;
    }
    long duration = (sleepTill - now) / 60;
    if (1 <= duration && (duration / 60) <= MAX_TRAVEL_SLEEP_TIME) {  //resuming sleep
      payload = "Travel Sleep Mode Engaged for " + String(duration) + " minutes";
      if (duration > 1440) {
        if ((duration % 1440) % 60 < 10) partialMessage = "0";
        else partialMessage = "";
        payload = "Travel Sleep Mode Engaged for " + String(duration / 1440) + " days, (" + String((duration % 1440) / 60) + ":" + partialMessage + String((duration % 1440) % 60) + ") hours";
      } else {
        if ((duration % 60) < 10) partialMessage = "0";
        else partialMessage = "";
        payload = "Travel Sleep Mode Engaged for (" + String(duration / 60) + ":" + partialMessage + String(duration % 60) + ") hours";
      }
      snprintf(globalMessage, MSG_BUFFER_SIZE, "%s", payload);
      publish((char *)payload.c_str());
      printSp();
      travelTimeStopTime = duration;  //setting travelTimeStopTime in minutes!
      if (hibernate) stopHibernate();
      if (publishState != -1) publishBoardState();
      publishState = -1;
      sleepMode(7);
      return;
    }
    if (MAX_ESP_SLEEP <= (duration * -1)) {
      publish("Travel Sleep Mode Ended!");
      client.publish(publishTopicSleep, "", true);
    } else publish("Invalid Data, Sleep Channel");
  } else publish("Invalid Data, Sleep Channel");
  if (rtcValid) clearSleepChannel();
  return;
}

/////////////////////////LED BLINK FUNCTIONS/////////////////////////////////////////////////////////////
void led_ON() {
  digitalWrite(BUILTIN_LED, LOW);
  ledState = true;
}

void led_OFF() {
  digitalWrite(BUILTIN_LED, HIGH);
  ledState = false;
}

void setupCompleteBlink() {
  int count = millis();
  while (millis() <= count + 1500) {
    led_ON();
    delay(14);
    led_OFF();
    delay(41);
  }
  for (int count = 12; count <= 45; count += 4) {
    led_ON();
    delay(count);
    led_OFF();
    delay(2 * count);
  }
  if (!outputSerial) pinMode(BUILTIN_LED, INPUT);
}

void sleepBlink() {
  if (!outputSerial) pinMode(BUILTIN_LED, OUTPUT);
  led_OFF();
  for (int count = 0; count < 11; count++) {
    led_ON();
    delay(250);
    led_OFF();
    delay(100);
    led_ON();
    delay(50);
    led_OFF();
    delay(100);
  }
  if (!outputSerial) pinMode(BUILTIN_LED, INPUT);
}

void heartbeatBlink() {
  bool led = ledState;
  if (!outputSerial) pinMode(BUILTIN_LED, OUTPUT);
  if (firstLoop) {
    led_OFF();
    delay(150);
    led_ON();
    delay(150);
    led_OFF();
    if (ledState) led_ON();
    if (!outputSerial) pinMode(BUILTIN_LED, INPUT);
    return;
  }
  if (led) led_OFF();
  delay(250);
  for (int outerCount = 0; outerCount < 2; outerCount++) {
    for (int innerCount = 0; innerCount < 2; innerCount++) {
      led_ON();
      delay(150);
      led_OFF();
      delay(250);
    }
    delay(500);
  }
  if (led) led_ON();
  if (!outputSerial) pinMode(BUILTIN_LED, INPUT);
}

void mqttConnFailBlink() {
  bool led = ledState;
  if (!outputSerial) pinMode(BUILTIN_LED, OUTPUT);
  if (led) led_OFF();
  delay(250);
  for (int count = 0; count < 5; count++) {
    led_ON();
    delay(250);
    led_OFF();
    delay(350);
  }
  delay(2600);
  if (led) led_ON();
  if (!outputSerial) pinMode(BUILTIN_LED, INPUT);
}

void acknowledgeCmdBlink() {
  bool led = ledState;
  if (!outputSerial) pinMode(BUILTIN_LED, OUTPUT);
  led_OFF();
  delay(150);
  led_ON();
  delay(150);
  led_OFF();
  delay(250);
  if (led) led_ON();
  if (!outputSerial) pinMode(BUILTIN_LED, INPUT);
}

/////////////////////////SLEEP FUNCTIONS/////////////////////////////////////////////////////////////////
void stopHibernate() {  //stops hibernation
  if (hibernate) hibernateMemUpdate = true;
  hibernate = false;
  publish("Hibernate to be Cancelled!");
}

void stopTravelSleep() {  // stops travel sleeping
  publish("Travel Sleep Cancelled!");
  travelTimeStopTime = 0;
  clearSleepChannel();
}

void sleepMode(int sleepMode, int duration) {
  switch (sleepMode) {
    case 0:
      timeAwake = duration * SIXTY_SECONDS;
      sleepCountdownCode = 0;
      publishTime(duration);
      awakeStartTime = millis();
      break;
    case 1:  //normal day sleep
      if (rtcValid) SLEEPTIME = rtcData.daySleep;
      else SLEEPTIME = DEFAULT_DAY_SLEEP_TIME;
      break;
    case 2:  //normal night sleep
      if (!coffee) {
        if (rtcValid) SLEEPTIME = rtcData.nightSleep;
        else SLEEPTIME = DEFAULT_NIGHT_SLEEP_TIME;
      }
      break;
    case 3:
      if (timeAwake < DEFAULT_AWAKE_TIME_AFTER_CALL * SIXTY_SECONDS) {
        timeAwake = DEFAULT_AWAKE_TIME_AFTER_CALL * SIXTY_SECONDS;
        publishTime(DEFAULT_AWAKE_TIME_AFTER_CALL);
      }
      reportCommandClear();
      sleepCountdownCode = 0;
      awakeStartTime = millis();
      break;
    case 4:  //deep night sleep
      if (!coffee) {
        if (rtcValid) SLEEPTIME = rtcData.nightSleep * 2;
        else SLEEPTIME = DEFAULT_NIGHT_SLEEP_TIME * 2;
      }
      break;
    case 5:  //travel sleep
      if (timeAwake < DEFAULT_AWAKE_TIME_AFTER_CALL * SIXTY_SECONDS) timeAwake = DEFAULT_AWAKE_TIME_AFTER_CALL * SIXTY_SECONDS;
      reportCommandClear(-1);
      sleepCountdownCode = -1;
      awakeStartTime = millis();
      break;
    case 6:  //hibernate
      if (timeAwake < DEFAULT_AWAKE_TIME_AFTER_CALL * SIXTY_SECONDS) timeAwake = DEFAULT_AWAKE_TIME_AFTER_CALL * SIXTY_SECONDS;
      reportCommandClear(-2);
      sleepCountdownCode = -2;
      awakeStartTime = millis();
      break;
    case 7:  // awake mode for in travel sleep or hibernate mode
      if (timeAwake < DEFAULT_AWAKE_TIME_AFTER_CALL * SIXTY_SECONDS) timeAwake = DEFAULT_AWAKE_TIME_AFTER_CALL * SIXTY_SECONDS;
      reportCommandClear();
      sleepCountdownCode = 0;
      awakeStartTime = millis();
      break;
    default:
      break;
  }
}

void startDeepSleep(int minutes) {
  if (rtcValid) {
    if (coffee) minutes = rtcData.coffeeSleep;  //establish sleeping time
    if (hibernate) minutes = rtcData.hibernateSleep;
    if (travelTimeStopTime > rtcData.travelSleep) minutes = rtcData.travelSleep;
    else if (travelTimeStopTime > 0) minutes = travelTimeStopTime;
  } else {
    if (coffee) minutes = COFFEE_SLEEP;  //establish sleeping time
    if (hibernate) minutes = DEFAULT_HIBERNATE_SLEEP;
    if (travelTimeStopTime > DEFAULT_TRAVEL_SLEEP) minutes = DEFAULT_TRAVEL_SLEEP;
    else if (travelTimeStopTime > 0) minutes = travelTimeStopTime;
  }
  if (minutes > MAX_ESP_SLEEP) minutes = MAX_ESP_SLEEP;
  uint64_t duration = minutes * SLEEP_MODIFIER;
  if (!noPublish) client.publish(publishTopicAwake, "", true);  //clear awake time
  if (hibernateMemUpdate && !hibernate) publish("Hibernation cancelled!");
  if (coffee) {  //if caffenated stop other sleep modes
    if (hibernate) stopHibernate();
    if (travelTimeStopTime > 0) stopTravelSleep();
  }
  if (sleepCountdownCode == -1 && hibernate) stopHibernate();
  if (sleepCountdownCode == -2) stopTravelSleep();
  if (checkRTCUpdate()) rtcMemUpdate();
  if (!noPublish) {
    snprintf(globalMessageAwake, MSG_BUFFER_SIZE, "%s", currentTime(local, minutes));
    publishNextWake(globalMessageAwake);
  }
  if (!noPublish) {  //output sleep outcome
    String localMessage, trailingZero, trailingZeroAlternate, told = "Resuming ";
    if (sleepCountdownCode < 0) told = "Starting ";
    if (coffee) localMessage = "Coffee Sleep Mode Engaged (" + String(minutes) + ") mins";
    else {
      if (!(sleepCountdownCode == -1 || travelTimeStopTime > 0 || hibernate)) {
        if (duration != SLEEP_MODIFIER) {
          if (dayTime) localMessage = "Day Sleep Mode Engaged (";
          else if (minutes > DEFAULT_NIGHT_SLEEP_TIME) localMessage = "Deep Night Sleep Mode Engaged (";
          else localMessage = "Night Sleep Mode Engaged (";
        } else localMessage = "Forced Sleep Mode Engaged (";
        localMessage += String(minutes) + " min)";
      } else {
        if ((travelTimeStopTime % 60) % 60 < 10) trailingZeroAlternate = "0";
        else trailingZeroAlternate = "";
        if (travelTimeStopTime % 60 < 10) trailingZero = "0";
        else trailingZero = "";
        if (sleepCountdownCode == -1 || travelTimeStopTime > 0) {
          if (travelTimeStopTime > 1440) localMessage = told + "Travel Sleep Mode for " + String(travelTimeStopTime / 1440) + " days, (" + String((travelTimeStopTime % 1440) / 60) + ":" + trailingZeroAlternate + String((travelTimeStopTime % 1440) % 60) + ") hours";
          else localMessage = told + "Travel Sleep Mode for (" + String(travelTimeStopTime / 60) + ":" + trailingZero + String(travelTimeStopTime % 60) + ") hours";
        } else {
          if (minutes % 60 < 10) trailingZero = "0";
          else trailingZero = "";
          if (hibernate) localMessage = told + " Hibernation, Sleep for (" + String(minutes / 60) + ":" + trailingZero + String(minutes % 60) + ") hours";
        }
      }
    }
    publishTravelTimeStopTime((char *)localMessage.c_str());
    if (sleepCountdownCode == -1 || travelTimeStopTime > 0) {
      printSp();
      if (minutes % 60 < 10) trailingZero = "0";
      else trailingZero = "";
      localMessage = "Sleeping for (" + String(minutes / 60) + ":" + trailingZero + String(minutes % 60) + ") hours";
      publishTravelTimeStopTime((char *)localMessage.c_str());
    }
  }
  if (!noPublish) client.disconnect();
  printSp();
  println("Going to Low Power Mode in 5s");
  sleepBlink();
  println("Goodbye!");
  ESP.deepSleep(duration, WAKE_RF_DISABLED);
}
//delay(200); ESP.restart(); } //DEBUGGING RESTART:

void twentySecondCall() {
  publish("20s until going to sleep");
  reportCommandClear();
  sleepCountdownCode = 2;
  printSp();
}

void sixtySecondCall() {
  publish("60s until going to sleep");
  reportCommandClear();
  sleepCountdownCode = 1;
  printSp();
}

void checkStayAsleep() {
  pinMode(STAY_ASLEEP_SW, INPUT_PULLUP);
  delay(50);
  if (digitalRead(STAY_ASLEEP_SW)) return;
  outputSerial = false;
  noPublish = true;
  startDeepSleep(STAY_ASLEEP_TIME);
}

void caffenate() {
  struct tm timeinfo;
  time_t now = time(nullptr);
  localtime_r(&now, &timeinfo);
  if (timeinfo.tm_hour == COFFEE_RESET) {
    publish("Can't Caffenate at this hour");
    return;
  }
  if (!coffee) {
    publish("Now Caffenated!");
    coffee = true;
    coffeeMemUpdate = true;
    reportCommandClear();
    clearSleepChannel();
    if (hibernate) stopHibernate();
    if (travelTimeStopTime > 0) stopTravelSleep();
  } else publish("Already Caffenated");
}

void setSleepDefault() {
  rtcData.daySleep = DEFAULT_DAY_SLEEP_TIME;
  rtcData.nightSleep = DEFAULT_NIGHT_SLEEP_TIME;
  rtcData.coffeeSleep = COFFEE_SLEEP;
  rtcData.travelSleep = DEFAULT_TRAVEL_SLEEP;
  rtcData.hibernateSleep = DEFAULT_HIBERNATE_SLEEP;
}

/////////////////////////PUBLISHING FUNCTIONS ///////////////////////////////////////////////////////////
void publishTime(unsigned long awake) {  //send awake in minutes
  if (!noTime) {
    time_t now = time(nullptr);
    now += (awake * 60);
    String message = String(now);
    client.unsubscribe(publishTopicSleep);
    client.unsubscribe(publishTopicAwake);
    client.publish(publishTopicAwake, message.c_str(), true);
  }
}

void publishSleep(int minutes) {  //send sleep target in minutes
  if (!noTime) {
    time_t now = time(nullptr);
    unsigned long seconds = minutes * 60;
    seconds += now;
    String message = String(seconds);
    client.unsubscribe(publishTopicSleep);
    client.publish(publishTopicSleep, message.c_str(), true);
  } else publish("Can't Perform without current time!");
}

void publishBoardState(char *display) {
  snprintf(globalMessage, MSG_BUFFER_SIZE, "%s at %s %s", display, tzname[0], currentTime(true));
  snprintf(globalMessageUTC, MSG_BUFFER_SIZE, "%s at UTC %s", display, currentTime());
  if (local) client.publish(publishTopicBoardState, globalMessage, true);
  else client.publish(publishTopicBoardState, globalMessageUTC, true);
}

void publishNextWake(char *display) {
  snprintf(globalMessage, MSG_BUFFER_SIZE, "Reawakening %s at %s %s", display, tzname[0], currentTime(true));
  snprintf(globalMessageUTC, MSG_BUFFER_SIZE, "Reawakening %s at UTC %s", display, currentTime());
  if (outputSerial) debugPrintNextAwake_aux(globalMessage);
  if (local) client.publish(publishTopicNextAwake, globalMessage, true);
  else client.publish(publishTopicNextAwake, globalMessageUTC, true);
}

void publishCompStatus(char *display) {
  char localMessage[MSG_BUFFER_SIZE], mssgutc[MSG_BUFFER_SIZE];
  snprintf(localMessage, MSG_BUFFER_SIZE, "%s at %s %s", display, tzname[0], currentTime(true));
  snprintf(mssgutc, MSG_BUFFER_SIZE, "%s at UTC %s", display, currentTime());
  if (outputSerial) debugPrintComputerStatus_aux(localMessage);
  if (local) client.publish(publishTopicStatus, localMessage, true);
  else client.publish(publishTopicStatus, mssgutc, true);
}

void publishTravelTimeStopTime(char *display, bool empty) {
  if (empty) {
    snprintf(globalMessage, 1, "");
    snprintf(globalMessageUTC, 1, "");
  } else {
    snprintf(globalMessage, MSG_BUFFER_SIZE, "%s at %s %s", display, tzname[0], currentTime(true));
    snprintf(globalMessageUTC, MSG_BUFFER_SIZE, "%s at UTC %s", display, currentTime());
  }
  if (outputSerial) debugPrint_aux(globalMessage);
  if (local) client.publish(publishTopic, globalMessage, true);
  else client.publish(publishTopic, globalMessageUTC, true);
}

void publish(char *display, int heartbeatCount) {
  if (heartbeatCount != -1) {
    snprintf(globalMessage, MSG_BUFFER_SIZE, "Heartbeat %ld at %s %s", heartbeatCount, tzname[0], currentTime(true));
    snprintf(globalMessageUTC, MSG_BUFFER_SIZE, "Heartbeat %ld at UTC %s", heartbeatCount, currentTime());
  } else {
    snprintf(globalMessage, MSG_BUFFER_SIZE, "%s at %s %s", display, tzname[0], currentTime(true));
    snprintf(globalMessageUTC, MSG_BUFFER_SIZE, "%s at UTC %s", display, currentTime());
  }
  if (outputSerial) debugPrint_aux(globalMessage);
  if (local) client.publish(publishTopic, globalMessage);
  else client.publish(publishTopic, globalMessageUTC);
}

void debugPrint_aux(char *globalMessage) {
  snprintf(globalMessageAlternate, MSG_BUFFER_SIZE, "Publish to [%s]: %s", publishTopic, globalMessage);
  print(globalMessageAlternate);
}

void debugPrintNextAwake_aux(char *globalMessage) {
  snprintf(globalMessageAlternate, MSG_BUFFER_SIZE, "Publish to [%s]: %s", publishTopicNextAwake, globalMessage);
  print(globalMessageAlternate);
}

void debugPrintComputerStatus_aux(char *globalMessage) {
  snprintf(globalMessageAlternate, MSG_BUFFER_SIZE, "Publish to [%s]: %s", publishTopicStatus, globalMessage);
  print(globalMessageAlternate);
}

void reportCommandClear(int ignore) {
  if (sleepCountdownCode == ignore) return;
  if (sleepCountdownCode == -2) {
    publish("Hibernate Command Cleared");
    sleepCountdownCode = 0;
  }
  if (sleepCountdownCode == -1) {
    publish("Travel Sleep Command Cleared");
    sleepCountdownCode = 0;
  }
}

void clearSleepChannel() {  //stops the travel sleep function
  client.unsubscribe(publishTopicSleep);
  delay(50);
  client.publish(publishTopicSleep, "", true);
}

/////////////////////////PRINTING FUNCTIONS /////////////////////////////////////////////////////////////
void print(String toPrint) {
  if (outputSerial) Serial.print(toPrint);
}

void println(String toPrint) {
  if (outputSerial) Serial.println(toPrint);
}

void printSp() {
  println();
  printSpace = false;
}

/////////////////////////STATUS PUBLISHING FUNCTIONS ////////////////////////////////////////////////////
void publishBoardState() {
  String line, trailingZero;
  if (hibernate) line = "In Hibernation";
  else if (travelTimeStopTime > 0) {
    line = "Travel Sleeping for " + String(travelTimeStopTime) + " minutes";
    if (travelTimeStopTime > 1440) {
      if ((travelTimeStopTime % 1440) % 60 < 10) trailingZero = "0";
      else trailingZero = "";
      line = "Travel Sleeping for " + String(travelTimeStopTime / 1440) + " days, (" + String((travelTimeStopTime % 1440) / 60) + ":" + trailingZero + String((travelTimeStopTime % 1440) % 60) + ") hours";
    } else {
      if ((travelTimeStopTime % 60) < 10) trailingZero = "0";
      else trailingZero = "";
      line = "Travel Sleeping for (" + String(travelTimeStopTime / 60) + ":" + trailingZero + String(travelTimeStopTime % 60) + ") hours";
    }
  } else if (coffee) line = "Caffenated";
  else if (stayAwake) line = "Board Forced Awake";
  else line = "Board in Normal State";
  publishBoardState((char *)line.c_str());
  if (outputSerial) println(line);
}

void boardStatus(unsigned long count) {
  String localMessage;
  unsigned long deltaMilliSeconds = (timeAwake - (millis() - awakeStartTime));
  int secs = deltaMilliSeconds / 60000, updatedSleepTime = count / 1000;
  if (rtcValid) publish("RTC was Valid");
  else publish("RTC was INVALID");
  if (checkRTCUpdate()) publish("Memory write called");
  else publish("No memory write called");
  if (coffee) publish("Caffeinated!");
  else publish("Decaffeinated");
  if (hibernate) publish("Hibernating");
  else publish("NOT Hibernating");
  if (local) localMessage = String("Publishing LOCAL Time (TZ#: ") + String(timeZone) + String(") ") + UTCoffset(timeZone);
  else localMessage = String("Publishing UTC (Local TZ#: ") + String(timeZone) + String(") ") + UTCoffset(timeZone);
  publish((char *)localMessage.c_str());
  print("\n");
  if (dayTime) publish("It is local DAY Time (8am - 11pm)");
  else publish("It is local NIGHT Time (11pm - 8am)");
  if (local) localMessage = String("Current Universal Time (TZ# 66): UTC ") + currentTime();
  else localMessage = String("Current Local Time (TZ#: ") + String(timeZone) + String("): ") + tzname[0] + " " + currentTime(true);
  publish((char *)localMessage.c_str());
  print("\n");
  statusPrint();
  if (travelTimeStopTime > 0) {
    String localMessage;
    if ((travelTimeStopTime % 60) % 60 < 10) localMessage = "0";
    else localMessage = "";
    if (travelTimeStopTime > 1440) localMessage = "Travel Sleep Mode Engaged for " + String(travelTimeStopTime / 1440) + " days, (" + String((travelTimeStopTime % 60) / 60) + ":" + localMessage + String((travelTimeStopTime % 60) % 60) + " hours)";
    else localMessage = "Travel Sleep Mode Engaged for (" + String(travelTimeStopTime / 60) + ":" + deltaMilliSeconds + String(travelTimeStopTime % 60) + ") hours";
  }
  if (updatedSleepTime > 60) {
    updatedSleepTime /= 60;
    if (updatedSleepTime > 1) localMessage = String("About ") + String(updatedSleepTime) + String(" mins till board would sleep, now ") + String(secs) + String(" mins");
    else localMessage = String("A little over ") + String(updatedSleepTime) + String(" min till board would sleep, now ") + String(secs) + String(" mins");
  } else localMessage = String(updatedSleepTime) + String("s till board would sleep, now ") + String(secs) + String(" mins");
  publish((char *)localMessage.c_str());
  print("\n");
}

void wifiStatus() {
  String localMessage = String("Wi-Fi: ") + String(ssid) + String("  || IP: ") + WiFi.localIP().toString();
  publish((char *)localMessage.c_str());
  println();
}

void rtcStatus() {
  String localMessage;
  if (checkRTCUpdate()) publish("Memory write called");
  else publish("No memory write called");
  localMessage = String("Wi-Fi Update: ") + boolToString(wifiMemUpdate);
  publish((char *)localMessage.c_str());
  localMessage = String("TZ Update: ") + boolToString(timeZoneMemUpdate);
  publish((char *)localMessage.c_str());
  localMessage = String("Light Mode Update: ") + boolToString(lightUpdate);
  publish((char *)localMessage.c_str());
  localMessage = String("Caffeine Update: ") + boolToString(coffeeMemUpdate);
  publish((char *)localMessage.c_str());
  localMessage = String("Time Publish Update: ") + boolToString(localTZMemUpdate);
  publish((char *)localMessage.c_str());
  localMessage = String("Hibernate Update: ") + boolToString(hibernateMemUpdate);
  publish((char *)localMessage.c_str());
}

void statusPrint() {
  delay(100);
  String localMessage = "LED Mode: ";
  if (lightStatus == AUTO) localMessage += "AUTO";
  if (lightStatus == OFF) localMessage += "OFF";
  if (lightStatus == ON) localMessage += "ON";
  publish((char *)localMessage.c_str());
}

void computerStatusPrint(int print) {
  String localMessage;
  if (print == 0) localMessage = "Comp Status: ";
  else localMessage = "Comp Status changed to: ";
  if (repStatus == ON) localMessage += COMMAND_2;
  else if (repStatus == OFF) localMessage += COMMAND_3;
  else if (repStatus == SLEEP) localMessage += "Sleeping";
  publishCompStatus((char *)localMessage.c_str());
}

void timePrint() {
  if (!rtcValid) publish("RTC Invalid! Showing default times.");
  String localMessage = String("Day Sleep Time: ") + String(rtcData.daySleep) + String(" minutes");
  publish((char *)localMessage.c_str());
  println();
  localMessage = String("Night Sleep Time: ") + String(rtcData.nightSleep) + String(" minutes");
  publish((char *)localMessage.c_str());
  println();
  localMessage = String("Coffee Sleep Time: ") + String(rtcData.coffeeSleep) + String(" minutes");
  publish((char *)localMessage.c_str());
  println();
  localMessage = String("Travel Sleep Time: ") + String((rtcData.travelSleep) / 60) + String(" hours");
  publish((char *)localMessage.c_str());
  println();
  localMessage = String("Hibernate Sleep Time: ") + String((rtcData.hibernateSleep) / 60) + String(" hours");
  publish((char *)localMessage.c_str());
  println();
}

/////////////////////////COMMAND FUNCTIONS //////////////////////////////////////////////////////////////
//ICACHE_RAM_ATTR void statusChange() {
void statusChange() {
  bool currentLightStatus = !digitalRead(PIN_POWER_LIGHT_IN_D5);
  recordComputerState();
  if (lastOn != 0 && lastOff != 0) {
    if ((millis() < lastOn + 4000) && (millis() < lastOff + 4000)) {
      if (currentStatus != SLEEP) currentStatus = SLEEP;
      recordLightStatusChange(currentLightStatus);
      return;
    }
  }
  if (currentLightStatus) {
    if (currentStatus != ON) currentStatus = ON;
  } else {
    if (currentStatus != OFF) currentStatus = OFF;
  }
  recordLightStatusChange(currentLightStatus);
}

void recordComputerState() {
  int computerState;
  if (payloadIndex >= STATUS_LOOPS) payloadIndex = 0;
  if (currentStatus == ON) computerState = 2;
  else if (currentStatus == SLEEP) computerState = 1;
  else computerState = 0;
  state[payloadIndex] = computerState;
  payloadIndex++;
}

void updateState() {
  char status = determineState();
  if (currentStatus == status) {
    repStatus = currentStatus;
    computerStatusPrint(1);
    commandCall = false;
  }
}

char determineState() {
  float roundedState = 0;
  for (int count = 0; count < STATUS_LOOPS; count++) roundedState += state[count];
  roundedState = round(roundedState / STATUS_LOOPS);
  if (roundedState == 2) return ON;
  else if (roundedState == 1) return SLEEP;
  else return OFF;
}

void recordLightStatusChange(bool lightIn) {
  if (lightIn) lastOn = millis();
  else lastOff = millis();
  if (lightStatus == AUTO)
    digitalWrite(PIN_POWER_LIGHT_OUT_D1, lightIn);
}

void setLightStatus(char newStatus) {
  if (newStatus == ON) {
    digitalWrite(PIN_POWER_LIGHT_OUT_D1, HIGH);
    lightStatus = ON;
    lightUpdate = true;
    publish("Light On");
    return;
  }
  if (newStatus == OFF) {
    digitalWrite(PIN_POWER_LIGHT_OUT_D1, LOW);
    lightStatus = OFF;
    lightUpdate = true;
    publish("Light Off");
    return;
  }
  if (newStatus == AUTO) {
    lightStatus = AUTO;
    lightUpdate = true;
    publish("Light Set to Auto");
    setLightStatus(currentStatus);
  }
}

void pressPowerButton(int duration) {
  powerButtonReleaseTime = millis() + (duration);
  digitalWrite(PIN_POWER_BUTTON_D6, HIGH);
}

void runRoutineChecks() {
  checkIfPowerNeedsToRelease();
  statusChange();
  if (currentStatus != repStatus) updateState();
}

void checkIfPowerNeedsToRelease() {
  if (powerButtonReleaseTime != 0) {
    if (millis() >= powerButtonReleaseTime) {
      digitalWrite(PIN_POWER_BUTTON_D6, LOW);
      powerButtonReleaseTime = 0;
      publish("Power Button Released");
      if (forceOff) {
        forceOff = false;
        publish("Force Power Off Complete");
      }
      printSp();
    }
  }
}

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ SETUP FUNCTION //////////////////////////////////////////////////////////////////////
void setup() {
  checkStayAsleep();    //check if board should stay asleep
  checkRTC();           //read memory and see if it is valid
  configure();          //configure board, I/Os and appropriate values
  startWiFi();          //start Wifi connection or start new
  setDateTime();        //set the time via server command
  determineDayNight();  //determine from time zone if it's day or night and set sleep timer appropriately
  setupMQTT();         //connect to mqtt and check for missed messages
  delay(50);
  awakeStartTime = heartbeat();
}  //fires off heartbeat before MQTT connection just to show sign of life

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ MAIN FUNCTION ///////////////////////////////////////////////////////////////////////
void loop() {
  runRoutineChecks();                    //check all board connections
  if (!client.connected()) reconnect();  //reconnect to mqtt if disconnected
  for (int count = 0; count < 10; count++) {         //client loop
    client.loop();
    delay(75);
  }
  if (printSpace) printSp();                                //called for serial display reasons
  if (mqttReconnectAttempts >= 5) startDeepSleep(SLEEPTIME);  // autosleep after too many attempts
  unsigned long now = millis();
  if (!publishState)
    if ((now - originalStartTime) > 1500) {
      publishBoardState();
      publishState = 3;
    }
  //TIME & Heartbeat CALLS
  if (now - lastMessage > HEARTBEAT) lastMessage = heartbeat();  //heartbeat
  if ((timeAwake - (now - awakeStartTime) < SIXTY_SECONDS) && sleepCountdownCode < 1) sixtySecondCall();
  if ((timeAwake - (now - awakeStartTime) < TWENTY_SECONDS) && sleepCountdownCode < 2) twentySecondCall();
  if (now - awakeStartTime > timeAwake) {
    if (commandCall) {
      commandCall = false;
      publish("+3 min (waiting for status change)");
      awakeStartTime += (3 * SIXTY_SECONDS);
    } else startDeepSleep(SLEEPTIME);
  }
}  //autosleep check
   //-----------------------------/END OF CODE/-----------------------------------------------------------------------//