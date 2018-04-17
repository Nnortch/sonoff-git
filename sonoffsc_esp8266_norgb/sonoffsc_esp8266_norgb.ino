/*  NETPIE ESP8266 basic sample                            */
/*  More information visit : https://netpie.io             */

// -----------------------------------------------------------------------------
// Library
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include <ESP8266WiFi.h>

#include "config/all.h"                 // Configuration

#include <MicroGear.h>                  // MicroGear library for netpie
#include <DebounceEvent.h>              // DebounceEvent library for button
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>                // WiFiManager library 
#include <Ticker.h>

#include "SerialLink.h"
SerialLink link(Serial, false);


// -----------------------------------------------------------------------------
// Microgear
// -----------------------------------------------------------------------------

WiFiClient client;

unsigned long LAST_TIMESTAMP_MICROGEAR_RECONNECT = 0;
MicroGear microgear(client);

String pmsg = "{\"temperature\":0, \"humidity\":0, \"light\":0, \"dust\":0, \"noise\":0, \"infrared\":0}";

unsigned long LAST_TIMESTAMP_SENSOR = 0;
unsigned long LAST_TIMESTAMP_CLAP = 0;
unsigned long LAST_TIMESTAMP_FEED = 0;
bool SEND_SENSOR_STATE = false;
bool SEND_CLAP_STATE = false;
bool MICROGEAR_STATE = false;

/*
Sonoff SC
Copyright (C) 2016 by Xose Pérez <xose dot perez at gmail dot com>
*/

// -----------------------------------------------------------------------------
// COMMUNICATIONS
// -----------------------------------------------------------------------------

const PROGMEM char at_push[] = "AT+PUSH";
const PROGMEM char at_every[] = "AT+EVERY";
const PROGMEM char at_temp[] = "AT+TEMP";
const PROGMEM char at_hum[] = "AT+HUM";
const PROGMEM char at_dust[] = "AT+DUST";
const PROGMEM char at_noise[] = "AT+NOISE";
const PROGMEM char at_light[] = "AT+LIGHT";
const PROGMEM char at_clap[] = "AT+CLAP";
const PROGMEM char at_code[] = "AT+CODE";
const PROGMEM char at_thld[] = "AT+THLD";
const PROGMEM char at_led[] = "AT+LED";
const PROGMEM char at_infrared[] = "AT+INFRA";

// -----------------------------------------------------------------------------
// VALUES
// -----------------------------------------------------------------------------

float temperature = 0;
int humidity = 0;
int light = 0;
float dust = 0;
int noise = 0;
int clap = 0;
int infrared = 0;
//uint32 rgb;

float getTemperature() { return temperature; }
float getHumidity() { return humidity; }
float getLight() { return light; }
float getDust() { return dust; }
float getNoise() { return noise; }
float getInfrared() {return infrared;}

//message string
String m;                 // ตัวแปรเก็บข้อความชนิด string

//Placeholders within input substring, which will jump from delimiter to delimiter
int boundLow;             // ตำแหน่งต่ำสุดของการ substring
int boundHigh;            // ตำแหน่งสูงสุดของการ substring

//Delimiter character
const char delimiter = ',';

// -----------------------------------------------------------------------------
// WiFi Mamager
// -----------------------------------------------------------------------------

WiFiManager wifiManager;
bool RESET_MODE_WIFI = false;
int WIFI_RECONNECT_TIMESTAMP = 0;

// -----------------------------------------------------------------------------
// Ticker
// -----------------------------------------------------------------------------

Ticker ticker;
unsigned long LAST_TIMESTAMP_LED = 0;
int LED_STATE = HIGH;


// -----------------------------------------------------------------------------
// METHODS WIFI
// -----------------------------------------------------------------------------

void initWiFi(){
  wifiManager.setTimeout(180);
  if(RESET_MODE_WIFI) wifiManager.resetSettings();
  wifiManager.setAPCallback(configModeCallback);
  RESET_MODE_WIFI = false;
  
  if(!wifiManager.autoConnect(AP_NAME)) {
    Serial.println("failed to connect and hit timeout");
    delay(5000);
  }
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("*PM: Entered config mode");
  Serial.print("*PM: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
  #ifdef LED_PIN
    ticker.attach(0.25, tick);
  #endif
}


// -----------------------------------------------------------------------------
// METHODS COMMUNICATIONS
// -----------------------------------------------------------------------------

bool commsGet(char * key) {
    return false;
}

bool commsSet(char * key, int value) {

    char buffer[50];

    if (strcmp_P(key, at_code) == 0) {
        sendClap(value);
        return true;
    }

    if (strcmp_P(key, at_temp) == 0) {
        temperature = (float) value / 10;
        return true;
    }

    if (strcmp_P(key, at_hum) == 0) {
        humidity = value;
        return true;
    }

    if (strcmp_P(key, at_light) == 0) {
        light = value;
        return true;
    }

    if (strcmp_P(key, at_dust) == 0) {
        dust = (float) value / 100;
        return true;
    }

    if (strcmp_P(key, at_noise) == 0) {
        noise = value;
        return true;
    }

    if (strcmp_P(key, at_infrared) == 0){
        infrared = value;
        return true;
    }
    return false;

}

void commConfigure() {
    link.send_P(at_every, SENSOR_EVERY);
    link.send_P(at_clap, SENSOR_CLAP_ENABLED == 1 ? 1 : 0);
}

void commsSetup() {
    link.onGet(commsGet);
    link.onSet(commsSet);
}

void commsLoop() {
    link.handle();
}


// -----------------------------------------------------------------------------
// METHODS BUTTON
// -----------------------------------------------------------------------------

DebounceEvent _button = DebounceEvent(BUTTON_PIN);

void buttonLoop() {
    if (_button.loop()) {
        uint8_t event = _button.getEvent();
        Serial.print("[BUTTON] Button pressed. Event: "); Serial.println(event);
        if (event == EVENT_DOUBLE_CLICK) RESET_MODE_WIFI = true;
        if (event == EVENT_LONG_CLICK) ESP.reset();
        //if (event == EVENT_SINGLE_CLICK) ESP.reset();
    }
}

// -----------------------------------------------------------------------------
// METHODS SONOFF SC
// -----------------------------------------------------------------------------

String getIdentifier() {
    char identifier[20];
    sprintf(identifier, "%s_%06X", DEVICE, ESP.getChipId());
    return String(identifier);
}

bool ledStatus(bool status) {
    digitalWrite(LED_PIN, status ? LOW : HIGH);
    return status;
}

bool ledStatus() {
    return (digitalRead(LED_PIN) == LOW);
}

void tick()
{
  #ifdef LEDPIN
    digitalWrite(LEDPIN, !digitalRead(LEDPIN));
  #endif
}

bool ledToggle() {
    return ledStatus(!ledStatus());
}

void ledBlink(unsigned long delayOff, unsigned long delayOn) {
    static unsigned long next = millis();
    if (next < millis()) {
        next += (ledToggle() ? delayOn : delayOff);
    }
}

void showStatus() {
    if (WiFi.status() != WL_CONNECTED) {
      ledBlink(250, 250);
    } else {
      ledBlink(50, 5000);
    }
}

void hardwareSetup() {
    Serial.begin(SERIAL_BAUDRATE);
    pinMode(LED_PIN, OUTPUT);
}

void hardwareLoop() {
    showStatus();
}

// -----------------------------------------------------------------------------
// METHODS Microgear
// -----------------------------------------------------------------------------

/* If a new message arrives, do this */
void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) {
    Serial.print("Incoming message --> ");
    msg[msglen] = '\0';
    Serial.println((char *)msg);
    
}

/* When a microgear is connected, do this */
void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) {
    Serial.println("Connected to NETPIE...");
    /* Set the alias of this microgear ALIAS */
    microgear.setAlias(ALIAS);
    microgear.subscribe("/#");
    
    commConfigure();
    link.send_P(at_push, 1);
    MICROGEAR_STATE = true;
}

void sendSensor(){
  pmsg = "{\"temperature\":"+String(temperature);
  pmsg+=",\"humidity\":"+String(humidity);
  pmsg+=",\"light\":"+String(light);
  pmsg+=",\"dust\":"+String(dust);
  pmsg+=",\"noise\":"+String(noise);
  pmsg+=",\"infrared\":"+String(infrared);
  pmsg+="}";
  Serial.println(pmsg);
  microgear.publish(NETPIE_TOPIC, pmsg);
}

void sendClap(int value){
  clap = value;
  Serial.print("clap: ");
  Serial.println(clap);
//  microgear.publish(NETPIE_TOPIC, pmsg);
}

// -----------------------------------------------------------------------------
// BOOTING
// -----------------------------------------------------------------------------

void welcome() {
    Serial.print("Device: "); Serial.println((char *) getIdentifier().c_str());
    Serial.print("ChipID: "); Serial.println(ESP.getChipId());
    Serial.print("CPU frequency: "); Serial.print(ESP.getCpuFreqMHz()); Serial.println(" MHz ");
    Serial.print("Last reset reason: "); Serial.print((char *) ESP.getResetReason().c_str());
    Serial.print("Memory size: "); Serial.print(ESP.getFlashChipSize()); Serial.println(" bytes");
    Serial.print("Free heap: "); Serial.print(ESP.getFreeHeap()); Serial.println(" bytes");
    Serial.print("Firmware size: "); Serial.print(ESP.getSketchSize()); Serial.println(" bytes");
    Serial.print("Free firmware space: "); Serial.print(ESP.getFreeSketchSpace()); Serial.println(" bytes");
    Serial.println("");

}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------

void setup() {

    hardwareSetup();
    welcome();
    commsSetup();
    commConfigure();
    initWiFi();
    
    /* Add Event listeners */

    /* Call onMsghandler() when new message arraives */
    microgear.on(MESSAGE,onMsghandler);

    /* Call onConnected() when NETPIE connection is established */
    microgear.on(CONNECTED,onConnected);

    /* Initial with KEY, SECRET and also set the ALIAS here */
    microgear.init(KEY,SECRET,ALIAS);

    if(WiFi.status()==WL_CONNECTED){
      #ifdef LED_PIN
        ticker.attach(0.05, tick);
      #endif
      /* connect to NETPIE to a specific APPID */
      microgear.connect(APPID);
    }
}

// -----------------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------------

void loop() {
  buttonLoop();
  hardwareLoop();
  commsLoop();

  if(RESET_MODE_WIFI) initWiFi();

  if(WiFi.status()!=WL_CONNECTED){
    Serial.println("Disconnect WiFi.");
    initWiFi();
  }else{
    /* To check if the microgear is still connected */
    if(microgear.connected()) {
      
      /* Call this method regularly otherwise the connection may be lost */
      microgear.loop();
          
      if(millis()-LAST_TIMESTAMP_SENSOR >= NETPIE_INTERVAL_SEND) {
        if(LAST_TIMESTAMP_SENSOR!=0){
          sendSensor();
        }
        LAST_TIMESTAMP_SENSOR = millis();
      }
      
      if(millis()-LAST_TIMESTAMP_FEED >= FEED_EVERY) {
        if(LAST_TIMESTAMP_FEED!=0 && pmsg!="") {
          if(FEEDAPIKEY!="") microgear.writeFeed(FEEDID,pmsg,FEEDAPIKEY);
          else microgear.writeFeed(FEEDID,pmsg);
        }
        LAST_TIMESTAMP_FEED = millis();
      }


      
    }else {
      Serial.println("Connection lost, reconnect...");
      if(MICROGEAR_STATE){
        MICROGEAR_STATE = false;
        link.send_P(at_push, 0);
      }
      if(WiFi.status()==WL_CONNECTED){
        #ifdef LED_PIN
          ticker.attach(0.05, tick);
        #endif
        if(millis()-LAST_TIMESTAMP_MICROGEAR_RECONNECT>=NETPIE_RECONNECT_DELAY){
          LAST_TIMESTAMP_MICROGEAR_RECONNECT = millis();
          link.send_P(at_push, 0);
          microgear.connect(APPID);
        }
      }
      if(WIFI_RECONNECT_TIMESTAMP==0){
        WIFI_RECONNECT_TIMESTAMP = millis();
      }
      if(millis()-WIFI_RECONNECT_TIMESTAMP>=WIFI_RECONNECT_TIMEOUT){
        if(WiFi.status()==WL_CONNECTED){
          WiFi.disconnect();
        }
        WIFI_RECONNECT_TIMESTAMP=0;
        initWiFi();
      }
    }
  }
}

