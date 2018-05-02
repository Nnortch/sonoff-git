/*

Sonoff SC
Copyright (C) 2017 by Xose Pérez <xose dot perez at gmail dot com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/



#include <DHT.h>
#include <SerialLink.h>

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

#define SERIAL_BAUDRATE         115200

#define LED_PIN                 7

#define LDR_PIN                 A3

#define SHARP_LED_PIN           9
#define SHARP_READ_PIN          A1
#define SHARP_SAMPLING_TIME	    280
#define SHARP_DELTA_TIME		40
#define SHARP_SLEEP_TIME		9680

#define DHT_PIN                 6
#ifndef DHT_TYPE
// Uncomment the sensor type that you have (comment the other if applicable)
//#define DHT_TYPE                DHT11
#define DHT_TYPE                DHT22
#endif

#define PIR_PIN  11 // can use 11-13

#define ADC_COUNTS              1023
#define MICROPHONE_PIN          A2
#define PIR_PIN 11
#define PIR_DEALAY_TIME 30000 //delay time PIR 30s

#define NOISE_READING_DELAY     100
#define NOISE_READING_WINDOW    20
#define NOISE_BUFFER_SIZE       20



#define MAX_SERIAL_BUFFER       20

#define DEFAULT_EVERY           60
#define DEFAULT_PUSH            0
//#define DEFAULT_CLAP            0
#define DEFAULT_THRESHOLD       0

// -----------------------------------------------------------------------------
// Keywords
// -----------------------------------------------------------------------------

const PROGMEM char at_hello[] = "AT+HELLO";
const PROGMEM char at_push[] = "AT+PUSH";
const PROGMEM char at_every[] = "AT+EVERY";
const PROGMEM char at_temp[] = "AT+TEMP";
const PROGMEM char at_hum[] = "AT+HUM";
const PROGMEM char at_dust[] = "AT+DUST";
const PROGMEM char at_noise[] = "AT+NOISE";
const PROGMEM char at_light[] = "AT+LIGHT";
//const PROGMEM char at_clap[] = "AT+CLAP";
const PROGMEM char at_code[] = "AT+CODE";
//const PROGMEM char at_thld[] = "AT+THLD";
const PROGMEM char at_led[] = "AT+LED";
const PROGMEM char at_sound[] = "AT+SOUND";
const PROGMEM char at_infrared[] = "AT+INFRA";

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

SerialLink link(Serial);
DHT dht(DHT_PIN, DHT_TYPE);
//int clapTimings[CLAP_BUFFER_SIZE];
byte clapPointer = 0;

// If push == false the slave waits for the master to request for values
// If push == true the slaves sends messages
//
// If every == 0 values are retrieved upon request
// If every > 0 values are retrieved every <every> seconds and cached/sent
//
// If push == true and every == 0 messages are never sent.

bool push = DEFAULT_PUSH;
//bool clap = DEFAULT_CLAP;
unsigned long every = 1000 * DEFAULT_EVERY;
//unsigned int threshold = DEFAULT_THRESHOLD;
//unsigned long lastEffectTime = 0;


float temperature;
int humidity;
float dust;
int light;
int noise;
//int sound;
bool infrared, previous_infrared, p_status=0;
unsigned long trigger =0;


////________smoothing____________
//const int numReadings = 20;
//int readings[numReadings];      // the readings from the analog input
//int readIndex = 0;              // the index of the current reading
//int total = 0;                  // the running total
//int average = 0;                // the average



unsigned int noise_buffer[NOISE_BUFFER_SIZE] = {0};
unsigned int noise_buffer_pointer = 0;
unsigned int noise_buffer_sum = 0;

// -----------------------------------------------------------------------------
// ACTUATORS
// -----------------------------------------------------------------------------

void ledStatus(bool status) {
    digitalWrite(LED_PIN, status ? HIGH : LOW);
}

bool ledStatus() {
    return digitalRead(LED_PIN) == HIGH;
}

// -----------------------------------------------------------------------------
// SENSORS
// -----------------------------------------------------------------------------

int getLight() {
    return map(analogRead(LDR_PIN), 0, ADC_COUNTS, 100, 0);
}

// 0.5V/0.1mg/m3
float getDust() {

    digitalWrite(SHARP_LED_PIN, LOW);
	delayMicroseconds(SHARP_SAMPLING_TIME);

	float reading = analogRead(SHARP_READ_PIN);

	delayMicroseconds(SHARP_DELTA_TIME);
	digitalWrite(SHARP_LED_PIN, HIGH);
	//delayMicroseconds(SHARP_SLEEP_TIME);

    // mg/m3
	float dust = 0.17 * reading * (5.0 / 1024.0) - 0.1;
    if (dust < 0) dust = 0;
    return dust;

}

float getTemperature() {
    return dht.readTemperature();
}

int getHumidity() {
    return dht.readHumidity();
}

bool getInfrared() {
  infrared = digitalRead(PIR_PIN);//read PIR
        if(infrared == 0){
          if(previous_infrared==1){
             p_status = 1;
              trigger = millis();
          }
          else if ((previous_infrared==0)&&(millis()-trigger < PIR_DEALAY_TIME)&&(trigger!=0)){p_status = 1;}
          else if ((previous_infrared==0)&&(millis()-trigger >= PIR_DEALAY_TIME)){p_status = 0;}
          else{p_status = 0;}
        }
        else{
          p_status = 1;
        }
        previous_infrared = infrared;
  return infrared;
}

int getNoise() {

    int value = 0;

    //if (noise_count > 0) {

        value = noise_buffer_sum / NOISE_BUFFER_SIZE;

    //}

    return value;

}

//int getSound(){
//  return average;
//}

// -----------------------------------------------------------------------------
// MICrophone
// -----------------------------------------------------------------------------



void noiseLoop() {

    static unsigned long last_reading = 0;
    static unsigned int triggered = 0;

    unsigned int sample;
    //unsigned int count = 0;
    //unsigned long sum;
    unsigned int min = ADC_COUNTS;
    unsigned int max = 0;

    // Check MIC every NOISE_READING_DELAY
    // if (millis() - last_reading < NOISE_READING_DELAY) return;
    last_reading = millis();

    while (millis() - last_reading < NOISE_READING_WINDOW) {
        sample = analogRead(MICROPHONE_PIN);
        //++count;
        //sum += sample;
        if (sample < min) min = sample;
        if (sample > max) max = sample;
    }

    //++noise_count;
    //unsigned int average = 100 * (sum / count) / ADC_COUNTS;
    //noise_sum += average;

    unsigned int peak = map(max - min, 0, ADC_COUNTS, 0, 100);
//    Serial.println(peak);
//    if (clap) clapRecord(peak);
//
    noise_buffer_sum = noise_buffer_sum + peak - noise_buffer[noise_buffer_pointer];
    noise_buffer[noise_buffer_pointer] = peak;
    noise_buffer_pointer = (noise_buffer_pointer + 1) % NOISE_BUFFER_SIZE;

//    //__________smoothing______________
//    total = total - readings[readIndex];
//    readings[readIndex] = peak;
//    total = total + readings[readIndex];
//    readIndex = readIndex + 1;
//    if (readIndex >= numReadings) {
//      // ...wrap around to the beginning:
//      readIndex = 0;
//    }
//    average = total / numReadings;

//    if (threshold > 0) {
//        unsigned int value = noise_buffer_sum / NOISE_BUFFER_SIZE;
//        if (value > threshold) {
//            if (value > triggered) {
//                link.send_P(at_noise, value);
//                triggered = value;
//            }
//        } else if (triggered > 0) {
//            link.send_P(at_noise, value);
//            triggered = 0;
//        }
//    }
    
      

}

// -----------------------------------------------------------------------------
// COMMUNICATION
// -----------------------------------------------------------------------------

// How to respond to AT+...=? requests
bool linkGet(char * key) {

    if (strcmp_P(key, at_push) == 0) {
        link.send(key, push ? 1 : 0, false);
        return true;
    }

    if (strcmp_P(key, at_led) == 0) {
        link.send(key, ledStatus() ? 1 : 0, false);
        return true;
    }



    if (strcmp_P(key, at_every) == 0) {
        link.send(key, every / 1000, false);
        return true;
    }

    if (strcmp_P(key, at_temp) == 0) {
        if (every == 0) temperature = getTemperature();
        link.send(key, 10 * temperature, false);
        return true;
    }

    if (strcmp_P(key, at_hum) == 0) {
        if (every == 0) humidity = getHumidity();
        link.send(key, humidity, false);
        return true;
    }

    if (strcmp_P(key, at_noise) == 0) {
        if (every == 0) noise = getNoise();
        link.send(key, noise, false);
        return true;
    }

    if (strcmp_P(key, at_dust) == 0) {
        if (every == 0) dust = getDust();
        link.send(key, 100 * dust, false);
        return true;
    }

    if (strcmp_P(key, at_light) == 0) {
        if (every == 0) light = getLight();
        link.send(key, light, false);
        return true;
    }

    if (strcmp_P(key, at_infrared) == 0) {
        if (every == 0) infrared = getInfrared();
        link.send(key, infrared, false);
        return true;
    }

//    if (strcmp_P(key, at_sound) == 0) {
//        if (every == 0) light = getSound();
//        link.send(key, sound, false);
//        return true;
//    }
    
    return false;

}


// Functions for responding to AT+...=<int> commands that set values and functions
bool linkSet(char * key, int value) {

    if (strcmp_P(key, at_push) == 0) {
        if (0 <= value && value <= 1) {
            push = value == 1;
            return true;
        }
    }


    if (strcmp_P(key, at_every) == 0) {
        if (5 <= value && value <= 300) {
            every = 1000 * value;
            return true;
        }
    }


    if (strcmp_P(key, at_led) == 0) {
        if (0 <= value && value <= 1) {
            ledStatus(value == 1);
            return true;
        }
    }

    return false;
}

//Setup callbacks when AT commands are recieved
void linkSetup() {
    link.onGet(linkGet);
    link.onSet(linkSet);
}

// Check for incoming AT+ commands
void linkLoop() {
    link.handle();
}

// -----------------------------------------------------------------------------
// MAIN
// -----------------------------------------------------------------------------

void setup() {

	// Setup Serial port
    Serial.begin(SERIAL_BAUDRATE);
    link.send_P(at_hello, 1);

    linkSetup();

	// Setup physical pins on the ATMega328
    pinMode(LED_PIN, OUTPUT);
	  pinMode(LDR_PIN, INPUT);
    pinMode(DHT_PIN, INPUT);
    pinMode(SHARP_LED_PIN, OUTPUT);
    pinMode(SHARP_READ_PIN, INPUT);
    pinMode(MICROPHONE_PIN, INPUT_PULLUP);
    pinMode(PIR_PIN,INPUT);

	// Switch LED off (just to be sure)
    ledStatus(false);

	// Setup the DHT Thermometer/Humidity Sensor
    dht.begin();


}

void loop() {

    static unsigned long last = 0;

    linkLoop();

	// If AT+EVERY>0 then we are sending a signal every so many seconds
    if ((every > 0) && ((millis() - last > every) || (last == 0))) {

        last = millis();

        temperature = getTemperature();
        if (push) link.send_P(at_temp, 10 * temperature, false);
        Serial.print("Temp:");
        Serial.print(temperature);
        
        noiseLoop();

        humidity = getHumidity();
        if (push) link.send_P(at_hum, humidity, false);
        Serial.print(" Humi:");
        Serial.print(humidity);
        
        noiseLoop();

        dust = getDust();
        if (push) link.send_P(at_dust, 100 * dust, false);
        Serial.print(" Dust:");
        Serial.print(dust);

        noiseLoop();

        light = getLight();
        if (push) link.send_P(at_light, light, false);
        Serial.print(" Light:");
        Serial.print(light);
   
        noiseLoop();

        infrared = getInfrared();
        if(push) link.send_P(at_infrared, infrared, false);
        Serial.print(" Infrared:");
        Serial.println(infrared);
        
        noiseLoop();

        noise = getNoise();
        if (push) link.send_P(at_noise, noise, false);
        Serial.print(" Noise:");
        Serial.print(noise);
        
//        sound = getSound();
//        if(push) link.send_P(at_sound, sound, false);
//        Serial.print(" Sound:");
//        Serial.println(sound);

        
    }

    noiseLoop();

}
