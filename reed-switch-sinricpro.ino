#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
#define DEBUG_ESP_PORT Serial
#define NODEBUG_WEBSOCKETS
#define NDEBUG
#endif

#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
#include <WiFi.h>
#endif

#include "SinricPro.h"
#include "SinricProContactsensor.h"

typedef enum {
  LED_ON = HIGH,
  LED_OFF = LOW
} LED_State_t;

typedef enum {
    REED_CLOSED = LOW,
    REED_OPEN   = HIGH
} Reed_State_t ;

gpio_num_t const REED_PIN = GPIO_NUM_15;
gpio_num_t const LED_PIN = GPIO_NUM_14;

#define WIFI_SSID  ""
#define WIFI_PASS  ""
#define APP_KEY    ""  // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET ""  // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define CONTACT_ID ""  // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define BAUD_RATE 115200 // Change baudrate to your need

bool lastContactState = false;
unsigned long lastChange = 0;
LED_State_t ledState;
Reed_State_t contactState;

void led_on() {
  ledState = LED_ON;
  digitalWrite(LED_PIN, ledState);
}

void led_off() {
  ledState = LED_OFF;
  digitalWrite(LED_PIN, ledState);
}

void handleContactsensor() {
  unsigned long actualMillis = millis();
  if (actualMillis - lastChange < 250) return;
  bool contactState = digitalRead(REED_PIN); /* reads HIGHT when open */

  if (contactState != lastContactState) {
    Serial.printf("contactState is %s\r\n", contactState ? "open" : "closed");

    lastContactState = contactState;
    lastChange = actualMillis;
    SinricProContactsensor& myContact = SinricPro[CONTACT_ID];
    bool success = myContact.sendContactEvent(!contactState); /* contactState true = contact is closed, false = contact is open */
    if (!success) {
      Serial.printf("Something went wrong...could not send Event to server!\r\n");
    }

    led_on();
    delay(1000);
    led_off();
  }
}

// setup function for WiFi connection
void setupWiFi() {
  Serial.printf("\r\n[Wifi]: Connecting");

#if defined(ESP8266)
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setAutoReconnect(true);
#elif defined(ESP32)
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  IPAddress localIP = WiFi.localIP();
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %d.%d.%d.%d\r\n", localIP[0], localIP[1], localIP[2], localIP[3]);
}

// setup function for SinricPro
void setupSinricPro() {
  // add device to SinricPro
  SinricProContactsensor& myContact = SinricPro[CONTACT_ID];

  // setup SinricPro
  SinricPro.onConnected([]() {
    Serial.printf("Connected to SinricPro\r\n");
  });
  SinricPro.onDisconnected([]() {
    Serial.printf("Disconnected from SinricPro\r\n");
  });
  SinricPro.begin(APP_KEY, APP_SECRET);
}

// main setup function
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial);
  Serial.printf("\r\n\r\n");

  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  led_on();
  delay(1250);
  led_off();

  setupWiFi();
  setupSinricPro();
}

void loop() {
  handleContactsensor();
  SinricPro.handle();
}
