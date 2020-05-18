#include <dummy.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// WIFI ACCESS POINT.
#define WLAN_SSID   "mely"
#define WLAN_PASS   "andy1406"

// ADAFRUIT.IO CONFIGURACION.
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "andy14munguia"
#define AIO_KEY         "aio_tNHm32lg83muVA3IH9kE67NWkVJl"


// CREATE AN ESP8266 WIFICLIENT CLASS TO CONNECT TO THE MQTT SERVER.
WiFiClient Client;


// SETUP THE MQTT CLIENT CLASS BY PASSING IN THE WIFI CLIENT AND MQTT SERVER AND LOGIN DETAILS.
Adafruit_MQTT_Client mqtt(&Client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);


// LED SETUP.
int ledPin1 = 5; // GPIO5 OF ESP8266
int ledPin2 = 4; // GPIO4 OF ESP8266
int ledPin3 = 2; // GPIO2 OF ESP8266


// FEEDS.
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/LED_ON_OFF", MQTT_QOS_1);


// SKETCH CODE BUG - ARDUINO 1.6.6, IT SEEMS TO NEED A FUNCTION DECLARATION REASON:ONLY AFFECTS ESP***, LIKELY AN ARDUINO-BUILED BUG.
void MQTT_connect();


void setup() {
  Serial.begin(115200);

  // LED SETUP.
  pinMode(ledPin1, OUTPUT);
  digitalWrite(ledPin1, LOW);
  pinMode(ledPin2, OUTPUT);
  digitalWrite(ledPin2, LOW);
  pinMode(ledPin3, OUTPUT);
  digitalWrite(ledPin3, LOW);
  Serial.println(F("Adafruit MQTT Demo"));


  // CONNECT WIFI ACCESS POINT.
  Serial.println();
  Serial.println();
  Serial.print("Conectando espere un momento por favor...");
  Serial.println(WLAN_SSID);


  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi Conectado Direccion IP");
  Serial.println("IP Adress: ");
  Serial.println(WiFi.localIP());

  // SETUP MQTT SUBSCRIPTION FOR ON-OFF-FEED.
  mqtt.subscribe(&onoffbutton);
}

uint32_t x = 0;

// ENSURE THE CONNECTION TO THE MQTT SERVER IS ALIVE (THIS WILL MAKE THE FIRST CONNECTION AND
// AUTOMATICALLY RECONNEXT WHEN DISCCONECTED).
// SEE THE MQTT_CONNECT FUNCTION DEFINITION FURTHER BELOW.

void loop() {
  MQTT_connect();

  // THIS IS OUR 'WAIT FOR INCOMING SUBSCRIPTION PACKETS' BUSU SUBLOOP

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &onoffbutton) {
      Serial.print("Got: ");
      Serial.println((char *)onoffbutton.lastread);

      if (strcmp((char *)onoffbutton.lastread, "ON") == 0) {
        digitalWrite(ledPin1, HIGH);
      }
      if (strcmp((char *)onoffbutton.lastread, "OFF") == 0) {
        digitalWrite(ledPin1, LOW);
      }
      if (strcmp((char *)onoffbutton.lastread, "1") == 0) {
        digitalWrite(ledPin2, HIGH);
      }
      if (strcmp((char *)onoffbutton.lastread, "0") == 0) {
        digitalWrite(ledPin2, LOW);
      }
      if (strcmp((char *)onoffbutton.lastread, "Lumos") == 0) {
        digitalWrite(ledPin3, HIGH);
      }
      if (strcmp((char *)onoffbutton.lastread, "Nox") == 0) {
        digitalWrite(ledPin3, LOW);
      }
    }
  }
}

void MQTT_connect() {
  int8_t ret;

  // STOP IF ALREADY CONNECTED.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Conectando a MQTT... Espere por favor");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Reintentando conexion espere 5 segundos...");
    mqtt.disconnect();
    delay(5000);
    retries--;
    Serial.println("Retry " + retries);

    //BASICALLY DIE AND WAIT FOR WDT (WATCH DOG TIMER).
    if (retries == 0) {
      while (1);
    }

  }
  Serial.println("Ready1,MQTT Conectado!");
}
