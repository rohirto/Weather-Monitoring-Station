/*
 Standalone Sketch to use with a Arduino UNO and a
 Sharp Optical Dust Sensor GP2Y1010AU0F
*/
/* It sends data over MQTT to adafruit server */
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <WiFiEspUdp.h>
#ifndef STM32
#include "SoftwareSerial.h"
#endif
#include <PubSubClient.h>
/************** WIFI and MQTT Stuff************************* */
char server[] = "io.adafruit.com";  //Server address
char ssid[] ="Your SSID";
char pwd[] = "Your Password";
char aqi_feed[] ="rohirto/feeds/aqi";

int status = WL_IDLE_STATUS;  //Wifi Radio Status
//Initialize Ethernet client object
WiFiEspClient espClient;
PubSubClient client(espClient);
/***************************************************************/
/******* TIMER DEFINES ***************************************/
unsigned long startMillis;  //Some global vaiable anywhere in program
unsigned long currentMillis;
volatile byte aqi_timer = 3;  // In 10 secs multiple //1 min timer
volatile byte aqi_timer_elapsed = false;
volatile byte ten_sec_counter = 0;

/**************************************************************/
int measurePin = A5; //Connect dust sensor to Arduino A0 pin
int ledPower = 12;   //Connect 3 led driver pins of dust sensor to Arduino D2

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

#ifndef STM32
//Software Serial
SoftwareSerial soft(6,7);  //RX,TX
#endif

#define DEBUG
void setup()
{
  Serial.begin(9600);
  pinMode(ledPower,OUTPUT);

  #ifndef STM32
  //Initialize Serial for ESP module
  soft.begin(57600);
  //Initialize ESP module
  WiFi.init(&soft);
  #endif

  //Check for presence of Shield
  if(WiFi.status() == WL_NO_SHIELD){
    #ifdef DEBUG
    Serial.println("No Device Detected");
    #endif
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pwd);
  }
  // you're connected now, so print out the data
  #ifdef DEBUG
  Serial.println("You're connected to the network");
  #endif
  //connect to MQTT server
  client.setServer(server, 1883);
  client.setCallback(callback);
}

void loop(){
  if (!client.connected()) {
    reconnect();
  }
  //Update Timers  
  timer_function();

  if (aqi_timer_elapsed == true )
  {
    //Send the AQI Data
    digitalWrite(ledPower,LOW); // power on the LED
    delayMicroseconds(samplingTime);

    voMeasured = analogRead(measurePin); // read the dust value

    delayMicroseconds(deltaTime);
    digitalWrite(ledPower,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);

    // 0 - 5V mapped to 0 - 1023 integer values
    // recover voltage
    calcVoltage = voMeasured * (5.0 / 1024.0);

    // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
    // Chris Nafis (c) 2012
    dustDensity = 170 * calcVoltage - 0.1;  //in microgram per meter cube

  
    Serial.println(dustDensity); // unit: ug/m3
    String str = String(dustDensity);
    client.publish(aqi_feed, str.c_str());

    delay(1000);

    aqi_timer_elapsed = false;
  }
  

  client.loop();
}
//print any message received for subscribed topic
//Mainly it can be for ON OFF Control
void callback(char* topic, byte* payload, unsigned int length) {
  char rx_topic[2] = {};
  //Payload will be of 2 bytes 
  for (int i=0;i<length;i++) 
  {
    rx_topic[i] = ((char)payload[i]);
  }

  if(topic[14] == 'o')  //character identifier for ON OFF Topic (yet not implemented)
  {
    
  }
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect, just a name to identify the client
    if (client.connect("arduinoClient","rohirto","aio_knbt71hFGYGTPU4zVjUhQTm7f1rc")) {
      #ifdef DEBUG
      Serial.println("connected");
      #endif
      //client.subscribe(on_off_feed);  //FUTURE IMPLEMENTATION
      delay(500);
    } else {
      #ifdef DEBUG
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      Serial.println("1");
      #endif
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void timer_function()
{
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if( currentMillis - startMillis >= 10000)
  {
    startMillis = currentMillis;
    ten_sec_counter++;

     if((ten_sec_counter % aqi_timer) == 0)
    {
      aqi_timer_elapsed = true;
    }   
  }
}
