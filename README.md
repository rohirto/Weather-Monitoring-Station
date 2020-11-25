# Weather-Monitoring-Station
Weather Monitoring Station based on Arduino 

Hardware Used: Arduino Uno 3 clone, gp2y1010au0f optical dust sensor (SHARP company), ESP8266-01 WiFi module

Software Environment: Arduino IDE
Libraries: WiFiEsp (It is used to connect Arduino UNO board with ESP8266-01 module via AT commands and directly give MQTT functionality)

Project Briefly Explained: After 30 seconds the controller samples the sensor to get AQI (PM10 ug/m3) and sends it Adafruit IO server.
