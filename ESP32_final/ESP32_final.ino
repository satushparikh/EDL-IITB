#include "WiFi.h"                             // including the Wifi library
#include "ESPAsyncWebServer.h"                // including the Web server library

/*
 * Here we declare the ssid (the name of the hotspot created by ESP32) and it's password.
 * Make sure to set the ssid as something unique for your team.
 */
const char* ssid = "hello";
const char* password = "12345678";
int mode; // 0 is manual 1 is auto
int output1 = 33;
int output2 = 26;
int output3 = 27;
AsyncWebServer server(80);                    // creating the webserver here

void setup(){
  pinMode(output1, OUTPUT);
  pinMode(output2, OUTPUT);
  pinMode(output3, OUTPUT);
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);

  // delay(100);
  // digitalWrite(2, LOW);
  // delay(1000);
  // digitalWrite(2, HIGH);
  // delay(1000);
  // digitalWrite(reset_pin, LOW);
  mode = 0;


  WiFi.softAP(ssid, password);                // creates a WiFi hotspot on ESP32
  
  /*
   * After this we can start adding some code to control the ESP32.
   * We can make requests to the server through a webpage and make it perform different tasks.
   */

  server.on("/stop-on", HTTP_GET, [](AsyncWebServerRequest *request){
    // the following code runs whenever we receive a "turn-on" request from the webpage
    digitalWrite(2,LOW);
    delay(1000);
    digitalWrite(2,HIGH);
    mode = 0;
    write_output(0);                    // sets the voltage of pin 2 as high, turning on the LED light on ESP32
    request->send_P(200, "text/plain", "");   
  });

  server.on("/manual-on", HTTP_GET, [](AsyncWebServerRequest *request){
    // the following code runs whenever we recieve a "turn-off" request from the webpage
    
    mode = 0;
    write_output(1);                     // sets the voltage of pin 2 as low, turning off the LED light on ESP32
    request->send_P(200, "text/plain", "");
  });
  server.on("/auto-on", HTTP_GET, [](AsyncWebServerRequest *request){
    // the following code runs whenever we recieve a "turn-off" request from the webpage
    
    mode = 1;
    write_output(1);                     // sets the voltage of pin 2 as low, turning off the LED light on ESP32
    request->send_P(200, "text/plain", "");
  });

  server.on("/fwd-on", HTTP_GET, [](AsyncWebServerRequest *request){
    // the following code runs whenever we recieve a "turn-off" request from the webpage
    
    if(mode == 0)
    {
      write_output(2);
    }                     // sets the voltage of pin 2 as low, turning off the LED light on ESP32
    request->send_P(200, "text/plain", "");
  });
  server.on("/rev-on", HTTP_GET, [](AsyncWebServerRequest *request){
    // the following code runs whenever we recieve a "turn-off" request from the webpage
    
    if(mode == 0)
    {
      write_output(5);
    }                     // sets the voltage of pin 2 as low, turning off the LED light on ESP32
    request->send_P(200, "text/plain", "");
  });
  server.on("/left-on", HTTP_GET, [](AsyncWebServerRequest *request){
    // the following code runs whenever we recieve a "turn-off" request from the webpage
    
    if(mode == 0)
    {
      write_output(3);
    }                     // sets the voltage of pin 2 as low, turning off the LED light on ESP32
    request->send_P(200, "text/plain", "");
  });
  server.on("/right-on", HTTP_GET, [](AsyncWebServerRequest *request){
    // the following code runs whenever we recieve a "turn-off" request from the webpage
    
    if(mode == 0)
    {
      write_output(4);
    }                     // sets the voltage of pin 2 as low, turning off the LED light on ESP32
    request->send_P(200, "text/plain", "");
  });
   server.on("/turn-on", HTTP_GET, [](AsyncWebServerRequest *request){
    // the following code runs whenever we receive a "turn-on" request from the webpage
    
    digitalWrite(2, HIGH);                    // sets the voltage of pin 2 as high, turning on the LED light on ESP32
    request->send_P(200, "text/plain", "");   
  });

  server.on("/turn-off", HTTP_GET, [](AsyncWebServerRequest *request){
    // the following code runs whenever we recieve a "turn-off" request from the webpage
    
    digitalWrite(2, LOW);                     // sets the voltage of pin 2 as low, turning off the LED light on ESP32
    request->send_P(200, "text/plain", "");
  });
  server.begin();                             // starts the web server
} 

void loop() {}
void write_output(int x){
    Serial.println(x);
    if(x == 0)
    {
      digitalWrite(output1, LOW);
      delay(1);
      digitalWrite(output2, LOW);
      delay(1);
      digitalWrite(output3, LOW);
    }
    else if (x == 1)
    {
      digitalWrite(output1, HIGH);
      delay(1);
      digitalWrite(output2, LOW);
      delay(1);
      digitalWrite(output3, LOW);
    }
    else if (x == 2)
    {
      digitalWrite(output1, LOW);
      delay(1);
      digitalWrite(output2, HIGH);
      delay(1);
      digitalWrite(output3, LOW);
    }
    else if (x == 3)
    {
      digitalWrite(output1, HIGH);
      delay(1);
      digitalWrite(output2, HIGH);
      delay(1);
      digitalWrite(output3, LOW);
    }
    else if (x == 4)
    {
      digitalWrite(output1, LOW);
      delay(1);
      digitalWrite(output2, LOW);
      delay(1);
      digitalWrite(output3, HIGH);
    }
    else if (x == 5)
    {
      digitalWrite(output1, HIGH);
      delay(1);
      digitalWrite(output2, LOW);
      delay(1);
      digitalWrite(output3, HIGH);
    }
    else if (x == 6)
    {
      digitalWrite(output1, LOW);
      delay(1);
      digitalWrite(output2, HIGH);
      delay(1);
      digitalWrite(output3, HIGH);
    }
    else if (x == 7)
    {
      digitalWrite(output1, HIGH);
      delay(1);
      digitalWrite(output2, HIGH);
      delay(1);
      digitalWrite(output3, HIGH);
    }
    else
    {
      digitalWrite(output1, LOW);
      delay(1);
      digitalWrite(output2, LOW);
      delay(1);
      digitalWrite(output3, LOW);
    }
    delay(10);
}