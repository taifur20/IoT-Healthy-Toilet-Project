/******************************************************************************************
Connect NodeMCU to AWS IoT Core.
Publish Messages from NodeMCU to the Core
   
Tutorial by https://nerdyelectronics.com :
1)  AWS IoT - Create a Thing
    https://nerdyelectronics.com/iot/how-to-create-a-thing-in-aws-iot/ :
2)  Convert Certificates from .pem to .der format
    https://nerdyelectronics.com/iot/how-to-convert-certificates-from-pem-to-der-format/ :
3)  Connect NodeMCU to AWS IoT Core
    https://nerdyelectronics.com/iot/how-to-connect-nodemcu-to-aws-iot-core/ :
  
*********************************************************************************************/
   
#include "FS.h"
#include <ESP8266WiFi.h>  //tested esp8266 core version: 2.5.2
#include <PubSubClient.h> //tested version: 2.7.0
#include <NTPClient.h>    //tested version: 3.2.0
#include <WiFiUdp.h>
#include <ArduinoJson.h>  //tested version: 6.18.4

#define RELAY D4 //you can use any digital pin here
String message = "";

DynamicJsonDocument doc(1024);

   
// Update these with values suitable for your network.
   
const char* ssid = "Your WiFi Name";   //Edit this line and put in your Wifi Name
const char* password = "PASSWORD";   //Edit this line and put in your Wifi Password
int count = 0;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
   
const char* AWS_endpoint = "XXXXXXXXXXXXX-ats.iot.us-west-2.amazonaws.com"; // Edit your AWS Endpoint here
  
void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) 
  {
    message.concat((char)payload[i]); 
  }
  Serial.print(message);
  deserializeJson(doc, message);
  bool fan_status = doc["state"]["desired"]["fan_status"];
  Serial.println();
  if(fan_status == true){
    //turn on fan
    digitalWrite(RELAY, LOW);
    Serial.println("Fan is ON");
    }
  else if(fan_status == false){
    //turn off fan
    digitalWrite(RELAY, HIGH);
    Serial.println("Fan is OFF");
    }
  message = "";
}
  
WiFiClientSecure espClient;
PubSubClient client(AWS_endpoint, 8883, callback, espClient); //set MQTT port number to 8883 as per //standard
long lastMsg = 0;
char msg[50];  //buffer to hold the message to be published
int value = 0;
  
void setup_wifi() 
{
  
  delay(10);
  // We start by connecting to a WiFi network
  espClient.setBufferSizes(512, 512);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  timeClient.begin();
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
  
  espClient.setX509Time(timeClient.getEpochTime());
  
}
  
void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESPthing")) 
    {
      Serial.println("connected");
      client.subscribe("node/mcu/fan/state");
      Serial.println("subscribed");
    } 
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
  
      char buf[256];
      espClient.getLastSSLError(buf, 256);
      Serial.print("WiFiClientSecure SSL error: ");
      Serial.println(buf);
  
      // Wait 5 seconds before retrying
      delay(5000);
    }

    
  }
}
  
void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(RELAY, OUTPUT);
  setup_wifi();
  delay(1000);
  if (!SPIFFS.begin()) 
  {
    Serial.println("Failed to mount file system");
    return;
  }
  
  Serial.print("Heap: "); Serial.println(ESP.getFreeHeap());
  
  // Load certificate file
  File cert = SPIFFS.open("/cert.der", "r"); //replace cert.crt with your uploaded file name
  if (!cert) 
  {
    Serial.println("Failed to open cert file");
  }
  else
    Serial.println("Successfully opened cert file");
  
  delay(1000);
  
  if (espClient.loadCertificate(cert)) // add the thing certificate to the client
    Serial.println("cert loaded");
  else
    Serial.println("cert not loaded");
  
  // Load private key file
  File private_key = SPIFFS.open("/private.der", "r"); //replace private with your uploaded file name
  if (!private_key) 
  {
    Serial.println("Failed to open private cert file");
  }
  else
    Serial.println("Successfully opened private cert file");
  
  delay(1000);
  
  if (espClient.loadPrivateKey(private_key))  // add the private key to the client
    Serial.println("private key loaded");
  else
    Serial.println("private key not loaded");
  
  // Load CA file
  File ca = SPIFFS.open("/ca.der", "r"); //replace ca with your uploaded file name
  if (!ca) 
  {
    Serial.println("Failed to open ca ");
  }
  else
    Serial.println("Successfully opened open ca");
  
  delay(1000);
  
  if (espClient.loadCACert(ca))   // add the AWS root certificate to the client
    Serial.println("ca loaded");
  else
    Serial.println("ca failed");
  
  Serial.print("Heap: "); Serial.println(ESP.getFreeHeap());
}
  
void loop() 
{ 
  if (!client.connected()) 
  {
    reconnect();
  }
  client.loop();
}
