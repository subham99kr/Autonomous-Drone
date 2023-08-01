#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include<DNSServer.h>
#include<ESP8266WebServer.h>
#include<WiFiManager.h>

#define MAGNET D2
const char* mqttServer="192.168.43.12";
const int mqttPort=1883;

void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived in Topic ");
  Serial.print(topic);
  
  Serial.print(" Message:");
  Serial.println();

  String message ="";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    message+=(char)payload[i];
  }
   Serial.println("----------------");

  if(String(topic)=="MAGNET"){
    if(message=="ON"){
        digitalWrite(MAGNET,HIGH);
        digitalWrite(BUILTIN_LED,LOW);
        Serial.println("Magnet is on");
    }
  else{
    digitalWrite(MAGNET,LOW);
    digitalWrite(BUILTIN_LED,HIGH);  
  }    
  }
}

WiFiClient espClient;
PubSubClient client (mqttServer, mqttPort, callback, espClient);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(MAGNET, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(MAGNET,LOW);
  digitalWrite(BUILTIN_LED,HIGH);
  //wifiManager
  //Local initialization. Once its business is done , there is no need to keep it arround
  WiFiManager wifiManager; 

wifiManager.autoConnect("AutoConnectAP");

Serial.println("connected...|");

client.setServer(mqttServer, mqttPort);
client.setCallback(callback);
 while (!client.connected())
 {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      client.subscribe("MAGNET");
      client.println("connected");
    } 
    else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }

}

void loop() {
  client.loop();

}
