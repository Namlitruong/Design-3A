/*
 WiFiEsp example: ConnectWPA
 
 This example connects to an encrypted WiFi network using an ESP8266 module.
 Then it prints the  MAC address of the WiFi shield, the IP address obtained
 and other network details.

 For more details see: http://yaab-arduino.blogspot.com/p/wifiesp-example-connect.html
*/
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <WiFiEspUdp.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

SoftwareSerial esp(6, 7); // RX, TX

char ssid[] = "HungTQ";            // your network SSID (name)
char pass[] = "9903098nam";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

char mqttServer[] = "broker.hivemq.com";
char mqttUserName[] = "nam610";
char mqttPass[] = "XRD6J7QS8IO5UK9W";
char writeAPIkey[] = "N4O02PEQUVQ7G46Z";
long channelID = 482288;

WiFiEspClient espClient;
PubSubClient client(espClient);

void setup()
{
  // initialize serial for debugging
  Serial.begin(9600);
  ConnectToWiFi ();
  client.setServer (mqttServer, 1883);
  client.setCallback (callback);
}

void loop()
{
  // print the network connection information every 10 seconds
  Serial.println();
  WifiInfor();
  //delay(10000);
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  mqttPublish();
}

void WifiInfor(){
  
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print your MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  char buf[20];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  Serial.print("MAC address: ");
  Serial.println(buf);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.println(rssi);
}

void ConnectToWiFi (){
    esp.begin(9600);
  // initialize ESP module
  WiFi.init(&esp);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("ESP is not connected");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  Serial.println("You're connected to the network");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println ("Attempting MQTT connection...");
    // Attempt to connect, just a name to identify the client
    if (client.connect("ArduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      /*client.publish("command","hello world");
      // ... and resubscribe
      client.subscribe("presence");*/
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqttPublish (){
  Serial.println ("Collecting data");

  int InputData = 10;
  int InputData2 = 20;

  String Data = String ("field1=" + String(InputData) + "&field2=" + String(InputData2));
  int length = Data.length();
  char Buff[length];
  Data.toCharArray (Buff, length + 1);
  Serial.println (Buff);

  String topicString ="channels/" + String( channelID ) + "/publish/"+String(writeAPIkey);
  length = topicString.length();
  char topicBuff[length];
  topicString.toCharArray(topicBuff,length+1);
 
  client.publish( topicBuff, Buff );

  delay(1000);
}

