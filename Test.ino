#include <Arduino.h>
#include <avr/io.h>
#include <SPI.h>
#include <MFRC522.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

//###################################################################
//WIFI configuration
SoftwareSerial esp(A5 ,2); // RX, TX

char ssid[] = "Namli";            // your network SSID (name)
char pass[] = "9903098610";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

//char mqttServer[] = "mqtt.thingspeak.com";
char mqttServer[] = "192.168.137.98";
char ClientID[] = "Robot1";
String arrivedData;
bool messageArrived = 0;

WiFiEspClient espClient;
PubSubClient client(espClient);

//######################################################################
//RFID Configuration
MFRC522 mfrc522(10, 9);       
unsigned long UID, UIDtemp;
//###################################################################
//=====DECLARATION=====
unsigned long count;
uint8_t sensor[5];
char Case;

  // VARIABLES FOR SENSOR READING
float activeSensor = 0; // Count active sensors
float totalSensor = 0;  // Total sensor readings
float avgSensor =3;     // Average sensor reading

  // VARIABLES FOR PID CONTROLLER
float Kp = 66;
float Ki = 0.05;
float Kd = 2;
float error = 0;
float *er_pt = &error;      // error pointer
float previousError = 0;
float totalError = 0;
int PIDstatus = 1;
int i;
float power = 0;

  // VARIABLES FOR ADJUSTING SPEED
int turnSpeed =200;
int adj = 3;
int adjRight =2;
int flag;

  // VARIABLES FOR PATH OPTIMISATION
bool finish = false;
char option[50];
int index = 0;
//###############################################################
//=====WiFi Functions=====

void RobotInfor(){
  
  // print the SSID of the network you're attached to
  Serial.print("###ClientID: ");
  Serial.print (ClientID);

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("---IP Address: ");
  Serial.println(ip);
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
  arrivedData = "";
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    arrivedData += (char)payload[i];
  }
  Serial.println();
  Serial.println("########################");
  Serial.println(arrivedData);
  Serial.println("########################");
  Serial.println();
  messageArrived = 1;
  //delay_ms(300);
  //turnLED();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println ("Attempting MQTT connection...");
    // Attempt to connect, just a name to identify the client
    if (client.connect(ClientID)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay_ms(5000);
    }
  }
}

void mqttPublish (){
  Serial.println ("Sending data");
  // Data
  //String Data = String(UID)+';'+String(v);
  String Data = String(UID) + ";6;6;85"; 
  int length = Data.length();
  char Buff[length];
  Data.toCharArray (Buff, length + 1);
  //Publish packet
  client.publish( ClientID, Buff );
  return;
}
//###############################################################
//========================
//=====RFID FUNCTION=====
void RFIDCard (){
  UID = 0;
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    UIDtemp = mfrc522.uid.uidByte[i];
    UID = UID*256+UIDtemp;
  } 
  Serial.println (UID);
  mfrc522.PICC_HaltA(); 
  return;
}
//=====DELAY FUNCTION=====
void delay_ms (uint16_t millisecond) {
  unsigned long sec;
  sec = ((16000000/12)/1000)*millisecond;
  for (count = 0; count < sec; count ++);
}

//=====MOTOR FUNCTIONS=====
void setPWM_leftmotor (uint8_t PWM6){
  OCR0B = PWM6;
}

void setPWM_rightmotor (uint8_t PWM5){
  OCR0A = PWM5;
}
 
void Forward (){
  PORTD |= (1 << PD3); 
  PORTD &=~ ((1 << PD4)); 
  PORTD |= (1 << PD7); 
  PORTB &=~ ((1 << PB0)); 
}

void Backward (){
  PORTD &=~ (1 << PD3); 
  PORTD |= ((1 << PD4)); 
  PORTD &=~ (1 << PD7); 
  PORTB |= ((1 << PB0)); 
}

void Stop (){
  PORTD &=~ ((1 << PD3)); 
  PORTD &=~ ((1 << PD4)); 
  OCR0B = 0; 
  PORTD &=~ ((1 << PD7)); 
  PORTB &=~ ((1 << PB0)); 
  OCR0A = 0;
}

void TurnLeft (){ //left motor runs backward, right motor runs forward
  PORTD &=~ ((1 << PD3));
  PORTD |= (1 << PD4);
  //OCR0B = startSpeed + rotate; //the speed of left motor = 65 + 30 = 95 
  PORTD |= (1 << PD7); 
  PORTB &=~ ((1 << PB0));
 // OCR0A = startSpeed + rotate; //the speed of right motor = 65 + 30 = 95
}

void TurnRight (){ //left motor runs forward, right motor runs backward
  PORTD |= (1 << PD3); 
  PORTD &=~ ((1 << PD4)); 
  //OCR0B = startSpeed + rotate; //the speed of left motor = 65 + 30 = 95
  PORTD &=~ ((1 << PD7));
  PORTB |= (1 << PB0);
  //OCR0A = startSpeed + rotate; //the speed of right motor = 65 + 30 = 95
}
//========================

//=====READ SENSORS AND ASSIGN VALUE TO SENSOR ARRAY=====
void ReadSensors (){
  // Sensor 0
    if (PINC & (1<<0)){
      sensor[0] = 1;
    }else{
      sensor[0] = 0;}

// Sensor 1
    if (PINC & (1<<1)){
      sensor[1] = 1;
    }else{
      sensor[1] = 0;}

// Sensor 2
    if (PINC & (1<<2)){
      sensor[2] = 1;
    }else{
      sensor[2] = 0;}

// Sensor 3
    if (PINC & (1<<3)){
      sensor[3] = 1;
    }else{
      sensor[3] = 0;}


// Sensor 4
    if (PINC & (1<<4)){
      sensor[4] = 1;
    }else{
      sensor[4] = 0;}

     //Print in the serial monitor 
    Serial.print (sensor[0]);
    Serial.print ("   ");
    Serial.print (sensor[1]);
    Serial.print ("   ");
    Serial.print (sensor[2]);
    Serial.print ("   ");
    Serial.print (sensor[3]);
    Serial.print ("   ");
    Serial.print (sensor[4]);
    Serial.print ("   ");
}
//========================

//=====DETERMINE RUNNING CASE=====
void SensorsCondition (){
    ReadSensors ();
    // STRAIGHT
     if ((sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0) //01100
    || (sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)   //11000
    || (sensor[0]==1 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)   //10000
    || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==0)   //00010
    || (sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0)   //00110
    || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1)   //00011
    || (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1)   //00001
    || (sensor[0]==0 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)   //01000
    ||(sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0)    //00100
    ) {    
      Case = 'S'; // Straight 00100
    }
  
   // FINISH
    else if(sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1){
      //if ( flag == 1)
      Case = 'F'; // Finish 11111
    }
    // TURN LEFT
    else if ((sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==0)
          || (sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0)) {
      Case = 'L'; // Turn Left 11100
                  //           11110
    }

// TURN RIGHT
    else if ((sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1)
          || (sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1)) {
      Case = 'R'; // Turn Right 00111
                  //            01111
    }

// DEADEND
    else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0){
      Case = 'D'; // Dead End 00000
      //Situation = 0;
    }
}

//=====TURNING FUNCTIONS=====
void turnRight(){
  setPWM_leftmotor (turnSpeed);
  setPWM_rightmotor (turnSpeed);
  TurnRight();
  delay_ms((adjRight*90)+1);
  Stop();
  delay_ms(20);
}
void turnLeft () {
  setPWM_leftmotor (turnSpeed);
  setPWM_rightmotor (turnSpeed);
  TurnLeft();
  delay_ms((adj*90)+15);
  Stop();
  delay_ms(20);
}
//========================

//=====CONTROL ROBOT ACCORDING TO OUTPUT FROM SENSORS=====
void RunCase (){
   SensorsCondition ();
   switch (Case){

      // GO STRAIGHT
      case 'S':
      //Situation =0;
        Forward ();
        PID_program();
        Serial.println ("Straight");
        break;
 
      case 'R':
        setPWM_leftmotor (turnSpeed);
        setPWM_rightmotor (turnSpeed);
        ReadSensors();
        turnRight ();
        Serial.println ("Turn Right");
        break;
        
      case 'L': // Left 
        setPWM_leftmotor (turnSpeed);
        setPWM_rightmotor (turnSpeed);
        ReadSensors ();
        turnLeft ();
        Serial.println ("TurnLeft");
        break;
        
      default:
        Stop ();
        Serial.println ("Errors");
        break;
    }
}
//=================================
//=====PID CONTROLLER=====
void PID_program()
{ 
    Error(er_pt);
    
    previousError = error; // save previous error for differential 
 
    totalError += error; // Accumulate error for integral
    
    power = (Kp*error) + (Kd*(error-previousError)) + (Ki*totalError);
    
    if( power>255 ) { power = 255; }
    if( power<-255 ) { power = -255; }
    
    if(power<0) // Turn left
    {
      setPWM_rightmotor(185);
      setPWM_leftmotor(180 - abs(int(power)));
    }
    
    else // Turn right
    {
      setPWM_rightmotor(185 - int(power));
      setPWM_leftmotor(180);
    }
    Serial.print ("PWM Right:   ");
    Serial.print (OCR0A);
    Serial.print ("   ");
    Serial.print ("PWM Left:   ");
    Serial.print (OCR0B);
    Serial.print ("   ");
}

void Error(float *error) {
  for(int i=0; i<=4; i++) 
    {
      if(sensor[i]==1) {
        activeSensor+=1; 
        }
      totalSensor += sensor[i] * (i+1);
    }
      
    avgSensor = totalSensor/activeSensor;
    *error = (avgSensor - 3);
    activeSensor = 0; totalSensor = 0;
}
//=================================
//=====MAIN PROGRAM=====
void setup() {
  //###################
  //General 
  Serial.begin(9600);
  SPI.begin(); 
  //####################
  //ESP
  ConnectToWiFi ();
  client.setServer (mqttServer, 1883);
  client.setCallback (callback);
  //####################
  //RFID
  mfrc522.PCD_Init();
  RobotInfor();
  //###################
  DDRD |=  (1 << 7) | (1 << 6)| (1 << 5)| (1 << 4)| (1 << 3); //set pin 3 OUPUT (motor) and pin 2-4-5-6-7 INPUT (sensors)
  DDRB |=  (1 << 7) | (1 << 6)| (1 << 5)| (1 << 4)| (1 << 3)| (1 << 0); //set pins 8-9-10-11-12 OUTPUT (motors)

  TCCR0A = 0;
  TCCR0B = 0;
  //reset 2 registers
  TCCR0A |= (1 << WGM21) | (1 << WGM20); //mode fast PWM
  TCCR0B |= (1 << CS22) | (1 << CS20);
  //prescaler= 128
  TCCR0A |= (1 << COM2B1); //(PD3) (pin 3) ( none-inverting)
  TCCR0A |= (1 << COM2A1); //(PB3) (pin 11) ( none-inverting)
  OCR0B = 255; //(PD3) (pin 3) motor left
  OCR0A = 255; //(PB3) (pin 11) motor right
  PORTD |= (1<<5) | (1<<6);
}

void loop() {
  RunCase ();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
    return;
  }
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return;
  }
  Serial.println ("#################################");
  RFIDCard();
  RobotInfor();
  mqttPublish();
  Serial.println ("#################################");
  return;
}
