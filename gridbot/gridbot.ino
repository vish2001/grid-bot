#define ENA   14          // Enable/speed motors left        GPIO14(D5)
#define ENB   12          // Enable/speed motors right         GPIO12(D6)
#define IN_1  16          // L298N in1 motors Rightx          GPIO15(D0)
#define IN_2  5          // L298N in2 motors Right           GPIO13(D1)
#define IN_3  4           // L298N in3 motors Left            GPIO2(D2)
#define IN_4  0           // L298N in4 motors Left            GPIO0(D3)
#include <ESP8266WiFi.h>  
#include <WiFiUdp.h>
int speedCar1 = 210;
int speedCar2 = 255;
int turnSpeed = 400;
int smolDelay = 10;
int bigDelay = 50;
int speed_Coeff = 3;
//************************************************//
const char* ssid = "KIRNET";
const char* password = "Buddy@123";
unsigned int localPort = 8888;
WiFiUDP Udp;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
//char ReplyBuffer[] = “acknowledged”; // a string to send back

void setup() { 
pinMode(ENA, OUTPUT);
pinMode(ENB, OUTPUT);  
pinMode(IN_1, OUTPUT);
pinMode(IN_2, OUTPUT);
pinMode(IN_3, OUTPUT);
pinMode(IN_4, OUTPUT);
Serial.begin(115200);
WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());  
  Serial.printf("UDP server on port %d\n", localPort);
  Udp.begin(localPort);
}
void goAhead(){ 
      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
      analogWrite(ENA, speedCar1);
      analogWrite(ENB, speedCar2);
  }

void goBack(){ 
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
      analogWrite(ENA, speedCar1);
      analogWrite(ENB, speedCar2);
  }

void goRight(){
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
      analogWrite(ENA, speedCar1);
      analogWrite(ENB, speedCar2);
}

void goLeft(){
      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
      analogWrite(ENA, speedCar1);
      analogWrite(ENB, speedCar2);
}

void stopRobot(){  
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, LOW);
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, LOW);
 }

void loop(){
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
    Serial.println("Contents:");
    String msg = packetBuffer;
    Serial.println(msg);
      if (msg == "w") goAhead();
      else if (msg == "s") goBack();
      else if (msg == "a") goLeft();
      else if (msg == "d") goRight();
      else if (msg == "0") speedCar1 += 10;
      else if (msg == "1") speedCar2 += 10;
      else if (msg == "2") speedCar1 += 5;
      else if (msg == "3") speedCar2 += 5;
      else if (msg == "4") speedCar1 -= 10;
      else if (msg == "5") speedCar2 -= 10;
      else if (msg == "6") speedCar1 -= 5;
      else if (msg == "7") speedCar2 -= 5;
      else if (msg == "8") speedCar1 -= 50;
      else if (msg == "9") speedCar2 -= 50;
      else if (msg == "10") speedCar1 += 50;
      else if (msg == "11") speedCar2 += 50;
      else stopRobot();
      if (speedCar1 > 255) {
        speedCar1 = 255;
      }
      if (speedCar2 > 255) {
        speedCar2 = 255;
      }
      if (speedCar1 < 100) {
        speedCar1 = 100;
      }
      if (speedCar2 < 100) {
        speedCar2 = 100;
      }
      
    }
}