/* Robot Assistance PMR — Arduino Mega 2560 — Hind Jabrane */
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include "arm_controller.h"
#include "obstacle_detection.h"
#include "motor_driver.h"

ArmController arm(4,5,6,7);
ObstacleDetector obstacles;
MotorDriver motors(2,24,26,3,28,30);
SoftwareSerial btSerial(10,11);
Adafruit_SSD1306 oled(128,64,&Wire,-1);
bool emergencyStop=false;

void setup(){
    Serial.begin(115200);pinMode(22,INPUT_PULLUP);
    arm.begin();obstacles.begin(32,33,34,35,36,37,38,39);
    motors.begin();btSerial.begin(9600);
    oled.begin(SSD1306_SWITCHCAPVCC,0x3C);
    arm.goHome();showOLED("Robot PMR","Pret!");
}
void loop(){
    if(btSerial.available()){String cmd=btSerial.readStringUntil('\n');processCommand(cmd);}
    if(!emergencyStop)readJoystick();
    static unsigned long t=0;if(millis()-t>100){checkObstacles();t=millis();}
}
void readJoystick(){
    int x=analogRead(A0)-512,y=analogRead(A1)-512;
    if(abs(x)<50)x=0;if(abs(y)<50)y=0;
    motors.drive(map(abs(y+x),0,512,0,255),y+x>=0,map(abs(y-x),0,512,0,255),y-x>=0);
    if(digitalRead(22)==LOW){arm.toggleGripper();delay(300);}
}
void processCommand(String cmd){
    cmd.trim();cmd.toLowerCase();
    if(cmd=="attrape")arm.grab();else if(cmd=="lache")arm.release();
    else if(cmd=="monte")arm.goUp();else if(cmd=="descend")arm.goDown();
    else if(cmd=="home")arm.goHome();
    else if(cmd=="stop"){emergencyStop=true;motors.stop();showOLED("URGENCE","ARRET");}
    else if(cmd=="go"){emergencyStop=false;showOLED("Robot","Reprise");}
    else if(cmd.startsWith("base:"))arm.setBase(cmd.substring(5).toInt());
    else if(cmd.startsWith("epaule:"))arm.setShoulder(cmd.substring(7).toInt());
    else if(cmd.startsWith("coude:"))arm.setElbow(cmd.substring(6).toInt());
}
void checkObstacles(){
    float d=obstacles.getDistance(FRONT);
    if(d>0&&d<30){motors.stop();btSerial.println("OBSTACLE:FRONT:"+String(d,0));}
}
void showOLED(String l1,String l2){
    oled.clearDisplay();oled.setTextSize(1);oled.setTextColor(WHITE);
    oled.setCursor(0,0);oled.println(l1);oled.setCursor(0,20);oled.println(l2);oled.display();
}
