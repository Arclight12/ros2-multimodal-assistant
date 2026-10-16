/*
 Fixed-pose demo: D=pickup, return home, hold 3s, release; H=home/open;
 O=open; C=close; x=stop commanded movement.
 CH0 base, CH1/CH2 coupled linkage, CH3 gripper; PCA9685 at 0x40.
 Before boot, physically support/place arm at HOME_POSE. No feedback sensors:
 pulses are commanded positions, not measured positions. Test path unloaded.
 Cut servo power immediately if any servo buzzes, grinds, or binds.
 Servo supply must be separate regulated 5V on PCA9685 V+, common GND with Uno.
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "arm_calibration.h"
Adafruit_PWMServoDriver pwm(0x40);
int cur[4] = {330,430,400,300};
bool stopping=false, busy=false;

int pv(const ArmPose &p, int c) {
  if(c==0) return p.ch0; if(c==1) return p.ch1;
  if(c==2) return p.ch2; return p.ch3;
}
bool valid(int c,int v){return c>=0&&c<4&&v>=CHANNEL_MIN[c]&&v<=CHANNEL_MAX[c];}
void writeAll(){for(int c=0;c<4;c++)pwm.setPWM(c,0,cur[c]);}
bool checkStop(){
  while(Serial.available()){
    char c=Serial.read();
    if(c=='x'||c=='X'){stopping=true; return true;}
  }
  return stopping;
}
bool moveTo(int t[4]){
  int from[4], steps=0;
  for(int c=0;c<4;c++){
    if(!valid(c,t[c])){Serial.println("ERR,out_of_range");return false;}
    from[c]=cur[c]; steps=max(steps,abs(t[c]-from[c]));
  }
  for(int s=1;s<=steps;s++){
    if(checkStop()) return false;
    for(int c=0;c<4;c++){
      cur[c]=from[c]+(long)(t[c]-from[c])*s/steps;
    }
    writeAll(); delay(STEP_DELAY_MS);
  }
  for(int c=0;c<4;c++)cur[c]=t[c];
  writeAll(); return !checkStop();
}
bool pose(const ArmPose &p,int grip){
  int t[4]={p.ch0,p.ch1,p.ch2,grip}; return moveTo(t);
}
bool grip(int v){
  int t[4]={cur[0],cur[1],cur[2],v}; return moveTo(t);
}
bool runDemo(){
  if(busy){Serial.println("ERR,busy");return false;}
  busy=true; stopping=false;
  Serial.println("STATE,open");
  if(!grip(GRIPPER_OPEN_PULSE))goto fail;
  Serial.println("STATE,pickup");
  if(!pose(PICKUP_POSE,GRIPPER_OPEN_PULSE))goto fail;
  Serial.println("STATE,close");
  if(!grip(GRIPPER_CLOSE_PULSE))goto fail;
  Serial.println("STATE,return_home");
  if(!pose(HOME_POSE,GRIPPER_CLOSE_PULSE))goto fail;
  Serial.println("STATE,hold_3s");
  for(unsigned long t=0;t<HANDOVER_HOLD_MS;t+=20){
    if(checkStop())goto fail; delay(20);
  }
  Serial.println("STATE,release");
  if(!grip(GRIPPER_OPEN_PULSE))goto fail;
  Serial.println("STATE,complete"); busy=false; return true;
fail:
  Serial.println("ERR,stopped"); busy=false; return false;
}
bool home(){stopping=false; return pose(HOME_POSE,GRIPPER_OPEN_PULSE);}
bool calibrationOK(){
  for(int c=0;c<4;c++){
    if(CHANNEL_MIN[c]>CHANNEL_MAX[c]||!valid(c,pv(HOME_POSE,c))||
       !valid(c,pv(PICKUP_POSE,c)))return false;
  }
  return valid(3,GRIPPER_OPEN_PULSE)&&valid(3,GRIPPER_CLOSE_PULSE);
}
void setup(){
  Serial.begin(115200); Wire.begin(); pwm.begin(); pwm.setPWMFreq(50); delay(10);
  for(int c=0;c<4;c++)pwm.setPWM(c,0,4096);
  if(!calibrationOK()){Serial.println("ERR,bad_calibration");while(true)delay(1000);}
  Serial.println("READY");
}
void loop(){
  static char b[24]; static int n=0; static bool overflow=false;
  while(Serial.available()){
    char c=Serial.read(); if(c=='\r')continue;
    if(c!='\n'){if(n<(int)sizeof(b)-1)b[n++]=c;else overflow=true;continue;}
    b[n]='\0'; bool ok=false;
    if(!overflow&&n==1){
      if(b[0]=='D')ok=runDemo();
      else if(b[0]=='H')ok=home();
      else if(b[0]=='O'){stopping=false;ok=grip(GRIPPER_OPEN_PULSE);}
      else if(b[0]=='C'){stopping=false;ok=grip(GRIPPER_CLOSE_PULSE);}
      else if(b[0]=='x'||b[0]=='X'){stopping=true;}
      else Serial.println("ERR,bad_command");
      if(ok)Serial.println("OK");
      else if(b[0]!='x'&&b[0]!='X')Serial.println("ERR,stopped");
    }else Serial.println("ERR,bad_command");
    n=0; overflow=false;
  }
}
