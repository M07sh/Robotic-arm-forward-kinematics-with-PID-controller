int pulses1;                              
int encoderA1 = 3;
int encoderB1 = 2;
const int pwm1 = 11;                  
const int M11 = 5;                       
const int M12 = 4;
int angle1 =0;
int angle1out=0;

int pulses2;                              
int encoderA2 = 18;
int encoderB2 = 19;
const int pwm2 = 12;                      
const int M21 = 6;                       
const int M22 = 7;
const int pin5V=22;
int angle2 =0;
int angle2out=0;

int pulses3;                              
int encoderA3 = 20;
int encoderB3 = 21;
const int pwm3 = 10;                      
const int M31 = 8;                       
const int M32 = 9;
int angle3 =0;
int angle3out=0;

#define total 2400                        

#include "math.h"
using namespace std;

//time difference
unsigned long time;
 float dt=0;
 float prev=0;

//PID_A
float e_A=0;
float I_A=0;
float D_A=0;
float prev_eA=0;
int oA=0;

//PID_B
float e_B=0;
float I_B=0;
float D_B=0;
double prev_eB=0;
int oB=0;

//PID_C
float e_C=0;
float I_C=0;
float D_C=0;
float prev_eC=0;
int oC=0;

//user input angle
String detha;
String detha1;
String detha2;
int angtopul1=0;
int angtopul2=0;
int angtopul3=0;

//position
float px=0;
float py=0;
float pz=0;

void setup() {
  
  Serial.begin(9600);
  pinMode(pin5V,OUTPUT);
  digitalWrite(pin5V, HIGH);
  pinMode(pwm1, OUTPUT);
  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  analogWrite(pwm1, 0);                     
  digitalWrite(M11, HIGH);
  digitalWrite(M12, HIGH);
  pinMode(encoderA1, INPUT);
  digitalWrite(encoderA1,HIGH);
  pinMode(encoderB1, INPUT);
  digitalWrite(encoderB1,HIGH);
  attachInterrupt(0, A1_CHANGE, CHANGE);
  attachInterrupt(1, B1_CHANGE, CHANGE);

  pinMode(pwm2, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);
  analogWrite(pwm2, 0);                     
  digitalWrite(M21, HIGH);
  digitalWrite(M22, HIGH);
  pinMode(encoderA2, INPUT);
  digitalWrite(encoderA2,HIGH);
  pinMode(encoderB2, INPUT);
  digitalWrite(encoderB2,HIGH);
  attachInterrupt(4, A2_CHANGE, CHANGE);
  attachInterrupt(5, B2_CHANGE, CHANGE);

  pinMode(pwm3, OUTPUT);
  pinMode(M31, OUTPUT);
  pinMode(M32, OUTPUT);
  analogWrite(pwm3, 0);                     
  digitalWrite(M31, HIGH);
  digitalWrite(M32, HIGH);
  pinMode(encoderA3, INPUT);
  digitalWrite(encoderA3,HIGH);
  pinMode(encoderB3, INPUT);
  digitalWrite(encoderB3,HIGH);
  attachInterrupt(2, A3_CHANGE, CHANGE);
  attachInterrupt(3, B3_CHANGE, CHANGE);

}//setup

void loop(){
  while(Serial.available() == 0){
     
  }
  
  if (Serial.available() >0){
    detha  = Serial.readStringUntil(',');
    Serial.read();
    detha1 = Serial.readStringUntil(',');
    Serial.read();
    detha2  = Serial.readStringUntil('\0'); 
    angle1=detha.toInt();
    angle2=detha1.toInt();
    angle3=detha2.toInt();
    angtopul1=angle1*6.66666667;
    angtopul2=angle2*6.66666667;
    angtopul3=angle3*6.66666667;
    Serial.print(angtopul1);
    Serial.print(" ");
    Serial.print(angtopul2);
    Serial.print(" ");
    Serial.println(angtopul3);  
  }
    while(1){
    timer();
    PID_A(angtopul1); 
    PID_B(angtopul2); 
    PID_C(angtopul3); 
    
    angle1out = pulses1*0.15;  
    Serial.print("1.) " );  
    Serial.print(angle1out);
    Serial.print(",  ");
    Serial.print(e_A);
    Serial.print(",  ");
    Serial.println(oA);
    
    angle2out = pulses2*0.15;
    Serial.print("2.) " );    
    Serial.print(angle2out);
    Serial.print(",  ");
    Serial.print(e_B);
    Serial.print(",  ");
    Serial.println(oB);
    
    angle3out = pulses3*0.15; 
    Serial.print("3.) " );
    Serial.print(pulses3);
    Serial.print(",  ");    
    Serial.print(angle3out);
    Serial.print(",  ");
    Serial.print(e_C);
    Serial.print(",  ");
    Serial.println(oC);
    
    //print the px py pz   
    pos();
    Serial.print("(Px,Py,Pz)=(" );    
    Serial.print(px);
    Serial.print(",  ");
    Serial.print(py);
    Serial.print(",  ");
    Serial.print(pz);
    Serial.println(")  ");
    char s = Serial.read(); 
    switch (s){    
       case 's':
       timer();
       angtopul1=0;
       angtopul2=0;
       angtopul3=0;
       PID_A(angtopul1); 
       PID_B(angtopul2); 
       PID_C(angtopul3); 
    }
    
}}

void timer()
{
  time = millis();
  dt=(time-prev)*0.001;
  prev=time;
  delay(10);
}

void PID_A(int setpoint)
{
  e_A=setpoint-pulses1;
  I_A=I_A+e_A*dt;
  D_A=(e_A-prev_eA)/dt;
  oA=e_A*0.7+0.04*D_A;
  prev_eA=e_A;
  delay(dt);
  
   if (oA<-255 ){
    oA=-255 ;     
   }
   else if(oA>255){
     oA=255;   
    }
    else{
      oA=oA;       
    }
      if (oA<0){
      oA=-1*oA;
      oA=oA*0.67058+75;
      analogWrite(pwm1,oA);
      digitalWrite(M11,LOW);
      digitalWrite(M12,HIGH);
      if (oA<83)
      {
        oA=0;    
      digitalWrite(M11,LOW);
      digitalWrite(M12,LOW);  
      }
      else{
        oA=oA;
      }
           
    }
    else{
      oA=oA*0.67058+75;
      analogWrite(pwm1,oA);
      digitalWrite(M11,HIGH);
      digitalWrite(M12,LOW);
      if (oA<83)
      {
        oA=0;    
      digitalWrite(M11,LOW);
      digitalWrite(M12,LOW);  
      }
      else{
        oA=oA;
      }
     
    }  
}

void PID_B(int setpoint)
{
  e_B=setpoint-pulses2;
  I_B=I_B+e_B*dt;
  D_B=(e_B-prev_eB)/dt;
  oB=e_B*0.75+0.08*D_B;
  prev_eB=e_B;
  delay(dt);
  
   if (oB<-255 ){
    oB=-255 ;     
   }
   else if(oB>255){
     oB=255;   
    }
    else{
      oB=oB;       
    }
      if (oB<0){
      oB=-1*oB;
      oB=oB*0.72549+70;
      analogWrite(pwm2,oB);
      digitalWrite(M21,LOW);
      digitalWrite(M22,HIGH); 
      if (oB<79)
      {
        oB=0;    
      digitalWrite(M21,LOW);
      digitalWrite(M22,LOW);  
      }
      else{
        oB=oB;
      }
      
    }
    else{
      oB=oB*0.72549+70;
      analogWrite(pwm2,oB);
      digitalWrite(M21,HIGH);
      digitalWrite(M22,LOW);
      if (oB<79)
      {
        oB=0;  
      digitalWrite(M21,LOW);
      digitalWrite(M22,LOW);  
      }
      else{
        oB=oB;
      } 
      }  
}

void PID_C(int setpoint)
{
  e_C=setpoint-pulses3;
  I_C=I_C+e_C*dt;
  D_C=(e_C-prev_eC)/dt;
  oC=e_C*0.75+0.07*D_C;
  prev_eC=e_C;
  delay(dt);
  
   if (oC<-255 ){
    oC=-255 ;     
   }
   else if(oC>255){
     oC=255;   
    }
    else{
      oC=oC;       
    }
      if (oC<0){
      oC=-1*oC;
      oC=oC*0.72549+70;
      analogWrite(pwm3,oC);
      digitalWrite(M31,LOW);
      digitalWrite(M32,HIGH);  
      if (oC<79)
      {
        oC=0;  
      digitalWrite(M31,LOW);
      digitalWrite(M32,LOW);  
      }
      else{
        oC=oC;
      }     
    }
    else{
      oC=oC*0.72549+70;
      analogWrite(pwm3,oC);
      digitalWrite(M31,HIGH);
      digitalWrite(M32,LOW);
       if (oC<79)
      {
        oC=0;  
      digitalWrite(M31,LOW);
      digitalWrite(M32,LOW);  
      }
      else{
        oC=oC;
      }    
    }
  
}
void pos()
{
  float d1 =angle1out*0.01745;
  float d2 =angle2out*0.01745;
  float d3 =angle3out*0.01745;
  px=7.5*sin(d1)+10*cos(d1)*cos(d2);
  py=-7.5*cos(d1)+10*sin(d1)*sin(d2);
  pz=10*sin(d2);  
}
void A1_CHANGE(){
  if( digitalRead(encoderB1) == 0 ) {
    if ( digitalRead(encoderA1) == 0 ) { // A fell, B is low
      pulses1--;
    } else {// A rose, B is low
      pulses1++;
    }
  }else {
    if ( digitalRead(encoderA1) == 0 ) { // B fell, A is high
      pulses1++;
    } else {// B rose, A is high
      pulses1--; 
    }
  }
}
void B1_CHANGE(){
 if ( digitalRead(encoderA1) == 0 ) {
    if ( digitalRead(encoderB1) == 0 ) { // B fell, A is low
      pulses1++; 
    } else {// B rose, A is low
      pulses1--; 
    }
 } else {
    if ( digitalRead(encoderB1) == 0 ) {// B fell, A is high
      pulses1--; 
    } else {// B rose, A is high
      pulses1++; 
    }
  }
}

void A2_CHANGE(){
 if( digitalRead(encoderB2) == 0 ) {
    if ( digitalRead(encoderA2) == 0 ) { // A fell, B is low
      pulses2--;
    } else {// A rose, B is low
      pulses2++;
    }
  }else {
    if ( digitalRead(encoderA2) == 0 ) { // B fell, A is high
      pulses2++;
    } else {// B rose, A is high
      pulses2--; 
    }
  }
}
void B2_CHANGE(){
  if ( digitalRead(encoderA2) == 0 ) {
    if ( digitalRead(encoderB2) == 0 ) { // B fell, A is low
      pulses2++; 
    } else {// B rose, A is low
      pulses2--; 
    }
 } else {
    if ( digitalRead(encoderB2) == 0 ) {// B fell, A is high
      pulses2--; 
    } else {// B rose, A is high
      pulses2++; 
    }
  }
}

void A3_CHANGE(){
 if( digitalRead(encoderB3) == 0 ) {
    if ( digitalRead(encoderA3) == 0 ) { // A fell, B is low
      pulses3--;
    } else {// A rose, B is low
      pulses3++;
    }
  }else {
    if ( digitalRead(encoderA3) == 0 ) { // B fell, A is high
      pulses3++;
    } else {// B rose, A is high
      pulses3--; 
    }
  }
}
void B3_CHANGE(){
  if ( digitalRead(encoderA3) == 0 ) {
    if ( digitalRead(encoderB3) == 0 ) { // B fell, A is low
      pulses3++; 
    } else {// B rose, A is low
      pulses3--; 
    }
 } else {
    if ( digitalRead(encoderB3) == 0 ) {// B fell, A is high
      pulses3--; 
    } else {// B rose, A is high
      pulses3++; 
    }
  }
}
