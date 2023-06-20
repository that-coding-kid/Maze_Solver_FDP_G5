#include <PID_v1.h>
#include<NewPing.h>

//Settings
int max_dist = 4000;
double Kp = 0.7, Ki = 0.1, Kd = 0.5;// propoprtional gain, differential gain and integeral gain for PID
int lExtSpeed = 30;  //Error
int turnTime = 300;
int goFowdTime = 400;
int thershold = 30;
int fThershold = 10; 

void readSensors();
void lTurn();
void rTurn();
void stopAll();
void stabilize();
void goFowd();
void decide();
void uTurn();
void Print();
 
int lTrig = A4;
int lEcho = A5;
int fTrig = A2;
int fEcho = A3;
int rTrig = A0;
int rEcho = A1;

const int lFowd = 6; //Left Forward pwm
const int lBack = 4;
const int rBack = 7;
const int rFowd = 5; //Right Forward pwm
const int En_A=9;
const int En_B=11;

double lDist, rDist, fDist;
double mean;   

double lms = 150, rms = 150;

NewPing lSensor(lTrig, lEcho, max_dist);
NewPing fSensor(fTrig, fEcho, max_dist);
NewPing rSensor(rTrig, rEcho, max_dist);

PID lPid(&lDist, &lms, &mean, Kp, Ki, Kd, DIRECT);
PID rPid(&rDist, &rms, &mean, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(En_A,OUTPUT);
  pinMode(En_B,OUTPUT);
  pinMode(rBack, OUTPUT);
  pinMode(rFowd, OUTPUT);
  pinMode(lBack, OUTPUT);
  pinMode(lFowd, OUTPUT);
  lPid.SetMode(AUTOMATIC);
  rPid.SetMode(AUTOMATIC);
  lPid.SetOutputLimits(outLowLim, outHigLim);
  rPid.SetOutputLimits(outLowLim, outHigLim);
  analogWrite(En_A,180);
  analogWrite(En_B,190);
}



void loop() {

  readSensors();
  stabilize();
  decide();
  stopAll();
}




void readSensors() {

  int iterations = 5;
  int lDura = lSensor.ping_median(iterations);
  delay(50);
  int fDura = fSensor.ping_median(iterations);
  delay(50);
  int rDura = rSensor.ping_median(iterations);
  delay(50);

  lDist = lSensor.convert_cm(lDura);
  fDist = fSensor.convert_cm(fDura);
  rDist = rSensor.convert_cm(rDura);
}

void lTurn(){
  
  analogWrite(lFowd, -183);
  analogWrite(lBack, 183);
  digitalWrite(rFowd, 183);
  digitalWrite(rFowd, -183);
  delay(turnTime);
}
void rTurn(){
  
  analogWrite(rFowd, -255);
  analogWrite(rBack, 255);
  analogWrite(lFowd, 255);
  analogWrite(lBack, -255);
  delay(turnTime);
}
void stopAll() {

  digitalWrite(rBack, LOW);
  digitalWrite(rFowd, LOW);
  digitalWrite(lBack, LOW);
  digitalWrite(lFowd, LOW);
}
void stabilize(){
  
  mean = (lDist+rDist)/2;
  lPid.Compute();
  rPid.Compute();
  
}
void goFowd() {

  analogWrite(lFowd, lms + lExtSpeed);
  digitalWrite(lFowd, HIGH);
  digitalWrite(lBack, LOW);
  digitalWrite(rFowd, LOW);
  digitalWrite(rFowd, HIGH);
  delay(goFowdTime);
}

void decide() {
  
  if (lDist > thershold)
  {
     goFowd();
     goFowd();
     goFowd();
     lTurn();
     lTurn();
     lTurn();
     lTurn();
     delay(100);
     goFowd();
     goFowd();
     
  }
  else if (fDist > fThershold)
  {
     if(lDist<5){ // Reallignment Function
      rTurn();
      digitalWrite(lFowd, HIGH);
      digitalWrite(lBack, LOW);
      digitalWrite(rFowd, LOW);
      digitalWrite(rFowd, HIGH);
      delay(50);
      lTurn();
     }
     else if(rDist<5){ // Reallignment Function
      lTurn();
      lTurn();
      digitalWrite(lFowd, HIGH);
      digitalWrite(lBack, LOW);
      digitalWrite(rFowd, LOW);
      digitalWrite(rFowd, HIGH);
      delay(50);
      rTurn();
     }
     else{
     goFowd();
     }
  }
  else if (rDist > thershold)
  {
     rTurn();
     rTurn();
     rTurn();
     rTurn();
     // observation: 4 rTurn() results in 90 degree turns in right
  }
  else 
  {
     rTurn();
     rTurn();
     rTurn();
     rTurn();
     rTurn();
     rTurn();
     rTurn();
     rTurn();
     // u turn is basically just taking two right turns
  }
}

void Print(){
Serial.print("Leftdistance = ");
Serial.println(lDist);
Serial.print("Rightdistance = ");
Serial.println(rDist);
Serial.print("Frontdistance = ");
Serial.println(fDist);  
Serial.print("Mean = ");
Serial.println(mean);
Serial.print("Lms = ");
Serial.println(lms);
Serial.print("Rms = ");
Serial.println(rms);
Serial.print("\n");
delay(300);
}