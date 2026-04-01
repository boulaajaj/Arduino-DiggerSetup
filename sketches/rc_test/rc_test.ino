// Digger Control V2.3 — Arduino Nano R4
// Expo + Inertia active | PID bypassed | X.BUS passive capture
// Serial = USB (debug/plot) | Serial1 = D0/D1 (X.BUS)

#include <Arduino.h>
#include <Servo.h>

// --- Pins ---
const uint8_t PIN_RC_LEFT  = 2;   // CH1 interrupt
const uint8_t PIN_RC_MODE  = 3;   // CH4 interrupt
const uint8_t PIN_RC_OVER  = 8;   // CH5 interrupt
const uint8_t PIN_RC_RIGHT = 12;  // CH2 interrupt
const uint8_t PIN_JOY_Y    = A0;  // Direct 5V, no divider
const uint8_t PIN_JOY_X    = A1;  // Direct 5V, no divider
const uint8_t PIN_ESC_LEFT = 9;   // PWM servo
const uint8_t PIN_ESC_RIGHT= 10;  // PWM servo

// --- Constants ---
const int ADC_MAX=16383, ADC_CENTER=8192;
const int JOY_DEADBAND=480, RC_DEADBAND=50;
const int SVC=1500, SVMIN=1000, SVMAX=2000;
const int MODE_LO=1250, MODE_HI=1750;
const float SOFT_RANGE=250.0f;
const float VMASS=3.0f, FRIC=8.0f, RESP=20.0f;
const unsigned long FAILSAFE_US=500000UL;

Servo escL, escR;

// --- RC interrupts ---
volatile unsigned long riseT[4]={0,0,0,0};
volatile int pw[4]={1500,1500,1000,1500};
volatile unsigned long pwT[4]={0,0,0,0};
enum{CL=0,CR=1,CO=2,CM=3};

void isrL(){unsigned long n=micros();if(digitalRead(PIN_RC_LEFT)==HIGH)riseT[CL]=n;
  else{unsigned long p=n-riseT[CL];if(p>=800&&p<=2200){pw[CL]=p;pwT[CL]=n;}}}
void isrR(){unsigned long n=micros();if(digitalRead(PIN_RC_RIGHT)==HIGH)riseT[CR]=n;
  else{unsigned long p=n-riseT[CR];if(p>=800&&p<=2200){pw[CR]=p;pwT[CR]=n;}}}
void isrO(){unsigned long n=micros();if(digitalRead(PIN_RC_OVER)==HIGH)riseT[CO]=n;
  else{unsigned long p=n-riseT[CO];if(p>=800&&p<=2200){pw[CO]=p;pwT[CO]=n;}}}
void isrM(){unsigned long n=micros();if(digitalRead(PIN_RC_MODE)==HIGH)riseT[CM]=n;
  else{unsigned long p=n-riseT[CM];if(p>=800&&p<=2200){pw[CM]=p;pwT[CM]=n;}}}

// --- X.BUS passive capture on Serial1 (D0/D1) ---
int xbtotal=0;
bool xlocked=false; int xbi=0; unsigned long xstart=0;
const long xbauds[]={115200,250000,100000,19200,57600,38400,9600};
const int XBAUD_CNT=7;

void xbusInit(){xbi=0;xlocked=false;Serial1.begin(xbauds[0]);xstart=millis();xbtotal=0;}
void xbusUpdate(){
  while(Serial1.available()){Serial1.read();xbtotal++;}
  if(!xlocked&&millis()-xstart>=3000){
    if(xbtotal>=10)xlocked=true;
    else{xbi++;if(xbi>=XBAUD_CNT)xbi=0;Serial1.end();Serial1.begin(xbauds[xbi]);xstart=millis();xbtotal=0;}
  }
}

// --- State ---
float velL=0,velR=0,posL=0,posR=0;
unsigned long prevUs=0, prevPrint=0;

// --- Math ---
float expoCurve(float x){
  float a=fabsf(x),sq=a*a,r=a;
  if(r>0.001f){r=0.5f*(r+a/r);r=0.5f*(r+a/r);}
  return sq*r;
}
float fastTanh(float x){
  float a=fabsf(x);
  return constrain(x/(1.0f+a+0.28f*a*a),-1.0f,1.0f);
}
int deadRC(int v){return(abs(v-SVC)<=RC_DEADBAND)?SVC:v;}
int deadJoy(int v){return(abs(v-ADC_CENTER)<=JOY_DEADBAND)?ADC_CENTER:v;}
int joyExpo(int adc){
  float n=(float)(adc-ADC_CENTER)/(float)ADC_CENTER;
  n=constrain(n,-1.0f,1.0f);
  float s=(n>=0)?1.0f:-1.0f;
  return SVC+(int)(s*expoCurve(n)*SOFT_RANGE);
}
int softLim(float d){
  if(fabsf(d)<0.5f)return SVC;
  return SVC+(int)(SOFT_RANGE*fastTanh(d/SOFT_RANGE)+0.5f);
}

void setup(){
  Serial.begin(115200);  // USB — goes straight to PC
  delay(500);
  Serial.println("=== Digger Control V2.3 — Nano R4 ===");
  Serial.println("Expo+Inertia | PID bypassed | XBUS capture");
  Serial.println("RC1,RC2,RC4,RC5,JoyY,JoyX,OutL,OutR,XbusB");

  analogReadResolution(14);

  pinMode(PIN_RC_LEFT,INPUT);
  pinMode(PIN_RC_MODE,INPUT);
  pinMode(PIN_RC_OVER,INPUT);
  pinMode(PIN_RC_RIGHT,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_LEFT),isrL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_MODE),isrM,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_OVER),isrO,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_RIGHT),isrR,CHANGE);

  escL.attach(PIN_ESC_LEFT);
  escR.attach(PIN_ESC_RIGHT);
  escL.writeMicroseconds(SVC);
  escR.writeMicroseconds(SVC);

  prevUs=micros();
  xbusInit();
}

void loop(){
  unsigned long now=micros();
  unsigned long dt=now-prevUs;
  if(dt==0)return;
  prevUs=now;
  float dts=(float)dt*0.000001f;

  xbusUpdate();

  noInterrupts();
  int rcL=pw[CL],rcR=pw[CR],rcO=pw[CO],rcM=pw[CM];
  unsigned long tL=pwT[CL],tR=pwT[CR];
  interrupts();

  int jy=analogRead(PIN_JOY_Y),jx=analogRead(PIN_JOY_X);

  bool lost=(now-tL>FAILSAFE_US)&&(now-tR>FAILSAFE_US);
  int sL=lost?SVC:deadRC(rcL);
  int sR=lost?SVC:deadRC(rcR);

  int jT=joyExpo(deadJoy(jy)),jS=joyExpo(deadJoy(jx));
  int jo=jS-SVC;
  int jL=constrain(jT+jo,SVMIN,SVMAX);
  int jR=constrain(jT-jo,SVMIN,SVMAX);

  int left,right;
  if(rcO<MODE_LO){left=sL;right=sR;}
  else if(rcO<=MODE_HI){
    bool act=(sL!=SVC)||(sR!=SVC);
    if(act&&!lost){left=sL;right=sR;}else{left=jL;right=jR;}
  }else{left=(sL+jL)/2;right=(sR+jR)/2;}

  int scL=softLim((float)(left-SVC));
  int scR=softLim((float)(right-SVC));

  float tgL=(float)(scL-SVC),tgR=(float)(scR-SVC);
  velL+=(RESP*(tgL-posL)-FRIC*velL)/VMASS*dts;
  velR+=(RESP*(tgR-posR)-FRIC*velR)/VMASS*dts;
  posL+=velL*dts;posR+=velR*dts;
  if(fabsf(tgL)<1&&fabsf(posL)<2&&fabsf(velL)<5){posL=0;velL=0;}
  if(fabsf(tgR)<1&&fabsf(posR)<2&&fabsf(velR)<5){posR=0;velR=0;}

  int outL=constrain(SVC+(int)(posL+0.5f),SVMIN,SVMAX);
  int outR=constrain(SVC+(int)(posR+0.5f),SVMIN,SVMAX);

  escL.writeMicroseconds(outL);
  escR.writeMicroseconds(outR);

  // 10 Hz CSV output via USB
  if(now-prevPrint>=100000UL){
    prevPrint=now;
    char buf[100];
    sprintf(buf,"%d,%d,%d,%d,%d,%d,%d,%d,%d",
            rcL,rcR,rcM,rcO,jy,jx,outL,outR,xbtotal);
    Serial.println(buf);
    xbtotal=0;
  }
}
