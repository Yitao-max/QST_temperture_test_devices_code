/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
//use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 4300.0

// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 1000.0


double startpoint,Setpoint,startpoint_first,control_value,temp,Setpoint_change,control_change;
double Setpoint_first = 25.0;
int temperatureMode = 0 ;//= 0; // 0 = change the temperture to the setpoint, 1 = rise,2 = decline.
bool firstloop = true; 
bool startpoint_print = true;
bool judge_case = false;

unsigned long lastTime,now;
double Input,Output,timeChange;
double errSum, lastErr;
double kp, ki, kd;

int controlpin = 3;
int interruptPin = 2;
int H_blue = 6;
int i=4;

void calculate()
{
  now = millis();
  timeChange = (double)(now - lastTime)/1000;
  
   /*Compute all the working error variables*/
   double error = Setpoint - Input;
   errSum += (error * timeChange);
    if(errSum>15){
      errSum = 15;
    }
    if(errSum<-15)
    {
      errSum = -15;
    }
    if(errSum<0.5&&lastErr>0.1&&error>0.1)
    {
      errSum += 1;
    }
    if(errSum>-0.5&&lastErr<-0.1&&error<-0.1)
    {
      errSum -= 1;
    }
   double dErr = (error - lastErr) / timeChange;
  
   /*Compute PID Output*/
   Output = kp * error + ki * errSum + kd * dErr;
  
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;;
}
  
void SetTunings(double Kp, double Ki, double Kd)
{
   kp = Kp;
   ki = Ki;
   kd = Kd;
}

void setup() {
  Serial.begin(115200);
  //Serial.println("Adafruit MAX31865 PT1000 Sensor Test!");
  pinMode(controlpin,OUTPUT); 
  pinMode(H_blue,OUTPUT);
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, FALLING);
  //pinMode(H_yellow,OUTPUT);
  thermo.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
}

void loop() {
    if(firstloop){
    startpoint_first = thermo.temperature(RNOMINAL, RREF);
    firstloop = false;
  }
  
  if (Serial.available() > 0) {
    String modeValue = Serial.readStringUntil('\n');
    modeValue.trim(); 
    temperatureMode = modeValue.toInt();
    modeValue = Serial.readStringUntil('\n');
    modeValue.trim();
    Setpoint_first = modeValue.toInt();
  }

  Input = thermo.temperature(RNOMINAL, RREF);

  switch(temperatureMode){

  case 0:
  //uint16_t rtd = thermo.readRTD();
  //Serial.print("RTD value: ");
  //Serial.println(rtd);
  //double ratio = rtd;
  //ratio /= 32768;
  SetTunings(10,2,3);
  startpoint = startpoint_first;
  Setpoint = Setpoint_first;
  Setpoint_change = thermo.temperature(RNOMINAL, RREF);
  calculate();
  temp = Setpoint+Output; 
  if(Setpoint>startpoint)
  { 
  control_value = 510*(0.01747*(Output)-0.02563);
  digitalWrite(H_blue,HIGH);
  }
  else{
  control_value = 510*(-0.01747*(Output)-0.02563);
  digitalWrite(H_blue,LOW);
  }
  if (control_value>255){
    control_value = 255.00;
  }
  if (control_value<0){
    control_value = 0.00;
  }
  analogWrite(controlpin,round(control_value));


  //Serial.print("Ratio = ");
  //Serial.println(ratio, 8);
  //Serial.print("Resistance = ");
  //Serial.println(RREF * ratio, 8);
  Serial.print("Temperature = ");
  Serial.println(thermo.temperature(RNOMINAL, RREF));
  //Serial.print("control_value = ");
  //Serial.println(control_value,8);
  //Serial.print("timeChange= ");
  //Serial.println(timeChange,8);
  //Serial.print("Setpoint= ");
  //Serial.println(Setpoint,8);
  //Serial.print("startpoint= ");
  //Serial.println(startpoint,8);
  //erial.print("errsum= ");
  //Serial.println(errSum,8);
  //Serial.println();
  delay(100);
  startpoint_print = true;
  break;

  case 1:
  SetTunings(4,0,6);
  startpoint_first = thermo.temperature(RNOMINAL, RREF);
  if(startpoint_print)
  {
    Setpoint = startpoint_first+0.75;
    startpoint = startpoint_first;
    i=4;
  }
  if(Setpoint < startpoint+19)
  {
    i--;
    if(i<1)
    {
    Setpoint += 0.75;
    i = 3;
    }
  }
  calculate();
  temp = Setpoint+Output;
  control_value = 510*(0.01747*(Output)-0.02563);
  digitalWrite(H_blue,HIGH);
  if (control_value>255){
    control_value = 255.00;
  }
  if (control_value<0){
    control_value = 0.00;
  }
  analogWrite(controlpin,round(control_value));
  //Serial.print("Temperature = ");
  if(startpoint_print){
  Serial.print("startpoint ");
  Serial.println(startpoint,8);
  startpoint_print = false;
  }
  Serial.println(thermo.temperature(RNOMINAL, RREF));
  //Serial.print("temperatureMode= ");
  //Serial.println(temperatureMode,8);
  //Serial.println();
  delay(100);
  break;

  case 2:
  SetTunings(5,0,3);
  startpoint_first = thermo.temperature(RNOMINAL, RREF);
  if(startpoint_print)
  {
    Setpoint = startpoint_first-0.9;
    startpoint = startpoint_first; 
    i=4;
  }
  if(Setpoint > startpoint-19)
  {
    i--;
    if(i<=0)
    {
    Setpoint -= 0.9;
    i = 3;
    }
  }
  calculate();
  temp = Setpoint+Output;
  control_value = 510*(-0.01747*(Output)-0.02563);
  digitalWrite(H_blue,LOW);
  if (control_value>255){
    control_value = 255.00;
  }
  if (control_value<0){
    control_value = 0.00;
  }
  analogWrite(controlpin,round(control_value));
  //Serial.print("Temperature = ");
  if(startpoint_print)
  {
  Serial.print("startpoint ");
  Serial.println(startpoint,8);
  startpoint_print = false;
  }
  Serial.println(thermo.temperature(RNOMINAL, RREF));
  //Serial.print("Setpoint ");
  //Serial.println(Setpoint,8);
  //Serial.print("i ");
  //Serial.println(i,8);
  //Serial.println();
  delay(100);
  break;
}

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x");
    Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage");
    }
    thermo.clearFault();
  }
}
void blink(){
  temperatureMode = 0;
  Setpoint_first= 32;
  startpoint_first = thermo.temperature(RNOMINAL, RREF); 
  errSum = 0;
}