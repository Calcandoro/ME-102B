/*Bail-out Bike
ME 102B: Spring 2013
*/


#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

void stopIfFault()//Prints a fault if error with motor controller
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

int leftIR = A2; //left IR pin
int rightIR = A3; //right IR pin
float valL = 0; //left reading
float valR = 0; //right reading
int valL1 = 0; //the averaging values for valL
int valL2 = 0;
int valL3 = 0;
int valR1 = 0; //the averaging values for valR
int valR2 = 0;
int valR3 = 0;
int firstRun = HIGH; //decides to reset all values necessary for readings and speed
int leftSpeed = 0; //left motor speed
int rightSpeed = 0; //right motor speed
float time = 0; //time as watched by the main function
float timePrev = 0; //previous time recorded by the system
int currState;
int prevState = HIGH; //previous state of the button
int latchState = LOW; //state of the latch
int buttPin = 3;
int onOffSwitch = LOW; //decides if the system is on or off
int clock = 0; //clock for the checkLatch function
int turnLCount = 0; //cycles of needing to turn left
int turnRCount = 0; //cycles of needing to turn right
int valLPrev; //previous IR left reading
int valRPrev; //previous IR right reading
int dPosL = 0;
int dPosR = 0;
int errorL;
int errorR;
int dPosLPrev = 0;
int dPosRPrev = 0;
int leftSpeedPrev = 0;
int rightSpeedPrev = 0;
int turnLSpeed = 0;
int turnRSpeed = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(3, INPUT);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
}
  
void loop()
{
  currState = digitalRead(buttPin);
  if (currState == HIGH && prevState == LOW && abs(millis()-clock)>50){
    if(latchState == HIGH){
      latchState = LOW;
    }
    else{
      latchState = HIGH;
    }
    clock = millis();
  }
  prevState = currState;
  onOffSwitch = latchState;
  
  /*This will take a distance reading from both IR sensors*/
  if (onOffSwitch == LOW){
    leftSpeed = 0;
    rightSpeed = 0;
    firstRun == HIGH;
    md.setM1Speed(0);
    md.setM2Speed(0);
    turnLCount = 0;
    turnRCount = 0;
    leftSpeedPrev = 0;
    rightSpeedPrev = 0;
  }
  else {
    if (firstRun == HIGH) //For initial condition case
    {
      valL1=analogRead(leftIR);
      valR1=analogRead(rightIR);
      delay(20);
      valL2=analogRead(leftIR);
      valR2=analogRead(rightIR);
      delay(20);
      valL3=analogRead(leftIR);
      valR3=analogRead(rightIR);
      leftSpeed = 0;
      rightSpeed = 0;
      firstRun == LOW;
      valLPrev = (valL1 + valL2 + valL3) / 3;
      valRPrev = (valR1 + valR2 + valR3) / 3;
    }
    else //All runs after first run
    {
      valL1=analogRead(leftIR);
      valR1=analogRead(rightIR);
    }
    //averages 3 readings to get less noise
    valL=(valL1+valL2+valL3)/3;
    valR=(valR1+valR2+valR3)/3;
    //maps the readings to get more helpful readings
    valL=map(valL,1,1022,0,255);
    valR=map(valR,1,1022,0,255);
    //passes on readings to future cases
    valL3=valL2;
    valL2=valL1;
    valR3=valR2;
    valR2=valR1;
    
    /*consider delaying the differences by a cycle or two.
    This would possibly create larger differences in dPos
    and eliminate changes caused simply by noise*/
    dPosL = valL - valLPrev;
    dPosR = valR - valRPrev;
    
    valL = valL;//offset due to changes in voltage
    valR = valR;
    
    if (valL > 40 || valR > 40){ //too close. initiates the braking sequence
      leftSpeed = 0;
      rightSpeed = 0;
      turnRSpeed = 0;
      turnLSpeed = 0;
      md.setM1Speed(75); 
      md.setM2Speed(-75);
      delay(600);
      md.setSpeeds(0,0);
      md.setM1Brake(400);
      md.setM2Brake(400);
      turnLCount = 0;
      turnRCount = 0;
    }
    if (((valL <= 40 && valL >= 22) && valR <=45 )|| ((valR <= 40 && valR >= 25) && valL <= 45)){
      errorL = 40 - valL;
      errorR = 40 - valR;
      turnRSpeed = 0;
      turnLSpeed = 0;
      if ((dPosLPrev < -3 && dPosL < -3 && errorL > 5) || (dPosRPrev < -3 && dPosR < -3 && errorR > 5)){
        if (leftSpeedPrev == 0 && rightSpeedPrev == 0){
          rightSpeedPrev -= 15;
        }
        if (leftSpeedPrev >= 170){
          leftSpeed = leftSpeedPrev;
          rightSpeed = rightSpeedPrev;
        }
        else {
          leftSpeed = 13 + leftSpeedPrev;
          rightSpeed = rightSpeedPrev - 13;
        }
      }
      else if ((dPosLPrev > 3 && dPosL > 3 && errorL < -5) || (dPosRPrev > 3 && dPosR > 3 && errorR < -5)){
        leftSpeed = leftSpeedPrev - 23;
        rightSpeed = 23 + rightSpeedPrev;
        if (leftSpeed <=0 || rightSpeed >=0){
          leftSpeed = 0;
          rightSpeed = 0;
          md.setM1Speed(50);
          md.setM2Speed(-50);
          delay(600);
          md.setSpeeds(0,0);
          md.setBrakes(400,400);
        }        
      }
      if (abs(valL-valR) >= 8){ //this is what will handle the turning
        if (valL > valR){
          turnLCount += 1;
          turnRCount = 0;
          if (turnLCount >= 2){ //left turn. Right(M1) is large while left(M2) is small
            turnRSpeed = -170;
            if (leftSpeed <= 150 && leftSpeed >= 50){
              turnLSpeed = 50;
            }
            else if (leftSpeed < 50){
              turnLSpeed = 0;
            }
            else {
              turnLSpeed = leftSpeed - 150;
            }
            md.setM1Speed(turnRSpeed);
            md.setM2Speed(turnLSpeed);
            delay(1000);
            md.setM1Speed(rightSpeed);
            md.setM2Speed(leftSpeed);
            delay(500);
          }
        }
        else if (valR > valL){
          turnRCount += 1;
          turnLCount = 0;
          if (turnRCount >= 2){ //right turn. Left(M2) is large while right(M1) is small
            turnLSpeed = 170;
            if (rightSpeed >= -150 && rightSpeed <= -50){
              turnRSpeed = -65;
            }
            else if (rightSpeed > -50){
              turnRSpeed = 0;
            }
            else {
              turnRSpeed = rightSpeed + 150;
            }
            md.setM1Speed(turnRSpeed);
            md.setM2Speed(turnLSpeed);
            delay(1000);
            md.setM1Speed(rightSpeed);
            md.setM2Speed(leftSpeed);
            delay(500);
          }
        }
      }
      else {
        turnLCount = 0;
        turnRCount = 0;
      }
    }
    else { //This is for no sight
      leftSpeed = 0;
      rightSpeed = 0;
      md.setBrakes(400,400);
    }
    
    if ((turnLCount >= 2 || turnRCount >= 2) && (valL <= 40 && valR <= 40)){ //second stage of turning
      md.setM1Speed(turnRSpeed);
      md.setM2Speed(turnLSpeed);
      delay(600);
    }
    else {
      md.setM1Speed(rightSpeed);
      md.setM2Speed(leftSpeed);
    }
    leftSpeedPrev = leftSpeed;
    rightSpeedPrev = rightSpeed;
    valLPrev = valL;
    valRPrev = valR;
    dPosLPrev = dPosL;
    dPosRPrev = dPosR;
  }
}
