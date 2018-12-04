/* Week 9 Pre-lab
 * IR Sensor
 * 
 * Gary Huarng
 * 
 * Detects if IR sensor is seeing "black" as opposed to "white"
 * based on the threshold setting and returns binary value of 
 * IR sensor status
 */
#include <Servo.h>
#include <math.h>

Servo myservo; //servo object
#define IR1 11 //left sensor
#define IR2 12 //left-mid sensor
#define IR3 13 //middle sensor
#define IR4 14 //right-mid sensor
#define IR5 15 //right sensor

#define pointingGain 20

float pathError;

const int motorA1 = 4; // IN1 (Motor A) 
const int motorA2 = 5; // IN2 (Motor A) 
const int motorB1 = 6; // IN3 (Motor B) 
const int motorB2 = 7; // IN4 (Motor B) 

const int ledPin = 13;
const int allMotors[] = {motorA1, motorA2, motorB1, motorB2}; 
const int rightMotor[] = {motorA1, motorA2}; 
const int leftMotor[] = {motorB1, motorB2}; 

const String turnsArray[] = {"L","R","F","B"}; 
int inputSpeed; 
String inputDirection; 
int inputTurnRatio;   

const int servoPin = 11;
byte sensorCode = 0; //byte binary representation of sensor status
byte previousSensorReads;
int sensors[] = {IR1, IR2, IR3, IR4, IR5}; //array of IR sensors
int sensorOut[] = {43, 45, 47, 49 , 51}; // array of sensor digital output pins
int const thresholds[] = {32,32,32,32,32}; //analog threshold value (0-1023) for the 5 IR sensors
int sensorA;
int currentState;
int previousState;
int previousError;

int leftMotorSpeed;
int rightMotorSpeed;
int speedAdjustmentControl;

int readLineSensor(int sensorAnalogInPin, int sensorDigitalOutPin){ //outputs 1 if analogRead() value of sensor is greater than threshold, 0 if less than.
  sensorA = analogRead(sensorAnalogInPin);
  if(sensorAnalogInPin == IR5){
    sensorA -= 10;
  }
  //Serial.println(sensorA);

  if(analogRead(sensorAnalogInPin) > thresholds[sensorAnalogInPin]){
    digitalWrite(sensorDigitalOutPin, HIGH);
    return 1; //black  
  }
  else{
    digitalWrite(analogRead(sensorDigitalOutPin), LOW);
    return 0; //white
  }
  
}

byte getPathSensorStates(){ //assigns binary value byte variable "sensorCode" based on sensor values
  for(int i = 0; i < 5; i++){
    if(readLineSensor(sensors[i], sensorOut[i]) == 1){
      bitWrite(sensorCode, i , 1);
    }
    else{
      bitWrite(sensorCode, i, 0);
    }    
  }
  //Serial.println(sensorCode, BIN);
  //Serial.println();
  return sensorCode;
}

float sensePathPositionError(byte PathSensorStates){
  if(bitRead(PathSensorStates, 0) == 1 && bitRead(PathSensorStates, 1) == 0 && bitRead(PathSensorStates, 2) == 0 && bitRead(PathSensorStates, 3) == 0 && bitRead(PathSensorStates, 4) == 0){
    return -2;
  }
  else if(bitRead(PathSensorStates, 0) == 1 && bitRead(PathSensorStates, 1) == 1 && bitRead(PathSensorStates, 2) == 0 && bitRead(PathSensorStates, 3) == 0 && bitRead(PathSensorStates, 4) == 0){
    return -1.5;
  }
  else if(bitRead(PathSensorStates, 0) == 0 && bitRead(PathSensorStates, 1) == 1 && bitRead(PathSensorStates, 2) == 0 && bitRead(PathSensorStates, 3) == 0 && bitRead(PathSensorStates, 4) == 0){
    return -1;
  }
  else if(bitRead(PathSensorStates, 0) == 0 && bitRead(PathSensorStates, 1) == 1 && bitRead(PathSensorStates, 2) == 1 && bitRead(PathSensorStates, 3) == 0 && bitRead(PathSensorStates, 4) == 0){
    return -0.5;
  }
  else if(bitRead(PathSensorStates, 0) == 0 && bitRead(PathSensorStates, 1) == 0 && bitRead(PathSensorStates, 2) == 1 && bitRead(PathSensorStates, 3) == 0 && bitRead(PathSensorStates, 4) == 0){
    return 0;
  }
  else if(bitRead(PathSensorStates, 0) == 0 && bitRead(PathSensorStates, 1) == 0 && bitRead(PathSensorStates, 2) == 1 && bitRead(PathSensorStates, 3) == 1 && bitRead(PathSensorStates, 4) == 0){
    return 0.5;
  }
  else if(bitRead(PathSensorStates, 0) == 0 && bitRead(PathSensorStates, 1) == 0 && bitRead(PathSensorStates, 2) == 0 && bitRead(PathSensorStates, 3) == 1 && bitRead(PathSensorStates, 4) == 0){
    return 1;
  }
  else if(bitRead(PathSensorStates, 0) == 0 && bitRead(PathSensorStates, 1) == 0 && bitRead(PathSensorStates, 2) == 0 && bitRead(PathSensorStates, 3) == 1 && bitRead(PathSensorStates, 4) == 1){
    return 1.5;
  }
  else if(bitRead(PathSensorStates, 0) == 0 && bitRead(PathSensorStates, 1) == 0 && bitRead(PathSensorStates, 2) == 0 && bitRead(PathSensorStates, 3) == 0 && bitRead(PathSensorStates, 4) == 1){
    return 2;
  }
  else{
    return 100; //bogus value  
  }  
} 

int updatePointingAngle(float pathError){
  if(pathError > 2 || pathError < -2){
    return;
  }
  return 90 + pointingGain*pathError;
}

void motorMove(int motorSide[], int motorSpeed){ //motor move function passing the motor to be moved (motorA1, motorB1, etc.) and a speed between -100 and 100% 
  if(motorSpeed == 0){ 
    analogWrite(motorSide[0], 0); 
    analogWrite(motorSide[1], 0);
  } 

  if(motorSpeed > 0){ 
  motorSpeed = map(motorSpeed, 0, 100, 0, 255); //converts 0 to 100 input value to the 0-256 bit analogWrite scale 
  analogWrite(motorSide[1], motorSpeed); 
  } 
  
  if(motorSpeed < 0){ 
  motorSpeed *= -1; 
  motorSpeed = map(motorSpeed, 0, 100, 0, 255); //converts 0 to 100 input value to the 0-256 bit analogWrite scale 
  analogWrite(motorSide[0], motorSpeed); 
  } 
} 

void straightMove(int straightSpeed){ //passes forward speed (0-100%) to the motor move for both motors 
  motorMove(rightMotor, straightSpeed); 
  motorMove(leftMotor, straightSpeed); 
} 

void robotMove(String moveDirection, int moveSpeed, float turnRatio){ //passes moveDirection("L", "R","F",B"), moveSpeed (0-100) and turn ratio (-1.00 - 1.00) to turn the robot  
  if(moveDirection != "R" && moveDirection != "L" && moveDirection != "F" && moveDirection != "B"){ 
    Serial.print(moveDirection); 
    Serial.println(" is not a valid direction! Use 'L' to turn left and 'R' to turn right"); 
    return; 
  } 

  if(turnRatio > 1 || turnRatio < -1){ 
    Serial.print(turnRatio); 
    Serial.println(" is not a valid value for the turning ratio. Valid range is -1 to 1"); 
    return; 
  } 

  if(moveDirection == "L"){ 
    motorMove(rightMotor, moveSpeed); 
    motorMove(leftMotor, moveSpeed* turnRatio);
  } 

  if(moveDirection == "R"){ 
    motorMove(leftMotor, moveSpeed); 
    motorMove(rightMotor, moveSpeed * turnRatio); 
  } 

  if(moveDirection == "F"){ 
    motorMove(leftMotor, moveSpeed); 
    motorMove(rightMotor, moveSpeed); 
  } 

  if(moveDirection == "B"){ 
    motorMove(leftMotor, -1*moveSpeed); 
    motorMove(rightMotor, -1*moveSpeed); 
  }     
} 

void stopMotors(){ //sets all motor gates to 0 to stop all motors 

  for(int i = 0; i < sizeof(allMotors)/sizeof(int);i++){ 
    analogWrite(allMotors[i],0); 
  } 
} 

void followLine(int followSpeed, float pathErrorIn, int gain){
    if(pathError > 2 || pathError < -2){
      return;
    }
    else{
    stopMotors();
    speedAdjustmentControl = gain * pathErrorIn;
    rightMotorSpeed = (100. - speedAdjustmentControl)*followSpeed / 100.;
    rightMotorSpeed = constrain(rightMotorSpeed, 0, followSpeed);
    leftMotorSpeed = (100. + speedAdjustmentControl)*followSpeed / 100.;
    leftMotorSpeed = constrain(leftMotorSpeed, 0, followSpeed);
    motorMove(rightMotor, rightMotorSpeed);
    motorMove(leftMotor, leftMotorSpeed);
    Serial.print("Path Error of "); Serial.print(pathErrorIn); Serial.print(" yields speeds of {");
    Serial.print("Left Motor: "); Serial.print(leftMotorSpeed); Serial.print("  ");
    Serial.print("Right Motor: "); Serial.print(rightMotorSpeed); Serial.println("}");
    }
 }

void debugLineState(){
  for(int i = 0; i<5; i++){
  if(bitRead(sensorCode, i) != bitRead(previousSensorReads,i)){
      for(int j = 0; j<5; j++){
        bitWrite(previousSensorReads, j, bitRead(sensorCode, j));  
        Serial.print("IR"); Serial.print(j+1); Serial.print(": ("); Serial.print(bitRead(sensorCode, j)); Serial.print(") ");
        }
    Serial.println();
    }
  }
  if(previousState != currentState){
    if(currentState == 1){
      Serial.println("On course: going straight");
    }
    else if(currentState == 2){
      Serial.print("Going Left");
    }
    else if(currentState == 3){
    Serial.print("Going Right");
    }
    else{
      Serial.print("Off Course: Searching");
    }
  }
  previousState = currentState;
}


void setup() {
  Serial.begin(9600);
  Serial.println("Serial connection established"); 
  myservo.attach(servoPin);
  myservo.write(180);
  delay(800);
  myservo.write(90);
  delay(800);

 pinMode(motorA1, OUTPUT); //Set digital pins as OUTPUTs 
 pinMode(motorA2, OUTPUT); 
 pinMode(motorB1, OUTPUT); 
 pinMode(motorB2, OUTPUT); 
 pinMode(ledPin, OUTPUT); 
}

void loop() {
  pathError = sensePathPositionError(getPathSensorStates());
  getPathSensorStates();
  debugLineState();
  myservo.write(updatePointingAngle(pathError));
  followLine(40, pathError, 15);
}
