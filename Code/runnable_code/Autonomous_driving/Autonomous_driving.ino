// **************************** ENCODERS **************************
/* Records encoder ticks for each wheel
 * and prints the number of ticks for
 * each encoder every 500ms
 */
// pins for the encoder inputs
//Encoder on right wheel
#define RH_ENCODER_OUT_A 2 // OutA (right motor) must be connected to an interrupt pin
#define RH_ENCODER_OUT_B 5
//Encoder on left wheel
#define LH_ENCODER_OUT_A 3 // OutA (left motor) must be connected to an interrupt pin
#define LH_ENCODER_OUT_B 4

int rightPos = 0; // right wheel position
int leftPos = 0; // right wheel position
// variables to store the number of encoder pulses
// for each motor
volatile unsigned long leftCount = 1000; // use of volatile variable because of the interrupts, 
volatile unsigned long rightCount = 1000;

// **************************** LIGHT SENSORS **************************

#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 16;
uint16_t sensorValues[SensorCount];
uint16_t position;

// **************************** MOTORS **************************

//Motor Pins in heading direction
int EN_A_RIGHTMOTOR = 13;      //Enable pin for right motor
int IN1_RIGHTMOTOR = 12;       //control pin for right motor
int IN2_RIGHTMOTOR = 11;       //control pin for right motor

int EN_B_LEFTMOTOR = 10;      //Enable pin for left motor
int IN3_LEFTMOTOR = 9;        //control pin for left motor
int IN4_LEFTMOTOR = 8;        //control pin for left motor

//Initializing variables to store data
int slowest_speed = 55;
int motor_speedL = slowest_speed;
int motor_speedR = slowest_speed;
// Set speeds accordingly 0-min, 255-max
int maxspeed = 200; //Max is 255

// **************************** PID regulator **************************
//PID constants
double kp = 30;
double ki = 0;
double kd = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
int tolerance;

void setup() {
// **************************** ENCODERS **************************
  pinMode(LH_ENCODER_OUT_A, INPUT);
  pinMode(LH_ENCODER_OUT_B, INPUT);

  pinMode(RH_ENCODER_OUT_A, INPUT);
  pinMode(RH_ENCODER_OUT_B, INPUT);

  // initialize hardware interrupts
  attachInterrupt(0, leftEncoderEvent, CHANGE);  // 0 equals pin 2 on arduino mega (reaad more here: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)
  attachInterrupt(1, rightEncoderEvent, CHANGE); // 1 equals pin 3 on arduino mega
  
// **************************** LIGHT SENSORS **************************
// configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A15,A14,A13,A12,A11,A10,A9,A8,A7,A6,A5,A4,A3,A2,A1,A0}, SensorCount);
  //qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

// **************************** MOTORS **************************

  //Right Motor
  pinMode(EN_A_RIGHTMOTOR, OUTPUT);
  pinMode(IN1_RIGHTMOTOR, OUTPUT);
  pinMode(IN2_RIGHTMOTOR, OUTPUT);
  //Left Motor
  pinMode(EN_B_LEFTMOTOR, OUTPUT);
  pinMode(IN3_LEFTMOTOR, OUTPUT);
  pinMode(IN4_LEFTMOTOR, OUTPUT);

  //Setting Direction of motors according to L298N motor controller
  // Left Motor
  digitalWrite(IN1_RIGHTMOTOR, HIGH);
  digitalWrite(IN2_RIGHTMOTOR, LOW);

  // Right motor
  digitalWrite(IN3_LEFTMOTOR, HIGH);
  digitalWrite(IN4_LEFTMOTOR, LOW);
  
// **************************** PID regulator **************************
  setPoint = 7500;   //set point at the middle of the sensor
  tolerance = 300;
  Serial.begin(9600);
}

void loop() {
// **************************** ENCODERS **************************
/*
  Serial.print("Right Count: ");
  Serial.println(rightCount);
  Serial.print("Left Count: ");
  Serial.println(leftCount);
  Serial.println();
  delay(500);
  */
// **************************** LIGHT SENSORS **************************
    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 5000 (for a white line, use readLineWhite() instead)
    
    position = qtr.readLineBlack(sensorValues);

    // print the sensor values as numbers from 0 to 1000, where 0 means maximum
    // reflectance and 1000 means minimum reflectance, followed by the line
    // position
    /*
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    */
    
    //Serial.println(position);
    
// **************************** PID regulator **************************
    input = position;                         // value of the light sensors array (7500 is in the middle)
    //input = qtr.readLineBlack(sensorValues);

    
    if (input < setPoint - tolerance){
      output = computePID(input);
      motor_speedR = slowest_speed + output;
      motor_speedL = slowest_speed - output*70;
      if (motor_speedL < 0){
        motor_speedL = 0;
        }
      }
    else if(input > setPoint + tolerance){
      output = computePID(input);
      motor_speedL = slowest_speed + output;
      motor_speedR = slowest_speed - output*70;
      if (motor_speedR < 0){
        motor_speedR = 0;
        }
      }
    else{
      motor_speedR = motor_speedL = slowest_speed;         
    }
    if(motor_speedR > maxspeed){
      motor_speedR = maxspeed;
      }  
    if(motor_speedL > maxspeed){
      motor_speedL = maxspeed;
      }  
      
    analogWrite(EN_A_RIGHTMOTOR, motor_speedR); //control the motors based on PID value
    analogWrite(EN_B_LEFTMOTOR, motor_speedL);     
    /*
    Serial.print("     Right speed: ");
    Serial.println(motor_speedR);
    Serial.print("                       Left speed: ");
    Serial.println(motor_speedL);
    
    Serial.print("Position: ");
    Serial.println(input);
    */
    //delay(200);
    
// **************************** OLD P-regulator **************************
    /*
    if (position < 3400) {
      motor_speedR = slowest_speed+((3400-position)/100);
      if (position < 3000) {
        motor_speedL = 0;
        }
      else {
        motor_speedL = slowest_speed*0.7-((3400-position)/100);
        motor_speedL = abs(motor_speedL);
        }
      }
    else if(position > 3600) {
      motor_speedL = slowest_speed+((position-3600)/100);
      if (position > 4000) {
        motor_speedR = 0;
      }
      else{
        motor_speedR = slowest_speed*0.7-((position-3500)/100);
        motor_speedR = abs(motor_speedR);
        }
      }
      
    else{
      motor_speedR = motor_speedL = slowest_speed;
      }
    analogWrite(EN_A_RIGHTMOTOR, motor_speedR);
    analogWrite(EN_B_LEFTMOTOR, motor_speedL);   
    */
   /*
    Serial.print("     Right speed: ");
    Serial.println(motor_speedR);
    Serial.print("                       Left speed: ");
    Serial.println(motor_speedL);
    delay(300);
    */    
}

// **************************** PID regulator **************************
double computePID(double inp){     
        currentTime = millis();                              //get current time
        elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation
        
        if (inp > setPoint){
          error = inp - setPoint;                         // determine error          
          }
        else if(inp < setPoint){
          error = setPoint - inp;                         // determine error          
          }  

        cumError = error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;    // compute derivative
        rateError = abs(rateError);
        
        double out = kp*error + ki*cumError + kd*rateError;        //PID output     
        /*
        Serial.print("P: ");
        Serial.println(kp*error);
        Serial.print("             I: ");
        Serial.println(ki*cumError);
        Serial.print("                           D: ");
        Serial.println(kd*rateError);
        Serial.print("                                      Output ");
        Serial.println(out/10000);          
        Serial.print("                                                  elapsedTime: ");
        Serial.println(elapsedTime);      
        */
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

        return out/10000;//14250000;                              //have function return the PID output
}

// **************************** INTERRUPT FUNCTIONS (ENCODERS) **************************

// encoder event for the interrupt call
// If output A is ahead of output B, then the motor is turning forward. 
// If output A is behind B, the motor is turning backward
void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_OUT_A) == HIGH) { // raising edge
    if (digitalRead(LH_ENCODER_OUT_B) == LOW) {
      leftCount++; // A=high, B=low --> backwards
      //Serial.println("RAISING EDGE ++ ");
    } else {
      leftCount--; // A=high, B=high --> forward
      //Serial.println("RAISING EDGE -- ");
    }
  } else {                                     // falling edge
    if (digitalRead(LH_ENCODER_OUT_B) == LOW) {
      leftCount--; // A=low, B=low --> forward
      //Serial.println("FALLING EDGE -- ");
    } else {
      leftCount++; // A=low, B=high --> backwards
      //Serial.println("FALLING EDGE ++ ");
    }
  }
}

// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_OUT_A) == HIGH) {
    if (digitalRead(RH_ENCODER_OUT_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(RH_ENCODER_OUT_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}
