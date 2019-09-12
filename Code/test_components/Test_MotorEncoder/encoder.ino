/*
 * Records encoder ticks for each wheel
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

// variables to store the number of encoder pulses
// for each motor
volatile unsigned long leftCount = 1000; // use of volatile variable because of the interrupts, 
volatile unsigned long rightCount = 1000;

void setup() {
  pinMode(LH_ENCODER_OUT_A, INPUT);
  pinMode(LH_ENCODER_OUT_B, INPUT);

  pinMode(RH_ENCODER_OUT_A, INPUT);
  pinMode(RH_ENCODER_OUT_B, INPUT);

  // initialize hardware interrupts
  attachInterrupt(0, leftEncoderEvent, CHANGE);  // 0 equals pin 2 on arduino mega (reaad more here: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)
  attachInterrupt(1, rightEncoderEvent, CHANGE); // 1 equals pin 3 on arduino mega
  
  Serial.begin(9600);
}

void loop() {
  Serial.print("Right Count: ");
  Serial.println(rightCount);
  Serial.print("Left Count: ");
  Serial.println(leftCount);
  Serial.println();
  delay(500);
}

// encoder event for the interrupt call
// If output A is ahead of output B, then the motor is turning forward. 
// If output A is behind B, the motor is turning backward
void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_OUT_A) == HIGH) { // raising edge
    if (digitalRead(LH_ENCODER_OUT_B) == LOW) {
      leftCount++; // A=high, B=low --> backwards
      Serial.println("RAISING EDGE ++ ");
    } else {
      leftCount--; // A=high, B=high --> forward
      Serial.println("RAISING EDGE -- ");
    }
  } else {                                     // falling edge
    if (digitalRead(LH_ENCODER_OUT_B) == LOW) {
      leftCount--; // A=low, B=low --> forward
      Serial.println("FALLING EDGE -- ");
    } else {
      leftCount++; // A=low, B=high --> backwards
      Serial.println("FALLING EDGE ++ ");
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
