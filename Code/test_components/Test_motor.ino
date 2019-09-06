//Motor Pins in heading direction
int EN_A_RIGHTMOTOR = 13;      //Enable pin for right motor
int IN1_RIGHTMOTOR = 12;       //control pin for right motor
int IN2_RIGHTMOTOR = 11;       //control pin for right motor

int EN_B_LEFTMOTOR = 10;      //Enable pin for left motor
int IN3_LEFTMOTOR = 9;        //control pin for left motor
int IN4_LEFTMOTOR = 8;        //control pin for left motor

//Initializing variables to store data
int motor_speedL;
int motor_speedR;
// Set speeds accordingly 0-min, 255-max
int maxspeed = 50; //Max is 255
int stopspeed = 0;

void setup ( ) {
  Serial.begin (9600); //Starting the serial communication at 9600 baud rate
  //Initializing the motor pins as output

  //Right Motor
  pinMode(EN_A_RIGHTMOTOR, OUTPUT);
  pinMode(IN1_RIGHTMOTOR, OUTPUT);
  pinMode(IN2_RIGHTMOTOR, OUTPUT);
  //Left Motor
  pinMode(EN_B_LEFTMOTOR, OUTPUT);
  pinMode(IN3_LEFTMOTOR, OUTPUT);
  pinMode(IN4_LEFTMOTOR, OUTPUT);

  //Setting Direction of motors according to L298N motor controller
  // Right Motor
  digitalWrite(IN1_RIGHTMOTOR, HIGH);
  digitalWrite(IN2_RIGHTMOTOR, LOW);

  // Left motor
  digitalWrite(IN3_LEFTMOTOR, HIGH);
  digitalWrite(IN4_LEFTMOTOR, LOW);
}
void loop()
{
    motor_speedL = maxspeed;
    motor_speedR = maxspeed;
    analogWrite(EN_A_RIGHTMOTOR, motor_speedL);
    analogWrite(EN_B_LEFTMOTOR, motor_speedR);
    delay(10);
}
