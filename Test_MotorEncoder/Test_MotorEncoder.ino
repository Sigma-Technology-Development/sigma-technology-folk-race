float encA;
float encB;

//Motor Pins
int EN_A = 11;      //Enable pin for first motor
int IN1 = 9;       //control pin for first motor
int IN2 = 8;       //control pin for first motor
//int IN3 = 7;        //control pin for second motor
//int IN4 = 6;        //control pin for second motor
//int EN_B = 10;      //Enable pin for second motor

//Encoder pins
int VCC = 2;
int outA = 4;
int outB = 5;

//Initializing variables to store data
int motor_speedL;
int motor_speedR;
// Set speeds accordingly 0-min, 255-max
int maxspeed = 0; //Max is 255
int stopspeed = 0;





void setup ( ) {
  Serial.begin (9600); //Starting the serial communication at 9600 baud rate
  //Initializing the motor pins as output

  //Left Motor
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);  
  pinMode(IN2, OUTPUT);
  //Right Motor
  /*pinMode(EN_B, OUTPUT);
  pinMode(IN3, OUTPUT);  
  pinMode(IN4, OUTPUT);
  */
  //Left Encoder 
  pinMode(VCC, OUTPUT);
  pinMode(outA, INPUT);
  pinMode(outB, INPUT);
  
 
  //Setting Direction of motors according to L298N motor controller
  // Left Motor 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Right motor 
  //digitalWrite(IN3, HIGH);
  //digitalWrite(IN4, LOW);

  //Enable encoder 
  digitalWrite(VCC, HIGH);
  
  
           
}

void loop() 
{       
    motor_speedL = maxspeed;
    //motor_speedR = maxspeed;
    analogWrite(EN_A, motor_speedL);
    encA = digitalRead(outA);
    encB = digitalRead(outB);
    Serial.print(encA);
    Serial.print(' ');
    Serial.println(encB);
    //analogWrite(EN_B, motor_speedR);
    delay(10);
    
    //motor_speedL = stopspeed;
    //motor_speedR = stopspeed;
    //analogWrite(EN_A, motor_speedL);
    //analogWrite(EN_B, motor_speedR);
    //delay(100000);
    }
