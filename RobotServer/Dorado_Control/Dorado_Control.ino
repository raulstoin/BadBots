#include "MPU6050API.h"

#define lDir 0 //pinul 0 este pinul de direcţie pentru roata stângă
#define lSpeed 3 //pinul 3 este pinul cu care putem controla viteza motorului stâng
#define rSpeed 4 //pinul 4 este pinul cu care putem controla viteza motorului drept 
#define rDir 1 //pinul 1 este pinul de direcţie pentru roata dreaptă

#define maxPWM 255 //valoarea maximă a PWM-ului
#define minPWM 50 //valoarea mminimă a PWM-ului
#define motorForward LOW //valoarea logică a pinului de direcţie ca motorul să se rotească înainte
#define motorBackward HIGH //valoarea logică a pinului de direcţie ca motorul să se rotească înapoi

#define START_DELAY 5 // in seconds

int precision = 2;
int incomingData = 0; //variabila de tip integer în care se memoriază caracterul recepţionat pe Bluetooth
byte incomingAngle = 0;
int motorSpeed = maxPWM; //variabila de tip integer în care se memoriază viteza actuală a motoarelor, starea iniţială este 255.
int debug = 0; //variabila de tip integer în care activeayă funcţiile de debug
int turningSpeed= 200; //variabila de tip integer în care se memoriază viteya motoarelor pentru rotiriile spre dreapta si stânga
int serialEventDone = 0; //variabila de tip integer în care se memoriază dacă s-a recepţionat cevpe serial
int maxSpeed = minPWM; // ???
int desiredAngle = 0;
int turnFlag = 0;
int printMe;
void setup()
{
  pinMode(lSpeed, OUTPUT); //definirea pinului lSpeed ca iesire
  pinMode(rSpeed, OUTPUT); //definirea pinului rSpeed ca iesire
  pinMode(lDir, OUTPUT); //definirea pinului lDir ca iesire
  analogWrite(lSpeed, 255); //setarea PWM-ului la valoarea de 255 pentru lSpeed
  pinMode(rDir, OUTPUT); //definirea pinului rDir ca iesire
  analogWrite(rSpeed, 255); //setarea PWM-ului la valoarea de 255 pentru rSpeed
  Serial.begin(115200); //pornirea comunicării pe serial
  if(debug)
  {
    Serial.println("Serial Running.");
  };
  setupMPU6050();
  delay(1000 * START_DELAY);
}

void serialEvent()
{
  if(Serial.available()>0)
  {
    if(turnFlag == 0) {
      incomingData = Serial.read(); //citirea datelor recepţionate pe serial
      if(incomingData == 'l' || incomingData == 'r')
        turnFlag = 1;
      else
        serialEventDone = 1;
    }
    else {
      incomingAngle = Serial.read();
      turnFlag = 0;
      serialEventDone = 1;      
    }
  }
}

void loop()
{
  if(serialEventDone)
  {
    switch (incomingData)
    {
    case 'f':
      motorDrive(1);  //start moving forward
      if(debug)
      {
        Serial.println("received f");
      };
      break;

    case 'b':
      motorDrive(2);  //start moving backward
      if(debug)
      {
        Serial.println("received b") ;
      };
      break;

    case 'l':
      motorDrive(3);  //start turing left
      if(debug)
      {
        Serial.print("received l with angle ") ;
        Serial.println(incomingAngle);
      };
      break;

    case 'r':
      motorDrive(4);  //start turning right
      if(debug)
      {
        Serial.print("received r with angle ") ;
        Serial.println(incomingAngle);
      };
      break;

    case 's':
      motorDrive(0);  //stop the motors
      if(debug)
      {
        Serial.println("received s")  ;
      };
      break;
    default:
      break;
    }
    serialEventDone = 0;
  }
  loopMPU6050();
  
}
void motorDrive(int mode)
{
  switch (mode)
  {
  case 0:  //stop
    brake();

    if(debug)
    {
      Serial.println(motorSpeed)  ;
    };
    break;

  case 1:  //forward
    digitalWrite(lDir, motorForward);
    digitalWrite(rDir, motorForward);
    if(accelerate() == 1)
    {
      // analogWrite(lSpeed, minPWM);
      // analogWrite(rSpeed, minPWM);
    }
    if(debug)
    {
      Serial.println(motorSpeed)  ;
    };
    break;

  case 2:  //backward
    digitalWrite(lDir, motorBackward);
    digitalWrite(rDir, motorBackward);

    if(accelerate() == 1)
    {
      // analogWrite(lSpeed, minPWM);
      // analogWrite(rSpeed, minPWM);
    }
    if(debug)
    {
      Serial.println(motorSpeed)  ;
    };
    break;

  case 3:  //left
    digitalWrite(lDir, motorBackward);
    digitalWrite(rDir, motorForward);

    motorSpeed = turningSpeed;
    analogWrite(lSpeed, motorSpeed);
    analogWrite(rSpeed, motorSpeed);
    // wait until we've rotated to the desired angle
    desiredAngle = currentAngleMPU6050() - incomingAngle;
    if(desiredAngle < -180)
      desiredAngle = 180 - (desiredAngle + 180);
    if(desiredAngle > 180)
      desiredAngle = -180 + (desiredAngle - 180); 
    while( !(currentAngleMPU6050() > (desiredAngle - precision) 
      && currentAngleMPU6050() < (desiredAngle + precision))) {
      loopMPU6050();
      if(debug) {
        Serial.print("Desired angle: ");
        Serial.print(desiredAngle);
        Serial.print(" current angle: ");
        Serial.println(currentAngleMPU6050());
      }
    }
    brake();

    break;

  case 4:  //right
    digitalWrite(lDir, motorForward);
    digitalWrite(rDir, motorBackward);
    motorSpeed = turningSpeed;
    analogWrite(lSpeed, motorSpeed);
    analogWrite(rSpeed, motorSpeed);

    // wait until we've rotated to the desired angle    
    desiredAngle = currentAngleMPU6050() + incomingAngle;
    if(desiredAngle < -180)
      desiredAngle = 180 - (desiredAngle + 180);
    if(desiredAngle > 180)
      desiredAngle = -180 + (desiredAngle - 180); 
    while( !(currentAngleMPU6050() > (desiredAngle - precision) 
      && currentAngleMPU6050() < (desiredAngle + precision))) {
      loopMPU6050();
      if(debug) {
        Serial.print("Desired angle: ");
        Serial.print(desiredAngle);
        Serial.print(" current angle: ");
        Serial.println(currentAngleMPU6050());
      }
    }
    brake();

    break;
  default:
    break;
  }
  Serial.write(0); // acknowledge for client that command has been executed
}
int brake()
{
  // for(motorSpeed; motorSpeed<maxPWM; motorSpeed++) 
  // {
    analogWrite(lSpeed, maxPWM);
    analogWrite(rSpeed, maxPWM);
    // delayMicroseconds(500);
  // }
  digitalWrite(lSpeed, HIGH);
  digitalWrite(rSpeed, HIGH);

  if(debug)
  {
    Serial.println("brake 1")  ;
  };
  return 1;
}
int accelerate() 
{
  // for(motorSpeed = maxPWM; motorSpeed>minPWM; motorSpeed--)
  // {
    analogWrite(lSpeed, maxSpeed);
    analogWrite(rSpeed, maxSpeed);
    // delay(2);
  // }

  if(debug)
  {
    Serial.println("accelerate 1")  ;
  };
  return 1;
}
