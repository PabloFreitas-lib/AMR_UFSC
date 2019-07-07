/*
#  Chefbot_ROS_Interface.ino
#
#  Copyright 2015 Lentin Joseph <qboticslabs@gmail.com>
#  Website : www.qboticslabs.com , www.lentinjoseph.com
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#
#  Some of the portion is adapted from I2C lib example code for MPU 6050
*/


//MPU 6050 Interfacing libraries

//Library to communicate with I2C devices
#include "Wire.h"
//I2C communication library for MPU6050
#include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"
//Processing incoming serial data
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>

//Creating MPU6050 Object
MPU6050 mpu;
//Messenger object
Messenger Messenger_Handler = Messenger();


#define OUTPUT_READABLE_QUATERNION

///////////////////////////////////////////////////////////////////////////////////////
//DMP options
//Set true if DMP init was successful
bool dmpReady = false;
//Holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;
//return status after each device operation
uint8_t devStatus;
//Expected DMP paclet size
uint16_t packetSize;
//count of all bytes currently in FIFO
uint16_t fifoCount;
//FIFO storate buffer
uint8_t fifoBuffer[64];


#define OUTPUT_READABLE_QUATERNION
////////////////////////////////////////////////////////////////////////////////////////////////

//orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

float euler[3];
float ypr[3];




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
   mpuInterrupt = true;
}




////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////
//Motor Pin definition
//Left Motor pins

#define A_1 PA_3


//PWM 1 pin number
#define PWM_1 PC_5


//Right Motor

#define B_1 PA_4

//PWM 2 pin number
#define PWM_2 PC_6

//HIGH ativa a ponte H
//LOW desativa
//#define STOP_MOTOR_R PB_0
//#define STOP_MOTOR_L PB_1

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Ultrasonic pins definition
const int echo = PE_4, Trig = PE_5;
long duration, cm;

//Reset pin for resetting Tiva C, if this PIN set high, Tiva C will reset

#define RESET_PIN PB_2

/////////////////////////////////////////////////////////////////////////////////////////
//Time  update variables

unsigned long LastUpdateMicrosecs = 0;
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

///////////////////////////////////////////////////////////////////////////////////////
//Motor speed from PC
//Motor left and right speed
float motor_left_speed = 0;
float motor_right_speed = 0;
/////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////
//Encoder pins definition

// Left encoder 5V( preto GND - Vermelho VCC)

#define Left_Encoder_PinA PD_7 // branco
#define Left_Encoder_PinB PF_4 // verde

volatile long Left_Encoder_Ticks = 0;

// Right encoder 5V( preto GND - Vermelho VCC)

#define Right_Encoder_PinA PC_7 // branco
#define Right_Encoder_PinB PD_6 // verde

volatile long Right_Encoder_Ticks = 0;


//SetupEncoders() Definition

void SetupEncoders()
{
  pinMode(Left_Encoder_PinA,INPUT_PULLUP);
  pinMode(Left_Encoder_PinB,INPUT_PULLUP);

  pinMode(Right_Encoder_PinA,INPUT_PULLUP);
  pinMode(Right_Encoder_PinB,INPUT_PULLUP);

  attachInterrupt(Left_Encoder_PinA, do_Left_Encoder, RISING);
  attachInterrupt(Right_Encoder_PinA, do_Right_Encoder, RISING);
}
int a=0,b=0;

void setup()
{

 //Init Serial port with 115200 baud rate
 Serial.begin(115200);

 //Setup Encoders
 SetupEncoders();
 //Setup Motors
 SetupMotors();
 //Setup Ultrasonic
 //SetupUltrasonic();
 //Setup MPU 6050
 //Setup_MPU6050();
 //Setup Reset pins
 SetupReset();
 //Set up Messenger
 Messenger_Handler.attach(OnMssageCompleted);


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup UltrasonicsSensor() function
void SetupUltrasonic()
{
pinMode(Trig, OUTPUT);
pinMode(echo, INPUT);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup Motors() function

void SetupMotors()
{

//Left motor
pinMode(A_1,OUTPUT);
pinMode(PWM_1,OUTPUT);

//Right Motor
pinMode(B_1,OUTPUT);
pinMode(PWM_2,OUTPUT);

//PIN RESET MOTOR
//pinMode(STOP_MOTOR_R,OUTPUT);
//pinMode(STOP_MOTOR_L,OUTPUT);

}


//Setup Reset() function

void SetupReset()

{


 pinMode(GREEN_LED,OUTPUT);
 pinMode(RESET_PIN,OUTPUT);


 ///Conenect RESET Pins to the RESET pin of launchpad,its the 16th PIN
 digitalWrite(RESET_PIN,HIGH);


}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//do_Left_Encoder() Definitions
void do_Left_Encoder()
{
 (digitalRead(Left_Encoder_PinB)==LOW) ? Left_Encoder_Ticks++ : Left_Encoder_Ticks--;

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//do_Right_Encoder() Definitions

void do_Right_Encoder()
{
 (digitalRead(Right_Encoder_PinB)==LOW) ? Right_Encoder_Ticks++ : Right_Encoder_Ticks--;
}

//Will update both encoder value through serial port
void Update_Encoders()
{

 Serial.print("e");
 Serial.print("\t");
 Serial.print(Left_Encoder_Ticks);
 Serial.print("\t");
 Serial.print(Right_Encoder_Ticks);
 Serial.print("\n");



}
void loop()
{

   //Read from Serial port
   Read_From_Serial();
   //Send time information through serial port
   Update_Time();
   //Send encoders values through serial port
   Update_Encoders();
   //Update motor values with corresponding speed and send speed values through serial port
   Update_Motors();
   //Send ultrasonic values through serial port
   //Update_Ultra_Sonic();

}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function

void Read_From_Serial()

{
  while(Serial.available() > 0)
   {

      int data = Serial.read();
      Messenger_Handler.process(data);


   }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition

void OnMssageCompleted()
{

 char reset[] = "r";
 char set_speed[] = "s";

 if(Messenger_Handler.checkString(reset))
 {

    Serial.println("Reset Done");
    Reset();

 }
 if(Messenger_Handler.checkString(set_speed))
 {

    //This will set the speed
    Set_Speed();
    return;


 }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Set speed
void Set_Speed()
{

 motor_left_speed = Messenger_Handler.readLong();
 motor_right_speed = Messenger_Handler.readLong();



}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Reset function
void Reset()
{

 digitalWrite(GREEN_LED,HIGH);
 delay(1000);
 digitalWrite(RESET_PIN,HIGH);
 digitalWrite(GREEN_LED,LOW);


}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update both motors
void Update_Motors()
{

 moveRightMotor(motor_right_speed);
 moveLeftMotor(motor_left_speed);

 Serial.print("s");
 Serial.print("\t");
 Serial.print(motor_left_speed);
 Serial.print("\t");
 Serial.print(motor_right_speed);
 Serial.print("\n");


}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update ultrasonic sensors through serial port

void Update_Ultra_Sonic()
{
 digitalWrite(Trig, LOW);
 delayMicroseconds(2);
 digitalWrite(Trig, HIGH);
 delayMicroseconds(10);
 digitalWrite(Trig, LOW);
 // The echo pin is used to read the signal from the PING))): a HIGH
 // pulse whose duration is the time (in microseconds) from the sending
 // of the ping to the reception of its echo off of an object.
 duration = pulseIn(echo, HIGH);
 // convert the time into a distance
 cm = microsecondsToCentimeters(duration);

 //Sending through serial port
 Serial.print("u");
 Serial.print("\t");
 Serial.print(cm);
 Serial.print("\n");

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update time function
void Update_Time()
{


 CurrentMicrosecs = micros();
 LastUpdateMillisecs = millis();
 MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
 if (MicrosecsSinceLastUpdate < 0)
   {
 MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;

   }
 LastUpdateMicrosecs = CurrentMicrosecs;
 SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;

 Serial.print("t");
 Serial.print("\t");
 Serial.print(LastUpdateMicrosecs);
 Serial.print("\t");
 Serial.print(SecondsSinceLastUpdate);
 Serial.print("\n");


}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Conversion of microsecond to centimeter
long microsecondsToCentimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29 / 2;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Motor running function


void moveRightMotor(float rightServoValue)
{
 if (rightServoValue>0)
 {
   //digitalWrite(STOP_MOTOR_R,HIGH);
   digitalWrite(A_1,HIGH);
   analogWrite(PWM_1,rightServoValue);
 }
 else if(rightServoValue<0)
 {
  //digitalWrite(STOP_MOTOR_R,HIGH);
  digitalWrite(A_1,LOW);

  analogWrite(PWM_1,abs(rightServoValue));

 }

 else if(rightServoValue == 0)
 {
   //reset PIN , mas esse desliga o TIVA
   //digitalWrite(STOP_MOTOR_R,LOW);
   analogWrite(PWM_1, 0);
   analogWrite(PWM_2, 0);
 }


}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveLeftMotor(float leftServoValue)
{
  if (leftServoValue > 0)
 {
   //digitalWrite(STOP_MOTOR_L,HIGH);
   digitalWrite(B_1,HIGH);
   analogWrite(PWM_2,leftServoValue);
 }
 else if(leftServoValue < 0)
 {
   //digitalWrite(STOP_MOTOR_L,HIGH);
   digitalWrite(B_1,LOW);
   analogWrite(PWM_2,abs(leftServoValue));
 }
 else if(leftServoValue == 0)
 {
  //reset PIN , mas esse desliga o TIVA
    //digitalWrite(STOP_MOTOR_L,LOW);
    analogWrite(PWM_1, 0);
    analogWrite(PWM_2, 0);
  }


}
