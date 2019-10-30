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
// Processing incoming serial data
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>

//Creating MPU6050 Object
// MPU6050 mpu;

//Messenger object
Messenger Messenger_Handler = Messenger();


#define OUTPUT_READABLE_QUATERNION

///////////////////////////////////////////////////////////////////////////////////////
// Codigo do Prof. Dourado


const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

Quaternion q;


///////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////
//DMP options
//Set true if DMP init was successful
// bool dmpReady = false;
// //Holds actual interrupt status byte from MPU
// uint8_t mpuIntStatus;
// //return status after each device operation
// uint8_t devStatus;
// //Expected DMP paclet size
// uint16_t packetSize;
// //count of all bytes currently in FIFO
// uint16_t fifoCount;
// //FIFO storate buffer
// uint8_t fifoBuffer[64];
//
//
// #define OUTPUT_READABLE_QUATERNION
// //////////////////////////////////////////////////////////////////////////////////////////////
//
// // orientation/motion vars
// Quaternion q;
// VectorInt16 aa;
// VectorInt16 aaReal;
// VectorInt16 aaWorld;
// VectorFloat gravity;
//
// float euler[3];
// float ypr[3];




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
//
// volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
// void dmpDataReady() {
//         mpuInterrupt = true;
// }




////////////////////////////////////////////////////////////////////////////////////////

unsigned const int limite = 155;

//Codigo novo adaptado para novo esquematico

//definicoes

// Motor DIREITA - LevelUP da ESQUERDA

int PWM_D1 = PC_5; //PWM1 - PH4
int PWM_D2 = PC_6; //PWM2 - PH3

int Direita_d = PA_3; // Sinal de 5v para as duas entradas encoder - PH1



// Motor ESQUERDA - LevelUP do Meio

int PWM_E1 = PB_0; //PWM1 - PH1
int PWM_E2 = PB_1; //PWM2 - PH2

int Esquerda_d = PE_4; //sinal de 5v para as duas entradas encoder - PH3




//Encoder DIREITA - levelUP DIREITA
/*
   int PF4 = PF_4; // - PH1
   int PD7 = PD_7; // - PH2
   //Encoder ESQUERDA
   int PD6 = PD_6; // - PH4
   int PC7 = PC_7; // - PH3*/

///////////////////////////////////////////////////////////////
//Encoder pins definition

// Left encoder 5V( preto GND - Vermelho VCC)

#define Left_Encoder_PinA PD_6 // branco
#define Left_Encoder_PinB PC_7 // verde

volatile long Left_Encoder_Ticks = 0;

// Right encoder 5V( preto GND - Vermelho VCC)

#define Right_Encoder_PinA PF_4 // branco
#define Right_Encoder_PinB PD_7 // verde

volatile long Right_Encoder_Ticks = 0;

// fim do codigo novo


/*
   /////////////////////////////////////////////////////////////////
   //Motor Pin definition
   //Left Motor pins
 #define A_1 PA_3
   //PWM 1 pin number
 #define PWM_1 PC_4 // PC_5 nao funcionou, evitar essa porta
   //Right Motor
 #define B_1 PA_5
   //PWM 2 pin number
 #define PWM_2 PC_6
 */
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
/*
   ///////////////////////////////////////////////////////////////
   //Encoder pins definition
   // Left encoder 5V( preto GND - Vermelho VCC)
 #define Left_Encoder_PinA PD_6 // branco Antiga PD_7
 #define Left_Encoder_PinB PC_7 // verde  Antiga PD_6
   volatile long Left_Encoder_Ticks = 0;
   // Right encoder 5V( preto GND - Vermelho VCC)
 #define Right_Encoder_PinA PF_4 // branco Antiga PE_2
 #define Right_Encoder_PinB PD_7 // verde  Antiga PE_3
   volatile long Right_Encoder_Ticks = 0;
 */
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
        Setup_MPU6050();

        //Setup Reset pins
        SetupReset();

        //Set up Messenger
        Messenger_Handler.attach(OnMssageCompleted);


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup UltrasonicsSensor() function
// void SetupUltrasonic()
// {
//         pinMode(Trig, OUTPUT);
//         pinMode(echo, INPUT);
// }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup Motors() function

void SetupMotors()
{

        //Left motor
        pinMode(Esquerda_d,OUTPUT);
        pinMode(PWM_E1,OUTPUT);
        pinMode(PWM_E2,OUTPUT);


        //Right Motor
        pinMode(Direita_d,OUTPUT);
        pinMode(PWM_D1,OUTPUT);
        pinMode(PWM_D2,OUTPUT);


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


void Setup_MPU6050()
{

        //
        // Wire.begin();
        // // initialize device
        // Serial.println("Initializing I2C devices...");
        // mpu.initialize();
        //
        // // verify connection
        // Serial.println("Testing device connections...");
        // Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
        //
        // //Initialize DMP in MPU 6050
        // Setup_MPU6050_DMP();
        //Serial.begin(9600);
        Wire.begin();                       // Initialize comunication
        Wire.beginTransmission(MPU);        // Start communication with MPU6050 // MPU=0x68
        Wire.write(0x6B);                   // Talk to the register 6B
        Wire.write(0x00);                   // Make reset - place a 0 into the 6B register
        Wire.endTransmission(true);         //end the transmission

        calculate_IMU_error();
        delay(20);

}

// Código do Wikipedia

void ToQuaternion(float yaw, float pitch, float roll) // yaw (Z), pitch (Y), roll (X)
{
        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        q.w = cy * cp * cr + sy * sp * sr;
        q.x = cy * cp * sr - sy * sp * cr;
        q.y = sy * cp * sr + cy * sp * cr;
        q.z = sy * cp * cr - cy * sp * sr;

}

//Setup MPU 6050 DMP
// void Setup_MPU6050_DMP()
// {
//
// //DMP Initialization
//
// devStatus = mpu.dmpInitialize();
//
// mpu.setXGyroOffset(220);
// mpu.setXGyroOffset(76);
// mpu.setXGyroOffset(-85);
// mpu.setXGyroOffset(1788);
//
//
// if(devStatus == 0)
// {
//
//         mpu.setDMPEnabled(true);
//
//         pinMode(PUSH2,INPUT_PULLUP);
//         attachInterrupt(PUSH2, dmpDataReady, RISING);
//
//         mpuIntStatus = mpu.getIntStatus();
//
//         dmpReady = true;
//
//         packetSize = mpu.dmpGetFIFOPacketSize();
//
// }
// /*else{
//    ;
//    }*/


// }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//do_Left_Encoder() Definitions
void do_Left_Encoder()
{
        (digitalRead(Left_Encoder_PinB)==LOW) ? Left_Encoder_Ticks-- : Left_Encoder_Ticks++;

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

        //Send MPU 6050 values through serial port
        Update_MPU6050();

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
        //Zera os encoders
        Left_Encoder_Ticks = 0;
        Right_Encoder_Ticks = 0;

        //Para o Motor
        motor_left_speed = 0;
        motor_right_speed = 0;

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

// void Update_Ultra_Sonic()
// {
//         digitalWrite(Trig, LOW);
//         delayMicroseconds(2);
//         digitalWrite(Trig, HIGH);
//         delayMicroseconds(10);
//         digitalWrite(Trig, LOW);
//         // The echo pin is used to read the signal from the PING))): a HIGH
//         // pulse whose duration is the time (in microseconds) from the sending
//         // of the ping to the reception of its echo off of an object.
//         duration = pulseIn(echo, HIGH);
//         // convert the time into a distance
//         cm = microsecondsToCentimeters(duration);
//
//         //Sending through serial port
//         Serial.print("u");
//         Serial.print("\t");
//         Serial.print(cm);
//         Serial.print("\n");
//
// }

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
                digitalWrite(Direita_d,HIGH);
                if(rightServoValue > limite)
                {
                        rightServoValue = limite;
                }
                analogWrite(PWM_D1,rightServoValue);
                analogWrite(PWM_D2,0);
        }
        else if(rightServoValue<0)
        {
                digitalWrite(Direita_d,HIGH);
                if(abs(rightServoValue) > limite)
                {
                        rightServoValue = limite;
                }
                analogWrite(PWM_D2,abs(rightServoValue));
                analogWrite(PWM_D1,0);
        }

        else if(rightServoValue == 0)
        {
                //reset PIN , mas esse desliga o TIVA
                digitalWrite(Direita_d,LOW);
                analogWrite(PWM_D1, 0);
                analogWrite(PWM_D2, 0);
        }


}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveLeftMotor(float leftServoValue)
{
        if (leftServoValue > 0)
        {
                digitalWrite(Esquerda_d,HIGH);
                if(leftServoValue > limite)
                {
                        leftServoValue = limite;
                }
                analogWrite(PWM_E2,leftServoValue);
                analogWrite(PWM_E1,0);

        }
        else if(leftServoValue < 0)
        {
                digitalWrite(Esquerda_d,HIGH);
                if(abs(leftServoValue) > limite)
                {
                        leftServoValue = limite;
                }
                analogWrite(PWM_E1,abs(leftServoValue));
                analogWrite(PWM_E2,0);

        }
        else if(leftServoValue == 0)
        {
                //reset PIN , mas esse desliga o TIVA
                digitalWrite(Esquerda_d,LOW);
                analogWrite(PWM_E1,0);
                analogWrite(PWM_E2,0);
        }


}

void Update_MPU6050()
{

        // int16_t ax, ay, az;
        // int16_t gx, gy, gz;
        //
        // ///Update values from DMP for getting rotation vector
        // Update_MPU6050_DMP();

        // Codigo do Dourado

        Wire.beginTransmission(MPU);
        Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
        //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
        AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
        AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
        AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
        // Calculating Roll and Pitch from the accelerometer data
        accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
        accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)

//  // === Read gyroscope data === //
        previousTime = currentTime; // Previous time is stored before the actual time read
        currentTime = millis();     // Current time actual time read
        elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
        Wire.beginTransmission(MPU);
        Wire.write(0x43); // Gyro data first register address 0x43
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
        GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
        GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
        GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
        // Correct the outputs with the calculated error values
        GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
        GyroY = GyroY - 2; // GyroErrorY ~(2)
        GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)

        // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
        gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
        gyroAngleY = gyroAngleY + GyroY * elapsedTime;
        yaw =  yaw + GyroZ * elapsedTime;

        // Complementary filter - combine acceleromter and gyro angle values
        roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
        pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;


        ToQuaternion(yaw, pitch, roll);


        // Print the values on the serial monitor
        //Serial.print("quat\t");
        //Serial.print(q.w);
        //Serial.print("\t");
        //Serial.print(q.x);
        //Serial.print("\t");
        //Serial.print(q.y);
        //Serial.print("\t");
        //Serial.println(q.z);


        //Serial.print(" Gx: ");
        //Serial.print(AccX);
        //Serial.print(" / ");
        // Serial.print(" Gy: ");
        //Serial.print(AccY);
        //Serial.print(" / ");
        // Serial.print(" Gz: ");
        //Serial.println(AccZ);
        //Serial.println("\n");
        //delay(50);

        Serial.print("i"); Serial.print("\t");
         Serial.print(q.x); Serial.print("\t");
         Serial.print(q.y); Serial.print("\t");
         Serial.print(q.z); Serial.print("\t");
         Serial.print(q.w);
         Serial.print("\n");



}


void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update MPU6050 DMP functions

// void Update_MPU6050_DMP()
// {
//
// //DMP Processing
//
// if (!dmpReady) return;
//
// while (!mpuInterrupt && fifoCount < packetSize) {}
//
// mpuInterrupt = false;
// mpuIntStatus = mpu.getIntStatus();
//
// //get current FIFO count
// fifoCount = mpu.getFIFOCount();
//
//
// if ((mpuIntStatus & 0x10) || fifoCount > 512)
// {
//         // reset so we can continue cleanly
//         mpu.resetFIFO();
// }
//
//
//
//
// else if (mpuIntStatus & 0x02) {
//         // wait for correct available data length, should be a VERY short wait
//         while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//
//         // read a packet from FIFO
//         mpu.getFIFOBytes(fifoBuffer, packetSize);
//
//         // track FIFO count here in case there is > 1 packet available
//         // (this lets us immediately read more without waiting for an interrupt)
//         fifoCount -= packetSize;
//
// #ifdef OUTPUT_READABLE_QUATERNION
//         // display quaternion values in easy matrix form: w x y z
//         mpu.dmpGetQuaternion(&q, fifoBuffer);
//
//
//         Serial.print("i"); Serial.print("\t");
//         Serial.print(q.x); Serial.print("\t");
//         Serial.print(q.y); Serial.print("\t");
//         Serial.print(q.z); Serial.print("\t");
//         Serial.print(q.w);
//         Serial.print("\n");
//
//
// #endif
//
// #ifdef OUTPUT_READABLE_EULER
//         // display Euler angles in degrees
//         mpu.dmpGetQuaternion(&q, fifoBuffer);
//         mpu.dmpGetEuler(euler, &q);
//         Serial.print("euler\t");
//         Serial.print(euler[0] * 180/M_PI);
//         Serial.print("\t");
//         Serial.print(euler[1] * 180/M_PI);
//         Serial.print("\t");
//         Serial.println(euler[2] * 180/M_PI);
// #endif
//
// #ifdef OUTPUT_READABLE_YAWPITCHROLL
//         // display Euler angles in degrees
//         mpu.dmpGetQuaternion(&q, fifoBuffer);
//         mpu.dmpGetGravity(&gravity, &q);
//         mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//         Serial.print("ypr\t");
//         Serial.print(ypr[0] * 180/M_PI);
//         Serial.print("\t");
//         Serial.print(ypr[1] * 180/M_PI);
//         Serial.print("\t");
//         Serial.println(ypr[2] * 180/M_PI);
// #endif
//
// #ifdef OUTPUT_READABLE_REALACCEL
//         // display real acceleration, adjusted to remove gravity
//         mpu.dmpGetQuaternion(&q, fifoBuffer);
//         mpu.dmpGetAccel(&aa, fifoBuffer);
//         mpu.dmpGetGravity(&gravity, &q);
//         mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//         Serial.print("areal\t");
//         Serial.print(aaReal.x);
//         Serial.print("\t");
//         Serial.print(aaReal.y);
//         Serial.print("\t");
//         Serial.println(aaReal.z);
// #endif
//
// #ifdef OUTPUT_READABLE_WORLDACCEL
//         // display initial world-frame acceleration, adjusted to remove gravity
//         // and rotated based on known orientation from quaternion
//         mpu.dmpGetQuaternion(&q, fifoBuffer);
//         mpu.dmpGetAccel(&aa, fifoBuffer);
//         mpu.dmpGetGravity(&gravity, &q);
//         mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//         mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//         Serial.print("aworld\t");
//         Serial.print(aaWorld.x);
//         Serial.print("\t");
//         Serial.print(aaWorld.y);
//         Serial.print("\t");
//         Serial.println(aaWorld.z);
// #endif
//
// #ifdef OUTPUT_TEAPOT
//         // display quaternion values in InvenSense Teapot demo format:
//         teapotPacket[2] = fifoBuffer[0];
//         teapotPacket[3] = fifoBuffer[1];
//         teapotPacket[4] = fifoBuffer[4];
//         teapotPacket[5] = fifoBuffer[5];
//         teapotPacket[6] = fifoBuffer[8];
//         teapotPacket[7] = fifoBuffer[9];
//         teapotPacket[8] = fifoBuffer[12];
//         teapotPacket[9] = fifoBuffer[13];
//         Serial.write(teapotPacket, 14);
//         teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
// #endif
//
//
// }


// }
