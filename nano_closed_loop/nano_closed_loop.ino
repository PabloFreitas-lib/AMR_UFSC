/*
 *
 * Arduino Nano Pins:

H-Bridge:
Left:

D5 Analog Write ( Forward) 5
D6 Analog Write  ( Backwards) 6
A0 Digital Write ( falta, desnecessauro)

Right:

D9 Analog Write ( Forward) 9
D10 Analog Write  ( Backwards) 10
A6 Digital Write ( falta, desnecessauro)


Encoder:
Left:
A1 Digital Read  15
A2 Digital Read  16
Right:
D7 Digital Read  7
D8 Digital Read  8


MPU:

A4 - SDA  18
A5 - SCL  19
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


////////////////////////////////////////////////////////////////////////////////////////

unsigned const int limite = 155;

//Codigo novo adaptado para novo esquematico

//definicoes

// Motor ESQUERDA - LevelUP do Meio

int PWM_FRENTE_E = 5; // Para frente
int PWM_TRAS_E = 6; // Para tras

// Motor DIREITA - LevelUP da ESQUERDA

int PWM_FRENTE_D = 9;   // Para frente
int PWM_TRAS_D = 10; // Para tras


///////////////////////////////////////////////////////////////
//Encoder pins definition

// Left encoder 5V( preto GND - Vermelho VCC)

#define Left_Encoder_PinA 2// branco
#define Left_Encoder_PinB 16 // verde

volatile long Left_Encoder_Ticks = 0;

// Right encoder 5V( preto GND - Vermelho VCC)

#define Right_Encoder_PinA 3 // branco
#define Right_Encoder_PinB 8 // verde

volatile long Right_Encoder_Ticks = 0;

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


//SetupEncoders() Definition

void SetupEncoders()
{
        pinMode(Left_Encoder_PinA,INPUT_PULLUP);
        pinMode(Left_Encoder_PinB,INPUT_PULLUP);

        pinMode(Right_Encoder_PinA,INPUT_PULLUP);
        pinMode(Right_Encoder_PinB,INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(Left_Encoder_PinA), do_Left_Encoder, RISING);
        attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, RISING);
}
int a=0,b=0;

void setup()
{

        //Init Serial port with 9600 baud rate
        Serial.begin(115200);

        //Setup Encoders
        SetupEncoders();

        //Setup Motors
        SetupMotors();

        //Setup Ultrasonic
        //SetupUltrasonic();

        //Setup MPU 6050
//        Setup_MPU6050();

        //Setup Reset pins
        //SetupReset();

        //Set up Messenger
        Messenger_Handler.attach(OnMssageCompleted);


}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup Motors() function

void SetupMotors()
{

        //Left motor
        pinMode(PWM_FRENTE_E,OUTPUT);
        pinMode(PWM_TRAS_E,OUTPUT);


        //Right Motor
        pinMode(PWM_FRENTE_D,OUTPUT);
        pinMode(PWM_TRAS_D,OUTPUT);

}



void Setup_MPU6050()
{
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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//do_Left_Encoder() Definitions
void do_Left_Encoder()
{
        (digitalRead(Left_Encoder_PinB)==HIGH) ? Left_Encoder_Ticks-- : Left_Encoder_Ticks++;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//do_Right_Encoder() Definitions

void do_Right_Encoder()
{
       (digitalRead(Right_Encoder_PinB)==HIGH) ? Right_Encoder_Ticks-- : Right_Encoder_Ticks++;
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
  //      Update_MPU6050();

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
        //Parar o Motor
        motor_left_speed = 0;
        motor_right_speed = 0;
        //Zera os encoders
        //Left_Encoder_Ticks = 0;
        //Right_Encoder_Ticks = 0;

        delay(1000);        
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
                //digitalWrite(Direita_d,HIGH);
                if(rightServoValue > limite)
                {
                        rightServoValue = limite;
                }
                analogWrite(PWM_FRENTE_D,rightServoValue);
                analogWrite(PWM_TRAS_D,0);
        }
        else if(rightServoValue<0)
        {
                //digitalWrite(Direita_d,HIGH);
                if(abs(rightServoValue) > limite)
                {
                        rightServoValue = limite;
                }
                analogWrite(PWM_TRAS_D,abs(rightServoValue));
                analogWrite(PWM_FRENTE_D,0);
        }

        else if(rightServoValue == 0)
        {
                //reset PIN , mas esse desliga o TIVA
                //digitalWrite(Direita_d,LOW);
                analogWrite(PWM_FRENTE_D, 0);
                analogWrite(PWM_TRAS_D, 0);
        }


}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveLeftMotor(float leftServoValue)
{
        if (leftServoValue > 0)
        {
                //digitalWrite(Esquerda_d,HIGH);
                if(leftServoValue > limite)
                {
                        leftServoValue = limite;
                }
                analogWrite(PWM_TRAS_E,leftServoValue);
                analogWrite(PWM_FRENTE_E,0);

        }
        else if(leftServoValue < 0)
        {
                //digitalWrite(Esquerda_d,HIGH);
                if(abs(leftServoValue) > limite)
                {
                        leftServoValue = limite;
                }
                analogWrite(PWM_FRENTE_E,abs(leftServoValue));
                analogWrite(PWM_TRAS_E,0);

        }
        else if(leftServoValue == 0)
        {
                //reset PIN , mas esse desliga o TIVA
                //digitalWrite(Esquerda_d,LOW);
                analogWrite(PWM_FRENTE_E,0);
                analogWrite(PWM_TRAS_E,0);
        }


}

void Update_MPU6050()
{

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
