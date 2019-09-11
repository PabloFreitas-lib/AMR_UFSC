// Motor DIREITA - LevelUP da ESQUERDA

int PC5 = PC_5; //PWM1 - PH4
int PC6 = PC_6; //PWM2 - PH3

int PA3 = PA_3; // Sinal de 5v para as duas entradas encoder - PH1



// Motor ESQUERDA - LevelUP do Meio

int PB0 = PB_0; //PWM1 - PH1
int PB1 = PB_1; //PWM2 - PH2

int PE4 = PE_4; //sinal de 5v para as duas entradas encoder - PH3




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



void SetupEncoders()
{
   pinMode(Left_Encoder_PinA,INPUT_PULLUP);
   pinMode(Left_Encoder_PinB,INPUT_PULLUP);

   pinMode(Right_Encoder_PinA,INPUT_PULLUP);
   pinMode(Right_Encoder_PinB,INPUT_PULLUP);

   attachInterrupt(Left_Encoder_PinA, do_Left_Encoder, RISING);
   attachInterrupt(Right_Encoder_PinA, do_Right_Encoder, RISING);
}

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

void setup() {
    Serial.begin(115200);
  // put your setup code here, to run once:
pinMode (PA3, OUTPUT);
//pinMode (PA4, OUTPUT);
pinMode (PC6, OUTPUT);
pinMode (PE4, OUTPUT);
//pinMode (PE5, OUTPUT);
pinMode (PC5, OUTPUT);
//pinMode (PF4, OUTPUT);
//pinMode (PD7, OUTPUT);
//pinMode (PD6, OUTPUT);
//pinMode (Left_Encoder_PinB, OUTPUT);


// encoder

SetupEncoders();

}

void loop() {
  // put your main code here, to run repeatedly:

analogWrite(PC5, 0);  //pwm1 - motor direita
analogWrite(PC6, 1000); //pwm2 - motor direita
digitalWrite(PA3, HIGH); //sinal direita
analogWrite(PB0, 0); //pwm1 - motor esquerda
analogWrite(PB1, 1000); //pwm2 - motor esquerda
digitalWrite(PE4, HIGH); //sinal esquerda

Update_Encoders();


}
