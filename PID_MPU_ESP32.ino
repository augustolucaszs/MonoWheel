//#include "BluetoothSerial.h"
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 accelgyro;
#define OUTPUT_READABLE_ACCELGYRO
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
#define RAD_A_DEG = 57.295779
#define HOVER_SERIAL_BAUD   38400       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         38400      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           10         // [ms] Sending time interval
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
#define LED_BUILTIN         2           // Led GPIO 2
// Definição dos pinos para TX e RX
#define RXD2 16
#define TXD2 17
const char* ssid = "Casa Sergio"; /* coloque aqui o nome da rede wi-fi que o ESP32 deve se conectar */
const char* password = "36269871"; /* coloque aqui a senha da rede wi-fi que o ESP32 deve se conectar */
// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;
typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;
int sumpadValue1 = 0, sumpadValue2 = 0, count =10;
const int padPin1 = 36;
const int padPin2 = 39;
int padValue1 = 0;
int padValue2 = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float angulo0 = 0.0, angulo1 = 0.0, angulo2 = 0.0;
float Acc[2];
float Gy[3];
float Angle[3];
float dt = 0;

long tiempo_prev;

double Setpoint = 0, Input = 0, Output = 0;

bool blinkState = false;

PID myPID(&Input, &Output, &Setpoint,60,25,0,REVERSE);
//BluetoothSerial SerialBT;

void setup() {
  
    hoverSetup();
    initGyro();
    myPID.SetOutputLimits(-1000,1000);
    myPID.SetMode(AUTOMATIC);
    //SerialBT.begin("MonoWheel"); //Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");    
}

unsigned long lastTime = 0;

void Receive();
void Send();

void loop() {

    const unsigned long MillisNow = millis();
    const unsigned long comparisonTime = MillisNow - lastTime;
    
    Receive();
    gyroCalc();
    myPID.Compute(); 

    if(comparisonTime <= TIME_SEND){
      digitalWrite(LED_BUILTIN, LOW);
      sumpadValue1 = analogRead(padPin1) + sumpadValue1;
      sumpadValue2 = analogRead(padPin2) + sumpadValue2;
      count = count + 1;
      }  

    else{
     
      padValue1 = sumpadValue1/count;
      padValue2 = sumpadValue2/count;
      //SerialBT.println(Feedback.batVoltage);    
      if ((padValue1 > 3000) && (padValue2 > 3000)){
        Send(1000, Output);  
        //SerialBT.write(Serial.read());
        digitalWrite(LED_BUILTIN, HIGH);
        lastTime = millis();
        sumpadValue1 = 0;
        sumpadValue2 = 0;
        count = 0;
        
      }
      else if((padValue1 < 3000) || (padValue2 < 3000)){
        Send(-900, 0);  
        digitalWrite(LED_BUILTIN, HIGH);
        lastTime = millis();
        sumpadValue1 = 0;
        sumpadValue2 = 0;
        count = 0;
      }
      Serial.print(10);
//      Serial.print(" ");
//      Serial.print(Output);
//      Serial.print(" ");
      //Serial.print(padValue1);
      Serial.print(" ");
      Serial.print(padValue2);
      Serial.println(" ");
      //Serial.println(Feedback.batVoltage); 
    }
   
}
    

    
