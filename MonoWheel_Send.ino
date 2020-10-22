//const char* ssid = "iPhone de Lucas"; /* coloque aqui o nome da rede wi-fi que o ESP32 deve se conectar */
//const char* password = "zanicoski"; /* coloque aqui a senha da rede wi-fi que o ESP32 deve se conectar */
// ########################## LIBRARIES ##########################
#include <PID_v1.h>                       // PID
#include "I2Cdev.h"                       // I2C for MPU6050
#include "MPU6050_6Axis_MotionApps20.h"   // MPU6050
#include "BluetoothSerial.h"            

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   38400         // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200        // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD        // [-] Start frme definition for reliable serial communication
#define TIME_SEND           15            // [ms] Sending time interval
//#define DEBUG_RX                          // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
#define LED_BUILTIN         2             // Led GPIO 2
#define padPin1             36
#define padPin2             39
#define RXD2                16            // TX
#define TXD2                17            //RX

// ########################## MPU6050 DEFINES ##########################
#define OUTPUT_READABLE_ACCELGYRO
#define A_R                 16384.0       // 32768/2
#define G_R                 131.0         // 32768/250
#define RAD_A_DEG = 57.295779

// ########################## MPU6050 VARIABLES ##########################
int16_t ax, ay, az, gx, gy, gz;
float Acc[2], Gy[3], Angle[3], dt = 0;
long tiempo_prev;

// ########################## PID VARIABLES ##########################
double Setpoint = 0, Input = 0, Output = 0;
int power = 1000;

// ########################## SENSOR PAD VARIABLES ##########################
int sumpadValue1 = 0, sumpadValue2 = 0, count =1;
int padValue1 = 0;
int padValue2 = 0;
int sig = 1000;

// ########################## RECEIVE VARIABLES ##########################

uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   //int16_t  speedR_meas;
   //int16_t  speedL_meas;
   int16_t  batVoltage;
   //int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SEND VARIABLES ##########################
typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

MPU6050 accelgyro;
float kp = 7.00, ki = 11.00, kd = 0.20, KP = 7;
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd,REVERSE);
BluetoothSerial SerialBT;
char command;
int showMsg = 0, calibration = 0, z = 0, enFunckp = 0;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");
  Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
  pinMode(LED_BUILTIN, OUTPUT);
  
// ########################## MPU6050 INIT ##########################

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println("Updating internal sensor offsets...");
    accelgyro.setXAccelOffset(-3568); accelgyro.setYAccelOffset(-1386); accelgyro.setZAccelOffset(1510);  accelgyro.setXGyroOffset(60); accelgyro.setYGyroOffset(44); accelgyro.setZGyroOffset(18);
    delay(200);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t");
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t");
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t");
    Serial.print("\n");
    
// ########################## MPU6050 END ##########################

// ########################## PID INIT ##########################

    myPID.SetOutputLimits(-1000,1000);
    myPID.SetMode(MANUAL);
    myPID.SetSampleTime(5);
    
// ########################## PID END ##########################

    SerialBT.begin("MonoWheel"); //Bluetooth device name
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  Serial2.write((uint8_t *) &Command, sizeof(Command));
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;

void loop(void){
  if(SerialBT.available() >= 1){
    command = SerialBT.read();
    switch(command){
      case 'z':
        z = 1;
        kp = 0;
        ki = 0;
        kd = 0;
        PrintBtmsg();
        break;        
      case 'P':
        kp = kp + 0.5;
        KP = KP + 0.5;
        PrintBtmsg();
        break;
      case 'p':
        kp = kp - 0.5;
        KP = KP - 0.5;
        PrintBtmsg();
        break;
      case 'I':
        ki = ki + 0.05;
        PrintBtmsg();
        break;
      case 'i':
        ki = ki - 0.05;
        PrintBtmsg();
        break;
      case 'D':
        kd = kd + 0.05;
        PrintBtmsg();
        break;
      case 'd':
        kd = kd - 0.05;
        PrintBtmsg();
        break;
      case 'L':
        if (power >= 1000){
          power = power;
        }
        else{
          power = power + 50;
        }
        SerialBT.print("Power = ");
        SerialBT.println(power);
        myPID.SetOutputLimits(-power,power);
        break;
      case 'l':
        if (power <= 0){
          power = 0;
        }
        else{
          power = power - 50;
        }
        SerialBT.print("Power = ");
        SerialBT.println(power);
        myPID.SetOutputLimits(-power,power);
        break;
      case 'S':
        showMsg = 1;
        break;
      case 's':
        showMsg = 0;
        break;
      case 'T':
        enFunckp = 1;
        SerialBT.println("kp Map ACTIVATED!!");
        break;
      case 't':
        enFunckp = 0;
        SerialBT.println("kp Map DEACTIVATED!!");
        break;
      case 'Q':
        calibration = 1;
        break;
      case 'q':
        calibration = 0;
        break;
      case 'c':
        PrintBtmsg();
        break;       
    }
  }
  unsigned long timeNow = millis();
  while(iTimeSend < 15){    
// ########################## MPU6050 CALCULATION ##########################

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Acc[1] = atan(-1 * (ax / A_R) / sqrt(pow((ay / A_R), 2) + pow((az / A_R), 2))) * RAD_TO_DEG;
    Acc[0] = atan((ay / A_R) / sqrt(pow((ax / A_R), 2) + pow((az / A_R), 2))) * RAD_TO_DEG;
    Gy[0] = gx / G_R;
    Gy[1] = gy / G_R;
    Gy[2] = gz / G_R;
    dt = (millis() - tiempo_prev) / 1000.0;
    tiempo_prev = millis();
    Angle[0] = 0.98 * (Angle[0] + Gy[0] * dt) + 0.02 * Acc[0];
    Angle[1] = 0.98 * (Angle[1] + Gy[1] * dt) + 0.02 * Acc[1];
    Angle[2] = Angle[2] + Gy[2] * dt;

// ########################## PID CALCULATION ##################################

    Input = Angle[0];
    if(enFunckp == 1){
      if((Input >= -7) && (Input <= -5))    {kp = (((Input*Input)/3) - 1);}
      else if((Input > -5) && (Input < 5))  {kp = KP;}
      else if((Input >= 5) && (Input <= 7)) {kp = (((Input*Input)/3) -2);}
      else if((Input < -7) && (Input > 7))  {kp = 15;}
    }
    else if (enFunckp == 0){kp = KP;} 
    myPID.SetTunings(kp, ki, kd);
    myPID.Compute();
    
// ########################## SENSOR PAD CALCULATION ###########################

    sumpadValue1 = analogRead(padPin1) + sumpadValue1;
    sumpadValue2 = analogRead(padPin2) + sumpadValue2;
    count = count + 1;
        
// ########################## END SENSOR PAD CALCULATION #######################

    iTimeSend = millis() - timeNow; //should aproximate to 10ms then get out of the loop    
  }
  iTimeSend = 0;
  padValue1 = sumpadValue1/count;
  padValue2 = sumpadValue2/count; 
     
  if (calibration == 1){
    myPID.SetMode(AUTOMATIC);
    Send(-1000,Output);
    PrintMsg();   
  }
  else{
    if((padValue1 >= 1700) && (padValue2 >= 1700)){
      myPID.SetMode(AUTOMATIC);
      Send(-1000, Output);
      sumpadValue1 = 0;
      sumpadValue2 = 0;
      count = 0;
      PrintMsg();
    }
    else if((padValue1 < 1700) || (padValue2 < 1700)){
      myPID.SetMode(MANUAL);
      Send(1000, 0);
      sumpadValue1 = 0;
      sumpadValue2 = 0;
      count = 0;
      PrintMsg();
    }    
  }
}
void PrintMsg(){
  if(showMsg == 1){
    SerialBT.print(Input);
    SerialBT.print(", ");
    SerialBT.print(Output);
    SerialBT.print(", ");
    SerialBT.print(padValue1);
    SerialBT.print(", ");
    SerialBT.println(padValue2);        
  }
  else{return;}
}

void PrintBtmsg(){
  myPID.SetTunings(kp, ki, kd);
  kp = myPID.GetKp();
  ki = myPID.GetKi();
  kd = myPID.GetKd();
  String remember = "kp:" + String(kp) + " ki:" + String(ki) + " kd:" + String(kd);
  SerialBT.println(remember);   
}
