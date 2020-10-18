// ########################## LIBRARIES ##########################
#include <PID_v1.h>                       // PID
#include "I2Cdev.h"                       // I2C for MPU6050
#include "MPU6050_6Axis_MotionApps20.h"   // MPU6050
#include "BluetoothSerial.h"            

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   38400         // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200        // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD        // [-] Start frme definition for reliable serial communication
#define TIME_SEND           10            // [ms] Sending time interval
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

// ########################## SENSOR PAD VARIABLES ##########################
int sumpadValue1 = 0, sumpadValue2 = 0, count =1;
int padValue1 = 0;
int padValue2 = 0;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

MPU6050 accelgyro;
int kp = 60, ki = 10, kd = 1;
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd,REVERSE);
BluetoothSerial SerialBT;
char command;

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
    myPID.SetMode(AUTOMATIC);
    
// ########################## PID END ##########################

    SerialBT.begin("MonoWheel"); //Bluetooth device name
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  Serial2.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;

void loop(void)
{
  if(SerialBT.available() > 0){
    command = SerialBT.read();
    kp = myPID.GetKp();
    ki = myPID.GetKi();
    kd = myPID.GetKd();
    String remember = "kp:" + String(kp) + " ki:" + String(ki) + " kd:" + String(kd);
    switch(command){
      case 'P':
        kp = kp + 5;
        myPID.SetTunings(kp, ki, kd);
        break;
      case 'p':
        kp = kp - 5;
        myPID.SetTunings(kp, ki, kd);
        break;
      case 'I':
        ki = ki + 1;
        myPID.SetTunings(kp, ki, kd);
        break;
      case 'i':
        ki = ki - 1;
        myPID.SetTunings(kp, ki, kd);
        break;
      case 'D':
        kd = kd + 1;
        myPID.SetTunings(kp, ki, kd);
        break;
      case 'd':
        kd = kd - 1;
        myPID.SetTunings(kp, ki, kd);
        break;
      case 'A':
        myPID.SetMode(AUTOMATIC);
        break;
      case 'M':
        Input = 0;
        Output = 0;
        myPID.SetMode(MANUAL);
        
        break;
      case 'c':
        SerialBT.println(remember);
    }
    SerialBT.println(remember);
  }
  unsigned long timeNow = millis();
  while(iTimeSend < 10){    
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
// ########################## END MPU6050 CALCULATION ##########################
// ########################## PID CALCULATION ##########################

    Input = Angle[0];
    myPID.Compute();
// ########################## END PID CALCULATION ##########################

    digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);  // Blink the LED
    iTimeSend = millis() + TIME_SEND;
    
// ########################## SENSOR PAD CALCULATION ##########################

    sumpadValue1 = analogRead(padPin1) + sumpadValue1;
    sumpadValue2 = analogRead(padPin2) + sumpadValue2;
    count = count + 1;    
// ########################## END SENSOR PAD CALCULATION ##########################
  }    
  iTimeSend = 0;
  
  padValue1 = sumpadValue1/count;
  padValue2 = sumpadValue2/count;
  
  if((padValue1 > 3000) && (padValue2 > 3000)){
    myPID.SetMode(AUTOMATIC);    
    Send(0, Output);
    sumpadValue1 = 0;
    sumpadValue2 = 0;
    count = 0;
  }
  else if((padValue1 < 3000) || (padValue2 < 3000)){
    Input = 0;
    Output = 0;
    myPID.SetMode(MANUAL);
    Send(1000, 0);
    sumpadValue1 = 0;
    sumpadValue2 = 0;
    count = 0; 
  }
  
  Serial.print(padValue1);
  Serial.print(" ");
  Serial.print(padValue2);
  Serial.print(" ");
  Serial.print(Input);
  Serial.print(" ");
  Serial.println(Output);

    
}
