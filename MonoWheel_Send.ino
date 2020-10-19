//#include <WiFi.h>
//#include <ESPmDNS.h>
//#include <WiFiUdp.h>
//#include <ArduinoOTA.h>

const char* ssid = "iPhone de Lucas"; /* coloque aqui o nome da rede wi-fi que o ESP32 deve se conectar */
const char* password = "zanicoski"; /* coloque aqui a senha da rede wi-fi que o ESP32 deve se conectar */
// ########################## LIBRARIES ##########################
#include <PID_v1.h>                       // PID
#include "I2Cdev.h"                       // I2C for MPU6050
#include "MPU6050_6Axis_MotionApps20.h"   // MPU6050
#include "BluetoothSerial.h"            

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   38400         // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200        // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD        // [-] Start frme definition for reliable serial communication
#define TIME_SEND           20            // [ms] Sending time interval
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

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

MPU6050 accelgyro;
float kp = 5, ki = 0.0, kd = 0.0;
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
    Serial.println("Booting");
    
//  WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);
//  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
//    Serial.println("Connection Failed! Rebooting...");
//    delay(5000);
//    ESP.restart();
//  }
//  // Port defaults to 3232                            // ArduinoOTA.setPort(3232);
//  // Hostname defaults to esp3232-[MAC]               // ArduinoOTA.setHostname("myesp32");
//  // No authentication by default                     // ArduinoOTA.setPassword("admin");
//  // Password can be set with it's md5 value as well  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
//  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
//  ArduinoOTA.onStart([]() {
//    String type;
//    if (ArduinoOTA.getCommand() == U_FLASH)
//      type = "sketch";
//    else // U_SPIFFS
//      type = "filesystem";
//    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
//    Serial.println("Start updating " + type);
//  });
//  ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
//  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
//    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
//  });
//  ArduinoOTA.onError([](ota_error_t error) {
//    Serial.printf("Error[%u]: ", error);
//    if (error == OTA_AUTH_ERROR)         Serial.println("Auth Failed");
//    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
//    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
//    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
//    else if (error == OTA_END_ERROR)     Serial.println("End Failed");
//  });
//  ArduinoOTA.begin();
//  Serial.println("Ready");
//  Serial.print("IP address: ");
//  Serial.println(WiFi.localIP());
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;

void loop(void)
{
//    ArduinoOTA.handle();
//  yield();
  
  if(SerialBT.available() >= 1){
    command = SerialBT.read();
    kp = myPID.GetKp();
    ki = myPID.GetKi();
    kd = myPID.GetKd();
    String remember = "kp:" + String(kp) + " ki:" + String(ki) + " kd:" + String(kd);
    switch(command){
      case 'P':
        kp = kp + 0.5;
        myPID.SetTunings(kp, ki, kd);
        SerialBT.println(remember);
        break;
      case 'p':
        kp = kp - 0.5;
        myPID.SetTunings(kp, ki, kd);
        SerialBT.println(remember);
        break;
      case 'I':
        ki = ki + 0.05;
        myPID.SetTunings(kp, ki, kd);
        SerialBT.println(remember);
        break;
      case 'i':
        ki = ki - 0.05;
        myPID.SetTunings(kp, ki, kd);
        SerialBT.println(remember);
        break;
      case 'D':
        kd = kd + 0.05;
        myPID.SetTunings(kp, ki, kd);
        SerialBT.println(remember);
        break;
      case 'd':
        kd = kd - 0.05;
        myPID.SetTunings(kp, ki, kd);
        SerialBT.println(remember);
        break;
      case 'A':
        myPID.SetMode(AUTOMATIC);
        sig = -1000;
        break;
      case 'M':
        Input = 0;
        Output = 0;
        sig = 1000;
        myPID.SetMode(MANUAL);        
        break;
      case 'L':
        power = power + 50;
        break;
      case 'l':
        power = power - 50;
        break;
      case 'c':
        SerialBT.println(remember);
    }
  }
  
  myPID.SetOutputLimits(-power,power);
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

    
// ########################## SENSOR PAD CALCULATION ##########################

    sumpadValue1 = analogRead(padPin1) + sumpadValue1;
    sumpadValue2 = analogRead(padPin2) + sumpadValue2;
    count = count + 1;
        
// ########################## END SENSOR PAD CALCULATION ##########################

    digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);  // Blink the LED
    //iTimeSend = millis() + TIME_SEND;
    iTimeSend = millis() - timeNow; //should aproximate to 10ms then get out of the loop
    
  }    
  
  iTimeSend = 0;
  
  padValue1 = sumpadValue1/count;
  padValue2 = sumpadValue2/count;
  
  if((padValue1 >= 2500) && (padValue2 >= 2500)){
    myPID.SetMode(AUTOMATIC);
    sig = -1000;    
    Send(sig, Output);
    sumpadValue1 = 0;
    sumpadValue2 = 0;
    count = 0;  
  }
  else if((padValue1 < 2500) || (padValue2 < 2500)){
    Output = 0;
    sig = 1000;
    myPID.SetMode(MANUAL);
    Send(sig, 0);
    sumpadValue1 = 0;
    sumpadValue2 = 0;
    count = 0;
  }    
}




//    Serial.print(padValue1);
//    Serial.print(" ");
//    Serial.print(padValue2);
//    Serial.print(" ");
//    Serial.print(Input);
//    Serial.print(" ");
//    Serial.println(Output);
