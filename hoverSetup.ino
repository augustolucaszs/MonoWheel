  void hoverSetup(){
    Serial.begin(SERIAL_BAUD);
    Serial.println("Hoverboard Serial v1.0");
  
    Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
    pinMode(LED_BUILTIN, OUTPUT);
  }
