void Receive()
{
  // Check for new data availability in the Serial buffer
  if (Serial2.available()) {
    incomingByte = Serial2.read();                                 // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;   // Construct the start frame    
  }
  else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
    Serial.print(incomingByte);
    return;
  #endif      
  
  // Copy received data
  if (bufStartFrame == START_FRAME) {
    //SerialBT.println("1");// Initialize if new data is detected
    p     = (byte *)&NewFeedback;
    *p++  = incomingBytePrev;
    *p++  = incomingByte;
    idx   = 2;  
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    //SerialBT.println("2");
    *p++  = incomingByte; 
    idx++;
  } 

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    //SerialBT.println("3");   
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
  
    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      
       //Print data to built-in Serial
//      SerialBT.print("1: ");   SerialBT.print(Feedback.cmd1);
//      SerialBT.print(" 2: ");  SerialBT.print(Feedback.cmd2);
//      SerialBT.print(" 3: ");  SerialBT.print(Feedback.speedR_meas);
//      SerialBT.print(" 4: ");  SerialBT.print(Feedback.speedL_meas);
//      SerialBT.print(" 5: ");  SerialBT.print(Feedback.batVoltage);
//      SerialBT.print(" 6: ");  SerialBT.print(Feedback.boardTemp);
//      SerialBT.print(" 7: ");  SerialBT.println(Feedback.cmdLed);

    } else {
//      SerialBT.println("Non-valid data skipped");
      //SerialBT.println("Non-valid data skipped");
    }
    //didx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }
  // Update previous states
  incomingBytePrev  = incomingByte;
  
}
