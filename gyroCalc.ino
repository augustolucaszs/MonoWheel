void gyroCalc(){
      // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    
//A partir de los valores del acelerometro, se calculan los angulos Y, X
//respectivamente, con la formula de la tangente.    
    Acc[1] = atan(-1 * (ax / A_R) / sqrt(pow((ay / A_R), 2) + pow((az / A_R), 2))) * RAD_TO_DEG;
    Acc[0] = atan((ay / A_R) / sqrt(pow((ax / A_R), 2) + pow((az / A_R), 2))) * RAD_TO_DEG;

//Calculo del angulo del Giroscopio
    Gy[0] = gx / G_R;
    Gy[1] = gy / G_R;
    Gy[2] = gz / G_R;

    dt = (millis() - tiempo_prev) / 1000.0;
    tiempo_prev = millis();

//Aplicar el Filtro Complementario
    Angle[0] = 0.98 * (Angle[0] + Gy[0] * dt) + 0.02 * Acc[0];
    Angle[1] = 0.98 * (Angle[1] + Gy[1] * dt) + 0.02 * Acc[1];

//Integración respecto del tiempo paras calcular el YAW
    Angle[2] = Angle[2] + Gy[2] * dt;
    
    angulo0 = Angle[0];
    angulo1 = Angle[1];
    angulo2 = Angle[2];
    Input = angulo0;
  }
