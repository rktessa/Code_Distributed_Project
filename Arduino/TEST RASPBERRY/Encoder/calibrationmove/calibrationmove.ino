void calibrationMove(){

 accSensitivity=0.05;
 speSensitivity=0.2;
 speError =0;


 delay(500);

           
while(n<250){
  start=millis();
  n++;

  gyro.update();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if(ultraSensor.distanceCm() <= distStop){
     motor1.setPWM(0);
     motor2.setPWM(0);
     break;
  }

  rotX = g.gyro.x - rotXError;
  rotY = g.gyro.y - rotYError;
  rotZ = g.gyro.z - rotZError;

  angX = gyro.getAngleX() - angXError;
  angY = gyro.getAngleY() - angYError;
  angZ = gyro.getAngleZ() - angZError;
  
  AaccX[0] = a.acceleration.x - accXError - accError*sin(angX*3.1415/180);
  AaccY[0] = a.acceleration.y - accYError - accError*sin(angY*3.1415/180);
  AaccZ[0] = a.acceleration.z - accZError - accError*cos(angX*3.1415/180)*cos(angY*3.1415/180);

  accX = AaccX[0] - AaccX[dim-1]/(dim-1);
  accY = AaccY[0] - AaccY[dim-1]/(dim-1);
  accZ = AaccZ[0] - AaccZ[dim-1]/(dim-1);

  for(int i=0;i<(dim-2);i++){
    AaccX[dim - 1 - i] = AaccX[dim - 2 - i] + AaccX[0];
    AaccY[dim - 1 - i] = AaccY[dim - 2 - i] + AaccY[0];
    AaccZ[dim - 1 - i] = AaccZ[dim - 2 - i] + AaccZ[0];
   }

  AaccX[1] = AaccX[0];
  AaccY[1] = AaccY[0];
  AaccZ[1] = AaccZ[0];
  
  accT = sqrt(pow(accX,2)+pow(accY,2)+pow(accZ,2));

  if(n<15||n>240){
    motor1.setPWM(0);
    motor2.setPWM(0);
  }else{
    motor1.setPWM(-motorSpeed*compsx/100);
    motor2.setPWM(motorSpeed*compdx/100);

/*
if(n<15||n>240){
    motor1.stop();
    motor2.stop();
  }else{
    if(n<180||n>210){
    motor1.run(-motorSpeed*compsx/100);
    motor2.run(motorSpeed*compdx/100);
    }else{
    motor1.run(motorSpeed*compsx/100);
    motor2.run(-motorSpeed*compdx/100);
    }
    */
    if(n>30){
    speX += accX*cycleTime/10;
    speY += accY*cycleTime/10;
    speZ += accZ*cycleTime/10;
    
    
    speT = sqrt(pow(speX,2)+pow(speY,2)+pow(speZ,2));
    
    speError += speT;

    if(sqrt((speStop-speT)*(speStop-speT))<speSensitivity*velocity){
      nStop++;
    }else{
      speStop=speT;
      nStop=0;
    }
    }
/*
      if(angZ>0){
        compsx /= cos(angZ*3.1415/180);
      }else{
        compdx /= cos(angZ*3.1415/180);
      }
      */
  }


  if(n>9&&n<245){
    
     if(accT>accSensitivity*power*resamotori&&accT<0.16*power*resamotori){
       accSensitivity = accT/(power*resamotori);
    }
/*
    if(accT>20){
      motor1.stop();
      motor2.stop();
      Serial.print("TEST NON VALIDO, RIESEGUIRE!!  ");
      Serial.println(accT);
      delay(200);
      resetFunc();
    }
*/
    if(nStop>100/sampTime){
          speSensitivity = sqrt((speStop-speT)*(speStop-speT))/velocity;
      nStop=0;
    }
  }

  delay(sampTime);

   cycleTime=millis()-start;
}

  speError /= (n-40);
  speError = 2 - speError/velocity;
/*
  if(speSensitivity<0.06){
    speSensitivity = 0.06;
  }
*/
  if(accSensitivity>0.16){
    accSensitivity += 0.05;
  }else{
  accSensitivity *=1.6;
  }

  speSensitivity *=1.4;
  /*
  Serial.print("accSensitivity: ");
  Serial.println(accSensitivity);
  Serial.print("speSensitivity: ");
  Serial.println(speSensitivity);
  Serial.print("speError ");
  Serial.println(speError);
  Serial.print("cycleTime ");
  Serial.println(cycleTime);
  Serial.print("Time travel ");
  Serial.println(cycleTime*225);
*/
n=0;
nStop=0;


motor1.setPWM(0);
motor2.setPWM(0);
delay(1500);

  for(int i=0;i<dim;i++){
    AaccX[i] = 0.00;
    AaccY[i] = 0.00;
    AaccZ[i] = 0.00;
   }

speX=0;
speY=0;
speZ=0;
accX=0;
accY=0;
accZ=0;

//rgbled_7.setColor(1,0,0,0);
//rgbled_7.setColor(2,0,0,0);
//rgbled_7.show();
}

