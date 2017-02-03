//CONSTANTS
//pid constants
#define PID_MIN_OUTPUT      -400     //PID min output
#define PID_MAX_OUTPUT      400      //PID max output

//comm lost constants
#define COMM_LOST_TIMEOUT             3000  //communication lost timeout
#define COMM_LOST_SHUTDOWN_TIMEOUT   15000  //communication lost timeout
#define LANDING_THROTTLE              1000  //landing throttle


//VARIABLES
//loop timing control
double starttime, endtime;    

//sensor control variables
double pitch, roll, yaw, heading, temperature, temperatureBarometer, pressure, altitudeBarometer, altitudeUltrasonic = 0;  //sensor readings
double gyroInputPitch, gyroInputRoll, gyroInputYaw = 0; //gyro rate tracking
double pitchLevelAdjust, rollLevelAdjust = 0; //autolevel feature


//motor control variables
int throttle = 1000; //motors throttle
int start = 0;    //start variable (for start control)
int m1Speed, m2Speed, m3Speed, m4Speed = 0; //motor speeds (for tracking)

//remote control variables
int rawRCPitch, rawRCRoll, rawRCYaw = 1500;
String commandString;
double lastCommandTime; //last command time (for tracking comm lost)
double lastSerialProcessTime = 0;   //last serial process time

//PID variables
double errSumPitch, lastErrPitch = 0;
double errSumRoll, lastErrRoll = 0;
double errSumYaw, lastErrYaw = 0;

double PIDPitchIn, PIDPitchOut, PIDPitchSetpoint = 0;
double PIDRollIn, PIDRollOut, PIDRollSetpoint = 0;
double PIDYawIn, PIDYawOut, PIDYawSetpoint = 0;

double KpPitch = 0.0;     //increase by step 0.2
double KiPitch = 0.0;     //increase by step 0.01
double KdPitch = 0.0;     //increase by step 1.0

double KpRoll = KpPitch;
double KiRoll = KiPitch;
double KdRoll = KdPitch;

double KpYaw = 0.0;
double KiYaw = 0.0;
double KdYaw = 0.0;

//print variables
double lastPrintTimeSlow = 0;
double lastPrintTimeFast = 0;

//motor testing
int motorTest = 0;

//testing variables
double lastCounter = 0;
double counter = 0;


void setup() {
  // put your setup code here, to run once:

  //init I2C communication
  I2CInit(); //start I2C communication

  //init serial communication
  Serial.begin(115200); //start serial to PC communication

  //init motors
  InitMotors();

  //init sensors
  SetupGY87(); //setup GY-87 sensor
  //SetupHYSRF05(); //setup HY-SRF05 sensor

  //turn on led
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  //time tracking variables
  lastSerialProcessTime = micros();
}

void loop() {
  // put your main code here, to run repeatedly:

  //save start time
  starttime = micros();

  //read serial commands
  if((lastSerialProcessTime + 10000) >= starttime) //100 Hz
  {
    SerialCommandProcess();
    lastSerialProcessTime = micros();
  }
  
  //read GY-87 sensor values (gyro rate & ypr angles)
  ReadDataMPU6050();    //read gyro & acc data
  GyroRate();           //get gyro rate
  YawPitchRollTemp();   //get ypr angles (in degrees)
  //HeadingAltitude();

  //read HY-SFR05 sensor (altitude)
  //altitudeUltrasonic = AltitudeCentimeterHYSRF05(temperature);


  if (start == 0) //stop
  {
    StopAllMotors();
    PIDSetpointReset(); //reset pid setpoints
  }

  if (start == 1) //start
  {
    //autolevel adjust
    pitchLevelAdjust = pitch * 15;    //calculate the pitch angle correction
    rollLevelAdjust = roll * 15;      //calculate the roll angle correction
    
    //raw RC to setpoint
    PIDPitchSetpoint = ((rawRCPitch - 1500 - pitchLevelAdjust) / 3.0);
    PIDRollSetpoint = ((rawRCRoll - 1500 - rollLevelAdjust) / 3.0);
    PIDYawSetpoint = ((rawRCYaw - 1500) / 3.0);
    

    //calculate output PID
    PIDPitchIn = gyroInputPitch;
    PIDRollIn = gyroInputRoll;
    PIDYawIn = gyroInputYaw;

    PIDCalculate();

    //calculate motor speeds
    int m1 = throttle - PIDPitchOut - PIDRollOut + PIDYawOut;
    int m2 = throttle - PIDPitchOut + PIDRollOut - PIDYawOut;
    int m3 = throttle + PIDPitchOut + PIDRollOut + PIDYawOut;
    int m4 = throttle + PIDPitchOut - PIDRollOut - PIDYawOut;

    //keep engines running
    if(throttle > 1100)
    {
      if(m1 < 1100) m1 = 1100;
      if(m2 < 1100) m2 = 1100;
      if(m3 < 1100) m3 = 1100;
      if(m4 < 1100) m4 = 1100;
    }

    //if throttle is zero - stop all motors
    if (throttle == 1000)
    {
      StopAllMotors();
    }
    else //else set motors speeds
    {
      if(motorTest == 0)
      {
        SetMotorSpeeds(m1, m2, m3, m4);
      }
      if(motorTest == 1)
      {
        SetMotorSpeeds(m1, 1000, 1000, 1000);
      }
      if(motorTest == 2)
      {
        SetMotorSpeeds(1000, m2, 1000, 1000);
      }
      if(motorTest == 3)
      {
        SetMotorSpeeds(1000, 1000, m3, 1000);
      }
      if(motorTest == 4)
      {
        SetMotorSpeeds(1000, 1000, 1000, m4);
      }
    }

    //safety - if communication lost
    if (millis() - lastCommandTime >= COMM_LOST_TIMEOUT) //if communication lost, begin landing
    {
      if (millis() - lastCommandTime >= COMM_LOST_SHUTDOWN_TIMEOUT) //if communication lost long time shutdown
      {
        start = 0;
        throttle = 1000;
      }
      else
      {
        throttle = LANDING_THROTTLE;
      }
     
      PIDSetpointReset(); //reset pid setpoints
    }


  }

  //testing
  counter = counter + 1; //for testing

  //debug output
  SerialDebugOutput();

  while(micros() - starttime < 4000); //250 Hz
}

/*
 * Process commands from serial port
 */
void SerialCommandProcess()
{
  //process serial commands
  while (Serial.available())
  {
    //delay(1);  //delay to allow buffer to fill
    delayMicroseconds(100); //delay to allow buffer to fill
    
    if (Serial.available() > 0)
    {
      char c = Serial.read();  //get one byte from serial buffer
      commandString += c;      //make the string
    }
  }
  
  if (commandString.length() > 0)
  {
    char command = commandString.charAt(0);
    String parameter = commandString.substring(1);

    //variables for PID tunning
    char subCommand1 = ' ';
    char subCommand2 = ' ';
    String subParameter = "";
    int iSubParameter = 0;
    
    //Serial.print(command); Serial.print(" , "); Serial.println(parameter);
    
    switch(command)
    {
      case 'C': //command
        if(parameter == "START")
        {
          if (start == 0) //if not started
          {
            PIDReset(); //reset PID variables
    
            start = 1;
            throttle = 1000;
          }
        }
        if(parameter == "STOP")
        {
          start = 0;
          throttle = 1000;
        }    
      break;
        
      case '#': //command with return value
        if(parameter == "S")
        {
          Serial.print(";");
          /*
          Serial.print(pitch);
          Serial.print(";");
          Serial.print(roll);
          Serial.print(";");
          Serial.print(yaw);
          Serial.print(";");
          */
          Serial.print(throttle);
          Serial.print(";");
          Serial.print(";");
          /*
          Serial.print(m1Speed);
          Serial.print(";");
          Serial.print(m2Speed);
          Serial.print(";");
          Serial.print(m3Speed);
          Serial.print(";");
          Serial.print(m4Speed);
          Serial.print(";");
          Serial.print(lastCounter);
          Serial.print(";");
          */
         
          Serial.print(pitch);
          Serial.print(";");
          Serial.print(roll);
          Serial.print(";");
          Serial.print(yaw);
          Serial.print(";");
          Serial.print(";");
          /*
          Serial.print(pitchLevelAdjust);
          Serial.print(";");
          Serial.print(rollLevelAdjust);
          Serial.print(";");
          Serial.print(PIDPitchSetpoint);
          Serial.print(";");
          Serial.print(PIDRollSetpoint);
          */
          Serial.print(PIDYawSetpoint);
          Serial.print(";");
          Serial.print(PIDYawIn);
          Serial.print(";");
          Serial.print(PIDYawOut);
          Serial.print(";");
          Serial.print(";");
          
          Serial.print(KpYaw);
          Serial.print(";");
          Serial.print(KiYaw);
          Serial.print(";");
          Serial.print(KdYaw);
          Serial.print(";");
          
          Serial.println(";");
        }    
      break;
      
      case 'P': //pitch      
        rawRCPitch = parameter.toInt();
      break;
      
      case 'R': //roll      
        rawRCRoll = parameter.toInt();
      break;
      
      case 'Y': //yaw      
        rawRCYaw = parameter.toInt();
      break;
      
      case 'L': //level
        PIDSetpointReset(); //reset pid setpoints
      break;
      
      case 'S': //speed
        throttle = constrain(parameter.toInt(), 1000, 1800);
      break;

      case 'K': //PID tunning
        subCommand1 = parameter.charAt(0);
        subCommand2 = parameter.charAt(1);
        subParameter = parameter.substring(2);  

        iSubParameter = subParameter.toInt();

        if(subCommand1 == 'P') //pitch & roll PID
        {
          if(subCommand2 == 'P')
          {
            KpPitch = KpRoll = iSubParameter / 1000.0;
          }
          else if(subCommand2 == 'I')
          {
            KiPitch = KiRoll = iSubParameter / 1000.0;
          }
          else if(subCommand2 == 'D')
          {
            KdPitch = KdRoll = iSubParameter / 1000.0;
          }
        }
        else if(subCommand1 = 'Y') //yaw PID
        {
          if(subCommand2 == 'P')
          {
            KpYaw = iSubParameter / 1000.0;
          }
          else if(subCommand2 == 'I')
          {
            KiYaw = iSubParameter / 1000.0;
          }
          else if(subCommand2 == 'D')
          {
            KdYaw = iSubParameter / 1000.0;
          }
        }
        
        /*
        Serial.print("subCom1=");
        Serial.print(subCommand1);
        Serial.print("\tsubCom2=");
        Serial.print(subCommand2);
        Serial.print("\tsubParam=");
        Serial.println(subParameter);
        
        Serial.print("KpPitch=");
        Serial.print(KpPitch);
        Serial.print("\tKiPitch=");
        Serial.print(KiPitch);
        Serial.print("\tKdPitch=");
        Serial.println(KdPitch);
        Serial.print("KpYaw=");
        Serial.print(KpYaw);
        Serial.print("\tKiYaw=");
        Serial.print(KiYaw);
        Serial.print("\tKdYaw=");
        Serial.println(KdYaw);
        */
        
        //reset pid variables
        PIDReset();
      break;
      
      case 'M': //motor test
        if(parameter == "0")
        {
          motorTest = 0;
        }
        else if(parameter == "1")
        {
          motorTest = 1;
        }
        else if(parameter == "2")
        {
          motorTest = 2;
        }
        else if(parameter == "3")
        {
          motorTest = 3;
        }
        else if(parameter == "4")
        {
          motorTest = 4;
        }  
          
      break;
      
      //FOR TESTING
      /*
      default:
        throttle = 1000;
        break;
      */
    }
    
    commandString = "";
    lastCommandTime = millis();
  }
  
}


/*
 * Reset PID variables
 */
void PIDReset()
{
  errSumPitch = lastErrPitch = 0;
  errSumRoll = lastErrRoll = 0;
  errSumYaw = lastErrYaw = 0;
}

/*
 * Reset PID setpoint variables
 */
void PIDSetpointReset()
{
  //level rc commands
  rawRCPitch = 1500;
  rawRCRoll = 1500;
  rawRCYaw = 1500;
  
  //reset setpoints
  PIDPitchSetpoint = 0;
  PIDRollSetpoint = 0;
  PIDYawSetpoint = 0;
}

/*
 * Calculate PID
 */
void PIDCalculate()
{
  //PID
  //Pitch PID
  //compute error variables
  double errorPitch = PIDPitchIn - PIDPitchSetpoint;
  errSumPitch += errorPitch;
  double dErrPitch = errorPitch - lastErrPitch;
  //compute pid variables
  double pPitch = KpPitch * errorPitch;
  double iPitch = KiPitch * errSumPitch;
  double dPitch = KdPitch * dErrPitch;

  //integral fix - windup (solution - limit growth)
  iPitch = constrain(iPitch, PID_MIN_OUTPUT, PID_MAX_OUTPUT);

  //integral fix - still running when destination reached (solution - reset integral)
  if (errorPitch == 0)
  {
    iPitch = 0;
    errSumPitch = 0;
  }

  //compute PID output
  PIDPitchOut = pPitch + iPitch + dPitch;

  //limit PID output
  PIDPitchOut = constrain(PIDPitchOut, PID_MIN_OUTPUT, PID_MAX_OUTPUT);

  //remember for next time
  lastErrPitch = errorPitch;


  //Roll PID
  //compute error variables
  double errorRoll = PIDRollIn - PIDRollSetpoint;
  errSumRoll += errorRoll;
  double dErrRoll = errorRoll - lastErrRoll;
  //compute pid variables
  double pRoll = KpRoll * errorRoll;
  double iRoll = KiRoll * errSumRoll;
  double dRoll = KdRoll * dErrRoll;

  //integral fix - windup (solution - limit growth)
  iRoll = constrain(iRoll, PID_MIN_OUTPUT, PID_MAX_OUTPUT);

  //integral fix - still running when destination reached (solution - reset integral)
  if (errorRoll == 0)
  {
    iRoll = 0;
    errSumRoll = 0;
  }

  //compute PID output
  PIDRollOut = pRoll + iRoll + dRoll;

  //limit PID output
  PIDRollOut = constrain(PIDRollOut, PID_MIN_OUTPUT, PID_MAX_OUTPUT);

  //remember for next time
  lastErrRoll = errorRoll;
  
  
  //Yaw PID
  //compute error variables
  double errorYaw = PIDYawIn - PIDYawSetpoint;
  errSumYaw += errorYaw;
  double dErrYaw = errorYaw - lastErrYaw;
  //compute pid variables
  double pYaw = KpYaw * errorYaw;
  double iYaw = KiYaw * errSumYaw;
  double dYaw = KdYaw * dErrYaw;

  //integral fix - windup (solution - limit growth)
  iYaw = constrain(iYaw, PID_MIN_OUTPUT, PID_MAX_OUTPUT);

  //integral fix - still running when destination reached (solution - reset integral)
  if (errorYaw == 0)
  {
    iYaw = 0;
    errSumYaw = 0;
  }

  //compute PID output
  PIDYawOut = pYaw + iYaw + dYaw;

  //limit PID output
  PIDYawOut = constrain(PIDYawOut, PID_MIN_OUTPUT, PID_MAX_OUTPUT);

  //remember for next time
  lastErrYaw = errorYaw;
}

/*
 * Output debug info to serial port
 */
void SerialDebugOutput()
{

  //fast output
  if (lastPrintTimeFast == 0 || millis() - lastPrintTimeFast >= 200)
  {
    lastPrintTimeFast = millis();

/*
    Serial.print("trottle = ");
    Serial.print(throttle);
    Serial.print(F("\tPIDYawSetpoint = "));
    Serial.print(PIDYawSetpoint);
    Serial.print(F("\trawRCYaw = "));
    Serial.print(rawRCYaw);
    Serial.print(F("\tPIDYawIn = "));
    Serial.print(PIDYawIn);
    Serial.print(F("\tPIDYawOut = "));
    Serial.println(PIDYawOut);
*/

/*
    Serial.print(F("KpPitch = "));
    Serial.print(KpPitch);
    Serial.print(F("\tKiPitch = "));
    Serial.print(KiPitch);
    Serial.print(F("\tKdPitch = "));
    Serial.print(KdPitch);
    Serial.print(F("\t"KpYaw = "));
    Serial.print(KpYaw);
    Serial.print(F("\tKiYaw = "));
    Serial.print(KiYaw);
    Serial.print(F("\tKdYaw = "));
    Serial.println(KdYaw);
*/

/*
    Serial.print(PIDPitchSetpoint);
    Serial.print(F(";"));
    Serial.print(throttle);
    Serial.print(F(";"));
    Serial.print(pitch);
    Serial.print(F(";"));
    Serial.print(m1Speed);
    Serial.print(F(";"));
    Serial.print(m2Speed);
    Serial.print(F(";"));
    Serial.print(m3Speed);
    Serial.print(F(";"));
    Serial.println(m4Speed);
    return;
*/
/*
    Serial.print(F("GPitch = "));
    Serial.print(gyroInputPitch);
    Serial.print(F("\tGRoll = "));
    Serial.print(gyroInputRoll);
    Serial.print(F("\tGYaw = "));
    Serial.print(gyroInputYaw);
    Serial.print(F("\tTemp = "));
    Serial.println(temperature);
*/
/*
    Serial.print(F("Pitch = "));
    Serial.print(pitch);
    Serial.print(F("\tRoll = "));
    Serial.print(roll);
    Serial.print(F("\tYaw = "));
    Serial.print(yaw);
    Serial.print(F("\tTemp = "));
    Serial.print(temperature);
    Serial.print(F("\tHeading = "));
    Serial.println(heading);
*/
/*
    if (start == 1)
    {
      Serial.print(F("m1 = "));
      Serial.print(m1Speed);
      Serial.print(F("\tm2 = "));
      Serial.print(m2Speed);
      Serial.print(F("\tm3 = "));
      Serial.print(m3Speed);
      Serial.print(F("\tm4 = "));
      Serial.println(m4Speed);
    }
*/
    
    //Serial.print(F("\tAltBar = "));
    //Serial.println(altitudeBarometer);
    //Serial.print(F("\tAltUltra = "));
    //Serial.println(altitudeUltrasonic);

    //Serial.print(F("TempBarometer = "));
    //Serial.print(temperatureBarometer);
    //Serial.print(F("\tPressure = "));
    //Serial.print(pressure);

  }


  //slow output
  if (lastPrintTimeSlow == 0 || millis() - lastPrintTimeSlow >= 1000)
  {
    lastPrintTimeSlow = millis();

    lastCounter = counter;    
    counter = 0;

    digitalWrite(13, !digitalRead(13));
    //Serial.print(F("Counter = "));
    //Serial.println(counter);
  }

}

