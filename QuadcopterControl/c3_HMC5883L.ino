// ----------------------------------------------------------
// HMC5883L (magnetometer) constants & variables
// ----------------------------------------------------------
//HMC5883L constants
#define HMC5883L_I2C_ADDRESS          0x1E      //I2C address of the HMC5883L
#define HMC5883L_REG_CONFIG_A         0x00      //CONFIG_A register
#define HMC5883L_REG_CONFIG_B         0x01      //CONFIG_B register (gain - sensitivity control)
#define HMC5883L_REG_MODE             0x02      //MODE register
#define HMC5883L_REG_DATA_OUTPUT_X_H  0x03      //DATA_OUTPUT_X_H register
#define HMC5883L_REG_IDENTIFICATION_A 0x0A      //IDENTIFICATION_A register
#define HMC5883L_REG_IDENTIFICATION_B 0x0B      //IDENTIFICATION_B register
#define HMC5883L_REG_IDENTIFICATION_C 0x0C      //IDENTIFICATION_C register
#define HMC5883L_REG_IDENTIFICATION_A_ID  0x48  //IDENTIFICATION_A value (who am i)
#define HMC5883L_REG_IDENTIFICATION_B_ID  0x34  //IDENTIFICATION_B value (who am i)
#define HMC5883L_REG_IDENTIFICATION_C_ID  0x33  //IDENTIFICATION_C value (who am i)

//magnetometer - sensitivity (Ga)
#define HMC5883_GAIN_0_88             0x00      // +/- 0.88
#define HMC5883_GAIN_1_3              0x20      // +/- 1.3 (default)
#define HMC5883_GAIN_1_9              0x40      // +/- 1.9
#define HMC5883_GAIN_2_5              0x60      // +/- 2.5
#define HMC5883_GAIN_4_0              0x80      // +/- 4.0
#define HMC5883_GAIN_4_7              0xA0      // +/- 4.7
#define HMC5883_GAIN_5_6              0xC0      // +/- 5.6
#define HMC5883_GAIN_8_1              0xE0      // +/- 8.1
#define HMC5883_GAIN_0_88_SENSITIVITY 0.73
#define HMC5883_GAIN_1_3_SENSITIVITY  0.92
#define HMC5883_GAIN_1_9_SENSITIVITY  1.22
#define HMC5883_GAIN_2_5_SENSITIVITY  1.52
#define HMC5883_GAIN_4_0_SENSITIVITY  2.27
#define HMC5883_GAIN_4_7_SENSITIVITY  2.56
#define HMC5883_GAIN_5_6_SENSITIVITY  3.03
#define HMC5883_GAIN_8_1_SENSITIVITY  4.35

//magnetometer - data output rate (frequency of data collecting)
#define HMC5883_DATA_OUTPUT_RATE_0_75   0x00    //0.75 Hz
#define HMC5883_DATA_OUTPUT_RATE_1_50   0x04    //1.5 Hz
#define HMC5883_DATA_OUTPUT_RATE_3_00   0x08    //3 Hz
#define HMC5883_DATA_OUTPUT_RATE_15_00  0x10    //15 Hz (default)
#define HMC5883_DATA_OUTPUT_RATE_30_00  0x14    //30 Hz
#define HMC5883_DATA_OUTPUT_RATE_75_00  0x18    //75 Hz

//magnetometer - number of samples averaged (1 to 8)
#define HMC5883_NUMBER_SAMPLES_1        0x00    //1 sample (default)
#define HMC5883_NUMBER_SAMPLES_2        0x20    //2 sample
#define HMC5883_NUMBER_SAMPLES_4        0x40    //4 sample
#define HMC5883_NUMBER_SAMPLES_8        0x60    //8 sample

//variables
double magnetometerSensitivity;
double magnetometerRawXOffset = 0, magnetometerRawYOffset = 0, magnetometerRawZOffset = 0;
double magnetometerRawXOffsetScale = 1, magnetometerRawYOffsetScale = 1, magnetometerRawZOffsetScale = 1;
double magnetometerRawX, magnetometerRawY, magnetometerRawZ; //raw values
double magnetometerHeading = 0, magnetometerLastHeading = 0;

// ----------------------------------------------------------
// HMC5883L METHODS
// ----------------------------------------------------------
/*
 * Setup HMC5883L sensor
 */
void SetupHMC5883L()
{
  //check HMC5883L sensor connected
  uint8_t i2cData[3];
  I2CReadRegister(HMC5883L_I2C_ADDRESS, HMC5883L_REG_IDENTIFICATION_A, i2cData, 3); //read 3 bytes of data starting at HMC5883L_REG_IDENTIFICATION_A register
  if(HMC5883L_REG_IDENTIFICATION_A_ID != i2cData[0] || HMC5883L_REG_IDENTIFICATION_B_ID != i2cData[1]
    || HMC5883L_REG_IDENTIFICATION_C_ID != i2cData[2]) //if not HMC5883L found
  {
    while(true)
    {
      Serial.println(F("HMC5883L sensor not found, check wiring!"));
      delay(1000);
    }
  }
  else
  {
    Serial.println(F("HMC5883L sensor online."));
  }

  //set gain, 1090 LSb/Gauss, 0.92 mG/LSb
  I2CWriteRegister(HMC5883L_I2C_ADDRESS, HMC5883L_REG_CONFIG_B, HMC5883_GAIN_8_1);
  magnetometerSensitivity = HMC5883_GAIN_8_1_SENSITIVITY;
  
  //set sampling rate and nuber of samples, 30 Hz and 8 samples
  I2CWriteRegister(HMC5883L_I2C_ADDRESS, HMC5883L_REG_CONFIG_A, HMC5883_DATA_OUTPUT_RATE_30_00 | HMC5883_NUMBER_SAMPLES_8);
  
  //set mode - continious measuring
  I2CWriteRegister(HMC5883L_I2C_ADDRESS, HMC5883L_REG_MODE, 0x00);
}

/*
 * Read data from HMC5883L sensor (magnetometer values)
 */
void ReadDataHMC5883L()
{
  //RAW VALUES
  //get magentometer data (from continuous registers)
  uint8_t i2cData[6];
  I2CReadRegister(HMC5883L_I2C_ADDRESS, HMC5883L_REG_DATA_OUTPUT_X_H, i2cData, 6); //read 6 bytes of data starting at DATA_OUTPUT_X_H register
  magnetometerRawX = i2cData[0]<<8|i2cData[1];    // 0x03 (DATA_OUTPUT_X_H) & 0x04 (DATA_OUTPUT_X_L)    
  magnetometerRawY = i2cData[4]<<8|i2cData[5];    // 0x07 (DATA_OUTPUT_Y_H) & 0x08 (DATA_OUTPUT_Y_L)
  magnetometerRawZ = i2cData[2]<<8|i2cData[3];    // 0x05 (DATA_OUTPUT_Z_H) & 0x06 (DATA_OUTPUT_Z_L)

  //convert raw values to gauss (scaled)
  magnetometerX = (magnetometerRawX - magnetometerRawXOffset) * magnetometerSensitivity * magnetometerRawXOffsetScale;
  magnetometerY = (magnetometerRawY - magnetometerRawYOffset) * magnetometerSensitivity * magnetometerRawYOffsetScale;
  magnetometerZ = (magnetometerRawZ - magnetometerRawZOffset) * magnetometerSensitivity * magnetometerRawZOffsetScale;
}

/*
 * Calibrate HMC5883L sensor
 * Calibration procedure:
 *  - during 30 seconds slowly rotate sensor around all axes (x, y, z)
 *  - calculate min and max magnetometer values for all axes (minX, maxX, minY, maxY, minZ, maxZ)
 *  - calculate offsets from minimal and maximal values (fix hard iron distorsions)
 *  - calculate scaling values (fix soft iron distorsions)
 */
void CalibrateHMC5883L()
{
  double minX = 0;
  double maxX = 0;
  double minY = 0;
  double maxY = 0;
  double minZ = 0;
  double maxZ = 0; 
  double distanceX = 0;
  double distanceY = 0;
  double distanceZ = 0;
  

  Serial.println(F("HMC5883L calibration started."));
  Serial.println(F("Please rotate the magnometer only one time slowly and precisly in complete circle around each axis in a 30 seconds."));
  //delay(2000);
  Serial.println(F("..."));

  //get current time
  double currTime = millis();

  //first reading (set min and max values)
  ReadDataHMC5883L();
  minX = maxX = magnetometerRawX;
  minY = maxY = magnetometerRawY;
  minZ = maxZ = magnetometerRawZ;

  //make multiple readings
  while (millis() < (currTime + 30000)) //run loop for 30 seconds (non blocking)
  {
    //read raw data
    ReadDataHMC5883L();
    
    //find x and y max and min values
    if (magnetometerRawX < minX) minX = magnetometerRawX;
    if (magnetometerRawX > maxX) maxX = magnetometerRawX;
    if (magnetometerRawY < minY) minY = magnetometerRawY;
    if (magnetometerRawY > maxY) maxY = magnetometerRawY;
    if (magnetometerRawZ < minZ) minZ = magnetometerRawZ;
    if (magnetometerRawZ > maxZ) maxZ = magnetometerRawZ;

/*
    Serial.print(magnetometerRawX);
    Serial.print(F(","));
    Serial.print(magnetometerRawY);
    Serial.print(F(","));
    Serial.println(magnetometerRawZ);
*/    

    delay(10);
  }

  //fix hard iron distorsion
  //calculate offsets (move x and y origin to 0,0)
  magnetometerRawXOffset = (maxX + minX) / 2.0;
  magnetometerRawYOffset = (maxY + minY) / 2.0;
  magnetometerRawZOffset = (maxZ + minZ) / 2.0;

  //fix soft iron distorsion
  //calculate scale factor
  //transform elipse (xy) into circle (xy)
  distanceX = abs(maxX - minX);
  distanceY = abs(maxY - minY);
  distanceZ = abs(maxZ - minZ);

  magnetometerRawXOffsetScale = 1.0; //distanceX/distanceX;
  magnetometerRawYOffsetScale = distanceX/distanceY;
  magnetometerRawZOffsetScale = distanceX/distanceZ;

/*
  Serial.println(F("HMC5883L calibration result:"));
  Serial.print(F("magnetometerRawXOffset = "));
  Serial.print(magnetometerRawXOffset);
  Serial.print(F(", "));
  Serial.print(F("magnetometerRawYOffset = "));
  Serial.print(magnetometerRawYOffset);
  Serial.print(F(", "));
  Serial.print(F("magnetometerRawZOffset = "));
  Serial.println(magnetometerRawZOffset);
  Serial.print(F("magnetometerRawXOffsetScale = "));
  Serial.print(magnetometerRawXOffsetScale);
  Serial.print(F(", "));
  Serial.print(F("magnetometerRawYOffsetScale = "));
  Serial.print(magnetometerRawYOffsetScale);
  Serial.print(F(", "));
  Serial.print(F("magnetometerRawZOffsetScale = "));
  Serial.println(magnetometerRawZOffsetScale);
  Serial.print(F("minX = "));
  Serial.print(minX);
  Serial.print(F(", "));
  Serial.print(F("maxX = "));
  Serial.print(maxX);
  Serial.print(F(", "));
  Serial.print(F("minY = "));
  Serial.print(minY);
  Serial.print(F(", "));
  Serial.print(F("maxY = "));
  Serial.print(maxY);
  Serial.print(F(", "));
  Serial.print(F("minZ = "));
  Serial.print(minZ);
  Serial.print(F(", "));
  Serial.print(F("maxZ = "));
  Serial.println(maxZ);
*/

  Serial.println(F("HMC5883L calibration done."));
}

/*
 * Calculate compass heading (horizontal version)
 */
double HeadingAngleHMC5883L()
{
   return atan2(magnetometerY, magnetometerX);
}

/*
 * Calucalte compass heading (tilted version)
 */
double HeadingAngleTiltCompensatedHMC5883L()
{
  //normalise acceleration data
  double accxnorm = accelerationX/sqrt(accelerationX*accelerationX+accelerationY*accelerationY+accelerationZ*accelerationZ);
  double accynorm = accelerationY/sqrt(accelerationX*accelerationX+accelerationY*accelerationY+accelerationZ*accelerationZ);

  //pitch & roll angles 
  double pitchHeading = asin(-accxnorm);
  double rollHeading = asin(accynorm/cos(pitchHeading));
  
  //algorithm cannot correct for tilt over 40 degrees, if the board is tilted as such, return -1000
  if (rollHeading > 0.78 || rollHeading < -0.78 || pitchHeading > 0.78 || pitchHeading < -0.78)
  {
    return -1000;
  }

  //magnetometer angles
  double cosRoll = cos(rollHeading);
  double sinRoll = sin(rollHeading);  
  double cosPitch = cos(pitchHeading);
  double sinPitch = sin(pitchHeading);
  
  //calculate tilt compensation
  double Xh = (magnetometerX * cosPitch) + (magnetometerZ * sinPitch);
  double Yh = (magnetometerX * sinRoll * sinPitch) + (magnetometerY * cosRoll) - (magnetometerZ * sinRoll * cosPitch);

  //tilt compensated heading
  return atan2(Yh, Xh);
}

double HeadingHMC5883L()
{
  //CALCULATE HEADING
  double headingAngle = 0;
  headingAngle = HeadingAngleTiltCompensatedHMC5883L();

  if(headingAngle != -1000)
  {
    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // Formula: (deg + (min / 60.0)) / RAD_TO_DEG;
    double declinationAngle = (3.0 + (43.0 / 60.0)) / RAD_TO_DEG; //3'43'' (positive) for Velika Gorica
    headingAngle += declinationAngle;
    
    //correct for heading < 0 deg and heading > 360 deg
    if (headingAngle < 0)
    {
      headingAngle += 2 * PI;
    }
    
    if (headingAngle > 2 * PI)
    {
      headingAngle -= 2 * PI;
    }
    
    //convert to degrees
    magnetometerHeading = headingAngle * RAD_TO_DEG;
  
    //save last heading
    magnetometerLastHeading = magnetometerHeading;
  }
  else
  {
    magnetometerHeading = magnetometerLastHeading;
  }

  return magnetometerHeading;

  
/*
  Serial.print(F("MagnetometerX = "));
  Serial.print(magnetometerX);
  Serial.print(F("\tMagnetometerY = "));
  Serial.print(magnetometerY);
  Serial.print(F("\tMagnetometerZ = "));
  Serial.println(magnetometerZ);
*/  
}

