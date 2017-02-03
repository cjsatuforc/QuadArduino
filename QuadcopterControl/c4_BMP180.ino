// ----------------------------------------------------------
// BMP180 (barometer) constants & variables
// ----------------------------------------------------------
//BMP180 constants
#define BMP180_I2C_ADDRESS          0x77      //I2C address of the BMP180
#define BMP180_REG_CHIP_ID          0xD0      //CHIP_ID register (who am i)
#define BMP180_REG_CHIP_ID_VALUE    0x55      //CHIP_ID value

//calibration registers
#define BMP180_REG_CALIBRATION_AC1_H          0xAA //AC1_H register
#define BMP180_REG_CALIBRATION_AC1_L          0xAB //AC1_L register
#define BMP180_REG_CALIBRATION_AC2_H          0xAC //AC2_H register
#define BMP180_REG_CALIBRATION_AC3_H          0xAE //AC3_H register
#define BMP180_REG_CALIBRATION_AC4_H          0xB0 //AC4_H register
#define BMP180_REG_CALIBRATION_AC5_H          0xB2 //AC5_H register
#define BMP180_REG_CALIBRATION_AC6_H          0xB4 //AC6_H register
#define BMP180_REG_CALIBRATION_B1_H           0xB6 //B1_H register
#define BMP180_REG_CALIBRATION_B2_H           0xB8 //B2_H register
#define BMP180_REG_CALIBRATION_MB_H           0xBA //MB_H register
#define BMP180_REG_CALIBRATION_MC_H           0xBC //MC_H register
#define BMP180_REG_CALIBRATION_MD_H           0xBE //MD_H register

//control & data registers
#define BMP180_REG_MEASUREMENT_CONTROL        0xF4 //MEASUREMENT_CONTROL register
#define BMP180_REG_MEASUREMENT_DATA_H         0xF6 //DATA_H register
#define BMP180_REG_MEASUREMENT_DATA_L         0xF7 //DATA_L register
#define BMP180_REG_MEASUREMENT_DATA_XL        0xF8 //DATA_XL register


//oversampling - number of samples averaged (1 to 8)
#define BMP180_OVERSAMPLING_RATIO_1           0  //1 sample
#define BMP180_OVERSAMPLING_RATIO_2           1  //2 samples
#define BMP180_OVERSAMPLING_RATIO_4           2  //4 samples
#define BMP180_OVERSAMPLING_RATIO_8           3  //8 samples
#define BMP180_OVERSAMPLING_RATIO_1_VALUE     0x00
#define BMP180_OVERSAMPLING_RATIO_2_VALUE     0x40
#define BMP180_OVERSAMPLING_RATIO_4_VALUE     0x80
#define BMP180_OVERSAMPLING_RATIO_8_VALUE     0xC0


#define BMP180_SEA_PRESSURE   1013.25f  //sea pressure in hPa

//variables
int _AC1, _AC2, _AC3, _B1, _B2, _MB, _MC, _MD; //calibration data
unsigned int _AC4, _AC5, _AC6; //calibration data
long X1, X2, _B5; //extra calibration data
long UT, T, UP, P; //uncompensated and compensated temperature & pressure

int barometerOversamplingRate, barometerOversamplingRateSensitivity;
double barometerTemperature, barometerPressure; //temperature in C, pressure in hPa
double barometerAltitude = 0, barometerInitialPressure = 0;

// ----------------------------------------------------------
// BMP180 METHODS
// ----------------------------------------------------------
/*
 * Setup BMP180 sensor
 */
void SetupBMP180()
{
  //check is BMP180 sensor connected
  uint8_t i2cData[1];
  I2CReadRegister(BMP180_I2C_ADDRESS, BMP180_REG_CHIP_ID, i2cData, 1); //read 1 byte of data starting at CHIP_ID register
  if(BMP180_REG_CHIP_ID_VALUE != i2cData[0]) //if not BMP180 found
  {
    while(true)
    {
      Serial.println(F("BMP180 sensor not found, check wiring!"));
      delay(1000);
    }
  }
  else
  {
    Serial.println(F("BMP180 sensor online."));
  }

  //CONFIGURE BMP180 SENSOR
  //set oversampling rate
  barometerOversamplingRate = BMP180_OVERSAMPLING_RATIO_8;
  barometerOversamplingRateSensitivity = BMP180_OVERSAMPLING_RATIO_8_VALUE;
}

/*
 * Calibrate BMP180
 *  Sensor is initialy calibrated, we only must read calibration data and use it later in readings
 */
void CalibrateBMP180()
{
  Serial.println(F("BMP180 calibration started."));
  
  //read calibration registers (from 0xAA to 0xBC)
  uint8_t i2cData[22];
  I2CReadRegister(BMP180_I2C_ADDRESS, BMP180_REG_CALIBRATION_AC1_H, i2cData, 22); //read 22 bytes of data starting at AC1_H register
  
  _AC1 = i2cData[0]<<8|i2cData[1];   // 0xAA (AC1_H) & 0xAB (AC1_L)
  _AC2 = i2cData[2]<<8|i2cData[3];   // 0xAC (AC2_H) & 0xAD (AC2_L)
  _AC3 = i2cData[4]<<8|i2cData[5];   // 0xAE (AC3_H) & 0xAF (AC3_L)
  _AC4 = i2cData[6]<<8|i2cData[7];   // 0xB0 (AC4_H) & 0xB1 (AC4_L)
  _AC5 = i2cData[8]<<8|i2cData[9];   // 0xB2 (AC5_H) & 0xB3 (AC5_L)
  _AC6 = i2cData[10]<<8|i2cData[11]; // 0xB4 (AC6_H) & 0xB5 (AC6_L)
  _B1 = i2cData[12]<<8|i2cData[13];  // 0xB6 (B1_H) & 0xB7 (B1_L)
  _B2 = i2cData[14]<<8|i2cData[15];  // 0xB8 (B2_H) & 0xB9 (B2_L)
  _MB = i2cData[16]<<8|i2cData[17];  // 0xBA (MB_H) & 0xBB (MB_L)
  _MC = i2cData[18]<<8|i2cData[19];  // 0xBC (MC_H) & 0xBD (MC_L)
  _MD = i2cData[20]<<8|i2cData[21];  // 0xBE (MD_H) & 0xBF (MD_L)

/*
  Serial.print(F("AC1 ="));
  Serial.println(_AC1);
  Serial.print(F("AC2 ="));
  Serial.println(_AC2);
  Serial.print(F("AC3 ="));
  Serial.println(_AC3);
  Serial.print(F("AC4 ="));
  Serial.println(_AC4);
  Serial.print(F("AC5 ="));
  Serial.println(_AC5);
  Serial.print(F("AC6 ="));
  Serial.println(_AC6);
  Serial.print(F("B1 ="));
  Serial.println(_B1);
  Serial.print(F("B2 ="));
  Serial.println(_B2);
  Serial.print(F("MB ="));
  Serial.println(_MB);
  Serial.print(F("MC ="));
  Serial.println(_MC);
  Serial.print(F("MD ="));
  Serial.println(_MD);
*/

  //get initial pressure
  GetTemperaturePressureBMP180();
  barometerInitialPressure = barometerPressure;
  
  Serial.println(F("BMP180 calibration done."));
}

/*
 * Read data from BMP180 (temperature in C and pressure in hPa)
 */
void GetTemperaturePressureBMP180()
{
  //RAW TEMPERATURE
  //send measure temperature command
  I2CWriteRegister(BMP180_I2C_ADDRESS, BMP180_REG_MEASUREMENT_CONTROL, 0x2E);

  delay(5); //wait 5 ms before reading register

  //get uncompensated temperature
  uint8_t i2cDataTemp[2];
  I2CReadRegister(BMP180_I2C_ADDRESS, BMP180_REG_MEASUREMENT_DATA_H, i2cDataTemp, 2); //read 3 bytes of data starting at MEASUREMENT_DATA_H register
  UT = i2cDataTemp[0]<<8|i2cDataTemp[1];

  //RAW PRESSURE
  //send measure pressure command
  I2CWriteRegister(BMP180_I2C_ADDRESS, BMP180_REG_MEASUREMENT_CONTROL, (0x34 | barometerOversamplingRateSensitivity));

  //delay depending on oversampling rate
  switch (barometerOversamplingRate) 
  {
    case 0: delay(5); break;
    case 1: delay(8); break;
    case 2: delay(14); break;
    case 3: delay(26); break;
  }

  //get uncompensated pressure
  uint8_t i2cDataPressure[3];
  I2CReadRegister(BMP180_I2C_ADDRESS, BMP180_REG_MEASUREMENT_DATA_H, i2cDataPressure, 3); //read 3 bytes of data starting at MEASUREMENT_DATA_H register
  unsigned long UP = ((unsigned long)i2cDataPressure[0]<<16|(unsigned long)i2cDataPressure[1]<<8|(unsigned long)i2cDataPressure[2]) >> (8 - barometerOversamplingRate);

/*
  Serial.print(F"UT = "));
  Serial.println(UT);
  Serial.print(F("UP = "));
  Serial.println(UP);
*/
 
  //CALCULATE TEMPERATURE
  //calculate true temperature (compensated)
  X1 = ((long)UT - (long)_AC6) * ((long)_AC5 / pow(2,15));
  X2 = ((long)_MC * pow(2,11)) / (X1 + _MD);
  _B5 = X1 + X2;
  T = (_B5 + 8) / pow(2,4); //temperature in 0.1 C

/*
  X1 = (UT - _AC6) * (_AC5 >> 15);
  X2 = (_MC << 11) / (X1 + _MD);
  _B5 = X1 + X2;
  T = (_B5 + 8) >> 4; //temperature in 0.1 C
*/

  barometerTemperature = T / 10.0f; //temperature in C

/*
//TEST
  _AC1 = 408;
  _AC2 = -72;
  _AC3 = -14383;
  _AC4 = 32741;
  _AC5 = 32757;
  _AC6 = 23153;
  _B1 = 6190;
  _B2 = 4;
  _MB = -32768;
  _MC = -8711;
  _MD = 2868;
  UT = 27898;

  X1 = 4743;
  X2 = -2344;
  _B5 = 2399;
  UP = 23843;
*/

  //CALCULATE PRESSURE
  //calculate true pressure (compensated)
  long B6, X1, X2, X3, B3;
  unsigned long B4, B7;
  
  B6 = _B5 - 4000;
  X1 = (_B2 * ((B6 * B6) >> 12)) >> 11;
  X2 = (_AC2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = (((((long)_AC1 * 4) + X3) << barometerOversamplingRate) + 2) >> 2;
  X1 = (_AC3 * B6) >> 13;
  X2 = (_B1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = (unsigned long)_AC4 * (unsigned long)(X3 + 32768) >> 15;
  B7 = ((unsigned long)UP - B3) * (50000 >> barometerOversamplingRate);
  
  if (B7 < 0x80000000)
  {
    P = (B7 << 1) / B4;
  }
  else
  {
    P = (B7 / B4) << 1;
  }
  
  X1 = (P >> 8) * (P >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * P) >> 16;
  P = P + ((X1 + X2 + 3791) >> 4); //pressure in Pa

  /*  
  B6 = _B5 - 4000;
  X1 = (_B2 * ((B6 * B6) / pow(2,12))) / pow(2,11);
  X2 = (_AC2 * B6) / pow(2,11);
  X3 = X1 + X2;
  B3 = ((((_AC1 * 4) + X3) << barometerOversamplingRate) + 2) / 4;
  X1 = (_AC3 * B6) / pow(2,13);
  X2 = (_B1 * ((B6 * B6) / pow(2,12))) / pow(2,16);
  X3 = ((X1 + X2) + 2) / 4;
  B4 = (unsigned long)_AC4 * (unsigned long)(X3 + 32768) / pow(2,15);
  B7 = ((unsigned long)UP - B3) * (50000 >> barometerOversamplingRate);
  
  if (B7 < 0x80000000)
  {
    P = (B7 * 2) / B4;
  }
  else
  {
    P = (B7 / B4) * 2;
  }
  
  X1 = (P / pow(2,8)) * (P / pow(2,8));
  X1 = (X1 * 3038) / pow(2,16);
  X2 = (-7357 * P) / pow(2,16);
  P = P + ((X1 + X2 + 3791) / pow(2,4)); //pressure in Pa
*/

  barometerPressure = P / 100.0f; //pressure in hPa
}
/*
 * Calculate apsolute altitude (altitude from start position, in meters)
 */
double AltitudeAbsolute()
{
  return (44330.0*(1-pow(barometerPressure/barometerInitialPressure,1/5.255)));
}

/*
 * Calculate altitude above sea level (in meters)
 */
double AltitudeAboveSeaLevel()
{
  return (44330*(1- pow((barometerPressure/BMP180_SEA_PRESSURE),0.190294496)));
}

/*
 * Calculate altitude (in meters)
 */
void AtitudeTemperaturePressureBMP180(double *atpArray)
{
  //CALCULATE ALTITUDE 
  barometerAltitude = AltitudeAbsolute();
  
  //return temperature, pressure & altitude
  atpArray[0] = barometerAltitude;
  atpArray[1] = barometerTemperature;
  atpArray[2] = barometerPressure;
}

