// ------------------------------------------------------------
// MPU-6050 (gyroscope and accelerometer) constants & variables
// ------------------------------------------------------------
//MPU-6050 constants
#define MPU6050_I2C_ADDRESS       0x68       //I2C address of the MPU-6050 (0x68 or 0x69)
#define MPU6050_REG_PWR_MGMT_1    0x6B       //PWR_MGMT_1 register
#define MPU6050_REG_ACCEL_XOUT_H  0x3B       //ACCEL_XOUT_H register
#define MPU6050_REG_GYRO_XOUT_H   0x43       //GYRO_XOUT_H register
#define MPU6050_REG_CONFIG        0x1A       //CONFIG register
#define MPU6050_REG_SMPLRT_DIV    0x19       //SMPLRT_DIV register
#define MPU6050_REG_GYRO_CONFIG   0x1B       //GYRO_CONFIG register
#define MPU6050_REG_ACCEL_CONFIG  0x1C       //ACCEL_CONFIG register
#define MPU6050_REG_WHO_AM_I      0x75       //WHO_AM_I register
#define MPU6050_REG_INT_PIN_CFG   0x37       //INT_PIN_CFG register

//gyro range - sensitivity (deg/sec)
#define MPU6050_GYRO_RANGE_250               0x00
#define MPU6050_GYRO_RANGE_500               0x08
#define MPU6050_GYRO_RANGE_1000              0x10
#define MPU6050_GYRO_RANGE_2000              0x18
#define MPU6050_GYRO_RATE_250_SENSITIVITY    131.0
#define MPU6050_GYRO_RATE_500_SENSITIVITY    65.5
#define MPU6050_GYRO_RATE_1000_SENSITIVITY   32.8
#define MPU6050_GYRO_RATE_2000_SENSITIVITY   16.4

//acceleration range - sensitivity (g)
#define MPU6050_ACCEL_RANGE_2G               0x00
#define MPU6050_ACCEL_RANGE_4G               0x08
#define MPU6050_ACCEL_RANGE_8G               0x10
#define MPU6050_ACCEL_RANGE_16G              0x18
#define MPU6050_ACCEL_RANGE_2G_SENSITIVITY   16384.0
#define MPU6050_ACCEL_RANGE_4G_SENSITIVITY   8192.0
#define MPU6050_ACCEL_RANGE_8G_SENSITIVITY   4096.0
#define MPU6050_ACCEL_RANGE_16G_SENSITIVITY  2048.0

//variables
double gyroSensitivity, accelerometerSensitivity; //gyro and accelerometer sensitivity
double accelerationRawX, accelerationRawY,accelerationRawZ, temperatureRaw, gyroRawX, gyroRawY, gyroRawZ; //raw values
double gyroRawXOffset = 0, gyroRawYOffset = 0, gyroRawZOffset = 0; //gyro calibration offsets
double accelerationRawXOffset = 0, accelerationRawYOffset = 0, accelerationRawZOffset = 0; //accelerometer calibration offsets
double accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ; // degree/sec values
double magnetometerX, magnetometerY, magnetometerZ; //gauss values
//double gyroRawXManualOffset = 0, gyroRawYManualOffset = 0, gyroRawZManualOffset = 0; //gyro calibration manual offsets

//YAW, PICTH, ROLL, TEMPERATURE calculation variables
double accGyroYaw, accGyroPitch, accGyroRoll, accGyroTemperature = 0;

//Madgwick filter variables
double accGyroLastUpdate = 0;                  // used to calculate integration interval
double accGyroNow = 0;                         // used to calculate integration interval

//Complementary filter variables
double gyroAngleX, gyroAngleY, gyroAngleZ =0;
double accAngleX, accAngleY, accAngleZ = 0;
double lastAngleX, lastAngleY, lastAngleZ = 0;
double deltat = 0; 

//gyro rate smoothed
double gyroInputX, gyroInputY, gyroInputZ = 0;

// ----------------------------------------------------------
// MPU-6050 METHODS
// ----------------------------------------------------------
/*
 * Setup MPU-6050 sensor
 */
void SetupMPU6050()
{
  //check is MPU-6050 sensor connected
  uint8_t i2cData[1];
  I2CReadRegister(MPU6050_I2C_ADDRESS, MPU6050_REG_WHO_AM_I, i2cData, 1); //read 1 byte of data starting at WHO_AM_I register
  if(MPU6050_I2C_ADDRESS != i2cData[0]) //if not MPU-6050 found
  {
    while(true)
    {
      Serial.println(F("MPU-6050 sensor not found, check wiring!"));
      delay(1000);
    }
  }
  else
  {
    Serial.println(F("MPU-6050 sensor online."));
  }

  //CONFIGURE MPU-6050 SENSOR
  //power on MPU-6050
  I2CWriteRegister(MPU6050_I2C_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x00);

  //set gyro sensitivity, 2000 deg/s (16.4 LSBs/deg/s)
  I2CWriteRegister(MPU6050_I2C_ADDRESS, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_RANGE_2000);
  gyroSensitivity = MPU6050_GYRO_RATE_2000_SENSITIVITY;

  //set accelerometer sensitivity, 8g (4096 LSB/g)
  I2CWriteRegister(MPU6050_I2C_ADDRESS, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_RANGE_8G);
  accelerometerSensitivity = MPU6050_ACCEL_RANGE_8G_SENSITIVITY;

  //set low pass filter, this lowers gyro sample rate to 1khz, delay 4.9ms equals to 200 Hz sampling rate 
  I2CWriteRegister(MPU6050_I2C_ADDRESS, MPU6050_REG_CONFIG, 0x03);

  //enable reading of slave sensors (needed for HMC5883L magentometer sensor)
  //set I2C Bypass mode - ON 
  I2CWriteRegister(MPU6050_I2C_ADDRESS, MPU6050_REG_INT_PIN_CFG, 0x02);
}

/*
 * Read data from MPU-6050 sensor (acceleration, gyro and temperature values)
 */
void ReadDataMPU6050()
{
  //RAW VALUES
  //get acceleration, gyroscope and temperature data (from continuous registers)
  uint8_t i2cData[14];
  I2CReadRegister(MPU6050_I2C_ADDRESS, MPU6050_REG_ACCEL_XOUT_H, i2cData, 14); //read 14 bytes of data starting at ACCEL_XOUT_H register
  accelerationRawX = i2cData[0]<<8|i2cData[1];    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  accelerationRawY = i2cData[2]<<8|i2cData[3];    // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accelerationRawZ = i2cData[4]<<8|i2cData[5];    // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temperatureRaw = i2cData[6]<<8|i2cData[7];      // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroRawX = i2cData[8]<<8|i2cData[9];            // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroRawY = i2cData[10]<<8|i2cData[11];          // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroRawZ = i2cData[12]<<8|i2cData[13];          // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //convert raw values to scaled values, g - accelerometer, degree/dec - gyro and celsius - temperature
  accelerationX = (accelerationRawX - accelerationRawXOffset) / accelerometerSensitivity;
  accelerationY = (accelerationRawY - accelerationRawYOffset)/ accelerometerSensitivity;
  accelerationZ = (accelerationRawZ - accelerationRawZOffset) / accelerometerSensitivity;
  accGyroTemperature = (temperatureRaw / 340) + 36.53; //convert raw temperature to celisus temperature (from documentation)
  gyroX = (gyroRawX - gyroRawXOffset) / gyroSensitivity;
  gyroY = (gyroRawY - gyroRawYOffset) / gyroSensitivity;
  gyroZ = (gyroRawZ - gyroRawZOffset) / gyroSensitivity;
}

/*
 * Calibrate MPU-6050 sensor
 * Calibration procedure:
 *  - calibration is only needed for gyroscope
 *  - wait 40 seconds for values to stabilize
 *  - take few tousands gyro sample values (for 20 seconds)
 *  - calculate average gyro sample values (offsets)
 *  - subtract manual calculated gyro offsets (if needed)
 */
void CalibrateMPU6050()
{
/*
  //PREVIOUS CALIBRATION DATA
  //gyro
  gyroRawXOffset = -30.77;
  gyroRawYOffset = -5.97; 
  gyroRawZOffset = 4.66;

  //accelerometer
  accelerationRawXOffset = 195.85;
  accelerationRawYOffset = 190.44;
  accelerationRawZOffset = 101.04;
  return;
*/

  int x;
  double gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  double accelerationXSum = 0, accelerationYSum = 0, accelerationZSum = 0;
  int totalSamples = 2000; //total gyro samples to process

  Serial.println(F("MPU-6050 calibration started."));
  Serial.println(F("MPU-6050 should be placed in horizontal position. Don't touch it until you see a finish message."));
  Serial.println(F("..."));
  delay(2000);
  
  //calculate gyro offsets
  for (x = 0; x < totalSamples; x++)
  {
    ReadDataMPU6050();

    //sum gyro values
    gyroXSum += gyroRawX;
    gyroYSum += gyroRawY;
    gyroZSum += gyroRawZ;

    //sum acceleration values
    accelerationXSum += accelerationRawX;
    accelerationYSum += accelerationRawY;
    accelerationZSum += accelerationRawZ;

    if(x % 1000 == 0)
    {
      Serial.println(F("..."));
    }

    delay(3);
  }

  //calculate average gyro values
  gyroRawXOffset = gyroXSum / (double)totalSamples;
  gyroRawYOffset = gyroYSum / (double)totalSamples;
  gyroRawZOffset = gyroZSum / (double)totalSamples;

  //calculate average acceleration values
  accelerationRawXOffset = accelerationXSum / (double)totalSamples;
  accelerationRawYOffset = accelerationYSum / (double)totalSamples;
  accelerationRawZOffset = accelerometerSensitivity - (accelerationZSum / (double)totalSamples);
  
/*
  //add manual offsets
  gyroRawXOffset += gyroRawXManualOffset;
  gyroRawYOffset += gyroRawYManualOffset;
  gyroRawZOffset += gyroRawZManualOffset;
*/

/*
  Serial.println(F("MPU6050 calibration result:"));
  Serial.print(F("gyroRawXOffset = "));
  Serial.print(gyroRawXOffset);
  Serial.print(F(", "));
  Serial.print(F("gyroRawYOffset = "));
  Serial.print(gyroRawYOffset);
  Serial.print(F(", "));
  Serial.print(F("gyroRawZOffset = "));
  Serial.println(gyroRawZOffset);
  Serial.print(F("accelerationRawXOffset = "));
  Serial.print(accelerationRawXOffset);
  Serial.print(F(", "));
  Serial.print(F("accelerationRawYOffset = "));
  Serial.print(accelerationRawYOffset);
  Serial.print(F(", "));
  Serial.print(F("accelerationRawZOffset = "));
  Serial.println(accelerationRawZOffset);
*/

  Serial.println(F("MPU-6050 calibration done."));
} 

/*
 * Calculate gyro rate for all axes
*/
void YPRGyroRate(double *yprtArray)
{
  //smooth gyro readings
  gyroInputX = gyroInputX * 0.8 + gyroX * 0.2;
  gyroInputY = gyroInputY * 0.8 + gyroY * 0.2;
  gyroInputZ = gyroInputZ * 0.8 + gyroZ * 0.2;
  
  //set array values
  yprtArray[0] = gyroInputZ;
  yprtArray[1] = gyroInputY;
  yprtArray[2] = gyroInputX;
  yprtArray[3] = accGyroTemperature;


/*
  Serial.print(F("GyroX = "));
  Serial.print(gyroInputX);
  Serial.print(F("\tGyroY = "));
  Serial.print(gyroInputY);
  Serial.print(F("\tGyroZ = "));
  Serial.print(gyroInputZ);
  Serial.print(F("\tTemperature = "));
  Serial.println(accGyroTemperature);
*/
}

/*
 * Calculate yaw, pitch & roll in degrees using complementary filter
*/
void YPRGyroAccComplementary(double *yprtArray)
{
  //CALCULATE YAW, PITCH, ROLL & TEMPERATURE
  accGyroNow = micros();
  deltat = ((accGyroNow - accGyroLastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  accGyroLastUpdate = accGyroNow;

  //acceleration angles
  accAngleX = atan2(accelerationY, accelerationZ) * RAD_TO_DEG;
  accAngleY = -1 * atan2(accelerationX, accelerationZ) * RAD_TO_DEG;
  accAngleZ = 0;

  //gyro angles (gyro * dt plus last complementary angle)
  gyroAngleX = gyroX * deltat + lastAngleX;
  gyroAngleY = gyroY * deltat + lastAngleY;
  gyroAngleZ = gyroZ * deltat + lastAngleZ;
  
  //final measurements
  accGyroPitch = 0.99 * gyroAngleX + 0.01 * accAngleX;
  accGyroRoll = 0.99 * gyroAngleY + 0.01 * accAngleY;
  accGyroYaw = gyroAngleZ;

  //constraint values
  accGyroPitch = constrain(accGyroPitch, -90, 90);
  accGyroRoll = constrain(accGyroRoll, -180, 180);
  accGyroYaw = constrain(accGyroYaw, -180, 180);

  //save values
  lastAngleX = accGyroPitch;
  lastAngleY = accGyroRoll;
  lastAngleZ = accGyroYaw;

  //set array values
  yprtArray[0] = accGyroYaw;
  yprtArray[1] = accGyroPitch;
  yprtArray[2] = accGyroRoll;
  yprtArray[3] = accGyroTemperature;


/*
  Serial.print(F("GX = "));
  Serial.print(gyroAngleX);
  Serial.print(F("\tGY = "));
  Serial.print(gyroAngleY);
  Serial.print(F("\tGZ = "));
  Serial.print(gyroAngleZ);
  Serial.print(F("\tAX = "));
  Serial.print(accAngleX);
  Serial.print(F("\tAgY = "));
  Serial.print(accAngleY);
  Serial.print(F("\tAZ = "));
  Serial.print(accAngleZ);
  Serial.print(F("Pitch = "));
  Serial.print(accGyroPitch);
  Serial.print(F("\tRoll = "));
  Serial.print(accGyroRoll);
  Serial.print(F("\tYaw = "));
  Serial.println(accGyroYaw);
*/
}
