/* GY-87 sensor (10 DOF sensor)
 *
 * Connecting to Arduino Uno
 * Arduino Uno   GY-87
 *  A4            SDA
 *  A5            SCL
 *  5V            VCC
 *  GND           GND
 *
 *
 *  Default at power-up:
 *  MPU-6050:
 *    The device is in sleep mode
 *    Gyro at 250 degrees second
 *    Acceleration at 2g
 *    Clock source at internal 8MHz
 *  HMC5883L:
 *    The device is in single-measurement mode
 *    Sensitivity +- 1.3 Ga
 *    Data sampling rate at 15 Hz
 *  BMP180:
 *    Device take measurement in single-measurement mode only
 *    At startup it is required to read calibration registers
 *    Before reading pressure you must read temperature from sensor
 *
 */

// ------------------------------------------------------------
// GY-87 (10DOF sensor) METHODS
// ------------------------------------------------------------
void SetupGY87()
{


  //SETUP MPU6050 sensor
  SetupMPU6050();

  //SETUP HMC5883L sensor
  //SetupHMC5883L();

  //SETUP BMP180 sensor
  //SetupBMP180();

  //CALIBRATE
  CalibrateMPU6050();
  //CalibrateHMC5883L();
  //CalibrateBMP180();
}

void GyroRate()
{
  double yprt[4];
  YPRGyroRate(yprt);

  //pitch & roll are switched because sensor forward axis is y, z axis is inverse
  gyroInputYaw = -1 * yprt[0];
  gyroInputPitch = yprt[2];
  gyroInputRoll = yprt[1];
  temperature = yprt[3];
}

void YawPitchRollTemp()
{
  //CALCULATE YAW, PITCH, ROLL & TEMPERATURE
  double yprt[4];
  //YPRGyroOnly(yprt);
  YPRGyroAccComplementary(yprt);

  //pitch & roll are switched because sensor forward axis is y
  yaw = floor(yprt[0]);
  pitch = floor(yprt[1]);
  roll = floor(yprt[2]);
  temperature = floor(yprt[3]);
}

void HeadingAltitude()
{
  //CALCULATE COMPASS HEADING
  heading = floor(HeadingHMC5883L());

  //CALCULATE ALTITUDE, TEMPERATURE & PRESSURE
  double atp[3];
  AtitudeTemperaturePressureBMP180(atp);
  altitudeBarometer = atp[0];
  temperatureBarometer = atp[1];
  pressure = atp[2];
}
