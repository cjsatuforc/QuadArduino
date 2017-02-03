#include <Wire.h> //I2C communication - for sensors

// ----------------------------------------------------------
// I2C COMMUNICATION HELPER METHODS
// ----------------------------------------------------------

/*
 * Initialize I2C communication
 */
void I2CInit()
{
  Wire.begin(); //start I2C communication
  
  //TWBR = 12;    //set 400kHz I2C clock
  TWBR = ((F_CPU / 400000) - 16) / 2; //set 400kHz I2C clock
}

/* 
 *  Write to device register
 *   mpu_address - device address
 *   mpu_register = register to write to
 *   data - data
 */
void I2CWriteRegister(int mpu_address, int mpu_register, uint8_t data)
{
  Wire.beginTransmission(mpu_address);
  Wire.write(mpu_register); 
  Wire.write(data);    
  Wire.endTransmission(true);
}

/*
 * Read from device register
 *  mpu_address - device address
 *  mpu_register = register to read from
 *  data - reference to data pointer that will hold data after read
 *  data_size - number of bytes to read from register
 */
void I2CReadRegister(int mpu_address, int mpu_register, uint8_t *data, int data_size)
{
  Wire.beginTransmission(mpu_address);
  Wire.write(mpu_register);  //registar to read from
  Wire.endTransmission(false);

  //read data
  Wire.requestFrom(mpu_address, data_size, true);
  int i = 0;
  while(Wire.available() && i < data_size)
  {
    data[i++] = Wire.read();
  }
}

/*
 * Test I2C connection with device
 *  mpu_address - device address
 * Returns: true if device exists
 */
bool I2CTestConnection(int mpu_address)
{
  bool result = false;

  //request data (1 byte)
  Wire.requestFrom(mpu_address, 1);

  //if device answers
  if(Wire.available())
  {
    result = true;
  }
  
  return result;
}
