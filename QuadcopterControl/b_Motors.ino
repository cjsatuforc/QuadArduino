/* Motors
 *
 * Arduino Uno:
 *    Has 6 PWM pins - D3, D5, D6, D9, D10, D11
 *    Pins D5 and D6 operate at frequency 980 Hz, controlled by Timer0 (used for system timing function such as delay(), millis(), ...)
 *    Pins D9 and D10 operate at frequency 490 HZ, controlled by Timer1 (used by Servo lib)
 *    Pins D3 and D11 operate at frequency 490 HZ, controlled by Timer2 (used by tone() function)
 *    We use pins D3, D9, D10 and D11 becuse their frequency (490 Hz) is close to ESC control frequency of 400 Hz and they have separate timers 
 *    that are not use by system timing functions
 *
 */

//MOTOR constraints
#define MOTOR_1_PIN           3     //front-left motor pin
#define MOTOR_2_PIN           10    //front-right motor pin
#define MOTOR_3_PIN           9     //back-right motor pin
#define MOTOR_4_PIN           11    //back-left motor pin

#define ESC_MIN_PULSE         1000  //esc minimum pulse length (in ms)
#define ESC_MAX_PULSE         2000  //esc maximum pulse length (in ms)
#define MOTOR_ZERO_SPEED      1000  //motors minimum speed (stop)
#define MOTOR_MAX_SPEED       2000  //motors maximum speed (full power)
#define MOTOR_MAX_SAFE_SPEED  1800  //motor maximum safety speed (full power is not recommended for longer period of time)

#define ANALOG_ZERO_SPEED     125   //analog write - zero motor speed (stop)
#define ANALOG_MIN_SPEED      125   //analog write - minumum motor speed (stop)
#define ANALOG_MAX_SPEED      254   //analog write - maximum motor speed (full power)

/*
 * Init motors - attach and initialize escs
 */
void InitMotors()
{
  pinMode(MOTOR_1_PIN,OUTPUT); 
  pinMode(MOTOR_2_PIN,OUTPUT);
  pinMode(MOTOR_3_PIN,OUTPUT);
  pinMode(MOTOR_4_PIN,OUTPUT);
  
  analogWrite(MOTOR_1_PIN, ANALOG_ZERO_SPEED);
  analogWrite(MOTOR_2_PIN, ANALOG_ZERO_SPEED);
  analogWrite(MOTOR_3_PIN, ANALOG_ZERO_SPEED);
  analogWrite(MOTOR_4_PIN, ANALOG_ZERO_SPEED);
  delay(3000);
}

/*
 * Set motors speed
 */
void SetMotorSpeeds(int m1, int m2, int m3, int m4)
{
  //constrain speed
  m1 = constrain(m1, MOTOR_ZERO_SPEED, MOTOR_MAX_SAFE_SPEED);
  m2 = constrain(m2, MOTOR_ZERO_SPEED, MOTOR_MAX_SAFE_SPEED);
  m3 = constrain(m3, MOTOR_ZERO_SPEED, MOTOR_MAX_SAFE_SPEED);
  m4 = constrain(m4, MOTOR_ZERO_SPEED, MOTOR_MAX_SAFE_SPEED);

  m1Speed = m1;
  m2Speed = m2;
  m3Speed = m3;
  m4Speed = m4;
 
  analogWrite(MOTOR_1_PIN, map(m1, MOTOR_ZERO_SPEED, MOTOR_MAX_SPEED, ANALOG_MIN_SPEED, ANALOG_MAX_SPEED));
  analogWrite(MOTOR_2_PIN, map(m2, MOTOR_ZERO_SPEED, MOTOR_MAX_SPEED, ANALOG_MIN_SPEED, ANALOG_MAX_SPEED));
  analogWrite(MOTOR_3_PIN, map(m3, MOTOR_ZERO_SPEED, MOTOR_MAX_SPEED, ANALOG_MIN_SPEED, ANALOG_MAX_SPEED));
  analogWrite(MOTOR_4_PIN, map(m4, MOTOR_ZERO_SPEED, MOTOR_MAX_SPEED, ANALOG_MIN_SPEED, ANALOG_MAX_SPEED));
}

/*
 * Stop all motors - set all motors speed to zero
 */
void StopAllMotors()
{
  m1Speed = m2Speed = m3Speed = m4Speed = MOTOR_ZERO_SPEED;
  
  SetMotorSpeeds(MOTOR_ZERO_SPEED, MOTOR_ZERO_SPEED, MOTOR_ZERO_SPEED, MOTOR_ZERO_SPEED);
}


