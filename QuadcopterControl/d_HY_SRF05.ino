/* HY-SRF05 ultrasonic sensor (distance measuring sensor)
 * 
 * Connecting to Arduino Uno
 * Arduino Uno   HY-SRF05
 *  D12           TRIGG
 *  D13           ECHO
 *  5V            VCC
 *  GND           GND
 *  
 *  HC-SR04 specs:
 *    - min range:   2 cm
 *    - max range: 400 cm
 *    - accuracy:  0.3 cm
 *    - working frequency: 40 Hz
 *    - measuring angle: 15 degrees
 *    
 *  Distance formulas:
 *    1 cm = 58 us 
 *    1 inch = 148 us
 *    sound speed = 340 m/s = 0.034 cm/us
 *    distance = ping time * sound speed / 2;
*/
// ------------------------------------------------------------
// HY-SRF05 (ultrasonic sensor) constants & variables
// ------------------------------------------------------------
#define HYSRF05_TRIGGER_PIN  12     //trigger pin
#define HYSRF05_ECHO_PIN     13     //echo pin
#define HYSRF05_MAX_DISTANCE 300    //max distance (in cm)

int ultrasonicPingTimeout = 0;      //ping timeout (in us)

// ------------------------------------------------------------
// HY-SRF05 (ultrasonic sensor) METHODS
// ------------------------------------------------------------
/*
 * Setup HC-SR04 sensor
 */
void SetupHYSRF05() 
{
  Serial.println(F("HY-SRF05 sensor online."));

  //set pins
  pinMode(HYSRF05_TRIGGER_PIN, OUTPUT);
  pinMode(HYSRF05_ECHO_PIN, INPUT);

  //caluculate ping timeout (in us)
  ultrasonicPingTimeout = HYSRF05_MAX_DISTANCE * 2 * 29;
}

/*
 * Read measured distance from HC-SR04 sensor (in centimeters)
 */
int AltitudeCentimeterHYSRF05(double temperature) 
{
  //send pulse
  digitalWrite(HYSRF05_TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(HYSRF05_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(HYSRF05_TRIGGER_PIN, LOW);

  //get pulse duration
  long duration = pulseIn(HYSRF05_ECHO_PIN, HIGH, ultrasonicPingTimeout);
  //if(duration == 0)
  //{
  //      duration = pingTimeout;
  //}

  //calculate distance
  long distance = floor(duration * 0.034 / 2.);

/*
  Serial.println(F("Distance = "));
  Serial.print(distance);
  Serial.print(F(" cm"));
*/

  return distance;
}

