#include <NewPing.h>

// Basic plan:
//  Circlebot has a Ping ultrasonic range sensor, a 3 axis gyro sensor, and a motor driver chip.
//
//  Gyro is a Parallax 3-Axis Gyroscope Module (L3G4200D)
//  The Parallax Ping is on pin 9.
//  The motor driver is a SN754410 on pins 5 through 8.
//
//  DriveForward State
//    Circlebot will drive forward for 5 seconds.
//    Go to Sense State
//
//  Sense State
//    Circlebot will turn left in a full circle, using the Ping sensor to see which way is best.
//    After completing a circle, Circlebot will turn right or left as best to point in the
//    target direction.
//    Go to Sense State
//
#include <NewPing.h>
#include <Wire.h>

const int sonarPin = 9;

const int motorRightSpeedPin=8;
const int motorRightDirPin=7;
const int motorLeftSpeedPin=6;
const int motorLeftDirPin=5;

#define  CTRL_REG1  0x203
#define  CTRL_REG2  0x21
#define  CTRL_REG3  0x22
#define  CTRL_REG4  0x23
#define  CTRL_REG5  0x24
#define  CTRL_REG6  0x25

int gyroI2CAddr=105;

int gyroRaw[3];                         // raw sensor data, each axis, pretty useless really but here it is.
double gyroDPS[3];                      // gyro degrees per second, each axis

float heading[3]={0.0f};                // heading[x], heading[y], heading [z]
float quaternion[4]={1.0f,0.0f,0.0f,0.0f};
float euler[3]={0.0f};

int gyroZeroRate[3];                    // Calibration data.  Needed because the sensor does center at zero, but rather always reports a small amount of rotation on each axis.
int gyroThreshold[3];                   // Raw rate change data less than the statistically derived threshold is discarded.

// Probably should be using a filter instead of discarding samples
#define  NUM_GYRO_SAMPLES  50           // As recommended in STMicro doc
#define  GYRO_SIGMA_MULTIPLE  3         // As recommended 

float dpsPerDigit=.00875f;              // for conversion to degrees per second
#define MAX_DISTANCE 600 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(sonarPin, sonarPin, MAX_DISTANCE); // NewPing setup of pin and maximum distance.

unsigned long pingTimer;
#define PING_INTERVAL 33  // Milliseconds between pings

void setup() {
  Serial.begin(115200);       // Open serial monitor at 115200 baud

  pinMode(motorRightSpeedPin,OUTPUT);
  pinMode(motorRightDirPin,OUTPUT);
  pinMode(motorLeftSpeedPin,OUTPUT);
  pinMode(motorLeftDirPin,OUTPUT);

  pingTimer=millis()+75+PING_INTERVAL;
  
  Wire.begin();
  setupGyro();
  calibrateGyro();
}

void loop() {
  updateGyroValues();
  updateHeadings();
  
/*  printQuaternion();
  Serial.print("    ->    ");
  printEuler();
  Serial.println();
*/
  if (millis() >=pingTimer)
  {
    Serial.print("Sonar result: ");
    Serial.println(sonar.ping()/US_ROUNDTRIP_CM);
  }

/*
      Serial.println("Motors Fast");
      digitalWrite(motorLeftDirPin,LOW);
      analogWrite(motorLeftSpeedPin,255);
      digitalWrite(motorRightDirPin,LOW);
      analogWrite(motorRightSpeedPin, 255);
      delay(1000);

      
      Serial.println("Motors Medium");
      digitalWrite(motorLeftDirPin,LOW);
      analogWrite(motorLeftSpeedPin,128);
      digitalWrite(motorRightDirPin,LOW);
      analogWrite(motorRightSpeedPin, 128);
      delay(1000);
      
      Serial.println("Motors Slow");
      digitalWrite(motorLeftDirPin,LOW);
      analogWrite(motorLeftSpeedPin,50);
      digitalWrite(motorRightDirPin,LOW);
      analogWrite(motorRightSpeedPin, 50);
      delay(1000);

      Serial.println("Motors Stop");
      digitalWrite(motorLeftDirPin,LOW);
      analogWrite(motorLeftSpeedPin,0);
      digitalWrite(motorRightDirPin,LOW);
      analogWrite(motorRightSpeedPin, 0);
      delay(2000);

      Serial.println("Motors Reverse Fast");
      digitalWrite(motorLeftDirPin,HIGH);
      analogWrite(motorLeftSpeedPin,0);
      digitalWrite(motorRightDirPin,HIGH);
      analogWrite(motorRightSpeedPin, 0);
      delay(1000);

      Serial.println("Motors Reverse Slow");
      digitalWrite(motorLeftDirPin,HIGH);
      analogWrite(motorLeftSpeedPin,128);
      digitalWrite(motorRightDirPin,HIGH);
      analogWrite(motorRightSpeedPin, 128);
      delay(1000);

      delay(2000);
      
      Serial.println("Motors Opposite Fast");
      digitalWrite(motorLeftDirPin,HIGH);
      analogWrite(motorLeftSpeedPin,0);
      digitalWrite(motorRightDirPin,LOW);
      analogWrite(motorRightSpeedPin, 255);
      delay(1000);

      Serial.println("Motors Opposite Slow");
      digitalWrite(motorLeftDirPin,HIGH);
      analogWrite(motorLeftSpeedPin,128);
      digitalWrite(motorRightDirPin,LOW);
      analogWrite(motorRightSpeedPin, 128);
      delay(1000);

      delay(2000);
      Serial.println("Motors Opposite Slow");
      digitalWrite(motorLeftDirPin,LOW);
      analogWrite(motorLeftSpeedPin,128);
      digitalWrite(motorRightDirPin,HIGH);
      analogWrite(motorRightSpeedPin, 128);
      delay(1000);
      
      delay (3000);
 */
}

void printQuaternion()
{
  Serial.print("Quat W: ");
  Serial.print(quaternion[0]);
  Serial.print("  Quat X: ");
  Serial.print(quaternion[1]);
  Serial.print("  Quat Y: ");
  Serial.print(quaternion[2]);
  Serial.print("  Quat Z: ");
  Serial.print(quaternion[3]);
}

void printEuler()
{
  Serial.print("Euler R: ");
  Serial.print(euler[0]);
  Serial.print("  P: ");
  Serial.print(euler[1]);
  Serial.print("  Y: ");
  Serial.print(euler[2]);
}

void printDPS()
{
  Serial.print("DPS X: ");
  Serial.print(gyroDPS[0]);
  Serial.print("  Y: ");
  Serial.print(gyroDPS[1]);
  Serial.print("  Z: ");
  Serial.print(gyroDPS[2]);
}

void printHeadings()
{
  Serial.print("Heading X: ");
  Serial.print(heading[0]);
  Serial.print("  Y: ");
  Serial.print(heading[1]);
  Serial.print("  Z: ");
  Serial.print(heading[2]);
}

float safe_asin(float v)
{
        if (isnan(v)) {
                return 0;
        }
        if (v >= 1.0) {
                return PI/2;
        }
        if (v <= -1.0) {
                return -PI/2;
        }
        return asin(v);
}

void updateHeadings()
{
    
  float deltaT=getDeltaTMicros();

  double gyroDPSDelta[3];

  for (int j=0;j<3;j++)
    {
      gyroDPSDelta[j]=(gyroDPS[j]*deltaT)/1000000.0;
      heading[j] -= gyroDPSDelta[j];
    }
    
  // get radians per second
  // z is yaw
  // y is pitch
  // x is bank
  double rps[3]={0.0};
  for (int j=0;j<3;j++)
    rps[j]=(gyroDPSDelta[j]/57.2957795);
  
  // convert the radians per second to a quaternion 
  double c1 = cos(rps[0]/2);
  double s1 = sin(rps[0]/2);
  double c2 = cos(rps[1]/2);
  double s2 = sin(rps[1]/2);
  double c3 = cos(rps[2]/2);
  double s3 = sin(rps[2]/2);
  double c1c2 = c1*c2;
  double s1s2 = s1*s2;
  double quatDelta[4];
  quatDelta[0] =c1c2*c3 - s1s2*s3;
  quatDelta[1] =c1c2*s3 + s1s2*c3;
  quatDelta[2] =s1*c2*c3 + c1*s2*s3;
  quatDelta[3] =c1*s2*c3 - s1*c2*s3;
  
  // now we just multiply quaternion by quatDelta
  float newQuat[4];
  newQuat[0]=(quaternion[0]*quatDelta[0] - quaternion[1]*quatDelta[1] - quaternion[2]*quatDelta[2] - quaternion[3]*quatDelta[3]);
  newQuat[1]=(quaternion[0]*quatDelta[1] + quaternion[1]*quatDelta[0] + quaternion[2]*quatDelta[3] - quaternion[3]*quatDelta[2]);
  newQuat[2]=(quaternion[0]*quatDelta[2] - quaternion[1]*quatDelta[3] + quaternion[2]*quatDelta[0] + quaternion[3]*quatDelta[1]);
  newQuat[3]=(quaternion[0]*quatDelta[3] + quaternion[1]*quatDelta[2] - quaternion[2]*quatDelta[1] + quaternion[3]*quatDelta[0]);
  for (int j=0;j<4;j++)
    quaternion[j]=newQuat[j];
    
  // Normalize quaternion
  float magnitude=sqrt(quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1]+quaternion[2]*quaternion[2]+quaternion[3]*quaternion[3]);
  if (magnitude > 50.0)
  {
    Serial.println("Normalizing");
    for (int j=0;j<4;j++)
      quaternion[j]=quaternion[j]/magnitude;
  }
  
  // Convert back to euler angles
  euler[2]=atan2(2.0*(quaternion[0]*quaternion[1]+quaternion[2]*quaternion[3]), 1-2.0*(quaternion[1]*quaternion[1]+quaternion[2]*quaternion[2]));
  euler[1]=safe_asin(2.0*(quaternion[0]*quaternion[2] - quaternion[3]*quaternion[1]));
  euler[0]=atan2(2.0*(quaternion[0]*quaternion[3]+quaternion[1]*quaternion[2]), 1-2.0*(quaternion[2]*quaternion[2]+quaternion[3]*quaternion[3]));
 
  // convert back to degrees
  for (int j=0;j<3;j++)
    euler[j]=euler[j] * 57.2957795;
}

// this simply returns the elapsed time since the last call.
unsigned long getDeltaTMicros()
{
  static unsigned long lastTime=0;
  
  unsigned long currentTime=micros();
  
  unsigned long deltaT=currentTime-lastTime;
  if (deltaT < 0.0)
     deltaT=currentTime+(4294967295-lastTime);
   
  lastTime=currentTime;
  
  return deltaT;
}

// I called this from the loop function to see what the right values were for the calibration constants.
// If you are trying to reduce the amount of time needed for calibration just try not to go so low that consecutive
// calibration calls give you completely unrelated data.  Some sensors are probably better than others.
void testCalibration()
{
  calibrateGyro();
  for (int j=0;j<3;j++)
  {
    Serial.print(gyroZeroRate[j]);
    Serial.print("  ");
    Serial.print(gyroThreshold[j]);
    Serial.print("  ");  
  }
  Serial.println();
  return; 
}

// The settings here will suffice unless you want to use the interrupt feature.
void setupGyro()
{
  gyroWriteI2C(CTRL_REG1, 0x1F);        // Turn on all axes, disable power down
  gyroWriteI2C(CTRL_REG3, 0x08);        // Enable control ready signal
  setGyroSensitivity500();

  delay(100);
}

// Call this at start up.  It's critical that your device is completely stationary during calibration.
// The sensor needs recalibration after lots of movement, lots of idle time, temperature changes, etc, so try to work that in to your design.
void calibrateGyro()
{
  long int gyroSums[3]={0};
  long int gyroSigma[3]={0};
 
  for (int i=0;i<NUM_GYRO_SAMPLES;i++)
  {
    updateGyroValues();
    for (int j=0;j<3;j++)
    {
      gyroSums[j]+=gyroRaw[j];
      gyroSigma[j]+=gyroRaw[j]*gyroRaw[j];
    }
  }
  for (int j=0;j<3;j++)
  {
    int averageRate=gyroSums[j]/NUM_GYRO_SAMPLES;
    
    // Per STM docs, we store the average of the samples for each axis and subtract them when we use the data.
    gyroZeroRate[j]=averageRate;
    
    // Per STM docs, we create a threshold for each axis based on the standard deviation of the samples times 3.
    gyroThreshold[j]=sqrt((double(gyroSigma[j]) / NUM_GYRO_SAMPLES) - (averageRate * averageRate)) * GYRO_SIGMA_MULTIPLE;    
  }
}

void updateGyroValues() {

  while (!(gyroReadI2C(0x27) & B00001000)){}      // Without this line you will get bad data occasionally
  
  //if (gyroReadI2C(0x27) & B01000000)
  //  Serial.println("Data missed!  Consider using an interrupt");
    
  int reg=0x28;
  for (int j=0;j<3;j++)
  {
    gyroRaw[j]=(gyroReadI2C(reg) | (gyroReadI2C(reg+1)<<8));
    reg+=2;
  }
 
  int deltaGyro[3];
  for (int j=0;j<3;j++)
  {
    deltaGyro[j]=gyroRaw[j]-gyroZeroRate[j];      // Use the calibration data to modify the sensor value.
    if (abs(deltaGyro[j]) < gyroThreshold[j])
      deltaGyro[j]=0;
    gyroDPS[j]= dpsPerDigit * deltaGyro[j];      // Multiply the sensor value by the sensitivity factor to get degrees per second.
  }
}

void setGyroSensitivity250(void)
{
  dpsPerDigit=.00875f;
  gyroWriteI2C(CTRL_REG4, 0x80);        // Set scale (250 deg/sec)
}

void setGyroSensitivity500(void)
{
  dpsPerDigit=.0175f;
  gyroWriteI2C(CTRL_REG4, 0x90);        // Set scale (500 deg/sec)
}

void setGyroSensitivity2000(void)
{
  dpsPerDigit=.07f;
  gyroWriteI2C(CTRL_REG4,0xA0); 
}

int gyroReadI2C (byte regAddr) {
  Wire.beginTransmission(gyroI2CAddr);
  Wire.write(regAddr);
  Wire.endTransmission();
  Wire.requestFrom(gyroI2CAddr, 1);
  while(!Wire.available()) {};
  return (Wire.read());
}

int gyroWriteI2C( byte regAddr, byte val){
  Wire.beginTransmission(gyroI2CAddr);
  Wire.write(regAddr);
  Wire.write(val);
  Wire.endTransmission();
}
