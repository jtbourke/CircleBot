#include <NewPing.h>

// Basic plan:
//  Circlebot has a Ping ultrasonic range sensor, a 3 axis gyro sensor, and a motor driver chip.
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

#define  CTRL_REG1  0x20
#define  CTRL_REG2  0x21
#define  CTRL_REG3  0x22
#define  CTRL_REG4  0x23

int gyroI2CAddr=105;

int gyroRaw[3];
int gyroDPS[3];

int gyroZeroRate[3];
int gyroThreshold[3];

#define  NUM_GYRO_SAMPLES  2000
#define  GYRO_SIGMA_MULTIPLE  3

#define PING_PIN  7  // Arduino pin tied to both trigger and echo pins on the ultrasonic sensor.
#define MAX_DISTANCE 600 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(PING_PIN, PING_PIN, MAX_DISTANCE); // NewPing setup of pin and maximum distance.

float heading[3];

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  Wire.begin();
  gyroWriteI2C(CTRL_REG1, 0x1F);        // Turn on all axes, disable power down
  gyroWriteI2C(CTRL_REG3, 0x08);        // Enable control ready signal
  gyroWriteI2C(CTRL_REG4, 0x90);        // Set scale (500 deg/sec)
  delay(1000);
  
  calibrateGyro();
}

void loop() {

  /*
  delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  Serial.print("Ping: ");
  Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.print("cm");
  
  */
  calibrateGyro();
  for (int i=0;i<3;i++)
  {
     Serial.print(" X Zero Rate: ");
     Serial.print(gyroZeroRate[0]);
     Serial.print(" Y Zero Rate: ");
     Serial.print(gyroZeroRate[1]);
     Serial.print(" Z Zero Rate: ");
     Serial.print(gyroZeroRate[2]);
     Serial.print(" X Threshold: ");
     Serial.print(gyroThreshold[0]);
     Serial.print(" Y Threshold: ");
     Serial.print(gyroThreshold[1]);
     Serial.print(" Z Threshold: ");
     Serial.print(gyroThreshold[2]);
     Serial.println();
  }
  updateGyroValues();
  /*
  Serial.print("Gyro:\tRaw X:");
  Serial.print(gyroX/114);
  Serial.print(" Raw Y:");
  Serial.print(gyroY/114);
  Serial.print(" Raw Z:");
  */
  /*
  int deltaGyro[3];
  for (int i=0;i<3;i++)
  {
    deltaGyro[i]=gyroRaw[i]-gyroZeroRate[i];
    if (abs(deltaGyro[i]) < gyroThreshold[i])
    deltaGyro[i]=0;
    gyroDPS[i]=0.00875 * (deltaGyro[i]);
  }
*/
/*
  Serial.print(" Z deg/sec: ");
  Serial.print(gyroDPS[2]);
  */
  unsigned long thisMicro=micros();
  float deltaT=getDeltaTMicros();          //     /1000000.0;
  
  if (deltaT > 0.0)
    for (int j=0;j<3;j++)
      heading[j] += (gyroDPS[j]*deltaT)/1000000.0;

/*
  Serial.print("  milli: ");
  Serial.print(thisMilli);
  Serial.print(" deltaT: ");
  Serial.print(deltaT);
  Serial.print(" headingchange: ");
  Serial.print(gyroDPS[2]/deltaT);
  */
  
  /*
  Serial.print("X heading: ");
  Serial.print(heading[0]);
  Serial.print("  Y heading: ");
  Serial.print(heading[1]);
  Serial.print("  Z heading: ");
  Serial.print(heading[2]);
  Serial.println();
*/
}

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

void calibrateGyro()
{
  int gyroSamples[NUM_GYRO_SAMPLES][3];
  int gyroSums[3];
  int gyroSigma[3];
  
  int ct=1;
  for (int i=0;i<NUM_GYRO_SAMPLES;i++)
  {
    updateGyroValues();
    for (int j=0;j<3;j++)
    {
      gyroSums[j]+=gyroRaw[j];
      if (ct > 1)
      {
        int tmp=(gyroSums[j] / ct - gyroRaw[j]);    
        gyroSigma[j]=gyroSigma[j]+ ct * tmp * tmp / (ct-1);
      }
    }
  }
  for (int j=0;j<3;j++)
  {
    gyroZeroRate[j]=gyroSums[j]/NUM_GYRO_SAMPLES;
    gyroThreshold[j]=sqrt(gyroSigma[j] / ct);
  }
 
 
 
 /*
  for (int i=0;i<NUM_GYRO_SAMPLES;i++)
  {
    updateGyroValues();
    for (int j=0;j<3;j++)
    {
      gyroSamples[i][j]=gyroRaw[j];
      gyroSums[j]+=gyroRaw[j];
    }
  }
  
  for (int j=0;j<3;j++)
    gyroZeroRate[j]=gyroSums[j]/NUM_GYRO_SAMPLES;
  
  // Calculate the std dev of samples
  int gyroDevs[NUM_GYRO_SAMPLES][3];
  for (int i=0;i<NUM_GYRO_SAMPLES;i++)
    for (int j=0;j<3;j++)
      gyroDevs[i][j]=(gyroSamples[i][j]-gyroZeroRate[j])^2;
  
  int gyroSigma[3];
  for (int i=0;i<NUM_GYRO_SAMPLES;i++)
    for (int j=0;j<3;j++)
       gyroSigma[j]+=gyroDevs[i][j];

  for (int j=0;j<3;j++)
    gyroThreshold[j]=sqrt(gyroSigma[j]/NUM_GYRO_SAMPLES)*GYRO_SIGMA_MULTIPLE;
    
    */
}

void updateGyroValues() {

  int reg=0x28;
  
  for (int j=0;j<3;j++)
  {
    gyroRaw[j]=(gyroReadI2C(reg) | (gyroReadI2C(reg+1)<<8));
    reg+=2;
  }
  
  int deltaGyro[3];
  for (int i=0;i<3;i++)
  {
    deltaGyro[i]=gyroRaw[i]-gyroZeroRate[i];
    if (abs(deltaGyro[i]) < gyroThreshold[i])
      deltaGyro[i]=0;
    gyroDPS[i]=2.0 * 0.00875 * (deltaGyro[i]);
  }
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
