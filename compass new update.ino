#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); //LCD address to 0x3F or sometimes 0x27 for a 20 chars and 4 line display
#include <Wire.h>
#define I2C_TX write
#define I2C_RX read
#define HMC5883_WriteAddress 0x1E
#define HMC5883_ModeRegisterAddress 0x02
#define HMC5883_ContinuousModeCommand (uint8_t)0x00
#define HMC5883_DataOutputXMSBAddress 0x03
int regb = 0x01;
int regbdata = 0x40;
int outputData[6];
void setup()
{
  lcd.begin();
  Serial.begin(9600);
  Wire.begin();   //Initiate the Wire library and join the I2C bus as a master
  Serial.write(254);
  Serial.write(1);
}
void loop() {
  int x,y,z;
  double angle;
  Serial.write(254);
  Serial.write(128); //Goto line-1
  Wire.beginTransmission(HMC5883_WriteAddress);
  Wire.I2C_TX(regb);
  Wire.I2C_TX(regbdata);
  Wire.endTransmission();
  delay(1000);
  Wire.beginTransmission(HMC5883_WriteAddress);
  Wire.I2C_TX(HMC5883_ModeRegisterAddress);
  Wire.I2C_TX(HMC5883_ContinuousModeCommand);
  Wire.endTransmission();
  delay(100);
 
  Wire.beginTransmission(HMC5883_WriteAddress);
  Wire.requestFrom(HMC5883_WriteAddress,6);
  delay(500);
  if(Wire.available() <= 6)
  {
    for(int i=0;i<6;i++)
    {
      outputData[i]=Wire.I2C_RX();
    }
  }
  x=outputData[0] << 8 | outputData[1];
  z=outputData[2] << 8 | outputData[3];
  y=outputData[4] << 8 | outputData[5];
  angle = (double)atan2(y, x);
  float declinationAngle = -0.019;
  angle += declinationAngle;
  if (angle < 0)    angle += 2*PI;
  if (angle > 2*PI) angle -= 2*PI;
  float bearing = angle * 180/PI; 
  Serial.println();
  Serial.println("Heading (degrees): " + String(bearing));
  Serial.print("\nYou are heading ");
  if((bearing > 337.5) || (bearing < 22.5))    Serial.print("North");
  if((bearing > 22.5)  && (bearing < 67.5 ))   Serial.print("North-East");
  if((bearing > 67.5)  && (bearing < 112.5 ))  Serial.print("East");
  if((bearing > 112.5) && (bearing < 157.5 ))  Serial.print("South-East");
  if((bearing > 157.5) && (bearing < 202.5 ))  Serial.print("South");
  if((bearing > 202.5) && (bearing < 247.5 ))  Serial.print("South-West");
  if((bearing > 247.5) && (bearing < 292.5 ))  Serial.print("West");
  if((bearing > 292.5) && (bearing < 337.5 ))  Serial.print("North-West");
}
