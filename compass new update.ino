#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); //LCD address to 0x3F or sometimes 0x27 for a 20 chars and 4 line display
#include <Wire.h>
#define I2C_TX write
#define I2C_RX read

// Shift the device assigned slave address (0x3C) for write operation, 1 bit right to compensate for how the TWI library only wants the
// 7 most significant bits (with the high bit padded with 0)
 
#define HMC5883_WriteAddress 0x1E //  i.e 0x3C >> 1
#define HMC5883_ModeRegisterAddress 0x02
#define HMC5883_ContinuousModeCommand (uint8_t)0x00     // cast to uint8_t added to get code to compile under Arduino v1.0
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
  Wire.beginTransmission(HMC5883_WriteAddress); //Initiate a transmission with HMC5883 (Write address).
  Wire.I2C_TX(HMC5883_ModeRegisterAddress);     //Place the Mode Register Address in send-buffer.
  Wire.I2C_TX(HMC5883_ContinuousModeCommand);   //Place the command for Continuous operation Mode in send-buffer.
  Wire.endTransmission();                       //Send the send-buffer to HMC5883 and end the I2C transmission.
  delay(100);
 
  Wire.beginTransmission(HMC5883_WriteAddress);  //Initiate a transmission with HMC5883 (Write address).
  Wire.requestFrom(HMC5883_WriteAddress,6);      //Request 6 bytes of data from the address specified.
 
  delay(500);
 
  //Read the value of magnetic components X,Y and Z
 
  if(Wire.available() <= 6) // If the number of bytes available for reading is <=6
  {
    for(int i=0;i<6;i++)
    {
      outputData[i]=Wire.I2C_RX();  //Store the data in outputData buffer
    }
  }
 
  x=outputData[0] << 8 | outputData[1];  //Combine MSB and LSB of X Data output register
  z=outputData[2] << 8 | outputData[3];  //Combine MSB and LSB of Z Data output register
  y=outputData[4] << 8 | outputData[5];  //Combine MSB and LSB of Y Data output register
  angle = (double)atan2(y, x); // angle in radians
  /*
   Refer the following application note for heading calculation.
   http://www.ssec.honeywell.com/magnetic/datasheets/lowcost.pdf
   ----------------------------------------------------------------------------------------
   atan2(y, x) is the angle in radians between the positive x-axis of a plane and the point
   given by the coordinates (x, y) on it.
   ----------------------------------------------------------------------------------------
   This sketch does not utilize the magnetic component Z as tilt compensation can not be done without an Accelerometer
    ----------------->y
   |
   \/
   x
  
         N
   NW    |    NE
         |
  W------+------E
         |
    SW   |   SE
         S
Most HMC5883 Compass modules have a pointer on the PCB like this:
          X  Point X towards the North
          ^
          |
          |
          |
          |
 Y<-------+
          |
          Z
  
  */
  // Find your declination angle here: https://www.ngdc.noaa.gov/geomag/declination.shtml
  // For example if you Declination is -1° 7' then that's -(1+7/60)*PI/180 in radians=-0.019
  float declinationAngle = -0.019;
  angle += declinationAngle;
  
  // Correct for when signs are reversed.
  if (angle < 0)    angle += 2*PI;
    
  // Check for wrap due to addition of declination.
  if (angle > 2*PI) angle -= 2*PI;
   
  // Convert radians to degrees for readability.
  float bearing = angle * 180/PI; 

  Serial.println();
  Serial.println("Heading (degrees): " + String(bearing));
  
//Print the angle on the LCD Screen
  lcd.setCursor(0,1);
  lcd.print("Angle="  + String(bearing));


  lcd.setCursor(0,0);
  lcd.print("Head: ");
  
  if((bearing < 22.5)  || (bearing > 337.5 ))
  { 
    lcd.setCursor(5,0); 
    lcd.print("North        ");
  }
  if((bearing > 22.5)  && (bearing < 67.5 ))
  {
    lcd.setCursor(5,0);   
    lcd.print("North-East");
  }
  if((bearing > 67.5)  && (bearing < 112.5 ))
  {
    lcd.setCursor(5,0);  
    lcd.print("East          ");
  }
  if((bearing > 112.5) && (bearing < 157.5 ))
  {
    lcd.setCursor(5,0);  
    lcd.print("South-East");
  }
  if((bearing > 157.5) && (bearing < 202.5 ))
  {
    lcd.setCursor(5,0);  
    lcd.print("South        ");
  }
  if((bearing > 202.5) && (bearing < 247.5 ))
  {
    lcd.setCursor(5,0);  
    lcd.print("South-West");
  }
  if((bearing > 247.5) && (bearing < 292.5 ))
  {
    lcd.setCursor(5,0);  
    lcd.print("West          ");
  }
  if((bearing > 292.5) && (bearing < 337.5 ))
  {
    lcd.setCursor(5,0);  
    lcd.print("North-West");
  }

  //Print the approximate direction
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
