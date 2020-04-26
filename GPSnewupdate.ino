#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial gpsSerial(8,9);
TinyGPSPlus gps;
double lattitude,longitude;

void setup() {
lcd.begin();
 gpsSerial.begin(9600);
 Serial.begin(9600);
  
  }

void loop()
{
  while (gpsSerial.available())
  {
    int data = gpsSerial.read();
    if (gps.encode(data))
    {
      lattitude = (gps.location.lat());
      longitude = (gps.location.lng());
      lcd.setCursor(0,0);
      lcd.print ("t: ");
      lcd.setCursor(2,0);
      lcd.println(gps.location.lat(), 10);
      lcd.setCursor(0,1);
      lcd.print ("g: ");
      lcd.setCursor(2,1);
      lcd.println(gps.location.lng(), 10 );
      Serial.print("lattitude: ");
      Serial.println(gps.location.lat(), 10);
      Serial.print("longitude: ");
      Serial.println(gps.location.lng(), 10);
      delay(1000);
    }
  }
}
