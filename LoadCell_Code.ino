#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>

HX711 scale(A2, A1);

float Weight,Zero,Tare;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup(void) 
  {
    lcd.begin();
    lcd.setCursor(0,0);lcd.print("  Scale  Meter  ");
    lcd.setCursor(0,1);lcd.print(" Weight =     g");
    Zero = scale.read_average(10);
  }

void loop() {
   Weight = (scale.read_average(10) - Zero) /415;
   lcd.setCursor(11,1);lcd.print;lcd.print("    ");
   lcd.setCursor(11,1);lcd.print;lcd.print(abs(Weight),0);
}
