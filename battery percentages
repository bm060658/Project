#include <ZMPT101B.h>
#include <Filters.h> //Easy library to do the calculations
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
float testFrequency = 50;                     // test signal frequency (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the signal, for statistist
int Sensor = 0; //Sensor analog input, here it's A0
float intercept = -0.04; // to be adjusted based on calibration testing
float slope = 0.0405; // to be adjusted based on calibration testing
float current_Volts; // Voltage
unsigned long printPeriod = 1000; //Refresh rate
unsigned long previousMillis = 0;
void setup() {
// initialize the LCD
lcd.begin();
}
void loop() {
  
  RunningStatistics inputStats;                //Easy life lines, actual calculation of the RMS requires a load of coding
  inputStats.setWindowSecs( windowLength );
   
  while( true ) {   
    Sensor = analogRead(A0);  // read the analog in value:
    inputStats.input(Sensor);  // log to Stats function
        
    if((unsigned long)(millis() - previousMillis) >= printPeriod) {
      previousMillis = millis();   // update time every second
      current_Volts = intercept + slope * inputStats.sigma(); //Calibartions for offset and amplitude
      current_Volts= current_Volts*(40.3231);                //Further calibrations for the amplitude

      if(current_Volts)
      lcd.setCursor(0, 0);
      lcd.print("volts =");
      {
        if(current_Volts>=200){
        lcd.setCursor(9, 0);
        lcd.println("100%   ");
        }
        if(current_Volts==150){
        lcd.setCursor(9, 0);
        lcd.println("90 %   ");
        }
        if(current_Volts==100){
        lcd.setCursor(9, 0);
        lcd.println("80 %   ");
        }
        if(current_Volts==50){
        lcd.setCursor(9, 0);
        lcd.println("70 %   ");
        }
        if(current_Volts<=20){
        lcd.setCursor(9, 0);
        lcd.println("0  %   ");
        }
      }
    }
  }
}
