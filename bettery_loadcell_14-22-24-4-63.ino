#include <ZMPT101B.h>
#include <Filters.h> //Easy library to do the calculations
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>
HX711 scale;
LiquidCrystal_I2C lcd(0x27, 16, 2);
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
float calibration_factor = -380000;

float testFrequency = 50;                     // test signal frequency (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the signal, for statistist
int Sensor = 0;                             //Sensor analog input, here it's A0
float intercept = -0.04;                        // to be adjusted based on calibration testing
float slope = 0.0405;                      // to be adjusted based on calibration testing
float current_Volts;                          // Voltage
unsigned long printPeriod = 1000;           //Refresh rate
unsigned long previousMillis = 0;

void setup() {

  lcd.begin();
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale();
  scale.tare();
  long zero_factor = scale.read_average();
  //long weight = scale.get_units();
}
void loop() 
{
  RunningStatistics inputStats;                //Easy life lines, actual calculation of the RMS requires a load of coding
  inputStats.setWindowSecs( windowLength );
   
  while( true ) {   
    Sensor = analogRead(A0);  // read the analog in value:
    inputStats.input(Sensor);  // log to Stats function

      scale.set_scale(calibration_factor);
      lcd.setCursor(0,1);
      lcd.print(scale.get_units(),4);
      lcd.setCursor(7,1);
      lcd.print(" kg       ");
      lcd.println();

        
    if((unsigned long)(millis() - previousMillis) >= printPeriod) {
      previousMillis = millis();   // update time every second
      current_Volts = intercept + slope * inputStats.sigma(); //Calibartions for offset and amplitude
      current_Volts= current_Volts*(40.3231);                //Further calibrations for the amplitude

      if(current_Volts)
      lcd.setCursor(0, 0);
      lcd.print("volts =");
      {
        if(current_Volts>=24){
        lcd.setCursor(9, 0);
        lcd.println("100%   ");
        }
        if(current_Volts==20){
        lcd.setCursor(9, 0);
        lcd.println("80 %   ");
        }
        if(current_Volts==15){
        lcd.setCursor(9, 0);
        lcd.println("80 %   ");
        }
        if(current_Volts==10){
        lcd.setCursor(9, 0);
        lcd.println("70 %   ");
        }
        if(current_Volts<=5){
        lcd.setCursor(9, 0);
        lcd.println("0  %   ");
        }
      }
    }
  }
}
