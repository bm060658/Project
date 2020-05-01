#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <Servo.h>
HMC5883L compass;
TinyGPSPlus gps;
SoftwareSerial ss(9, 8);

//loadcell
#include "HX711.h"
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 11;
int volt=12;
HX711 scale;
float calibration_factor = -380000;


int p = 2;
static const double POINT[][2] = {
  {13.914046, 100.510954},
  {13.913975, 100.511016},
  {13.914046, 100.510954},
  {13.913975, 100.511016}
 };
int headingDegrees;
//motor right
int ENA =5; //analog pin5
int IN1 =6;
int IN2 =7;

//motor leaft

int ENB =3; //analog pin3
int IN3 =2;
int IN4 =4;


unsigned long targetMillis = 0;
const long intervalQuick = 850;
const long intervalSlow = 5000;
long intervalCalculated = 0;
float angle = 0; //Measured angle from feedback --- Servo Feedback
 /*
Servo servo_test;        //initialize a servo object for the connected servo
int pinFeedback = 5;
float tHigh = 0;
float tLow = 0;
int tCycle = 0;
float dc = 0;

float dcMin = 2.9; //From Parallax spec sheet
float dcMax = 97.1; //From Parallax spec sheet
float Kp = .7; //Proportional Gain, higher values for faster response, higher values contribute to overshoot.
float Ki = .2; //Integral Gain, higher values to converge faster to zero error, higher values produce oscillations. Higher values are more unstable near a target_angle = 0.
float iLimit = 5; //Arbitrary Anti-wind-up
float Kd = 1; //Derivative Gain, higher values dampen oscillations around target_angle. Higher values produce more holding state jitter. May need filter for error noise.
float prev_error = 0;
float prev_pError = 0;
float error = 0;
float pError = 0;
float iError = 0;


*/
void setup()
{
  Serial.begin(115200);
  ss.begin(9600);
  
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);

  pinMode(6,OUTPUT); //Motor right
  pinMode(7,OUTPUT); //Motor right
  pinMode(5,OUTPUT); //Speed Motor right
  pinMode(2,OUTPUT); //Motor left
  pinMode(4,OUTPUT); //Motor left
  pinMode(3,OUTPUT); //Speed Motor left

  delay(2000);
 
  scale.begin(12, 11);
  Serial.println("Press + or - to calibration factor");
  Serial.println("ArduinoAll Calibration 0 Please Wait ... ");
  scale.set_scale();
  scale.tare();
  long zero_factor = scale.read_average();
  Serial.print("Zero factor: ");
  Serial.println(zero_factor);
  

}
void loop()
{
  //loadcell
  scale.set_scale(calibration_factor);
  double load_w = scale.get_units();
  loadcell(load_w);
  
  if(Serial.available() > 0)  {
    int incomingData= Serial.read(); // can be -1 if read error
    switch(incomingData) {
        case '1':
           volt = 1;
           break;
        case '2':
           volt = 2;
           break;
        case '3':
           volt = 3;
           break;
        case '4':
           volt = 4;
           break;
        case '5':
           volt = 5;
           break;
        case '6':
           volt = 6;
           break;
        case '7':
           volt = 7;
           break;
        case '8':
           volt = 8;
           break;
        case '9':
           volt = 9;
           break;
        case '10':
           volt = 10;
           break;
        case '11':
           volt = 11;
           break;
        case '12':
           volt = 12;
           break;
        default:
           break;
   }
 }
  bettery(volt);
  Serial.print("Reading: ");
  Serial.print(load_w, 4);
  Serial.print(" kg");
  Serial.print(" bettery: ");
  Serial.print(volt);
  Serial.print(" volt");
  Serial.println();  
  
  
    //Serial.println(POINT[p][0]);

  //Serial.println(POINT[p][1]);

  unsigned long Distance_M_To_POINT =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      POINT[p][0],
      POINT[p][1]);
  printInt(Distance_M_To_POINT, gps.location.isValid(), 9);

  double Course_To_POINT =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      POINT[p][0],
      POINT[p][1]);

  printFloat(Course_To_POINT, gps.location.isValid(), 7, 2);

  const char *Cardinal_To_POINT = TinyGPSPlus::cardinal(Course_To_POINT);

  printStr(gps.location.isValid() ? Cardinal_To_POINT : "*** ", 6);

  int heading_control = displayCompassInfo();

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  unsigned long currentMillis = millis();
  bool updateGPS = (currentMillis >= targetMillis);

  // only read the GPS/Compass periodialy, to give the servo more processing time
  if (updateGPS) {
    targetMillis = currentMillis + intervalCalculated;

    displayCompassInfo();  // updates heading degrees
    
  }
  //moveMotor( headingDegrees );

  float differenceFactor = abs( headingDegrees - angle ) / 360;  // 180 degrees should be the maximum value that the compass and servo can differ by, to scale this between 0 and 1
  intervalCalculated = intervalSlow * differenceFactor + intervalQuick * (1 - differenceFactor);

  //Serial.print(F(" intervalCalculated = "));
  //Serial.print(intervalCalculated);
  
  if(Distance_M_To_POINT < 5)
  {
    p++;
    Serial.println("Check point ===============================================================");
    delay(5000);
  }


  
  int numRows = sizeof(POINT)/sizeof(POINT[0]);
  if(p >= numRows)
  {
    p = 0;
  }
 
  double Forward_A = Course_To_POINT-20;
  double Forward_B = Course_To_POINT+20;
  Serial.print(F(" Point = "));
  Serial.print(p);
  Serial.println();
  Serial.print(F(" Forward_A = "));
  Serial.print(Forward_A);
  Serial.print(F(" Forward_B = "));
  Serial.print(Forward_B);
  Serial.print(F(" heading_control = "));
  Serial.print(heading_control);
  
  if ((heading_control >=Forward_A) && (heading_control<= Forward_B))
  {
    forward();
    Serial.print(F(" forward "));
  }
  else
  {
    if (heading_control > Course_To_POINT){
      left();
      Serial.print(F(" left "));
    }
    else if (heading_control < Course_To_POINT){
      right();
      Serial.print(F(" right "));
     }
  }
  Serial.println();
}
int cal_compass(float val)//แก้ error
{
  if(val > 360)
  {
    val = val-360;
  }
  if(val > 360)
  {
    val = 360+(val);
  }

  return val;
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}
static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}
static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}
int displayCompassInfo()
{
 Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 

  // Output
  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();

  return headingDegrees;

  delay(100);
}
void forward()
{
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
  analogWrite(5,170);
  digitalWrite(2,HIGH);
  digitalWrite(4,LOW);
  analogWrite(3,170);
  
}
void right()
{
  digitalWrite(6,LOW);
  digitalWrite(7,LOW);
  analogWrite(5,170);
  digitalWrite(2,HIGH);
  digitalWrite(4,LOW);
  analogWrite(3,170);
}
void left()
{
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
  analogWrite(5,170);
  digitalWrite(2,LOW);
  digitalWrite(4,LOW);
  analogWrite(3,170);
}


void bettery(int volt_val)
{
  if(volt_val <= 5)
  {
    //ให้กลับไปจุดเริ่มต้น
    p=0;
  }
  if(volt_val >=5)
  {
    p =2;
  }
}
void loadcell(double val)
{
  if(val>=3.00)
  {
    //ให้กลับไปจุดเริ่มต้น
    p=0;
  }
   if(val <=3.00)
  {
    p =2;
  }
}
/*loadcell







/*
void moveMotor(int targetAngle) {
    Serial.println(targetAngle);

  while (1) //From Parallax spec sheet
  {
    tHigh = pulseIn(pinFeedback, HIGH);
    tLow = pulseIn(pinFeedback, LOW);
    tCycle = tHigh + tLow;
    if ( tCycle > 1000 && tCycle < 1200)
    {
      break; //valid tCycle;
    }
  }

  dc = (100 * tHigh) / tCycle; //From Parallax spec sheet, you are trying to determine the percentage of the HIGH in the pulse

  angle = ((dc - dcMin) * 360) / (dcMax - dcMin + 1); //From Parallax spec sheet

  //Keep measured angles within bounds
  if (angle < 0)
  {
    angle = 0;
  }
  else if (angle > 359)
  {
    angle = 359;
  }

  if (targetAngle < 0)
  {
    targetAngle = 360 + targetAngle; //handles negative targetAngles;
  }

  error = targetAngle - angle;

  if (error > 180)
  {
    error = error - 360; //tells it to rotate in the other direction because it is a smaller angle that way.
  }
  if (error < -180)
  {
    error = 360 - error - 360; //tells it to rotate in the other direction because it is a smaller angle that way.
  }

  // PID controller stuff, Adjust values of Kp, Ki, and Kd above to tune your system
  float pError = Kp * error;
  float iError = Ki * (error + prev_error);

  if  (iError > iLimit)
  {
    iError = iLimit;
  }
  if (iError <  -iLimit)
  {
    iError = -iLimit;
  }

  prev_error = error;
  float dError = Kd * (pError - prev_pError);
  prev_pError = pError;

  error = error / 2; //max 180 error will have max 90 offset value

  int val = 93 - (Kp * error) - iError - dError; // 93 is the middle of my servo's "no motion" dead-band

  servo_test.write(val); //Move the servo
  
}*/
