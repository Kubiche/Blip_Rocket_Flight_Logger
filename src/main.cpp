#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_BMP280.h> 
#include <SPIFlash.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"

//First, we'll set up the LEDs and buzzer
const byte RGB[] = {9, 2 , 6}; // Array containing LED pins.
byte color_selector = 0;
const byte Buzzer = 10;
const float pressure_at_sea_level = 1018.63; // obtained on day of lauch from nearest airport
float altitude_offset;
int relative_altitude;
unsigned long timer1; //LED timer
unsigned long timer2; // NAV timer
bool LED_State = 0;   // State to pass to LED function. 0 equals LED ON.




byte Stage = 0;
/* Tracker for flight stage
0 = Ground Idle / Calibrating
1 = Powered Flight
2 = Ballistic Descent
3 = Chute Descent
4 = Landed
*/
unsigned long timer3; // Stage toggling timer for development.


//This is for the BMP280 barometer
Adafruit_BMP280 bmp;

//This is for the MPU6050 IMU
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO

void lightLED(int color, bool State){
  switch (color){
    case 9:
      digitalWrite(RGB[1], HIGH);
      digitalWrite(RGB[2], HIGH);
      digitalWrite(RGB[0], State);
      break;
    case 2:
      digitalWrite(RGB[0], HIGH);
      digitalWrite(RGB[2], HIGH);
      digitalWrite(RGB[1], State);
      break;
    case 6:
      digitalWrite(RGB[0], HIGH);
      digitalWrite(RGB[1], HIGH);
      digitalWrite(RGB[2], State);
      break;
  }
}

int relativeAltitude (float offset){
  return bmp.readAltitude(pressure_at_sea_level) - offset;
}

void setup() {
  //Set Startup time  
  timer1 = millis();
  timer2 = timer1;
  timer3 = timer1;

  //set system led pins as outputs
  while (color_selector < 3){
    pinMode(RGB[color_selector], OUTPUT);
    color_selector ++;
  }

  
  if (!bmp.begin()) {
    Serial.println("Could not find the BMP280 sensor :( Check your soldering and inspect for bad connections");
    delay(500);
    Serial.println("Proceeding with the rest of the startup process");
  }
  else{
   //Configure the barometer
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    Serial.println("Found the BMP280 sensor!");
  }
    
  Serial.println("Now we'll look for the the MPU6050 IMU. Standby...");
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "Found it! MPU6050 connection successful." : "MPU6050 connection failed :(");
  delay(2000);
}

void loop() { 
  
  // ************************************************************************ Ground Idle / Calibrating **************************************************************
  while (Stage == 0){

    if (millis() - timer1 >= 1000){
      if (color_selector == 3){
        color_selector = 0;
        lightLED(RGB[color_selector], LED_State);
        color_selector ++;
      }
      else{
        lightLED(RGB[color_selector], LED_State);
        color_selector ++;      
      }            
      timer1 = millis();
    }
  
    altitude_offset = bmp.readAltitude(pressure_at_sea_level); // set altitude reference point from pad.
    

    if (millis() - timer2 >= 250){
      Serial.println(relativeAltitude(altitude_offset));
      timer2 = millis();
    }
    
    //Develpment auto stage toggler
    if (millis() - timer3 >=30000){
      Stage ++;
      timer3 = millis();
    }  
    
  }  
  
  //***************************************************************************** Powered Flight ***********************************************************************
  while (Stage == 1){
    if(millis() - timer2 >= 25){
      Serial.println();
      Serial.print(F("Temperature = "));
      Serial.print(bmp.readTemperature());
      Serial.println(" *C");

      Serial.print(F("Pressure = "));
      Serial.print(bmp.readPressure() / 100);
      Serial.println(" hPa");

      Serial.print(F("Approx altitude = "));
      Serial.println(relativeAltitude(altitude_offset));
      Serial.println(" m");
    
      //Now the IMU
  
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      Serial.print("a/g:\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
      timer2 = millis();
    }
  }  
}

