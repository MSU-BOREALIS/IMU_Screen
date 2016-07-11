/*
 * Author: Trevor Gahl (CpE)              
 * Developed for use by MSGC Borealis program     
 * Date of Last Edit: 07/01/2016            
 * Purpose: To read a BNO055 IMU and print angles to an LCD screen 
 */



#include <LiquidCrystal.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55); 
LiquidCrystal lcd(9,8,6,5,4,3);
bool calibrated = false;

void setup(){
	Serial.begin(9600);
	if(!bno.begin())                                          //Launches the IMU. It returns a true value if it successfully launches. 
  	{
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  	}

	delay(500);                                               //Wait for 0.5s
  	bno.setExtCrystalUse(true);                                //Use the external clock in the IMU (true for better accuracy)
  	bno.setMode(bno.OPERATION_MODE_NDOF);

	lcd.begin(16, 2);
}

void loop(){
	lcd.setCursor(0,1);


	sensors_event_t event;                                                   //Creates a sensors event called event
  	float x,y,z;                                                             //Creates floats for the x, y, and z axis

  	imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

	uint8_t system, gyro, accel, mag;                                        //Create local variables gyro, accel, mag
  	system = gyro = accel = mag = 0;
  	if(calibrated == false){  		  		
  		bno.getCalibration(&system, &gyro, &accel, &mag);                    //Read the calibration values from the IMU
  		bno.getEvent(&event);                                                //Get a new event and store it to... event
  		lcd.setCursor(0,0);   
      lcd.print("Calibrating...");                                           //Will continually read calibration values until "calibrated" is set to true
      lcd.setCursor(0,1);
      lcd.print(system,DEC);
      lcd.print(gyro,DEC);
      lcd.print(accel,DEC);
      lcd.print(mag,DEC); 
  		Serial.print(system, DEC);
  		Serial.print(",");
  		Serial.print(gyro, DEC);
  		Serial.print(",");
		Serial.print(accel,DEC);
		Serial.print(",");
		Serial.println(mag, DEC);
     		if(bno.isFullyCalibrated() == true){                                 //Will return true if all calibration values are 3
      			lcd.clear();
      			calibrated = true;                                                  //Sets "calibrated" to true, preventing the calibration loop from executing again
     		}
  	}
  	else{
    delay(500);
    x = euler.x();                                                      //Creates Euler Vectors for the x, y, and z axis
    y = euler.y();
    z = euler.z();
  	  
    lcd.setCursor(0,0);                                                 //prints angle values to an LCD screen
    lcd.print("X: ");
    lcd.setCursor(3,0);
    lcd.print(x);
    lcd.setCursor(8,0);
    lcd.print("Y: ");
    lcd.setCursor(11,0);
    lcd.print(y);
    lcd.setCursor(0,1);
    lcd.print("Z: ");
    lcd.setCursor(3,1);
    lcd.print(z);


    Serial.println();                                                   //Prints the angles out over the serial port for x, y, and z axis
  	Serial.println("X Axis ");
  	Serial.print(x);
  	Serial.println("Y Axis ");
  	Serial.print(y);
  	Serial.println("Z Axis ");
  	Serial.print(z);
	}
}
