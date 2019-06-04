#define DIR_M1 2
#define DIR_M2 3
#define DIR_M3 4
#define DIR_M4 7
#define PWM_M1 9
#define PWM_M2 10
#define PWM_M3 5
#define PWM_M4 6
#define ENABLE_MOTORS 8
#include <SPI.h>
#include <PixyI2C.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float headingDegrees;
int offSet, initReading;
/ Assign a unique ID to this sensor at the same time /
float reading;


const int range1=31, range2=33, range3 = 35;
int dist1, dist2;
 

float heading;
// This is the main Pixy object 

PixyI2C pixy;

  int height;
  int width;
  int blocks;

void setup() {
   Serial.begin(9600);
  pixy.init();
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
  pinMode(DIR_M1, OUTPUT);
  pinMode(PWM_M1, OUTPUT);  
  digitalWrite(PWM_M1, LOW);
  pinMode(DIR_M2, OUTPUT);
  pinMode(PWM_M2, OUTPUT);  
  digitalWrite(PWM_M2, LOW);
  pinMode(DIR_M3, OUTPUT);
  pinMode(PWM_M3, OUTPUT);  
  digitalWrite(PWM_M3, LOW);
  pinMode(DIR_M4, OUTPUT);
  pinMode(PWM_M4, OUTPUT);  
  digitalWrite(PWM_M4, LOW);
  pinMode(ENABLE_MOTORS, OUTPUT); 
  digitalWrite(ENABLE_MOTORS, LOW);  // HIGH = disabled
  
  / Initialise the sensor /
  if(!mag.begin())
  {
    / There was a problem detecting the HMC5883 ... check your connections /
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }
  initReading = compass();
  offSet=180-initReading; //future readings are now calculated to be 180 in the loop
}

void right(){//moves right
  int dir1 = 1;
    int dir0 = 0;
    int pwm1 = 150; //set direction and speed 
    digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M1, pwm1); // write to pins
    digitalWrite(DIR_M2, dir1);
    analogWrite(PWM_M2, pwm1); // write to pins
    digitalWrite(DIR_M3, dir0);
    analogWrite(PWM_M3, pwm1); // write to pins
    digitalWrite(DIR_M4, dir0);
    analogWrite(PWM_M4, pwm1); // write to pins
    
}

void left(){//moves left
  int dir1 = 1;
    int dir0 = 0;
    int pwm1 = 150; //set direction and speed 
    digitalWrite(DIR_M1, dir0);
    analogWrite(PWM_M1, pwm1); // write to pins
    digitalWrite(DIR_M2, dir0);
    analogWrite(PWM_M2, pwm1); // write to pins
    digitalWrite(DIR_M3, dir1);
    analogWrite(PWM_M3, pwm1); // write to pins
    digitalWrite(DIR_M4, dir1);
    analogWrite(PWM_M4, pwm1); // write to pins
}

void forward(){//moves forward
  int dir1 = 1;
    int dir0 = 0;
    int pwm1 = 150; //set direction and speed 
    digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M1, pwm1); // write to pins
    digitalWrite(DIR_M2, dir0);
    analogWrite(PWM_M2, pwm1); // write to pins
    digitalWrite(DIR_M3, dir0);
    analogWrite(PWM_M3, pwm1); // write to pins
    digitalWrite(DIR_M4, dir1);
    analogWrite(PWM_M4, pwm1); // write to pins
    
}
void back(){//moves back
  int dir1 = 1;
    int dir0 = 0;
    int pwm1 = 150; //set direction and speed 
    digitalWrite(DIR_M1, dir0);
    analogWrite(PWM_M1, pwm1); // write to pins
    digitalWrite(DIR_M2, dir1);
    analogWrite(PWM_M2, pwm1); // write to pins
    digitalWrite(DIR_M3, dir1);
    analogWrite(PWM_M3, pwm1); // write to pins
    digitalWrite(DIR_M4, dir0);
    analogWrite(PWM_M4, pwm1); // write to pins
    
}
void backward2(){//movesbackward slow
int dir1 = 1;
    int dir0 = 0;
    int pwm1 = 100; //set direction and speed 
    digitalWrite(DIR_M1, dir0);
    analogWrite(PWM_M1, pwm1); // write to pins
    digitalWrite(DIR_M2, dir1);
    analogWrite(PWM_M2, pwm1); // write to pins
    digitalWrite(DIR_M3, dir1);
    analogWrite(PWM_M3, pwm1); // write to pins
    digitalWrite(DIR_M4, dir0);
    analogWrite(PWM_M4, pwm1); // write to pins
}

void findball(){
   blocks = pixy.getBlocks();
    if (blocks>0){
      pixy.blocks[0].print();
      width= pixy.blocks[0].width;
      height=pixy.blocks[0].height;
           
       if (pixy.blocks[0].x<90){
       left();
       }
              
       else if(pixy.blocks[0].x>230){
        right();
       }
       
       
       else{
         forward();
       }
       if(blocks==0){
         backward2();
    }
}
}
void loop() {
   findball();
   
    reading = compass();
  reading += offSet;  //to ensure all reading are relative to initial calculated 180
  if (reading<0)      
    reading+=360;
  else if (reading>360)      
    reading-=360;
 
  Serial.print("Heading (degrees): "); 
  Serial.println(reading);

  int dir1 = 1;
  int dir2 = 0;
  int pwm1 = 100;
  int pwm2 = 0;
  if(reading > 190){
    
   digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M1, pwm1); // write to pins
    digitalWrite(DIR_M2, dir1);
    analogWrite(PWM_M2, pwm1); // write to pins
    digitalWrite(DIR_M3, dir1);
    analogWrite(PWM_M3, pwm1); // write to pins
    digitalWrite(DIR_M4, dir1);
    analogWrite(PWM_M4, pwm1); // write to pins
  

    Serial.println(reading);
 
  }
  else if(reading < 170){
    digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M1, pwm1); // write to pins
    digitalWrite(DIR_M2, dir1);
    analogWrite(PWM_M2, pwm1); // write to pins
    digitalWrite(DIR_M3, dir2);
    analogWrite(PWM_M3, pwm1); // write to pins
    digitalWrite(DIR_M4, dir2);
    analogWrite(PWM_M4, pwm1); // write to pins

    Serial.println(reading);
  
  }

  else{ 

   digitalWrite(DIR_M1, 0);
    analogWrite(PWM_M1, 0); // write to pins
    digitalWrite(DIR_M2, 0);
    analogWrite(PWM_M2, 0); // write to pins
     digitalWrite(DIR_M3, 0);
    analogWrite(PWM_M3, 0); // write to pins
    digitalWrite(DIR_M4, 0);
    analogWrite(PWM_M4, 0); // write to pins

}
}

float compass() {
  sensors_event_t event; 
  mag.getEvent(&event);
 
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  return headingDegrees;
}
