/*
 * Change Log
 * 
 * code refactored for readability
 * const keyword added were needed
 * some veriable types changed to match expeceted inputs/outputs
 * some if statments changed to else if
 * removed and added some serial prints for debug
 * added test for radio disconect
 * motors now power down if radio disconected or throttle near zero
 */



#include <Wire.h>

#include <Servo.h>
Servo north_prop;
Servo south_prop;
Servo east_prop;
Servo west_prop;

#include "PIDclass.h"
PIDclass Xaxis(1, 0, 0.1);
PIDclass Yaxis(1, 0, 0.1);


//pin def
const byte PWM_X_pin = 12;
const byte PWM_Y_pin = 10;
const byte PWM_THROTTLE_pin = 11;

const byte NORTH_PROP_pin = 6;
const byte SOUTH_PROP_pin = 5;
const byte EAST_PROP_pin = 9;
const byte WEST_PROP_pin = 3;

//constants
const float rad_to_deg = 180/3.141592654;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
double throttle = 1300; //initial value of throttle to the motors
float desired_X_angle = 0;
float desired_Y_angle = 0;
bool radio_receved = false;
bool throttle_zero = true;

void read_radio(){
  unsigned long pwm_value_x = pulseIn(PWM_X_pin, HIGH, 10000); //added timeout as default is one second (1 msec now)
  unsigned long pwm_value_y = pulseIn(PWM_Y_pin, HIGH, 10000); //(which is too long and will cause strange behavure)
  unsigned long pwm_value_throttle = pulseIn(PWM_THROTTLE_pin, HIGH, 10000); //this may still be too long 

  if ((pwm_value_x == 0) || (pwm_value_y == 0) || (pwm_value_throttle == 0)){ //no data receved if zero is read
    radio_receved = false;
  }
  else radio_receved = true;

  desired_X_angle = map(pwm_value_x,800,2200,-40,40);
  desired_Y_angle = map(pwm_value_y,800,2200,-40,40);
  throttle = pwm_value_throttle;

  if (throttle < 1200){
    throttle_zero = true;
  }
  else throttle_zero = false;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
unsigned long elapsedTime, timePrev;
unsigned long time = 0;

void get_time(){
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

void read_IMU(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);
   
  Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();

  /*---X---*/
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  /*---Y---*/
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
   
  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); //Just 4 registers
   
  Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
  Gyr_rawY=Wire.read()<<8|Wire.read();

  /*---X---*/
  Gyro_angle[0] = Gyr_rawX/131.0;
  /*---Y---*/
  Gyro_angle[1] = Gyr_rawY/131.0;
  /*---X axis angle---*/
  Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
  /*---Y axis angle---*/
  Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
  /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
  //Serial.println(Total_angle[1]);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void setup() {
  //debug coms
  Serial.begin(250000);

  //radio stuff
  pinMode(PWM_X_pin, INPUT);
  pinMode(PWM_Y_pin, INPUT);
  pinMode(PWM_THROTTLE_pin, INPUT);

  //IMU comunication
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  //initialise motors
  north_prop.attach(NORTH_PROP_pin);
  south_prop.attach(SOUTH_PROP_pin);
  east_prop.attach(EAST_PROP_pin);
  west_prop.attach(WEST_PROP_pin);

  //start time
  time = millis(); //Start counting time in milliseconds

  north_prop.writeMicroseconds(1000);
  south_prop.writeMicroseconds(1000);
  east_prop.writeMicroseconds(1000);
  west_prop.writeMicroseconds(1000);

  //delay 7 seconds (why?)
  delay(7000);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void loop() {
  read_radio();
  if (!radio_receved) Serial.println("radio not conected"); //debug
  if (throttle_zero) Serial.println("throttle zero"); //debug

  get_time();
  
  if (radio_receved && !throttle_zero){
    read_IMU();
    float Xerror = Total_angle[0] - desired_X_angle;
    float Yerror = Total_angle[0] - desired_Y_angle;
    Xaxis.calculate_PID(elapsedTime, throttle, Xerror);
    Yaxis.calculate_PID(elapsedTime, throttle, Yerror);
    
    north_prop.writeMicroseconds(Yaxis.pwmLeft);
    south_prop.writeMicroseconds(Yaxis.pwmRight);
    east_prop.writeMicroseconds(Xaxis.pwmLeft);
    west_prop.writeMicroseconds(Xaxis.pwmRight);
  }
  else{
    north_prop.writeMicroseconds(1000);
    south_prop.writeMicroseconds(1000);
    east_prop.writeMicroseconds(1000);
    west_prop.writeMicroseconds(1000);
  }
}
