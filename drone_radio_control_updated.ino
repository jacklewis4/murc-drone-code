
#include <Wire.h>
#include <Servo.h>


Servo north_prop;
Servo south_prop;
Servo east_prop;
Servo west_prop;

//radio control stuff
byte PWM_X = 12;
byte PWM_Y = 10;
byte PWM_THROTTLE = 11;
 
int pwm_value_x;
int pwm_value_y;
int pwm_value_throttle;
 
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];




float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

//X axis PID values //
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

//Y axis PID values //
float PID2, pwmLeft2, pwmRight2, error2, previous_error2;
float pid_p2=0;
float pid_i2=0;
float pid_d2=0;

/////////////////PID CONSTANTS/////////////////
double kp=1;
double ki=0;
double kd=0.1;

double kp2=1;
double ki2=0;
double kd2=0.1;
///////////////////////////////////////////////

double throttle=1300; //initial value of throttle to the motors
float desired_X_angle = 0;
float desired_Y_angle = 0;


void setup() {
  //radio stuff
  pinMode(PWM_X, INPUT);
  pinMode(PWM_Y, INPUT);
  pinMode(PWM_THROTTLE, INPUT);
 
 
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);
  north_prop.attach(6);
  south_prop.attach(5);
  east_prop.attach(9);
  west_prop.attach(3);
 

  time = millis(); //Start counting time in milliseconds

  north_prop.writeMicroseconds(1000);
  south_prop.writeMicroseconds(1000);
  east_prop.writeMicroseconds(1000);
  west_prop.writeMicroseconds(1000);
 
  delay(7000);
}

void loop() {

  //radio stuff
  pwm_value_x = pulseIn(PWM_X, HIGH);
  pwm_value_y = pulseIn(PWM_Y, HIGH);
  pwm_value_throttle = pulseIn(PWM_THROTTLE, HIGH);
 
  desired_X_angle = map(pwm_value_x,800,2200,-40,40);
  desired_Y_angle = map(pwm_value_y,800,2200,-40,40);
  throttle = pwm_value_throttle;

/////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000;
   
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

   
 
/*///////////////////////////P I D XXXXXXXX///////////////////////////////////*/

error = Total_angle[0] - desired_X_angle;
pid_p = kp*error;
if(-3 <error <3)
{
  pid_i = pid_i+(ki*error);  
}
pid_d = kd*((error - previous_error)/elapsedTime);
PID = 4*(pid_p + pid_i + pid_d);

if(PID < -1000){
  PID= -1000;}
if(PID > 1000){
  PID= 1000;}

pwmLeft = throttle + PID;
pwmRight = throttle - PID;

//Right
if(pwmRight < 1000){
  pwmRight= 1000;}
if(pwmRight > 2000){
  pwmRight=2000;}
//Left
if(pwmLeft < 1000){
  pwmLeft= 1000;}
if(pwmLeft > 2000){
  pwmLeft=2000;}

////////////////////////////P  I  D  2  /  YYYYYYYY ///////////////////////////
 
error2 = Total_angle[1] - desired_Y_angle;
pid_p2 = kp2*error2;
if(-3 <error2 <3)
{
  pid_i2 = pid_i2+(ki2*error2);  
}
pid_d2 = kd2*((error2 - previous_error2)/elapsedTime);
PID2 = 4*(pid_p2 + pid_i2 + pid_d2);

if(PID2 < -1000){
  PID2= -1000;}
if(PID2 > 1000){
  PID2= 1000;}


pwmLeft2 = throttle + PID2;
pwmRight2 = throttle - PID2;

//Right
if(pwmRight2 < 1000){
  pwmRight2= 1000;}
if(pwmRight2 > 2000){
  pwmRight2=2000;}
//Left
if(pwmLeft2  < 1000){
  pwmLeft2 = 1000;}
if(pwmLeft2  > 2000){
  pwmLeft2 =2000;}

if (throttle < 1200){
  pwmLeft2 = 1000;
  pwmRight2 = 1000;
  pwmLeft = 1000;
  pwmRight = 1000;
}

north_prop.writeMicroseconds(pwmLeft2);
south_prop.writeMicroseconds(pwmRight2);
east_prop.writeMicroseconds(pwmLeft);
west_prop.writeMicroseconds(pwmRight);

previous_error = error; //Remember to store the previous error.
previous_error = error2;

Serial.print("X angle input = ");
Serial.print(desired_X_angle);
Serial.print("  Y angle input = ");
Serial.print(desired_Y_angle);
Serial.print("  Throttle input = ");
Serial.print(throttle);
Serial.print(" X axis angle = ");
Serial.print(Total_angle[0]);
Serial.print(" Y axis angle = ");
Serial.println(Total_angle[1]);
Serial.print(" pwm left2 = ");
Serial.print(pwmLeft2);
Serial.print(" pwm left = ");
Serial.println(pwmLeft);

}//end of loop void
