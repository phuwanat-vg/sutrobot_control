#include <ros.h>
#include "Arduino.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include "PinChangeInterrupt.h"
#include <Servo.h>    //servo addition
ros::NodeHandle nh;

char ns = "sutrobot1";

int cam_servo_pin = 11;
Servo cam_servo; //servo add
int cur_angle;
int new_angle;
int pos = 0;
#define LED_BUILTIN 13


#define L_MOTOR_PWM 6
#define L_MOTOR_DIR1 8
#define L_MOTOR_DIR2 9
#define R_MOTOR_PWM 5
#define R_MOTOR_DIR1 10
#define R_MOTOR_DIR2 7

/*Left Wheel Control*/
float PPR = 840.0;
float kp = 0.5;
float ki = 0.05;

unsigned long curTimeL, prevTimeL, diffTimeL;
long curTickL, prevTickL, diffTickL;
double errorL,setRPML,lastErrorL,cumErrorL;
double control_outL;
double measuredRPML;
double desiredRPMR,desiredRPML;

/*Right Wheel Control*/
unsigned long curTimeR, prevTimeR, diffTimeR;
long curTickR, prevTickR, diffTickR;
double errorR, setRPMR,lastErrorR,cumErrorR;
double control_outR;
double measuredRPMR;


std_msgs::Int32 rticks_msg;
std_msgs::Int32 lticks_msg;
std_msgs::Float32 rpm_left_msg;
std_msgs::Float32 rpm_right_msg;

ros::Publisher rticks_pub("sutrobot1/tick_wheel_right", &rticks_msg);
ros::Publisher lticks_pub("sutrobot1/tick_wheel_left", &lticks_msg);
ros::Publisher rpm_left_pub("sutrobot1/rpm_left", &rpm_left_msg);
ros::Publisher rpm_right_pub("sutrobot1/rpm_right", &rpm_right_msg);

void turnWheelL(float setpoint,long inTick) {
  unsigned int lpwm_value;
  curTickL = inTick;
  curTimeL = millis();
  setRPML = setpoint;
  diffTickL = curTickL - prevTickL;
  diffTimeL =  (curTimeL - prevTimeL);
  measuredRPML = ((diffTickL / PPR) / (diffTimeL * 0.001)) * 60;

  rpm_left_msg.data = measuredRPML;

  if (setRPML >= 0) {
    errorL = setRPML - measuredRPML;
  } else if (setRPML < 0) {
    errorL = -setRPML + measuredRPML;
  }
  cumErrorL += errorL * diffTimeL;
  control_outL = kp * errorL+ ki*cumErrorL;


  lastErrorL = errorL;


  prevTickL = curTickL;
  prevTimeL = curTimeL;

  if (setRPML > 0)
  {
    digitalWrite(L_MOTOR_DIR1, LOW);
    digitalWrite(L_MOTOR_DIR2, HIGH);
    lpwm_value  = control_outL;
  }
  else if (setRPML < 0)
  {
    digitalWrite(L_MOTOR_DIR1, HIGH);
    digitalWrite(L_MOTOR_DIR2, LOW);
    lpwm_value  = control_outL;
  }
  else{
    digitalWrite(L_MOTOR_DIR1, LOW);
    digitalWrite(L_MOTOR_DIR2, LOW);
    lpwm_value = 0;
  }
  if (lpwm_value < 0) {
    lpwm_value = 0;
  }
  analogWrite(L_MOTOR_PWM, lpwm_value);   
}


void turnWheelR(float setpoint,long inTick) {
  unsigned int rpwm_value;
  curTickR = inTick;
  curTimeR = millis();
  setRPMR = setpoint;
  diffTickR = curTickR - prevTickR;
  diffTimeR =  (curTimeR - prevTimeR);
  measuredRPMR = ((diffTickR / PPR) / (diffTimeR * 0.001)) * 60;
  rpm_right_msg.data = measuredRPMR;
  if (setRPMR >= 0) {
    errorR = setRPMR - measuredRPMR;
  } else if (setRPMR < 0) {
    errorR = -setRPMR + measuredRPMR;
  }

  cumErrorR += errorR * diffTimeR;
  control_outR = kp * errorR+ ki*cumErrorR;


  lastErrorR = errorR;


  prevTickR = curTickR;
  prevTimeR = curTimeR;

  if (setRPMR > 0)
  {
    digitalWrite(R_MOTOR_DIR1, LOW);
    digitalWrite(R_MOTOR_DIR2, HIGH);
    rpwm_value  = control_outR;
  }
  else if (setRPMR < 0)
  {
    digitalWrite(R_MOTOR_DIR1, HIGH);
    digitalWrite(R_MOTOR_DIR2, LOW);
    rpwm_value  = control_outR;
  }
  else{
    digitalWrite(R_MOTOR_DIR1, LOW);
    digitalWrite(R_MOTOR_DIR2, LOW);
    rpwm_value = 0;
  }
  if (rpwm_value < 0) {
    rpwm_value = 0;
  }
  analogWrite(R_MOTOR_PWM, rpwm_value);   
}
void rightWheelCb( const std_msgs::Float32 &wheel_power ) {


    desiredRPMR = wheel_power.data;
    
}

void leftWheelCb( const std_msgs::Float32 &wheel_power ) {
  
    desiredRPML = wheel_power.data;
}

void servoCb( const std_msgs::Int16 &angle)
{
  new_angle = angle.data;
  if(new_angle>(cur_angle-90)){
    
  for (pos = cur_angle;pos<new_angle+90; pos+=1 ){
    cam_servo.write(pos);
    delay(5);
  }
  cur_angle = new_angle+90;
  }
  else if((new_angle<(cur_angle-90))&&(new_angle>=-45)){
  for (pos = cur_angle;pos>new_angle+90; pos-=1 ){
    cam_servo.write(pos);
    delay(5);
  }
  cur_angle = new_angle+90;
  }
}

ros::Subscriber<std_msgs::Float32> sub_right("sutrobot1/wheel_power_right",
                                            &rightWheelCb );
ros::Subscriber<std_msgs::Float32> sub_left("sutrobot1/wheel_power_left",
                                           &leftWheelCb );
ros::Subscriber<std_msgs::Int16> sub_servo("sutrobot1/servo", &servoCb);

int r_encoder_pinA = 2; //Interrupt1
int r_encoder_pinB = 3; //Interrupt0
int l_encoder_pinA = A1;
int l_encoder_pinB = A0;

volatile long pulse_l=0;
volatile long pulse_r=0;
unsigned long timeold=0;


//unsigned int pulse_per_rev=20; //Pulse per revolustion of index disc

void counter_l()
{

  if( digitalRead(l_encoder_pinB) == 0 ) {
    if ( digitalRead(l_encoder_pinA) == 0 ) {
      // A fell, B is low
      pulse_l--; // moving reverse
    } else {
      // A rose, B is low
      pulse_l++; // moving forward
    }
  }

}
void counter_r()
{

   if( digitalRead(r_encoder_pinB) == 0 ) {
    if ( digitalRead(r_encoder_pinA) == 0 ) {
      // A fell, B is low
      pulse_r=pulse_r-1; // moving reverse
    } else {
      // A rose, B is low
      pulse_r=pulse_r+1; // moving forward
    }
  }
 
}





void setup()
{
  cam_servo.attach(cam_servo_pin);
  cur_angle=90;
  cam_servo.write(cur_angle);
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(L_MOTOR_DIR1, OUTPUT);
  pinMode(L_MOTOR_DIR2, OUTPUT);
  
  pinMode(R_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_DIR1, OUTPUT);
  pinMode(R_MOTOR_DIR2, OUTPUT);
  
  // Init motors to stop
  digitalWrite(L_MOTOR_DIR1, LOW);
  digitalWrite(L_MOTOR_DIR2, LOW);
  digitalWrite(R_MOTOR_DIR1, LOW);
  digitalWrite(R_MOTOR_DIR2, LOW);
  analogWrite(L_MOTOR_PWM, 0);
  analogWrite(R_MOTOR_PWM, 0);

  //Encoder setup
  pinMode(r_encoder_pinA, INPUT);
  pinMode(r_encoder_pinB, INPUT);
  pinMode(l_encoder_pinA, INPUT);
  pinMode(l_encoder_pinB, INPUT);


  //Initialize Value
  pulse_l = 0;
  pulse_r = 0;
  timeold = 0;

  attachInterrupt(digitalPinToInterrupt(2), counter_r,CHANGE);
 
  attachPCINT(digitalPinToPCINT(l_encoder_pinA), counter_l, CHANGE);

  
  
  //ROS setup
  nh.initNode();
  Serial.begin(57600);
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
  nh.subscribe(sub_servo);
  nh.advertise(rpm_left_pub);
  nh.advertise(rpm_right_pub);

  //nh.advertise(rpm_left_pub);
  //nh.advertise(rpm_right_pub);
  nh.advertise(rticks_pub);
  nh.advertise(lticks_pub);
  delay(20);
}

void loop()
{
  turnWheelR(desiredRPMR,-pulse_r);
  turnWheelL(desiredRPML,pulse_l);
  if (millis() - timeold >= 100){  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
 
   rticks_msg.data = -pulse_r;
   lticks_msg.data = pulse_l;

   //Publish data
   
   rticks_pub.publish(&rticks_msg);
   lticks_pub.publish(&lticks_msg);
   rpm_left_pub.publish(&rpm_left_msg);
   rpm_right_pub.publish(&rpm_right_msg);
   //reset parameter
   timeold = millis();
   //pulse_r = 0;
   //pulse_l = 0;
   
   //Write it out to serial port
   //Serial.print("RPM = ");
   //Serial.println(rpm_r,DEC);
   
   }

  
   //nh.loginfo("Log Me");
   nh.spinOnce();
  
   // wait for a second
   delay(20);
   
}
