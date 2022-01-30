#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>

#include <geometry_msgs/Twist.h>
#include <Encoder.h>
#include <geometry_msgs/Vector3.h>

//////////////// pin setup /////////////////

#define LED_PIN 13
#define r_dir1_pin 5
#define r_dir2_pin 6
#define r_pwm_pin 8

#define l_dir1_pin 20
#define l_dir2_pin 1
#define l_pwm_pin 21

#define l_encoder_a 15
#define l_encoder_b 14

#define r_encoder_a 11
#define r_encoder_b 12

// ========= PID COntrol ===========
// PID control parameters
double kp = 0.9;
double ki = 0.05;
double kd = 0.03;
float PPR = 1600;
long pl, pr;

/*Left Wheel Control*/
unsigned long curTimeL, prevTimeL, diffTimeL;
long curTickL, prevTickL, diffTickL;
double errorL, lastErrorL, cumErrorL, rateErrorL, setRPML;
double control_outL;
double measuredRPML;
double desiredRPMR,desiredRPML;

/*Right Wheel Control*/
unsigned long curTimeR, prevTimeR, diffTimeR;
long curTickR, prevTickR, diffTickR;
double errorR, lastErrorR, cumErrorR, rateErrorR, setRPMR;
double control_outR;
double measuredRPMR;

ros::NodeHandle nh;
////////////////Variables/////////////////////
std_msgs::Int32 right_tick;
std_msgs::Int32 left_tick;
std_msgs::Float32 rpm_left_msg;
std_msgs::Float32 rpm_right_msg;


ros::Publisher rticks_pub("/right_wheel_tick", &right_tick);
ros::Publisher lticks_pub("/left_wheel_tick", &left_tick);

ros::Publisher rpm_left_pub("/rpm_left", &rpm_left_msg);
ros::Publisher rpm_right_pub("/rpm_right", &rpm_right_msg);

double computePIDL(double control_cmd,
                   long inTick)
{
  unsigned int lpwm_value;
  curTickL = inTick;
  curTimeL = millis();
  setRPML = control_cmd;
  diffTickL = curTickL - prevTickL;
  diffTimeL =  (curTimeL - prevTimeL);
  measuredRPML = ((diffTickL / PPR) / (diffTimeL * 0.001)) * 60;
  if (setRPML >= 0) {
    errorL = setRPML - measuredRPML;
  } else if (setRPML < 0) {
    errorL = -setRPML + measuredRPML;
  }

  cumErrorL += errorL * diffTimeL;
  rateErrorL = (errorL - lastErrorL) / diffTimeL;

  control_outL = kp * errorL + ki * cumErrorL + kd * rateErrorL;
  if (control_outL < 0) {
    control_outL = 0;
  }
  lastErrorL = errorL;
  prevTickL = curTickL;
  prevTimeL = curTimeL;

  if (setRPML > 0)
  {
    digitalWrite(l_pwm_pin, HIGH);
    digitalWrite(l_dir2_pin, LOW);
    lpwm_value  = control_outL;
  }
  else if (setRPML < 0)
  {
    digitalWrite(l_pwm_pin, HIGH);
    digitalWrite(l_dir2_pin, HIGH);
    lpwm_value  = control_outL;
  }
  else{
    digitalWrite(l_pwm_pin, HIGH);
    digitalWrite(l_dir2_pin, LOW);
    lpwm_value = 0;
  }

  analogWrite(l_dir1_pin, lpwm_value);

  return control_outL;
}


double computePIDR(double control_cmd,long inTick)
{
  unsigned int rpwm_value;
  curTickR = inTick;
  curTimeR = millis();
  setRPMR = control_cmd;
  diffTickR = curTickR - prevTickR;
  diffTimeR =  (curTimeR - prevTimeR);
  measuredRPMR = ((diffTickR / PPR) / (diffTimeR * 0.001)) * 60;

  if (setRPMR >= 0) {
    errorR = setRPMR - measuredRPMR;
  } else if (setRPMR < 0) {
    errorR = -setRPMR + measuredRPMR;
  }
  cumErrorR += errorR * diffTimeR;
  rateErrorR = (errorR - lastErrorR) / diffTimeR;

  control_outR = kp * errorR + ki * cumErrorR + kd * rateErrorR;


  lastErrorR = errorR;


  prevTickR = curTickR;
  prevTimeR = curTimeR;

  if (setRPMR > 0)
  {
    digitalWrite(r_pwm_pin, HIGH);
    digitalWrite(r_dir2_pin, LOW);
    rpwm_value  = control_outR;
  }
  else if (setRPMR < 0)
  {
    digitalWrite(r_pwm_pin, HIGH);
    digitalWrite(r_dir2_pin, HIGH);
    rpwm_value  = control_outR;
  }
  else{
    digitalWrite(r_pwm_pin, HIGH);
    digitalWrite(r_dir2_pin, LOW);
    rpwm_value = 0;
  }
  if (rpwm_value < 0) {
    rpwm_value = 0;
  }
  analogWrite(r_dir1_pin, rpwm_value);

  return control_outR;
}

void rightwheel_callback(const std_msgs::Float32 &power)
{
  desiredRPMR = power.data;

}

void leftwheel_callback(const std_msgs::Float32 &power)
{

  desiredRPML = power.data;

}

ros::Subscriber<std_msgs::Float32> sub_right("/wheel_power_right",
    &rightwheel_callback );
ros::Subscriber<std_msgs::Float32> sub_left("/wheel_power_left",
    &leftwheel_callback );


void setup() {
  nh.initNode();
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
  nh.advertise(rticks_pub);
  nh.advertise(lticks_pub);

  nh.advertise(rpm_left_pub);
  nh.advertise(rpm_right_pub);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(r_dir1_pin, OUTPUT);
  pinMode(r_dir2_pin, OUTPUT);
  pinMode(r_pwm_pin, OUTPUT);
  pinMode(l_dir1_pin, OUTPUT);
  pinMode(l_dir2_pin, OUTPUT);
  pinMode(l_pwm_pin, OUTPUT);
  pinMode(r_encoder_a, INPUT);
  pinMode(r_encoder_b, INPUT);
  pinMode(l_encoder_a, INPUT);
  pinMode(l_encoder_b, INPUT);

 
  nh.loginfo("Gyro calibration is in process");
 
}
Encoder r_encoder(r_encoder_a, r_encoder_b);
Encoder l_encoder(l_encoder_a, l_encoder_b);
long old_pl = -999;
long old_pr = -999;

long curRPM_time;
long prevRPM_time;
long curPl;
long curPr;
long prevPl;
long prevPr;
long diffRPM_time;
float RPM_l, RPM_r;

void counter_tick()
{
  pl = l_encoder.read();

  if (pl != old_pl) {
    old_pl = pl;
  }

  pr = r_encoder.read();

  if (pr != old_pr) {
    old_pr = pr;
  }

  left_tick.data = pl;
  right_tick.data = pr;
   
  rticks_pub.publish(&right_tick);
  lticks_pub.publish(&left_tick);
  delay(20);
}
void counter_RPM(long inPl, long inPr) {

  curPl = inPl;
  curPr = inPr;
  curRPM_time = millis();
  diffRPM_time = curRPM_time - prevRPM_time ;
  RPM_l = (((curPl - prevPl) / PPR) / (diffRPM_time * 0.001)) * 60;
  RPM_r = (((curPr - prevPr) / PPR) / (diffRPM_time * 0.001)) * 60;


  prevPl = curPl;
  prevPr = curPr;
  prevRPM_time = curRPM_time;
  rpm_left_msg.data = RPM_l; // RPM_l;
  rpm_right_msg.data = RPM_r;
  rpm_left_pub.publish(&rpm_left_msg);
  rpm_right_pub.publish(&rpm_right_msg);
}

void loop()
{
  counter_tick();
  computePIDR(desiredRPMR,pr);
  computePIDL(desiredRPML,pl);
  counter_RPM(pl, pr);

  nh.spinOnce();
  delay(35);
}
