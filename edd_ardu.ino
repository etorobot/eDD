#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <FastLED.h>
#include <std_msgs/Float32.h>
#include <TimerOne.h>

#define EN_A 13
#define IN1 12
#define IN2 11
#define EN_B 8
#define IN4 9
#define IN3 10
#define LOOP_TIME 200000 
#define enc_L_pin 18
#define enc_R_pin 19
#define head_pin 30
#define tail_pin 31
#define headlites 40
#define tailites 20

CRGB headLeds[headlites];
CRGB taiLeds[tailites];

double rightwheel=0.0, leftwheel=0.0, radius = 0.08, wheel_base = 0.20,
       lite, angular=0, linear=0;
unsigned int PWM_min = 0, PWM_max = 255, left_count=0, right_count = 0;
  
ros::NodeHandle nh;
std_msgs::Float32 left_vel;
std_msgs::Float32 right_vel;

void cmd_vel_CB( const geometry_msgs::Twist& msg){
  angular = msg.angular.z;
  linear = msg.linear.x;
  rightwheel = (linear/radius) + ((angular*wheel_base)/(2.0*radius));
  leftwheel = (linear/radius) - ((angular*wheel_base)/(2.0*radius));  }

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_CB );
void Motors_init();
void MotorL(int PWML);
void MotorR(int PWMR);
//void lights();

ros::Publisher left_vel_pub("/left_velocity", &left_vel);
ros::Publisher right_vel_pub("/right_velocity", &right_vel);

void setup(){
 Motors_init();
 enc_setup();
 nh.initNode();
 nh.subscribe(cmd_vel_sub); 
 nh.advertise(left_vel_pub);
 nh.advertise(right_vel_pub);
 Timer1.attachInterrupt( timerIsr );
 FastLED.addLeds<WS2812B, head_pin,RGB>(headLeds,headlites);
 FastLED.addLeds<WS2812B, tail_pin,RGB>(taiLeds,tailites);}

void loop(){
 MotorL(leftwheel*10);
 MotorR(rightwheel*10);
 lights();
 nh.spinOnce(); }
 
void enc_setup(){
pinMode(enc_R_pin, INPUT_PULLUP);
pinMode(enc_L_pin, INPUT_PULLUP);
Timer1.initialize(LOOP_TIME);
attachInterrupt(digitalPinToInterrupt(enc_L_pin), count_L, RISING); 
attachInterrupt(digitalPinToInterrupt(enc_R_pin), count_R, CHANGE); }
void count_L(){ left_count++; }
void count_R(){ right_count++; }

void Motors_init(){
 pinMode(EN_A, OUTPUT);
 pinMode(EN_B, OUTPUT);
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(IN3, OUTPUT);
 digitalWrite(EN_A, LOW);
 digitalWrite(EN_B, LOW);
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, LOW);
 digitalWrite(IN4, LOW);
 digitalWrite(IN3, LOW);  }

void timerIsr(){
  Timer1.detachInterrupt();
  left_vel.data = left_count;
  left_vel_pub.publish(&left_vel);
  right_vel.data = right_count;
  right_vel_pub.publish(&right_vel);
  right_count=0;
  left_count=0;
  Timer1.attachInterrupt( timerIsr ); }

void MotorL(int PWML){
 if (PWML > 0){
     analogWrite(EN_A, abs(PWML));
     digitalWrite(IN1, HIGH);
     digitalWrite(IN2, LOW);  }

 if (PWML < 0){
     analogWrite(EN_A, abs(PWML));
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, HIGH); }

 else{
     analogWrite(EN_A, PWML);
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, LOW);  }
}

void MotorR(int PWMR){

 if (PWMR > 0){
     analogWrite(EN_B, abs(PWMR));
     digitalWrite(IN4, HIGH);
     digitalWrite(IN3, LOW);   }

 if (PWMR < 0){
     analogWrite(EN_B, abs(PWMR));
     digitalWrite(IN4, LOW);
     digitalWrite(IN3, HIGH);  }

  else{
     analogWrite(EN_B, PWMR);
     digitalWrite(IN4, LOW);
     digitalWrite(IN3, LOW);  }
}

void lights(){
  if (leftwheel == rightwheel > 0){
    for(int i =0; i<headlites; i++){
      headLeds[i]=CRGB::White;}
    for(int i=0; i < tailites; i++){
      taiLeds[i]=CRGB::White;}}

  if (leftwheel == rightwheel < 0){
    for(int i =0; i<headlites; i++){
      headLeds[i]=CRGB::Red;}
    for(int i=0; i < tailites; i++){
      taiLeds[i]=CRGB::White;
      FastLED.show();  
      delay(650);    
      taiLeds[i]=CRGB::Black;
      //FastLED.show();
      //delay(150);
      }}  

   if (leftwheel && rightwheel == 0){
    for(int i =0; i<headlites; i++){
      headLeds[i]=CRGB::Red;}
    for(int i=0; i < tailites; i++){
      taiLeds[i]=CRGB::Red;}}

   if (leftwheel > rightwheel){
    for(int i =0; i<(headlites/2); i++){
      headLeds[i]=CRGB::DarkOrange;}
    for(int i=0; i < (tailites/2); i++){
      taiLeds[i]=CRGB::DarkOrange;}}

   if (leftwheel < rightwheel){
    for(int i =((headlites/2)-1); i >=0; i--){
      headLeds[i]=CRGB::DarkOrange;}
    for(int i =((tailites/2)-1); i >=0; i--){
      taiLeds[i]=CRGB::DarkOrange;}}
             
   FastLED.show();
   delay(100);}
   
