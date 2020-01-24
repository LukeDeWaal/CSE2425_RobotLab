#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <std_msgs/String.h>

// Yellow LED
const int LED_pin = 13;

// Ultrasonwsic Sensor Pins
const int trigger_pin = 23;
const int echo_pin = 22;
const float stopping_distance = 7.5; // cm

// Motor Pins
// Left
const int rev_1 = 7;
const int en_1  = 24;
const int fwd_1 = 6;

// Right
const int rev_2 = 3;
const int en_2  = 25;
const int fwd_2 = 2;

// Bluetooth Pins
const int BT_TX = 18;
const int BT_RX = 19;

// Further Useful Variables
float dist = 0;
float joystick_X = 0.0;
float joystick_Y = 0.0;
float minval_range = 2.0;
char input;
ii
// ROS Stuff
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

//ros::Timer my_timer = nh.createTimer(ros::Duration(1), callback);

int vx = 0;
int vy = 0;
int vz = 0; 

int ax = 0;
int ay = 0;
int az = 0;

char buffer[100];

int counter = 0;

void velCallback(  const geometry_msgs::Twist& vel)
    {

                
        vx = (int)(vel.linear.x*255);
        vy = (int)(vel.linear.y*255);
        vz = (int)(vel.linear.z*255);

        ax = (int)(vel.angular.x*255);
        ay = (int)(vel.angular.y*255);
        az = (int)(vel.angular.z*255);

        dist = distance();
        
        nh.loginfo("Method Called");
        snprintf ( buffer, 100, "(Vx, Vy, Vz) = (%d, %d, %d)", vx, vy, vz);
        nh.loginfo(buffer);
        snprintf ( buffer, 100, "(ax, ay, az) = (%d, %d, %d)", ax, ay, az);
        nh.loginfo(buffer);

        if(stop_car()){
          return;
        }
        else{
          if (vx >= 1 && az == 0) //Forward move.
          {
          set_low(rev_1, rev_2);
          forward();
          counter = 0;
          }
          else if (vx <= -1 && az == 0)   //Backward move
          {
             set_low(fwd_1, fwd_2);
             reverse();
             counter = 0;
          }
          else if(-1 < vx < 1 && az == 0)
          {
             set_low(rev_1, rev_2);
             set_low(fwd_1, fwd_2);
             hold();
             counter = 0;
          }
          else if(az >= 1 && vx == 0){
            set_low(rev_1, rev_2);
            set_low(fwd_1, fwd_2);
            turn_left();
            counter = 0;
          }
          else if(az <= -1 && vx == 0){
            set_low(rev_1, rev_2);
            set_low(fwd_1, fwd_2);
            turn_right();
            counter = 0;
          }
          else{
            set_low(rev_1, rev_2);
            set_low(fwd_1, fwd_2);
            hold();
            counter = 0;
          }
          
        }
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);

void setup(){
 
  nh.initNode();
  nh.subscribe(sub);

  set_inputs( fwd_1, fwd_2, 
              rev_1, rev_2, 
              en_1, en_2, 
              BT_RX, 
              echo_pin, LED_pin
  );
              
  set_outputs(BT_RX, trigger_pin);

  set_low(rev_1, rev_2);
  set_low(fwd_1, fwd_2);
  
  digitalWrite(en_1, HIGH);
  digitalWrite(en_2, HIGH);
}



void loop() {
  
  nh.spinOnce();
  stop_car();
  if(counter >= 140){
    snprintf ( buffer, 100, "Have not received a command for %d loops", counter);
    nh.loginfo(buffer);
    hold();
    counter = 0;
  }
  counter++;
}

bool stop_car(){
  dist = distance();
  if(!free_path(dist) && !(dist < 0.1)){
    snprintf ( buffer, 100, "Object detected at %d [cm]", (int)(dist));
    nh.loginfo(buffer);
    if (vx <= -1 && az == 0){
      reverse();
    }
    else{
      hold();  
    }
    return true;
  }
  return false;
}

template<typename T>
void set_inputs(T first){
  pinMode(first, INPUT);
}

template<typename T, typename... Pins >
void set_inputs(T first, Pins... pins){
  pinMode(first, INPUT);
  set_inputs(pins...);
}

template<typename T>
void set_outputs(T first){
  pinMode(first, OUTPUT);
}

template<typename T, typename... Pins >
void set_outputs(T first, Pins... pins){
  pinMode(first, OUTPUT);
  set_outputs(pins...);
}

void set_low(const int pin_1, const int pin_2){
  analogWrite(pin_1, 0);
  analogWrite(pin_2, 0);
}


void forward(){
  hold();
  analogWrite(fwd_1, vx);
  analogWrite(fwd_2, vx);
}

void turn_left(){
  hold();
  analogWrite(rev_1, az);
  analogWrite(fwd_2, az);
}

void turn_right(){
  hold();
  analogWrite(rev_2, -1*az);
  analogWrite(fwd_1, -1*az);
}

void reverse(){
  hold();
  analogWrite(rev_1, -1*vx);
  analogWrite(rev_2, -1*vx);
}

void hold(){

  set_low(fwd_1, fwd_2);
  set_low(rev_1, rev_2);
  delay(10);
}

double microsecondsToCentimeters(double microseconds) {
  return microseconds * 0.0343 / 2;
}


float distance(){

  float duration, cm;

  //pinMode(trigger, OUTPUT);
  digitalWrite(trigger_pin, LOW);
  digitalWrite(trigger_pin, HIGH);
  digitalWrite(trigger_pin, LOW);

  //pinMode(echo, INPUT);
  duration = pulseIn(echo_pin, HIGH);
  
  cm = microsecondsToCentimeters(duration);

  return cm;
}

bool free_path(float distance){
  
  if(distance <= stopping_distance){
    return false;
  } else {
    return true;
  }
}

//void keyboard_to_movement(char input){
//
//  if(input == 'w'){
//    forward();
//    //Serial.print("^");
//  }
//  else if(input == 's'){
//    reverse();
//    //Serial.print("v");
//  }
//  else if(input == 'a'){
//    turn_left();
//    //Serial.print("<");
//  }
//  else if(input == 'd'){
//    turn_right();
//    //Serial.print(">");
//  }
//  else {
//    hold();
//    //Serial.print("X");
//  }  
//  //Serial.println();
//}
