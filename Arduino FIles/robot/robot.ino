#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <std_msgs/String.h>

// Yellow LED
const int LED_pin = 13;

// Ultrasonwsic Sensor Pins
const int trigger_pin = 23;
const int echo_pin = 22;
const float stopping_distance = 8.0; // cm
const float critical_distance = 4.0; // cm

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

// ROS Stuff
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

int x;
int y; 

char buffer[100];

void velCallback(  const geometry_msgs::Twist& vel)
    {
        x = (int)(vel.linear.x*1000);
        y = vel.linear.y;

        nh.loginfo("Method Called");
        snprintf ( buffer, 100, "X = %d", x);
        nh.loginfo(buffer);
        
        if (x > 1) //Forward move.
          {
          //set_low(fwd_1, fwd_2);
          set_low(rev_1, rev_2);
          forward();
          nh.loginfo("LED");
          digitalWrite(LED_pin, HIGH);
          delay(250);
          digitalWrite(LED_pin, LOW);
          }
        else if (x < -1)   //Backward move
        {
           reverse();
        }
        else 
        {
           hold();
        }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);


void setup(){

  set_inputs( fwd_1, fwd_2, 
              rev_1, rev_2, 
              en_1, en_2, 
              BT_RX, 
              echo_pin, LED_pin);
              
  set_outputs(BT_RX, trigger_pin);
  
  set_low(rev_1, rev_2);
  set_low(fwd_1, fwd_2);
  
  nh.initNode();
  nh.subscribe(sub);
  digitalWrite(en_1, HIGH);
  digitalWrite(en_2, HIGH);
}



void loop() {
  dist = distance();
  snprintf ( buffer, 100, "D = %f", dist);
  nh.loginfo(buffer);
  //analogWrite(fwd_1, 255);
  //analogWrite(fwd_2, 255);
  
  nh.spinOnce();
  
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
  digitalWrite(pin_1, LOW);
  digitalWrite(pin_2, LOW);
}

void forward(){
  hold();
  digitalWrite(fwd_1, HIGH);
  digitalWrite(fwd_2, HIGH);
}

void turn_left(){
  hold();
  digitalWrite(rev_1, HIGH);
  digitalWrite(fwd_2, HIGH);

}

void turn_right(){
  hold();
  digitalWrite(rev_2, HIGH);
  digitalWrite(fwd_1, HIGH);
}

void reverse(){
  hold();
  digitalWrite(rev_1, HIGH);
  digitalWrite(rev_2, HIGH);
}

void hold(){

  set_low(fwd_1, fwd_2);
  set_low(rev_1, rev_2);
}

double microsecondsToCentimeters(double microseconds) {
  return microseconds / 29 / 2;
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

void keyboard_to_movement(char input){

  if(input == 'w'){
    forward();
    //Serial.print("^");
  }
  else if(input == 's'){
    reverse();
    //Serial.print("v");
  }
  else if(input == 'a'){
    turn_left();
    //Serial.print("<");
  }
  else if(input == 'd'){
    turn_right();
    //Serial.print(">");
  }
  else {
    hold();
    //Serial.print("X");
  }  
  //Serial.println();
}
