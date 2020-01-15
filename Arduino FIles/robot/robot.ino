#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>


// Ultrasonwsic Sensor Pins
const int trigger_pin = 11; //23;
const int echo_pin = 10; //22;
const float stopping_distance = 8.0; // cm
const float critical_distance = 4.0; // cm

// Motor Pins
// Left
const int rev_1 = 9;//7;
const int en_1  = 7;//24;
const int fwd_1 = 6;//6;

// Right
const int rev_2 = 3;//3;
const int en_2  = 4;//25;
const int fwd_2 = 5;//2;

// Bluetooth Pins
const int BT_TX = 18;
const int BT_RX = 19;

//// Joystick Pins
//const int SW_pin = 2;
//const int X_pin = 3;
//const int Y_pin = 4;

// Further Useful Variables
float dist = 0;
float joystick_X = 0.0;
float joystick_Y = 0.0;
float minval_range = 2.0;
char input;

// ROS Stuff
ros::NodeHandle nh;

void velCallback(const geometry_msgs::Twist& vel){
  Serial.print((float)vel.linear.x);
  Serial.println();
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);

void setup(){
  Serial.begin(9600);
  set_inputs(fwd_1, fwd_2, rev_1, rev_2, en_1, en_2, BT_RX, 
              echo_pin);
  set_outputs(BT_RX, trigger_pin);

  //nh.initNode();
  //nh.subscribe(sub);
  
}

void loop() {
  dist = distance();
  Serial.print("~");
  Serial.print(dist);
  Serial.print("cm  -  ");
  //nh.spinOnce();
  
  if(free_path(dist)){
    if(Serial.available()){
      input = Serial.read();
    }
  }
  else{
    input = 'x';   
  }
  keyboard_to_movement(input);
  delay(20);
  
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

  set_low(fwd_1, fwd_2);
  set_low(rev_1, rev_2);
  digitalWrite(fwd_1, HIGH);
  digitalWrite(fwd_2, HIGH);
}

void turn_left(){

  set_low(fwd_1, rev_2);
  digitalWrite(rev_1, HIGH);
  digitalWrite(fwd_2, HIGH);

}

void turn_right(){

  set_low(fwd_2, rev_1);
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
  set_low(en_1, en_2);
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

bool emergency_stop(float distance){
  
 if(distance <= critical_distance){
    return true;
  } else {
    return false;
  } 
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
    Serial.print("^");
  }
  else if(input == 's'){
    reverse();
    Serial.print("v");
  }
  else if(input == 'a'){
    turn_left();
    Serial.print("<");
  }
  else if(input == 'd'){
    turn_right();
    Serial.print(">");
  }
  else {
    hold();
    Serial.print("X");
  }  
  Serial.println();
}

//void joystick_to_movement(){
//
//  read_joystick();
//  
//  if(joystick_X > minval_range && -minval_range < joystick_Y < minval_range){
//    turn_right();
//    Serial.print("Right");
//  }
//  else if(joystick_X < -minval_range && -minval_range < joystick_Y < minval_range){
//    turn_left();
//    Serial.print("Left");
//  }
//  else if(-minval_range < joystick_X < minval_range && joystick_Y > minval_range){
//    forward();
//    Serial.print("Forward");
//  }
//  else if(-minval_range < joystick_X < minval_range && joystick_Y < -minval_range){
//    reverse();
//    Serial.print("Backward");
//  }
//  else{
//    hold();
//    Serial.print("Holding");
//    
//  }
//  Serial.println();
//}
//
//void read_joystick(){
//  joystick_X = ((float)analogRead(X_pin) - 512)/52.15;
//  joystick_Y = ((float)analogRead(Y_pin) - 512)/-52.13;
//}
//
//void joystick_input(){
//  Serial.print(joystick_X);
//  Serial.print(" - ");
//  Serial.print(joystick_Y);
//  Serial.println();
//}
