// Ultrasonic Sensor Pins
const int trigger_pin = 23;
const int echo_pin = 22;
const double stopping_distance = 5.0; // cm

// Motor Pins
// Left
const int rev_1 = 7;
const int en_1   = 24;
const int fwd_1 = 6;

// Right
const int rev_2 = 3;
const int en_2  = 25;
const int fwd_2 = 2;

// Bluetooth Pins
const int BT_TX = 18;
const int BT_RX = 19;

void setup(){
  Serial.begin(9600);
}

void loop() {
  Serial.print(free_path(stopping_distance, trigger_pin, echo_pin));
  Serial.print("cm");
  Serial.println();

  forward();
  Serial.print(digitalRead(fwd_1));
  delay(500);
  turn_left();

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

void set_low(const int pin_1, const int pin_2){
  
  set_inputs(pin_1, pin_2);
  digitalWrite(pin_1, LOW);
  digitalWrite(pin_2, LOW);
}

void forward(){
  pinMode(fwd_1, INPUT);
  pinMode(fwd_2, INPUT);
  pinMode(rev_1, INPUT);
  pinMode(rev_2, INPUT);

  set_low(fwd_1, fwd_2);
  set_low(rev_1, rev_2);
  digitalWrite(fwd_1, HIGH);
  digitalWrite(fwd_2, HIGH);
}

void turn_left(){
  pinMode(fwd_1, INPUT);
  pinMode(fwd_2, INPUT);
  pinMode(rev_1, INPUT);
  pinMode(rev_2, INPUT);

  set_low(fwd_1, rev_2);
  digitalWrite(rev_1, HIGH);
  digitalWrite(fwd_2, HIGH);

}

void turn_right(){
  pinMode(fwd_1, INPUT);
  pinMode(fwd_2, INPUT);
  pinMode(rev_1, INPUT);
  pinMode(rev_2, INPUT);

  set_low(fwd_2, rev_1);
  digitalWrite(rev_2, HIGH);
  digitalWrite(fwd_1, HIGH);
}

void reverse(const int rev_pin_1, const int rev_pin_2){
  pinMode(fwd_1, INPUT);
  pinMode(fwd_2, INPUT);
  pinMode(rev_1, INPUT);
  pinMode(rev_2, INPUT);

  set_low(rev_pin_1, rev_pin_2);
  digitalWrite(rev_pin_1, HIGH);
  digitalWrite(rev_pin_2, HIGH);
}

void stop(){
  pinMode(fwd_1, INPUT);
  pinMode(fwd_2, INPUT);
  pinMode(rev_1, INPUT);
  pinMode(rev_2, INPUT);

  set_low(fwd_1, fwd_2);
  set_low(rev_1, rev_2);
}

double microsecondsToCentimeters(double microseconds) {
  return microseconds / 29 / 2;
}

bool free_path(const double distance, const int trigger, const int echo){
  double duration, cm;
  pinMode(trigger, OUTPUT);
  digitalWrite(trigger, LOW);
  digitalWrite(trigger, HIGH);
  digitalWrite(trigger, LOW);
  pinMode(echo, INPUT);
  duration = pulseIn(echo, HIGH);
  cm = microsecondsToCentimeters(duration);
  
  if(cm <= distance){
    return false;
  } else {
    return true;
  }
}
