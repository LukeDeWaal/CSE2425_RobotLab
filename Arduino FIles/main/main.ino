#include <iostream>

double microsecondsToCentimeters(double microseconds) {
   return microseconds / 29 / 2;
};

bool free_path(double distance, int trigger, int echo){

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
	}
	else{
		return true;
	}
}

void setup(){

	const int trigger_pin = 23;
	const int echo_pin = 22;
  	
}


void loop(){

  	Serial.print(free_path(5.0, trigger_pin, echo_pin));
	Serial.print("cm");
	Serial.println();

}
