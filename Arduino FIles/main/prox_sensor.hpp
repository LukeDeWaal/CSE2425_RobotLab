
const int trigger_pin = 23;
const int echo_pin = 22;

double microsecondsToCentimeters(double microseconds) {
   return microseconds / 29 / 2;
};

bool free_path(double distance){

	double duration, cm;
	pinMode(trigger_pin, OUTPUT);

	digitalWrite(trigger_pin, LOW);
	digitalWrite(trigger_pin, HIGH);
	digitalWrite(trigger_pin, LOW);

	pinMode(echoPin, INPUT);
	duration = pulseIn(echo_pin, HIGH);
	cm = microsecondsToCentimeters(duration);
	Serial.print(cm);
	Serial.print("cm");
	Serial.println();

	if(cm <= distance){
		return false;
	}
	else{
		return true;
	}
}