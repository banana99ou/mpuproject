const int enPin=8;
const int stepPin = 3;
const int dirPin = 6;
const int stepsPerRev=200;
int pulseWidthMicros = 100;   // microseconds
int millisBtwnSteps = 1000;
void setup() {
  Serial.begin(9600);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  Serial.println(F("CNC Shield Initialized"));
}
void loop() {
  Serial.println(F("Running clockwise"));
  digitalWrite(dirPin, HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for (int i = 0; i < stepsPerRev; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(millisBtwnSteps);
  }
  delay(1000); // One second delay
  Serial.println(F("Running counter-clockwise"));
  digitalWrite(dirPin, LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for (int i = 0; i < 2*stepsPerRev; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(millisBtwnSteps);
  }
  delay(1000);
}