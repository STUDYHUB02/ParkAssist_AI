#define TRIG_PIN 9   // Trig pin of HC-SR04
#define ECHO_PIN 10  // Echo pin of HC-SR04
#define BUZZER_PIN 11 // V/O pin of the buzzer module

float duration, distance;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);  // Set Trig pin as output
  pinMode(ECHO_PIN, INPUT);   // Set Echo pin as input
  pinMode(BUZZER_PIN, OUTPUT); // Set V/O pin as output
  digitalWrite(BUZZER_PIN, HIGH); // Ensure buzzer is OFF initially
  Serial.begin(9600);         // Initialize Serial monitor
}

void loop() {
  // Send a 10-microsecond pulse to trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pin and calculate the distance
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2; // Convert to distance in cm

  // Trigger the buzzer if the distance is less than the threshold
  if (distance > 2 && distance < 30) { // Distance threshold: 30 cm
    digitalWrite(BUZZER_PIN, HIGH);  // Turn on the buzzer
    Serial.println("ALERT");        // Send an alert message
    delay(70);                      // Buzzer on for 70ms
    digitalWrite(BUZZER_PIN, LOW);  // Turn off the buzzer
    delay(70);                      // Buzzer off for 70ms
  } else {
    digitalWrite(BUZZER_PIN, HIGH); // Ensure buzzer is off if no object nearby
  }
  delay(100); // Wait for a short interval before next measurement
}