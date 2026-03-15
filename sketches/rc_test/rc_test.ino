// Test: Read D4 FIRST to prove pulseIn blocking causes missed pulses
void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);
  pinMode(4, INPUT);
  pinMode(7, INPUT);
}

void loop() {
  // Read D4 first this time (was D2 first before)
  int ch2 = pulseIn(4, HIGH, 25000);
  int ch1 = pulseIn(2, HIGH, 25000);
  int ch5 = pulseIn(7, HIGH, 25000);

  Serial.print("D4=");
  Serial.print(ch2);
  Serial.print("  D2=");
  Serial.print(ch1);
  Serial.print("  D7=");
  Serial.println(ch5);

  delay(50);
}
