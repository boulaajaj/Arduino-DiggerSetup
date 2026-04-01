// Quick pin test — pulseIn (no interrupts) vs digitalRead
// Tests whether D4 and D7 can see RC signals at all

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  delay(2000);  // Extra delay so we can read startup output

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(7, INPUT);

  Serial.println("# === PIN TEST: pulseIn on all RC pins ===");
  Serial.println("# digitalPinToInterrupt values:");
  Serial.print("#   D2="); Serial.println(digitalPinToInterrupt(2));
  Serial.print("#   D3="); Serial.println(digitalPinToInterrupt(3));
  Serial.print("#   D4="); Serial.println(digitalPinToInterrupt(4));
  Serial.print("#   D7="); Serial.println(digitalPinToInterrupt(7));
  Serial.println("#");
  Serial.println("# Testing pulseIn (blocking, no interrupts)...");
  Serial.println("# 0us = no signal detected (timeout)");
  Serial.println("# Valid RC = 900-2100us");
  Serial.println("#");
}

void loop() {
  // Read each pin with pulseIn (100ms timeout each)
  unsigned long pw2 = pulseIn(2, HIGH, 100000);
  unsigned long pw3 = pulseIn(3, HIGH, 100000);
  unsigned long pw4 = pulseIn(4, HIGH, 100000);
  unsigned long pw7 = pulseIn(7, HIGH, 100000);

  Serial.print("# D2(CH1):"); Serial.print(pw2);
  Serial.print("us  D4(CH2):"); Serial.print(pw4);
  Serial.print("us  D3(CH4):"); Serial.print(pw3);
  Serial.print("us  D7(CH5):"); Serial.print(pw7);
  Serial.println("us");

  delay(200);
}
