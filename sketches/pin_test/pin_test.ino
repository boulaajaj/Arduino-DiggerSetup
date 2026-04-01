// D0 Signal Detection Test — Arduino Nano R4
// Tests whether Serial1 is on D0, and whether the pin sees any signal.
// Disables Serial1 and does raw digitalRead to detect toggling.

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  delay(1000);

  Serial.println("# === D0 SIGNAL DETECTION TEST ===");
  Serial.println("#");

  // --- Test 1: Raw digitalRead on D0 (no Serial1) ---
  Serial.println("# TEST 1: Raw digitalRead on D0 (Serial1 disabled)");
  pinMode(0, INPUT);

  int toggles = 0;
  bool last = digitalRead(0);
  unsigned long start = micros();
  // Sample D0 for 1 second at max speed
  while (micros() - start < 1000000UL) {
    bool now = digitalRead(0);
    if (now != last) { toggles++; last = now; }
  }
  Serial.print("#   D0 toggles in 1s: ");
  Serial.println(toggles);
  Serial.print("#   D0 current state: ");
  Serial.println(digitalRead(0) ? "HIGH (3.3V — UART idle)" : "LOW");

  if (toggles == 0) {
    Serial.println("#   -> NO activity. ESC is not transmitting (line idle).");
  } else {
    Serial.println("#   -> SIGNAL DETECTED! ESC is transmitting.");
    // Estimate baud from toggle rate (each bit = 1 toggle for alternating pattern)
    Serial.print("#   -> Estimated toggle rate: ");
    Serial.print(toggles);
    Serial.println(" Hz");
  }

  // --- Test 2: Check Serial1 on D0 at each baud rate ---
  Serial.println("#");
  Serial.println("# TEST 2: Serial1 at each baud rate (3s each)");

  const long bauds[] = {9600, 19200, 38400, 57600, 100000, 115200, 250000};
  const int nbauds = 7;

  for (int i = 0; i < nbauds; i++) {
    Serial1.begin(bauds[i]);
    delay(100);  // Let UART settle

    // Drain any buffered garbage
    while (Serial1.available()) Serial1.read();

    // Count bytes for 3 seconds
    int bytes = 0;
    uint8_t firstBytes[16];
    int captured = 0;
    unsigned long t = millis();
    while (millis() - t < 3000) {
      if (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (captured < 16) firstBytes[captured] = b;
        captured++;
        bytes++;
      }
    }

    Serial.print("#   ");
    Serial.print(bauds[i]);
    Serial.print(" baud: ");
    Serial.print(bytes);
    Serial.print(" bytes");

    if (bytes > 0) {
      Serial.print(" -> RECEIVED! First bytes: ");
      for (int j = 0; j < min(captured, 16); j++) {
        if (firstBytes[j] < 16) Serial.print("0");
        Serial.print(firstBytes[j], HEX);
        Serial.print(" ");
      }
    }
    Serial.println();

    Serial1.end();
  }

  // --- Test 3: Check if Serial (not Serial1) is on D0 ---
  Serial.println("#");
  Serial.println("# TEST 3: Pin identity check");
  Serial.print("#   digitalPinToInterrupt(0) = ");
  Serial.println(digitalPinToInterrupt(0));
  Serial.print("#   digitalPinToInterrupt(1) = ");
  Serial.println(digitalPinToInterrupt(1));

  // Check if we can read D0 while Serial is active
  // (if Serial uses D0, digitalRead would conflict)
  Serial.print("#   D0 read while Serial active: ");
  Serial.println(digitalRead(0) ? "HIGH" : "LOW");

  Serial.println("#");
  Serial.println("# TEST COMPLETE");
  Serial.println("# If all tests show 0 bytes: ESC requires polling (won't auto-transmit)");
  Serial.println("# If toggles > 0 but 0 bytes: baud rate not in our list");
}

void loop() {
  // Just idle
  delay(1000);
}
