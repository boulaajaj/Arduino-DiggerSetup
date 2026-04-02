// Pin Interrupt Test — Arduino Nano R4
// Tests whether attachInterrupt() works on D2, D3, D4, D7.
// D2/D3 should work (documented). D4/D7 are the question marks.
//
// Wire: connect a jumper from any PWM output (or just tap the pin
// with a jumper to 3.3V/GND) to trigger edges on each pin.
// Or connect RC receiver channels to generate real signals.
//
// Every second, prints interrupt counts for all four pins.

volatile unsigned long countD2 = 0;
volatile unsigned long countD3 = 0;
volatile unsigned long countD4 = 0;
volatile unsigned long countD7 = 0;

// Also track whether attachInterrupt returned without error
bool d2_attached = false;
bool d3_attached = false;
bool d4_attached = false;
bool d7_attached = false;

void isrD2() { countD2++; }
void isrD3() { countD3++; }
void isrD4() { countD4++; }
void isrD7() { countD7++; }

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  delay(500);

  Serial.println("# === PIN INTERRUPT TEST ===");
  Serial.println("# Testing attachInterrupt on D2, D3, D4, D7 (CHANGE mode)");
  Serial.println("#");

  // Check digitalPinToInterrupt for each pin
  int irqD2 = digitalPinToInterrupt(2);
  int irqD3 = digitalPinToInterrupt(3);
  int irqD4 = digitalPinToInterrupt(4);
  int irqD7 = digitalPinToInterrupt(7);

  Serial.print("# digitalPinToInterrupt(D2) = "); Serial.println(irqD2);
  Serial.print("# digitalPinToInterrupt(D3) = "); Serial.println(irqD3);
  Serial.print("# digitalPinToInterrupt(D4) = "); Serial.println(irqD4);
  Serial.print("# digitalPinToInterrupt(D7) = "); Serial.println(irqD7);
  Serial.println("#");

  // Set all pins as inputs
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(7, INPUT);

  // Attempt to attach interrupts on all four pins
  // If digitalPinToInterrupt returns NOT_AN_INTERRUPT (-1),
  // attachInterrupt will silently fail on most cores.
  if (irqD2 >= 0) {
    attachInterrupt(irqD2, isrD2, CHANGE);
    d2_attached = true;
    Serial.println("# D2: interrupt ATTACHED");
  } else {
    Serial.println("# D2: NOT_AN_INTERRUPT — cannot attach");
  }

  if (irqD3 >= 0) {
    attachInterrupt(irqD3, isrD3, CHANGE);
    d3_attached = true;
    Serial.println("# D3: interrupt ATTACHED");
  } else {
    Serial.println("# D3: NOT_AN_INTERRUPT — cannot attach");
  }

  if (irqD4 >= 0) {
    attachInterrupt(irqD4, isrD4, CHANGE);
    d4_attached = true;
    Serial.println("# D4: interrupt ATTACHED");
  } else {
    Serial.println("# D4: NOT_AN_INTERRUPT — cannot attach");
  }

  if (irqD7 >= 0) {
    attachInterrupt(irqD7, isrD7, CHANGE);
    d7_attached = true;
    Serial.println("# D7: interrupt ATTACHED");
  } else {
    Serial.println("# D7: NOT_AN_INTERRUPT — cannot attach");
  }

  Serial.println("#");
  Serial.println("# Printing counts every second. Tap pins to generate edges.");
  Serial.println("# Format: D2=count D3=count D4=count D7=count");
  Serial.println("#");
}

unsigned long lastPrint = 0;

void loop() {
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    // Snapshot counts with interrupts disabled
    noInterrupts();
    unsigned long c2 = countD2;
    unsigned long c3 = countD3;
    unsigned long c4 = countD4;
    unsigned long c7 = countD7;
    interrupts();

    Serial.print("D2=");
    Serial.print(c2);
    Serial.print(d2_attached ? "(ok) " : "(no) ");

    Serial.print("D3=");
    Serial.print(c3);
    Serial.print(d3_attached ? "(ok) " : "(no) ");

    Serial.print("D4=");
    Serial.print(c4);
    Serial.print(d4_attached ? "(ok) " : "(no) ");

    Serial.print("D7=");
    Serial.print(c7);
    Serial.print(d7_attached ? "(ok) " : "(no) ");

    Serial.println();
  }
}
