// USB output test — Arduino Nano R4
// Simple Serial test over USB. No Router Bridge needed.

int count = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // Wait for USB Serial to connect

  pinMode(LED_BUILTIN, OUTPUT);

  delay(1000);
  Serial.println("USB_TEST: Arduino Nano R4 Serial working!");
  Serial.println("SETUP_DONE");
}

void loop() {
  // Toggle LED every second
  digitalWrite(LED_BUILTIN, (count % 2) ? LOW : HIGH);

  // Output on Serial (USB)
  Serial.print("COUNT:");
  Serial.println(count);

  count++;
  delay(1000);
}
