// PANOPTES LED for the latency and sync check. Board: Seeed XIAO ESP32S3.
// Arduino IDE: Tools > USB CDC On Boot > Enabled.
//
// One byte in, one line out, sent right after the pin changes:
//   'F'  LED on, off again after FLASH_MS    reply "F"
//   'H'  LED on until 'L'                    reply "H"
//   'L'  LED off                             reply "L"

const int LED_PIN = 1;               // D0
const unsigned long FLASH_MS = 200;  // python/led.py FLASH_S must match

bool flashing = false;
unsigned long on_at = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'F') {
      digitalWrite(LED_PIN, HIGH);
      flashing = true;
      on_at = millis();
      Serial.println("F");
    } else if (c == 'H') {
      digitalWrite(LED_PIN, HIGH);
      flashing = false;
      Serial.println("H");
    } else if (c == 'L') {
      digitalWrite(LED_PIN, LOW);
      flashing = false;
      Serial.println("L");
    }
  }
  
  if (flashing && millis() - on_at >= FLASH_MS) {
    digitalWrite(LED_PIN, LOW);
    flashing = false;
  }
}
