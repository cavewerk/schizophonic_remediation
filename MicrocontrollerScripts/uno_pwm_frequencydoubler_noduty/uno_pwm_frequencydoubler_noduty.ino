// UNO R3 accurate signal generator using Timer1 hardware PWM
// D9 (OC1A) output
// A0 = coarse frequency pot (log-mapped 1..10000 Hz)
// A2 = fine tune pot (1x..2x multiplier over one octave)
// Button on D2 cycles harmonic multiplier: x1, x2, x3, x4
// Onboard LED (D13) blinks 1/2/3/4 times to show current multiplier (NON-BLOCKING)
// Duty cycle fixed at 50%

#include <Arduino.h>
#include <math.h>

const uint8_t PWM_PIN        = 9;   // OC1A
const uint8_t COARSE_POT_PIN = A0;
const uint8_t FINE_POT_PIN   = A2;

const uint8_t BUTTON_PIN     = 2;   // momentary to GND (INPUT_PULLUP)
const uint8_t LED_PIN        = 13;  // onboard "L" LED

const uint32_t F_MIN = 1;
const uint32_t F_MAX = 10000;

uint32_t freqHz = 10;
uint32_t lastFreq = 0;

unsigned long lastPrintMs = 0;

// Harmonic multiplier state: 1,2,3,4
uint8_t multIndex = 0;                 // 0..3
const uint8_t multTable[4] = {1, 2, 3, 4};

// Debounce
bool lastButtonReading = HIGH;
bool buttonStableState = HIGH;
unsigned long lastDebounceMs = 0;
const unsigned long debounceMs = 25;

// ---------- Non-blocking LED blink state ----------
struct BlinkState {
  bool active = false;
  uint8_t blinksRemaining = 0;   // number of ON pulses left
  bool ledOn = false;
  unsigned long lastToggleMs = 0;
  const unsigned long onMs = 120;
  const unsigned long offMs = 120;
  const unsigned long gapMs = 350; // pause after finishing sequence
} blink;

void startBlink(uint8_t count) {
  blink.active = true;
  blink.blinksRemaining = count;
  blink.ledOn = false;
  digitalWrite(LED_PIN, LOW);
  blink.lastToggleMs = millis();
}

void ledTask() {
  if (!blink.active) return;

  unsigned long now = millis();

  if (blink.ledOn) {
    // currently ON -> turn OFF after onMs
    if (now - blink.lastToggleMs >= blink.onMs) {
      blink.ledOn = false;
      digitalWrite(LED_PIN, LOW);
      blink.lastToggleMs = now;
      // we completed one ON pulse
      if (blink.blinksRemaining > 0) blink.blinksRemaining--;
      // if that was the last blink, we’ll wait gapMs before ending
    }
  } else {
    // currently OFF
    if (blink.blinksRemaining == 0) {
      // finished sequence: wait a gap then stop
      if (now - blink.lastToggleMs >= blink.gapMs) {
        blink.active = false;
        digitalWrite(LED_PIN, LOW);
      }
    } else {
      // more blinks to do: turn ON after offMs
      if (now - blink.lastToggleMs >= blink.offMs) {
        blink.ledOn = true;
        digitalWrite(LED_PIN, HIGH);
        blink.lastToggleMs = now;
      }
    }
  }
}

// --- ADC stabilization helper ---
int readAnalogStable(uint8_t pin) {
  analogRead(pin);                 // throwaway after mux switch
  delayMicroseconds(50);
  return analogRead(pin);
}

// --- Coarse log mapping ---
uint32_t coarsePotToBaseFreq(int adc) {
  float a = (float)adc / 1023.0f;               // 0..1
  float ratio = (float)F_MAX / (float)F_MIN;
  float f = (float)F_MIN * powf(ratio, a);
  uint32_t out = (uint32_t)(f + 0.5f);
  if (out < F_MIN) out = F_MIN;
  if (out > F_MAX) out = F_MAX;
  return out;
}

// --- Fine tune: 1x..2x over one octave ---
float finePotToMultiplier(int adc) {
  float a = (float)adc / 1023.0f;               // 0..1
  return powf(2.0f, a);                         // 1..2
}

// --- Timer1 setup (Mode 14, OC1A, 50% duty) ---
void timer1Set(uint32_t fHz) {
  if (fHz < 1) fHz = 1;

  const uint16_t prescalers[] = {1, 8, 64, 256, 1024};
  const uint8_t  csBits[]     = {
    (1 << CS10),
    (1 << CS11),
    (1 << CS11) | (1 << CS10),
    (1 << CS12),
    (1 << CS12) | (1 << CS10)
  };

  uint16_t top = 0;
  uint8_t cs = csBits[4];

  // choose smallest prescaler with TOP <= 65535
  for (int i = 0; i < 5; i++) {
    uint32_t N = prescalers[i];
    uint32_t calcTop = (F_CPU / (N * fHz)) - 1UL;
    if (calcTop <= 65535UL) {
      top = (uint16_t)calcTop;
      cs = csBits[i];
      break;
    }
  }
  if (top < 1) top = 1;

  // 50% duty
  uint16_t ocr1a = (top + 1) / 2;

  uint8_t sreg = SREG;
  cli();

  TCCR1A = 0;
  TCCR1B = 0;

  // Fast PWM, TOP = ICR1 (Mode 14)
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // OC1A (D9) non-inverting
  TCCR1A |= (1 << COM1A1);

  ICR1  = top;
  OCR1A = ocr1a;

  // start timer
  TCCR1B |= cs;

  SREG = sreg;
}

// Debounced button press detection (returns true on each press)
bool buttonPressed() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceMs = millis();
    lastButtonReading = reading;
  }

  if ((millis() - lastDebounceMs) > debounceMs) {
    if (reading != buttonStableState) {
      buttonStableState = reading;
      // INPUT_PULLUP => pressed is LOW
      if (buttonStableState == LOW) return true;
    }
  }
  return false;
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(9600);

  // initial
  int cAdc = readAnalogStable(COARSE_POT_PIN);
  int fAdc = readAnalogStable(FINE_POT_PIN);

  uint32_t base = coarsePotToBaseFreq(cAdc);
  float fineMult = finePotToMultiplier(fAdc);
  uint32_t fundamental = (uint32_t)((base * fineMult) + 0.5f);

  uint32_t finalF = fundamental * multTable[multIndex];
  if (finalF < F_MIN) finalF = F_MIN;

  freqHz = finalF;
  timer1Set(freqHz);
  lastFreq = freqHz;

  // show starting multiplier (x1) non-blocking
  startBlink(multTable[multIndex]);
}

void loop() {
  // keep LED task running all the time (non-blocking)
  ledTask();

  // Handle button cycling + start blink pattern (non-blocking)
  if (buttonPressed()) {
    multIndex = (multIndex + 1) & 0x03; // 0..3
    startBlink(multTable[multIndex]);
  }

  int cAdc = readAnalogStable(COARSE_POT_PIN);
  int fAdc = readAnalogStable(FINE_POT_PIN);

  uint32_t base = coarsePotToBaseFreq(cAdc);
  float fineMult = finePotToMultiplier(fAdc);
  uint32_t fundamental = (uint32_t)((base * fineMult) + 0.5f);
  if (fundamental < F_MIN) fundamental = F_MIN;

  uint8_t harm = multTable[multIndex];
  uint32_t finalF = fundamental * harm;
  if (finalF < F_MIN) finalF = F_MIN;

  if (finalF != lastFreq) {
    timer1Set(finalF);
    lastFreq = finalF;
    freqHz = finalF;
  }

  // Serial print
  if (millis() - lastPrintMs >= 250) {
    lastPrintMs = millis();
    Serial.print("Fundamental: ");
    Serial.print(fundamental);
    Serial.print(" Hz | Mult: x");
    Serial.print(harm);
    Serial.print(" | Output: ");
    Serial.print(freqHz);
    Serial.println(" Hz (50% duty)");
  }
}
