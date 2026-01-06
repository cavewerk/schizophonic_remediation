/*
  UNO R3 accurate frequency generator (Timer1 hardware PWM)
  Output: D9 (OC1A), fixed 50% duty

  Controls:
    - A0: Coarse frequency (log mapped 1..10000 Hz)
    - A2: Fine tune multiplier (x1.0 .. x1.5)  <-- UPDATED
    - 6 harmonic buttons:
        D2->x1, D3->x2, D4->x3, D5->x4, D6->x5, D7->x6
    - SHIFT button (D8):
        Hold to shift harmonic buttons to x7..x12
        Release to return to x1..x6 next press

  LED feedback:
    - Onboard LED (D13) blinks 1..12 times to show selected harmonic (non-blocking)
    - While SHIFT is held: LED also does a short "double-flash" heartbeat (non-blocking)
      without blocking PWM or button reads.
*/

#include <Arduino.h>
#include <math.h>

// ---------------- Pins ----------------
const uint8_t PIN_PWM_OUT    = 9;   // D9 = OC1A
const uint8_t PIN_LED        = 13;  // onboard LED

const uint8_t PIN_COARSE_POT = A0;
const uint8_t PIN_FINE_POT   = A2;

const uint8_t BUTTON_PINS[6] = {2, 3, 4, 5, 6, 7}; // D2..D7 => x1..x6 (or x7..x12 with shift)
const uint8_t PIN_SHIFT      = 8;                  // SHIFT button to GND

// ---------------- Range ----------------
const uint32_t FREQ_MIN_HZ = 1;
const uint32_t FREQ_MAX_HZ = 10000;

// ---------------- Timing ----------------
const unsigned long BUTTON_DEBOUNCE_MS = 25;
const unsigned long PRINT_EVERY_MS     = 250;

// Harmonic blink timings
const unsigned long LED_ON_MS  = 120;
const unsigned long LED_OFF_MS = 120;
const unsigned long LED_GAP_MS = 350;

// SHIFT indicator timings (a repeating double-flash)
const unsigned long SHIFT_FLASH_ON_MS   = 60;
const unsigned long SHIFT_FLASH_OFF_MS  = 70;
const unsigned long SHIFT_FLASH_GAP_MS  = 800;  // pause between double-flashes

// ---------------- State ----------------
uint8_t selectedHarmonic = 1;       // 1..12
uint32_t lastProgrammedHz = 0;

unsigned long lastPrintMs = 0;

// Per-harmonic-button debounce state
bool lastButtonReading[6];
bool buttonStableState[6];
unsigned long lastDebounceMs[6];

// SHIFT debounce state
bool lastShiftReading = HIGH;
bool shiftStableState = HIGH;
unsigned long lastShiftDebounceMs = 0;

// ---------------- LED: harmonic blink (non-blocking) ----------------
struct HarmBlinkState {
  bool active = false;
  uint8_t pulsesRemaining = 0;
  bool ledOn = false;
  unsigned long lastToggleMs = 0;
} harmBlink;

void harm_startBlinkCount(uint8_t count) {
  harmBlink.active = true;
  harmBlink.pulsesRemaining = count;
  harmBlink.ledOn = false;
  digitalWrite(PIN_LED, LOW);
  harmBlink.lastToggleMs = millis();
}

bool harm_isActive() {
  return harmBlink.active;
}

void harm_task() {
  if (!harmBlink.active) return;

  unsigned long now = millis();

  if (harmBlink.ledOn) {
    if (now - harmBlink.lastToggleMs >= LED_ON_MS) {
      harmBlink.ledOn = false;
      digitalWrite(PIN_LED, LOW);
      harmBlink.lastToggleMs = now;
      if (harmBlink.pulsesRemaining > 0) harmBlink.pulsesRemaining--;
    }
  } else {
    if (harmBlink.pulsesRemaining == 0) {
      if (now - harmBlink.lastToggleMs >= LED_GAP_MS) {
        harmBlink.active = false;
        digitalWrite(PIN_LED, LOW);
      }
    } else {
      if (now - harmBlink.lastToggleMs >= LED_OFF_MS) {
        harmBlink.ledOn = true;
        digitalWrite(PIN_LED, HIGH);
        harmBlink.lastToggleMs = now;
      }
    }
  }
}

// ---------------- LED: SHIFT double-flash indicator (non-blocking) ----------------
// Runs ONLY when shift is held, and ONLY when harmonic blink is not active.
struct ShiftFlashState {
  bool active = false;       // should we run? (shift held)
  uint8_t phase = 0;         // 0 idle gap, 1 on1, 2 off1, 3 on2, 4 off2
  unsigned long lastMs = 0;
} shiftFlash;

void shiftFlash_reset() {
  shiftFlash.phase = 0;
  shiftFlash.lastMs = millis();
  digitalWrite(PIN_LED, LOW);
}

void shiftFlash_task(bool shiftHeld) {
  // If shift not held, ensure indicator is off and reset.
  if (!shiftHeld) {
    if (shiftFlash.active) {
      shiftFlash.active = false;
      shiftFlash_reset();
    }
    return;
  }

  // Shift IS held
  shiftFlash.active = true;

  // If harmonic blinking is running, don't compete for the LED.
  if (harm_isActive()) return;

  unsigned long now = millis();

  switch (shiftFlash.phase) {
    case 0: // idle gap before starting flashes
      if (now - shiftFlash.lastMs >= SHIFT_FLASH_GAP_MS) {
        shiftFlash.phase = 1;
        shiftFlash.lastMs = now;
        digitalWrite(PIN_LED, HIGH);
      }
      break;

    case 1: // first ON
      if (now - shiftFlash.lastMs >= SHIFT_FLASH_ON_MS) {
        shiftFlash.phase = 2;
        shiftFlash.lastMs = now;
        digitalWrite(PIN_LED, LOW);
      }
      break;

    case 2: // between flashes OFF
      if (now - shiftFlash.lastMs >= SHIFT_FLASH_OFF_MS) {
        shiftFlash.phase = 3;
        shiftFlash.lastMs = now;
        digitalWrite(PIN_LED, HIGH);
      }
      break;

    case 3: // second ON
      if (now - shiftFlash.lastMs >= SHIFT_FLASH_ON_MS) {
        shiftFlash.phase = 4;
        shiftFlash.lastMs = now;
        digitalWrite(PIN_LED, LOW);
      }
      break;

    case 4: // after second OFF, go back to idle gap
    default:
      if (now - shiftFlash.lastMs >= SHIFT_FLASH_OFF_MS) {
        shiftFlash.phase = 0;
        shiftFlash.lastMs = now;
        digitalWrite(PIN_LED, LOW);
      }
      break;
  }
}

// ---------------- ADC helper ----------------
int analogReadStable(uint8_t pin) {
  analogRead(pin);          // throwaway after mux switch
  delayMicroseconds(50);
  return analogRead(pin);
}

// ---------------- Mappings ----------------
uint32_t mapCoarsePotToBaseHz(int adc0_1023) {
  float a = (float)adc0_1023 / 1023.0f;                 // 0..1
  float ratio = (float)FREQ_MAX_HZ / (float)FREQ_MIN_HZ;
  float f = (float)FREQ_MIN_HZ * powf(ratio, a);        // log mapping
  uint32_t out = (uint32_t)(f + 0.5f);
  if (out < FREQ_MIN_HZ) out = FREQ_MIN_HZ;
  if (out > FREQ_MAX_HZ) out = FREQ_MAX_HZ;
  return out;
}

// UPDATED: Fine knob now maps smoothly from x1.0 .. x1.5
// We use an exponential curve so it still "feels" smooth across travel.
// multiplier = 1.5^(a) where a is 0..1  => 1.0 .. 1.5
float mapFinePotToMultiplier(int adc0_1023) {
  float a = (float)adc0_1023 / 1023.0f;  // 0..1
  return powf(1.5f, a);                  // 1..1.5
}

// ---------------- SHIFT (debounced) ----------------
bool shiftIsHeld() {
  unsigned long now = millis();
  bool reading = digitalRead(PIN_SHIFT); // HIGH idle, LOW pressed

  if (reading != lastShiftReading) {
    lastShiftDebounceMs = now;
    lastShiftReading = reading;
  }

  if (now - lastShiftDebounceMs > BUTTON_DEBOUNCE_MS) {
    shiftStableState = reading;
  }

  return (shiftStableState == LOW);
}

// ---------------- Harmonic Buttons ----------------
// Returns selected harmonic 1..12 on a NEW press, else 0.
uint8_t harmonicButtonPressed(bool shiftHeld) {
  unsigned long now = millis();
  uint8_t baseOffset = shiftHeld ? 6 : 0; // shift -> +6 => 7..12

  for (uint8_t i = 0; i < 6; i++) {
    bool reading = digitalRead(BUTTON_PINS[i]); // HIGH idle, LOW pressed

    if (reading != lastButtonReading[i]) {
      lastDebounceMs[i] = now;
      lastButtonReading[i] = reading;
    }

    if (now - lastDebounceMs[i] > BUTTON_DEBOUNCE_MS) {
      if (reading != buttonStableState[i]) {
        buttonStableState[i] = reading;
        if (buttonStableState[i] == LOW) {
          return (uint8_t)(i + 1 + baseOffset); // 1..6 or 7..12
        }
      }
    }
  }
  return 0;
}

// ---------------- Timer1: accurate 50% PWM on D9 ----------------
void timer1_setFrequency_50duty(uint32_t fHz) {
  if (fHz < 1) fHz = 1;

  const uint16_t prescalers[] = {1, 8, 64, 256, 1024};
  const uint8_t csBits[] = {
    (1 << CS10),                         // N=1
    (1 << CS11),                         // N=8
    (1 << CS11) | (1 << CS10),           // N=64
    (1 << CS12),                         // N=256
    (1 << CS12) | (1 << CS10)            // N=1024
  };

  uint16_t top = 1;
  uint8_t cs = csBits[4];

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

  uint16_t ocr1a = (top + 1) / 2; // 50% duty

  uint8_t sreg = SREG;
  cli();

  TCCR1A = 0;
  TCCR1B = 0;

  // Mode 14: Fast PWM, TOP=ICR1
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // OC1A (D9) non-inverting
  TCCR1A |= (1 << COM1A1);

  ICR1  = top;
  OCR1A = ocr1a;

  TCCR1B |= cs; // start timer

  SREG = sreg;
}

void setup() {
  pinMode(PIN_PWM_OUT, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Harmonic buttons
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    lastButtonReading[i] = HIGH;
    buttonStableState[i] = HIGH;
    lastDebounceMs[i] = 0;
  }

  // Shift button
  pinMode(PIN_SHIFT, INPUT_PULLUP);

  Serial.begin(9600);

  // Indicate starting harmonic (x1)
  harm_startBlinkCount(selectedHarmonic);
  shiftFlash_reset();
}

void loop() {
  // Read shift state first (debounced)
  bool shiftHeld = shiftIsHeld();

  // Run LED tasks (harmonic blink has priority over shift indicator)
  harm_task();
  shiftFlash_task(shiftHeld);

  // Select harmonic via buttons (shifted or not)
  uint8_t pressedHarm = harmonicButtonPressed(shiftHeld);
  if (pressedHarm >= 1 && pressedHarm <= 12 && pressedHarm != selectedHarmonic) {
    selectedHarmonic = pressedHarm;
    harm_startBlinkCount(selectedHarmonic); // show new harmonic
  }

  // Read pots
  int coarseADC = analogReadStable(PIN_COARSE_POT);
  int fineADC   = analogReadStable(PIN_FINE_POT);

  // Fundamental from knobs
  uint32_t baseHz = mapCoarsePotToBaseHz(coarseADC);
  float fineMult  = mapFinePotToMultiplier(fineADC);
  uint32_t fundamentalHz = (uint32_t)((baseHz * fineMult) + 0.5f);
  if (fundamentalHz < 1) fundamentalHz = 1;

  // Apply selected harmonic
  uint32_t outHz = fundamentalHz * (uint32_t)selectedHarmonic;
  if (outHz < 1) outHz = 1;

  // Update Timer1 only when needed
  if (outHz != lastProgrammedHz) {
    timer1_setFrequency_50duty(outHz);
    lastProgrammedHz = outHz;
  }

  // Serial debug print
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_EVERY_MS) {
    lastPrintMs = now;
    Serial.print("Base: ");
    Serial.print(baseHz);
    Serial.print(" Hz | FineMult: x");
    Serial.print(fineMult, 3);
    Serial.print(" | Fundamental: ");
    Serial.print(fundamentalHz);
    Serial.print(" Hz | Harm: x");
    Serial.print(selectedHarmonic);
    Serial.print(" | Output: ");
    Serial.print(outHz);
    Serial.print(" Hz | ShiftHeld: ");
    Serial.println(shiftHeld ? "YES" : "NO");
  }
}
