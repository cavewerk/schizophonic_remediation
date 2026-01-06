/*
  UNO R3 accurate frequency generator (Timer1 hardware PWM)
  Output: D9 (OC1A), fixed 50% duty

  CONTROLS
  - A0: Coarse FUNDAMENTAL frequency (log mapped 5..800 Hz)   <-- UPDATED
  - A2: Fine multiplier (x1.0 .. x1.5, smooth)               <-- UPDATED
  - 6 harmonic buttons:
      D2->x1, D3->x2, D4->x3, D5->x4, D6->x5, D7->x6
  - SHIFT button (D8):
      Hold = buttons become x7..x12
      Release = next presses go back to x1..x6

  LED FEEDBACK (D13 onboard LED)
  - Blinks 1..12 times to show the currently selected harmonic (non-blocking)
  - While SHIFT is held: a repeating “double-flash” heartbeat (non-blocking),
    but harmonic-blink always has priority.

  IMPORTANT CHANGE FOR SMOOTHNESS
  - Timer1 is configured ONCE with a FIXED prescaler (64).
  - When frequency changes we only update ICR1 (TOP) + OCR1A (50% duty).
  - This avoids the big “interruptions” caused by fully resetting the timer.
*/

#include <Arduino.h>
#include <math.h>

// ---------------- Pins ----------------
const uint8_t PIN_PWM_OUT    = 9;    // D9 = OC1A
const uint8_t PIN_LED        = 13;   // onboard LED

const uint8_t PIN_COARSE_POT = A0;
const uint8_t PIN_FINE_POT   = A2;

const uint8_t BUTTON_PINS[6] = {2, 3, 4, 5, 6, 7}; // harmonic buttons
const uint8_t PIN_SHIFT      = 8;                  // shift button

// ---------------- Fundamental range (A0) ----------------
const uint32_t FUND_MIN_HZ = 5;     // <-- UPDATED
const uint32_t FUND_MAX_HZ = 800;   // <-- UPDATED

// ---------------- Fine multiplier range (A2) ----------------
const float FINE_MAX_MULT = 1.5f;  // x1.0 .. x1.5

// ---------------- Timer1 settings ----------------
// Fixed prescaler = 64 gives us smooth updates for 5 Hz up to many kHz.
const uint32_t TIMER1_PRESCALER = 64;

// ---------------- Timing ----------------
const unsigned long BUTTON_DEBOUNCE_MS = 25;
const unsigned long PRINT_EVERY_MS     = 250;

// Harmonic blink timings
const unsigned long LED_ON_MS  = 120;
const unsigned long LED_OFF_MS = 120;
const unsigned long LED_GAP_MS = 350;

// SHIFT indicator timings (repeating double-flash)
const unsigned long SHIFT_FLASH_ON_MS   = 60;
const unsigned long SHIFT_FLASH_OFF_MS  = 70;
const unsigned long SHIFT_FLASH_GAP_MS  = 800;

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
// Runs only while shift held and only when harmonic blink isn't active.
struct ShiftFlashState {
  bool active = false;
  uint8_t phase = 0; // 0 gap, 1 on1, 2 off1, 3 on2, 4 off2
  unsigned long lastMs = 0;
} shiftFlash;

void shiftFlash_reset() {
  shiftFlash.phase = 0;
  shiftFlash.lastMs = millis();
  digitalWrite(PIN_LED, LOW);
}

void shiftFlash_task(bool shiftHeld) {
  if (!shiftHeld) {
    if (shiftFlash.active) {
      shiftFlash.active = false;
      shiftFlash_reset();
    }
    return;
  }

  shiftFlash.active = true;

  // Harmonic blink has priority
  if (harm_isActive()) return;

  unsigned long now = millis();

  switch (shiftFlash.phase) {
    case 0:
      if (now - shiftFlash.lastMs >= SHIFT_FLASH_GAP_MS) {
        shiftFlash.phase = 1;
        shiftFlash.lastMs = now;
        digitalWrite(PIN_LED, HIGH);
      }
      break;

    case 1:
      if (now - shiftFlash.lastMs >= SHIFT_FLASH_ON_MS) {
        shiftFlash.phase = 2;
        shiftFlash.lastMs = now;
        digitalWrite(PIN_LED, LOW);
      }
      break;

    case 2:
      if (now - shiftFlash.lastMs >= SHIFT_FLASH_OFF_MS) {
        shiftFlash.phase = 3;
        shiftFlash.lastMs = now;
        digitalWrite(PIN_LED, HIGH);
      }
      break;

    case 3:
      if (now - shiftFlash.lastMs >= SHIFT_FLASH_ON_MS) {
        shiftFlash.phase = 4;
        shiftFlash.lastMs = now;
        digitalWrite(PIN_LED, LOW);
      }
      break;

    case 4:
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
// Coarse: log mapping 5..800 Hz (so it's not bunched up)
uint32_t mapCoarsePotToFundHz(int adc0_1023) {
  float a = (float)adc0_1023 / 1023.0f;              // 0..1
  float ratio = (float)FUND_MAX_HZ / (float)FUND_MIN_HZ;
  float f = (float)FUND_MIN_HZ * powf(ratio, a);     // log curve
  uint32_t out = (uint32_t)(f + 0.5f);
  if (out < FUND_MIN_HZ) out = FUND_MIN_HZ;
  if (out > FUND_MAX_HZ) out = FUND_MAX_HZ;
  return out;
}

// Fine: smooth multiplier x1.0..x1.5
float mapFinePotToMultiplier(int adc0_1023) {
  float a = (float)adc0_1023 / 1023.0f;  // 0..1
  return powf(FINE_MAX_MULT, a);         // 1..1.5 (exponential feel)
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

// ---------------- Timer1: fixed prescaler, smooth updates ----------------
// Configure Timer1 ONCE (Mode 14, TOP=ICR1, OC1A on D9, prescaler 64)
void timer1_begin_fixedPrescaler64() {
  uint8_t sreg = SREG;
  cli();

  TCCR1A = 0;
  TCCR1B = 0;

  // Fast PWM, TOP = ICR1 (Mode 14)
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // OC1A (D9) non-inverting
  TCCR1A |= (1 << COM1A1);

  // Prescaler = 64
  TCCR1B |= (1 << CS11) | (1 << CS10);

  // Set a safe initial frequency (use FUND_MIN_HZ)
  uint32_t top32 = (F_CPU / (TIMER1_PRESCALER * FUND_MIN_HZ)) - 1UL;
  if (top32 > 65535UL) top32 = 65535UL;
  if (top32 < 1UL) top32 = 1UL;

  uint16_t top = (uint16_t)top32;
  ICR1  = top;
  OCR1A = (top + 1) / 2; // 50% duty

  SREG = sreg;
}

// Update frequency by only updating TOP + compare (no full reset)
void timer1_setFrequency_fixedPrescaler64(uint32_t fHz) {
  if (fHz < 1) fHz = 1;

  uint32_t top32 = (F_CPU / (TIMER1_PRESCALER * fHz)) - 1UL;
  if (top32 > 65535UL) top32 = 65535UL;
  if (top32 < 1UL) top32 = 1UL;

  uint16_t top = (uint16_t)top32;

  // Write 16-bit regs atomically
  uint8_t sreg = SREG;
  cli();
  ICR1  = top;
  OCR1A = (top + 1) / 2; // keep 50% duty
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

  // Start Timer1 once (smooth updates from here on)
  timer1_begin_fixedPrescaler64();

  // Indicate starting harmonic (x1)
  harm_startBlinkCount(selectedHarmonic);
  shiftFlash_reset();
}

void loop() {
  // Read shift state (debounced)
  bool shiftHeld = shiftIsHeld();

  // LED tasks (harmonic blink has priority over shift indicator)
  harm_task();
  shiftFlash_task(shiftHeld);

  // Select harmonic via buttons (shifted or not)
  uint8_t pressedHarm = harmonicButtonPressed(shiftHeld);
  if (pressedHarm >= 1 && pressedHarm <= 12 && pressedHarm != selectedHarmonic) {
    selectedHarmonic = pressedHarm;
    harm_startBlinkCount(selectedHarmonic);
  }

  // Read pots
  int coarseADC = analogReadStable(PIN_COARSE_POT);
  int fineADC   = analogReadStable(PIN_FINE_POT);

  // Fundamental from knobs (5..800 Hz)
  uint32_t fundHz = mapCoarsePotToFundHz(coarseADC);
  float fineMult  = mapFinePotToMultiplier(fineADC);

  // Apply fine multiplier to fundamental
  uint32_t fundamentalHz = (uint32_t)((fundHz * fineMult) + 0.5f);
  if (fundamentalHz < 1) fundamentalHz = 1;

  // Apply selected harmonic (1..12)
  uint32_t outHz = fundamentalHz * (uint32_t)selectedHarmonic;
  if (outHz < 1) outHz = 1;

  // Smooth timer update: only update TOP/compare when frequency changes
  if (outHz != lastProgrammedHz) {
    timer1_setFrequency_fixedPrescaler64(outHz);
    lastProgrammedHz = outHz;
  }

  // Serial debug print
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_EVERY_MS) {
    lastPrintMs = now;
    Serial.print("Fund(knob): ");
    Serial.print(fundHz);
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
