/*
  MEGA 2560 DDS square-wave oscillator (phase-continuous)
  - Output: D11 (PB5 fast write)

  FREQUENCY CONTROLS
  - Coarse pot: A0  (log mapped 5..800 Hz fundamental)
  - Fine pot:   A2  (multiplier x1.0 .. x1.5)
  - SAMPLE/HOLD button: D10 (to GND, INPUT_PULLUP)
      * While held: reads pots and updates STORED fundamental
      * When released: stored fundamental freezes

  HARMONIC CONTROLS
  - 6 buttons: D2..D7 (to GND, INPUT_PULLUP)
      D2->x1, D3->x2, D4->x3, D5->x4, D6->x5, D7->x6
  - SHIFT: D8 (to GND, INPUT_PULLUP)
      Hold SHIFT while pressing harmonic buttons => x7..x12

  DUTY CONTROL
  - Joystick X: A1 controls duty continuously
      * auto-calibrates center at startup => idle ~50%
      * deadzone to prevent twitch

  DDS
  - Timer2 ISR at 62.5 kHz sample rate
  - phaseInc sets frequency; dutyThreshold sets duty

  Wiring:
    Pots/joystick (10k recommended):
      outer legs -> 5V and GND
      wiper -> A0 (coarse) / A2 (fine) / A1 (joy X)
    Buttons:
      one side -> pin, other side -> GND (INPUT_PULLUP)
*/

#include <Arduino.h>
#include <math.h>

// --------- Pins ----------
const uint8_t PIN_OUT = 11;  // Mega D11 = PB5 (fast)
const uint8_t PIN_COARSE_POT = A0;
const uint8_t PIN_FINE_POT = A2;
const uint8_t PIN_JOY_X = A1;

const uint8_t PIN_SAMPLE_BTN = 10;  // HOLD to sample pots (moved to avoid D2 conflict)

const uint8_t BUTTON_PINS[6] = { 2, 3, 4, 5, 6, 7 };  // harmonic keys
const uint8_t PIN_SHIFT = 8;                          // shift key

// --------- Fundamental frequency range (before harmonic) ----------
const float FUND_MIN = 5.0f;
const float FUND_MAX = 800.0f;

// Fine multiplier range
const float FINE_MAX_MULT = 1.5f;  // x1.0 .. x1.5

// --------- DDS sample rate ----------
// Timer2 CTC at 62.5 kHz: 16MHz / 1 / (255+1) = 62,500 Hz
const uint32_t DDS_SAMPLE_RATE = 62500UL;

// --------- Timing ----------
const unsigned long UPDATE_EVERY_MS = 5;   // main control update rate
const unsigned long PRINT_EVERY_MS = 250;  // debug print rate
const unsigned long BUTTON_DEBOUNCE_MS = 25;

unsigned long lastUpdateMs = 0;
unsigned long lastPrintMs = 0;

// --------- DDS state ----------
volatile uint32_t phaseAcc = 0;
volatile uint32_t phaseInc = 0;

// Duty control: HIGH when phaseAcc < dutyThreshold
volatile uint32_t dutyThreshold = 0x80000000UL;  // 50%

// --------- Stored fundamental (sample/hold) ----------
float storedFundHz = 40.0f;  // fundamental after coarse+fine, before harmonic
uint8_t selectedHarmonic = 1;

// --------- Joystick center calibration ----------
int joyCenter = 512;
const int JOY_DEADZONE = 20;  // +/- counts around center that snaps to 50%

// --------- Debounce state ----------
bool lastShiftReading = HIGH;
bool shiftStableState = HIGH;
unsigned long lastShiftDebounceMs = 0;

bool lastSampleReading = HIGH;
bool sampleStableState = HIGH;
unsigned long lastSampleDebounceMs = 0;

bool lastButtonReading[6];
bool buttonStableState[6];
unsigned long lastButtonDebounceMs[6];

// --------- Fast pin write (MEGA 2560) ----------
// Mega2560: D11 is PB5
#if defined(__AVR_ATmega2560__)
#define OUT_HIGH() (PORTB |= _BV(5))
#define OUT_LOW() (PORTB &= ~_BV(5))
#else
inline void OUT_HIGH() {
  digitalWrite(PIN_OUT, HIGH);
}
inline void OUT_LOW() {
  digitalWrite(PIN_OUT, LOW);
}
#endif

// --------- Timer2 ISR: DDS core ----------
ISR(TIMER2_COMPA_vect) {
  phaseAcc += phaseInc;

  // Variable duty square
  if (phaseAcc < dutyThreshold) OUT_HIGH();
  else OUT_LOW();
}

// --------- ADC helpers ----------
int analogReadStable(uint8_t pin) {
  // Helps when switching channels (A0/A1/A2)
  (void)analogRead(pin);
  (void)analogRead(pin);
  delayMicroseconds(200);
  return analogRead(pin);
}

int analogReadAveraged(uint8_t pin, uint8_t samples = 4) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogReadStable(pin);
  }
  return (int)(sum / samples);
}

// --------- Mapping ----------
float mapCoarseToFundHz_Log(int adc0_1023) {
  float a = (float)adc0_1023 / 1023.0f;
  float ratio = FUND_MAX / FUND_MIN;
  float f = FUND_MIN * powf(ratio, a);
  if (f < FUND_MIN) f = FUND_MIN;
  if (f > FUND_MAX) f = FUND_MAX;
  return f;
}

float mapFineToMult(int adc0_1023) {
  float a = (float)adc0_1023 / 1023.0f;
  return powf(FINE_MAX_MULT, a);  // 1..1.5
}

// dutyThreshold = duty% * 2^32
uint32_t dutyPctToThreshold(uint8_t dutyPct) {
  if (dutyPct <= 0) return 0;
  if (dutyPct >= 100) return 0xFFFFFFFFUL;
  uint64_t thr = (uint64_t)dutyPct * 4294967296ULL / 100ULL;
  return (uint32_t)thr;
}

// Joystick X -> duty %
uint8_t joystickToDutyPct(int xRaw) {
  int dx = xRaw - joyCenter;

  if (abs(dx) <= JOY_DEADZONE) return 50;

  if (dx < 0) {
    float norm = (float)dx / (float)(joyCenter);  // ~ -1 .. 0
    int duty = (int)(50.0f + norm * 50.0f);
    if (duty < 0) duty = 0;
    return (uint8_t)duty;
  } else {
    float norm = (float)dx / (float)(1023 - joyCenter);  // 0 .. ~ +1
    int duty = (int)(50.0f + norm * 50.0f);
    if (duty > 100) duty = 100;
    return (uint8_t)duty;
  }
}

// --------- DDS phase increment ----------
uint32_t computePhaseInc(float freqHz) {
  if (freqHz < 0.0f) freqHz = 0.0f;
  uint64_t num = (uint64_t)(freqHz * 4294967296.0f);  // freq * 2^32
  return (uint32_t)(num / (uint64_t)DDS_SAMPLE_RATE);
}

void setDDSFrequency(float freqHz) {
  uint32_t inc = computePhaseInc(freqHz);
  noInterrupts();
  phaseInc = inc;
  interrupts();
}

void setDutyPct(uint8_t dutyPct) {
  uint32_t thr = dutyPctToThreshold(dutyPct);
  noInterrupts();
  dutyThreshold = thr;
  interrupts();
}

// --------- Timer2 setup (62.5 kHz) ----------
void setupTimer2_62k5() {
  cli();

  TCCR2A = 0;
  TCCR2B = 0;

  // CTC mode
  TCCR2A |= (1 << WGM21);

  // prescaler = 1
  TCCR2B |= (1 << CS20);

  // OCR2A = 255 => 62.5 kHz
  OCR2A = 255;

  // enable interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();
}

// --------- Debounced buttons ----------
bool debouncedIsHeld(uint8_t pin, bool &lastRead, bool &stable, unsigned long &t0) {
  unsigned long now = millis();
  bool r = digitalRead(pin);  // HIGH idle, LOW pressed

  if (r != lastRead) {
    lastRead = r;
    t0 = now;
  }
  if (now - t0 > BUTTON_DEBOUNCE_MS) {
    stable = r;
  }
  return (stable == LOW);
}

// Return harmonic number 1..12 on a NEW press, else 0
uint8_t harmonicButtonPressed(bool shiftHeld) {
  unsigned long now = millis();
  uint8_t offset = shiftHeld ? 6 : 0;  // +6 => 7..12

  for (uint8_t i = 0; i < 6; i++) {
    bool r = digitalRead(BUTTON_PINS[i]);  // HIGH idle, LOW pressed

    if (r != lastButtonReading[i]) {
      lastButtonReading[i] = r;
      lastButtonDebounceMs[i] = now;
    }

    if (now - lastButtonDebounceMs[i] > BUTTON_DEBOUNCE_MS) {
      if (r != buttonStableState[i]) {
        buttonStableState[i] = r;
        if (r == LOW) {
          return (uint8_t)(i + 1 + offset);
        }
      }
    }
  }
  return 0;
}

void calibrateJoystickCenter() {
  uint32_t sum = 0;
  const uint8_t N = 32;
  for (uint8_t i = 0; i < N; i++) {
    sum += analogReadAveraged(PIN_JOY_X, 4);
    delay(2);
  }
  joyCenter = (int)(sum / N);
}

void setup() {
  pinMode(PIN_OUT, OUTPUT);
#if defined(__AVR_ATmega2560__)
  DDRB |= _BV(5);  // PB5 output
#endif

  pinMode(PIN_SAMPLE_BTN, INPUT_PULLUP);
  pinMode(PIN_SHIFT, INPUT_PULLUP);

  for (uint8_t i = 0; i < 6; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    lastButtonReading[i] = HIGH;
    buttonStableState[i] = HIGH;
    lastButtonDebounceMs[i] = 0;
  }

  Serial.begin(9600);
  Serial.println("Mega DDS: A0+A2 sample/hold (D10) + harmonics (D2-7) + shift (D8) + joystick duty (A1).");

  calibrateJoystickCenter();
  Serial.print("Joystick center: ");
  Serial.println(joyCenter);

  // Start: fund=40Hz, harm=1, duty=50%
  storedFundHz = 40.0f;
  selectedHarmonic = 1;

  setDutyPct(50);
  setDDSFrequency(storedFundHz * (float)selectedHarmonic);

  setupTimer2_62k5();
}

void loop() {
  unsigned long now = millis();
  if (now - lastUpdateMs < UPDATE_EVERY_MS) return;
  lastUpdateMs = now;

  // --- Duty always follows joystick ---
  int joyX = analogReadAveraged(PIN_JOY_X, 4);
  uint8_t duty = joystickToDutyPct(joyX);
  setDutyPct(duty);

  // --- Shift state (debounced) ---
  bool shiftHeld = debouncedIsHeld(PIN_SHIFT, lastShiftReading, shiftStableState, lastShiftDebounceMs);

  // --- Harmonic selection (on new press) ---
  uint8_t h = harmonicButtonPressed(shiftHeld);
  bool harmonicChanged = false;
  if (h >= 1 && h <= 12 && h != selectedHarmonic) {
    selectedHarmonic = h;
    harmonicChanged = true;
  }

  // --- Sample/Hold (debounced hold) ---
  bool sampling = debouncedIsHeld(PIN_SAMPLE_BTN, lastSampleReading, sampleStableState, lastSampleDebounceMs);

  float fundHz = storedFundHz;
  float fineMult = 1.0f;
  int coarseADC = 0, fineADC = 0;

  if (sampling) {
    coarseADC = analogReadAveraged(PIN_COARSE_POT, 4);
    fineADC = analogReadAveraged(PIN_FINE_POT, 4);

    float coarseHz = mapCoarseToFundHz_Log(coarseADC);
    fineMult = mapFineToMult(fineADC);

    storedFundHz = coarseHz * fineMult;
    if (storedFundHz < 0.1f) storedFundHz = 0.1f;
    fundHz = storedFundHz;
  }

  // --- Apply frequency whenever sampling OR harmonic changed ---
  if (sampling || harmonicChanged) {
    float outHz = fundHz * (float)selectedHarmonic;
    if (outHz < 0.1f) outHz = 0.1f;
    setDDSFrequency(outHz);
  }

  // --- Debug print ---
  if (now - lastPrintMs >= PRINT_EVERY_MS) {
    lastPrintMs = now;

    Serial.print(sampling ? "[SAMPLE] " : "[HOLD] ");
    Serial.print("fundStored=");
    Serial.print(storedFundHz, 2);
    Serial.print("Hz | harm=x");
    Serial.print(selectedHarmonic);
    Serial.print(" | duty=");
    Serial.print(duty);
    Serial.print("% | joyX=");
    Serial.print(joyX);

    if (sampling) {
      Serial.print(" | A0=");
      Serial.print(coarseADC);
      Serial.print(" A2=");
      Serial.print(fineADC);
      Serial.print(" fine x");
      Serial.print(fineMult, 3);
    }

    Serial.print(" | shift=");
    Serial.println(shiftHeld ? "YES" : "NO");
  }
}
