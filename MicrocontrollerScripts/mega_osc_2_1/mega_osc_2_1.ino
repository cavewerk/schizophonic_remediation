/*
  MEGA 2560 DDS Oscillator - FIXED ONE-SHOT LOGIC
  ------------------------------------------------------------------------------
  - Fixes "Bouncing" by ensuring buttons only trigger ONCE per touch.
  - You must release the button to toggle again.
*/

#include <Arduino.h>

// ============================================================================
// --- CONFIGURATION & CONSTANTS ---
// ============================================================================

// Pins
const uint8_t PIN_DDS_OUT     = 11; // PB5
const uint8_t PIN_COARSE_POT  = A0;
const uint8_t PIN_FINE_POT    = A2;
const uint8_t PIN_JOY_X       = A1;
const uint8_t PIN_SAMPLE_BTN  = 10;
const uint8_t PIN_SHIFT_BTN   = 8;
const uint8_t PIN_HARMONICS[] = {2, 3, 4, 5, 6, 7};

// Audio / DDS Settings
const uint32_t DDS_SAMPLE_RATE = 62500UL;
const float    FUND_MIN_HZ     = 80.0f;
const float    FUND_MAX_HZ     = 1000.0f;
const float    FINE_MAX_MULT   = 1.2f;

// Pre-calculated constant for Duty Cycle math (2^32 / 100)
const uint32_t DUTY_MULTIPLIER = 42949673UL; 

// ============================================================================
// --- COMPONENT CLASS: ONE-SHOT TOUCH BUTTON ---
// ============================================================================

class TouchButton {
  private:
    uint8_t _pin;
    bool _activeLow;
    bool _lastReading;     
    bool _debouncedState;
    bool _hasTriggered;      // Tracks if we have already handled this press
    
    unsigned long _lastDebounceTime; 
    const unsigned long DEBOUNCE_DELAY = 50; // 50ms noise filter

  public:
    void begin(uint8_t pin, bool activeLow) {
      _pin = pin;
      _activeLow = activeLow;
      // If your sensors have their own pull-ups, regular INPUT is fine.
      // But INPUT_PULLUP is safer for Active Low.
      pinMode(_pin, _activeLow ? INPUT_PULLUP : INPUT);
      
      _debouncedState = false;
      _lastReading = false;
      _hasTriggered = false;
      _lastDebounceTime = 0;
    }

    void update() {
      // 1. Read Raw
      bool raw = (digitalRead(_pin) == HIGH);
      if (_activeLow) raw = !raw; // Flip logic if Active Low

      // 2. Filter Noise
      if (raw != _lastReading) {
        _lastDebounceTime = millis();
      }
      _lastReading = raw;

      if ((millis() - _lastDebounceTime) > DEBOUNCE_DELAY) {
        _debouncedState = raw;
      }
    }

    // Returns true ONLY on the very first moment of a press.
    // It will NOT return true again until you release and repress.
    bool justPressed() {
      if (_debouncedState) {
        // Button is currently held down
        if (!_hasTriggered) {
          _hasTriggered = true; // Mark as handled
          return true;          // Fire event!
        }
      } else {
        // Button is released
        _hasTriggered = false; // Reset for next time
      }
      return false;
    }
};

// ============================================================================
// --- GLOBAL STATE ---
// ============================================================================

// DDS Interrupt Variables
volatile uint32_t ddsPhaseAcc = 0;
volatile uint32_t ddsPhaseInc = 0;
volatile uint32_t ddsDutyThr  = 0x80000000; // Init to 50%

// Application State
struct AppState {
    float storedFundHz     = 40.0f;
    uint8_t harmonic       = 1;
    bool shiftActive       = false; // x7..x12
    bool sampleMode        = true;  // true = continuous, false = hold
    int joyCenter          = 512;
};

AppState state;
TouchButton btnShift;
TouchButton btnSample;
TouchButton btnHarmonics[6];

// ============================================================================
// --- FAST GPIO (MEGA 2560 SPECIFIC) ---
// ============================================================================
#if defined(__AVR_ATmega2560__)
  #define OUT_HIGH() (PORTB |=  _BV(5))
  #define OUT_LOW()  (PORTB &= ~_BV(5))
#else
  #define OUT_HIGH() digitalWrite(PIN_DDS_OUT, HIGH)
  #define OUT_LOW()  digitalWrite(PIN_DDS_OUT, LOW)
#endif

// ============================================================================
// --- INTERRUPT SERVICE ROUTINE ---
// ============================================================================
ISR(TIMER2_COMPA_vect) {
  ddsPhaseAcc += ddsPhaseInc;
  if (ddsPhaseAcc < ddsDutyThr) OUT_HIGH();
  else                          OUT_LOW();
}

// ============================================================================
// --- SETUP ---
// ============================================================================
void setup() {
  Serial.begin(9600);
  pinMode(PIN_DDS_OUT, OUTPUT);
  #if defined(__AVR_ATmega2560__)
    DDRB |= _BV(5); 
  #endif

  // Initialize Buttons
  // Sample/Shift are Active LOW (true). Harmonics are Active HIGH (false).
  btnSample.begin(PIN_SAMPLE_BTN, true);
  btnShift.begin(PIN_SHIFT_BTN, true);
  for(int i=0; i<6; i++) btnHarmonics[i].begin(PIN_HARMONICS[i], false);

  // Calibrate Joystick
  long sum = 0;
  for(int i=0; i<16; i++) { sum += analogRead(PIN_JOY_X); delay(2); }
  state.joyCenter = sum / 16;
  
  // Setup DDS Timer
  noInterrupts();
  TCCR2A = (1 << WGM21);   // CTC Mode
  TCCR2B = (1 << CS20);    // No Prescaler
  OCR2A  = 255;            // ~62.5kHz
  TIMSK2 |= (1 << OCIE2A); // Enable Interrupt
  interrupts();
  
  Serial.println(F("--- Mega DDS One-Shot Ready ---"));
}

// ============================================================================
// --- HELPER FUNCTIONS ---
// ============================================================================

void applyFrequency(float freqHz) {
  if (freqHz < 0.1f) freqHz = 0.1f;
  uint32_t inc = (uint32_t)((freqHz * 4294967296.0f) / (float)DDS_SAMPLE_RATE);
  noInterrupts();
  ddsPhaseInc = inc;
  interrupts();
}

void applyDutyCycle(uint8_t dutyPct) {
  if (dutyPct > 100) dutyPct = 100;
  uint32_t thr = (uint32_t)dutyPct * DUTY_MULTIPLIER; 
  noInterrupts();
  ddsDutyThr = thr;
  interrupts();
}

int readAnalogSmooth(uint8_t pin) {
  int val = 0;
  val += analogRead(pin); val += analogRead(pin);
  val += analogRead(pin); val += analogRead(pin);
  return val >> 2; 
}

// ============================================================================
// --- MAIN LOOP ---
// ============================================================================
void loop() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 5) return; 
  lastUpdate = millis();

  // 1. UPDATE BUTTONS
  btnShift.update();
  btnSample.update();
  for(int i=0; i<6; i++) btnHarmonics[i].update();

  // 2. HANDLE TOGGLES (One-Shot Logic)
  if (btnShift.justPressed())  state.shiftActive = !state.shiftActive;
  if (btnSample.justPressed()) state.sampleMode  = !state.sampleMode;

  // 3. HANDLE HARMONICS
  for(int i=0; i<6; i++) {
    if (btnHarmonics[i].justPressed()) {
      state.harmonic = (i + 1) + (state.shiftActive ? 6 : 0);
    }
  }

  // 4. READ POTS (Only if Sample Mode is ON)
  if (state.sampleMode) {
    int rawCoarse = readAnalogSmooth(PIN_COARSE_POT);
    int rawFine   = readAnalogSmooth(PIN_FINE_POT);

    float normCoarse = rawCoarse / 1023.0f;
    float fund = FUND_MIN_HZ * powf((FUND_MAX_HZ / FUND_MIN_HZ), normCoarse);
    float mult = powf(FINE_MAX_MULT, rawFine / 1023.0f);
    
    state.storedFundHz = fund * mult;
  }

  // 5. READ JOYSTICK (Duty Cycle)
  int joyRaw = readAnalogSmooth(PIN_JOY_X);
  int duty = 50;
  int deadzone = 20;
  
  if (abs(joyRaw - state.joyCenter) > deadzone) {
    if (joyRaw < state.joyCenter) {
      duty = map(joyRaw, 0, state.joyCenter - deadzone, 0, 50);
    } else {
      duty = map(joyRaw, state.joyCenter + deadzone, 1023, 50, 100);
    }
  }
  
  // 6. APPLY CHANGES
  applyFrequency(state.storedFundHz * state.harmonic);
  applyDutyCycle((uint8_t)constrain(duty, 0, 100));

  // 7. DEBUG OUTPUT (Every 250ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    lastPrint = millis();
    Serial.print(F("Fund: ")); Serial.print(state.storedFundHz, 1);
    Serial.print(F("Hz | Harm: x")); Serial.print(state.harmonic);
    Serial.print(F(" | Shift: ")); Serial.print(state.shiftActive ? "ON" : "OFF");
    Serial.print(F(" | Mode: ")); Serial.print(state.sampleMode ? "SAMPLE" : "HOLD");
    Serial.print(F(" | Duty: ")); Serial.print(duty);
    Serial.println(F("%"));
  }
}