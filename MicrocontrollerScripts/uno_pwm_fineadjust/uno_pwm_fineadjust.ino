// UNO R3 accurate signal generator using Timer1 hardware PWM
// D9 (OC1A) output
// A0 = coarse frequency pot (log-mapped 1..10000 Hz)
// A2 = fine tune pot (1x..2x multiplier over one octave)
// Duty cycle fixed at 50%

#include <Arduino.h>
#include <math.h>

const uint8_t PWM_PIN        = 9;   // OC1A
const uint8_t COARSE_POT_PIN = A0;
const uint8_t FINE_POT_PIN   = A2;

const uint32_t F_MIN = 1;
const uint32_t F_MAX = 10000;

const uint8_t DUTY_FIXED = 50;  // percent

uint32_t freqHz = 10;
uint32_t lastFreq = 0;

unsigned long lastPrintMs = 0;

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

// --- Timer1 setup (Mode 14, OC1A) ---
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

  // 50% duty: OCR1A = (TOP + 1) / 2
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

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  Serial.begin(9600);

  int cAdc = readAnalogStable(COARSE_POT_PIN);
  int fAdc = readAnalogStable(FINE_POT_PIN);

  uint32_t base = coarsePotToBaseFreq(cAdc);
  float mult = finePotToMultiplier(fAdc);
  uint32_t finalF = (uint32_t)((base * mult) + 0.5f);

  if (finalF < F_MIN) finalF = F_MIN;
  if (finalF > F_MAX) finalF = F_MAX;

  freqHz = finalF;
  timer1Set(freqHz);
  lastFreq = freqHz;
}

void loop() {
  int cAdc = readAnalogStable(COARSE_POT_PIN);
  int fAdc = readAnalogStable(FINE_POT_PIN);

  uint32_t base = coarsePotToBaseFreq(cAdc);
  float mult = finePotToMultiplier(fAdc);
  uint32_t finalF = (uint32_t)((base * mult) + 0.5f);

  if (finalF < F_MIN) finalF = F_MIN;
  if (finalF > F_MAX) finalF = F_MAX;

  if (finalF != lastFreq) {
    timer1Set(finalF);
    lastFreq = finalF;
    freqHz = finalF;
  }

  if (millis() - lastPrintMs >= 250) {
    lastPrintMs = millis();
    Serial.print("CoarseADC: ");
    Serial.print(cAdc);
    Serial.print(" | FineADC: ");
    Serial.print(fAdc);
    Serial.print(" | Freq: ");
    Serial.print(freqHz);
    Serial.println(" Hz (50% duty)");
  }
}
