// UNO R3 accurate signal generator using Timer1 hardware PWM
// D9 (OC1A) output
// A0 = frequency pot (log-mapped 1..10000 Hz)
// A1 = duty pot (0..100%)
// Serial prints at 9600

#include <Arduino.h>
#include <math.h>

const uint8_t PWM_PIN      = 9;   // OC1A
const uint8_t FREQ_POT_PIN = A0;
const uint8_t DUTY_POT_PIN = A1;

const uint32_t F_MIN = 1;
const uint32_t F_MAX = 10000;

uint32_t freqHz = 10;
uint8_t  dutyPercent = 50;

uint32_t lastFreq = 0;
uint8_t  lastDuty = 255;
unsigned long lastPrintMs = 0;

static inline uint16_t clamp16(uint32_t v) {
  if (v > 65535UL) return 65535;
  return (uint16_t)v;
}

// Log mapping: even “decades” across the knob
uint32_t potToFreqLog(int adc) {            // adc: 0..1023
  float a = (float)adc / 1023.0f;           // 0..1
  float ratio = (float)F_MAX / (float)F_MIN;
  float f = (float)F_MIN * powf(ratio, a);  // 1..10000
  uint32_t out = (uint32_t)(f + 0.5f);
  if (out < F_MIN) out = F_MIN;
  if (out > F_MAX) out = F_MAX;
  return out;
}

// Choose prescaler for best resolution while keeping TOP <= 65535
// Mode 14: f_pwm = F_CPU / (prescaler * (TOP + 1))
void timer1Set(uint32_t fHz, uint8_t dutyPct) {
  if (fHz < 1) fHz = 1;

  // Candidate prescalers for Timer1 on ATmega328P
  const uint16_t prescalers[] = {1, 8, 64, 256, 1024};
  const uint8_t  csBits[]     = {
    (1 << CS10),                           // 1
    (1 << CS11),                           // 8
    (1 << CS11) | (1 << CS10),             // 64
    (1 << CS12),                           // 256
    (1 << CS12) | (1 << CS10)              // 1024
  };

  uint16_t top = 0;
  uint8_t cs = csBits[4]; // default 1024

  // pick smallest prescaler that keeps TOP in range
  for (int i = 0; i < 5; i++) {
    uint32_t N = prescalers[i];
    uint32_t calcTop = (F_CPU / (N * fHz)) - 1UL;
    if (calcTop <= 65535UL) {
      top = clamp16(calcTop);
      cs = csBits[i];
      break;
    }
  }
  if (top < 1) top = 1; // avoid degenerate TOP

  // duty in counts: 0..TOP
  if (dutyPct > 100) dutyPct = 100;
  uint32_t ocr = ((uint32_t)dutyPct * (uint32_t)(top + 1)) / 100UL;
  if (ocr > top) ocr = top; // keep in range
  uint16_t ocr1a = (uint16_t)ocr;

  // --- Configure Timer1 mode 14, non-inverting on OC1A ---
  // Update atomically to avoid glitches
  uint8_t sreg = SREG;
  cli();

  TCCR1A = 0;
  TCCR1B = 0;

  // Fast PWM, TOP = ICR1  (WGM13:0 = 14 -> 1110)
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // OC1A (D9) non-inverting: clear on compare, set at BOTTOM
  TCCR1A |= (1 << COM1A1);

  ICR1  = top;
  OCR1A = ocr1a;

  // start timer with chosen prescaler
  TCCR1B |= cs;

  SREG = sreg;
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  Serial.begin(9600);

  // initial set
  int fAdc = analogRead(FREQ_POT_PIN);
  int dAdc = analogRead(DUTY_POT_PIN);
  freqHz = potToFreqLog(fAdc);
  dutyPercent = (uint8_t)((uint32_t)dAdc * 100UL / 1023UL);

  timer1Set(freqHz, dutyPercent);
}

void loop() {
  int fAdc = analogRead(FREQ_POT_PIN);
  int dAdc = analogRead(DUTY_POT_PIN);

  uint32_t fNew = potToFreqLog(fAdc);
  uint8_t  dNew = (uint8_t)((uint32_t)dAdc * 100UL / 1023UL);
  if (dNew > 100) dNew = 100;

  // Only reprogram timer when values change (reduces jitter)
  if (fNew != lastFreq || dNew != lastDuty) {
    timer1Set(fNew, dNew);
    lastFreq = fNew;
    lastDuty = dNew;
    freqHz = fNew;
    dutyPercent = dNew;
  }

  // Serial print (not too fast)
  if (millis() - lastPrintMs >= 250) {
    lastPrintMs = millis();
    Serial.print("Freq pot: ");
    Serial.print(fAdc);
    Serial.print(" -> ");
    Serial.print(freqHz);
    Serial.print(" Hz | Duty pot: ");
    Serial.print(dAdc);
    Serial.print(" -> ");
    Serial.print(dutyPercent);
    Serial.println(" %");
  }
}
