#include <avr/sleep.h>
#include <avr/power.h>
#include <Arduino.h>

//#define DEBUG_OUTPUT

const uint8_t LED_PIN = 13;

#ifdef DEBUG_OUTPUT
volatile uint16_t pins;

ISR(PCINT0_vect) {
  pins &= ~(0B00011111 << 8);
  pins |= (PINB & 0B00011111) << 8;
}

ISR(PCINT2_vect) {
  pins &= ~(0B11111100);
  pins |= PIND & 0B11111100;
}
#else
EMPTY_INTERRUPT(PCINT0_vect);
EMPTY_INTERRUPT(PCINT2_vect);
#endif

void sleep() {
#ifdef DEBUG_OUTPUT
  Serial.println(F("Going to sleep..."));
  Serial.flush();
#endif
  ADCSRA = 0; // disable ADC
  power_all_disable(); // turn off various modules
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts(); // timed sequence follows
  sleep_enable();
  MCUCR = bit(BODS) | bit(BODSE); // turn off brown-out enable in software
  MCUCR = bit(BODS);
#ifdef DEBUG_OUTPUT
  pins = ((PINB & 0B00011111) << 8) | (PIND & 0B11111100);
#endif
  interrupts(); // guarantees next instruction executed
  sleep_cpu(); // sleep within 3 clock cycles of above
  sleep_disable();
  power_all_enable();
#ifdef DEBUG_OUTPUT
  Serial.println(F("Waking up"));
  Serial.print(F("GPIO: "));
  for (uint8_t i = LED_PIN - 1; i > 1; --i) {
    Serial.print((pins >> i) & 0x01);
  }
  Serial.println();
#endif
}

void setup() {
#ifdef DEBUG_OUTPUT
  Serial.begin(115200);
#endif
  for (uint8_t pin = 2; pin < LED_PIN; ++pin) {
    pinMode(pin, INPUT_PULLUP);
  }
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  PCMSK0 |= 0B00011111; // D8 - D12
  PCMSK2 |= 0B11111100; // D2 - D7
  PCIFR |= ((1 << PCIF0) | (1 << PCIF2));
  PCICR |= ((1 << PCIE0) | (1 << PCIE2));
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_PIN, LOW);
  delay(2000);
  sleep();
}
