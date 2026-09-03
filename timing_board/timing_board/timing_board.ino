volatile uint8_t counter_1 = 0;
volatile uint8_t counter_2 = 0;

void setup() {
  // Interrupt pin
  pinMode(2, INPUT);

  // Green LEDs
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  // Blue LEDs
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  // Set up ISR
  attachInterrupt(digitalPinToInterrupt(2), triggerISR, RISING);
}

void loop() {
  // Do nothing
}

void triggerISR() {
  // Green LEDs
  PORTC = (PORTC & 0b11110000) | (1 << counter_1);

  // Blue LEDs
  PORTB = (PORTB & 0b11110000) | counter_2;

  // Update counter
  counter_2++;
  if (counter_2 > 15) {
    counter_2 = 0;
    counter_1++;
  }
  if (counter_1 > 3) {
    counter_1 = 0;
  }
}
