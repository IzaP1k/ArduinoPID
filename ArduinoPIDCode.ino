#define LM35 A0

float Kp = 5.0, Ki = 0.2, Kd = 1.0;    // Wzmocnienia PID, stale
float integral = 0, lastError = 0;
float setpoint = 25, hysteresis = 0.3;  // Zadana temperatura i histereza

bool displayed_value = true;

const int dataPin = 2, clockPin = 4, latchPin = 9;   // 74HC595 - info
const int digitPin[3] = {11, 12, 13};                // Anody wyświetlacza - com
const uint8_t cathodeMap[10] = {                    // Wzorce segmentów 0–9
  0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,
  0b01101101,0b01111101,0b00000111,0b01111111,0b01101111
};
const uint8_t DOT = 0b10000000;                      // Kropka po drugiej cyfrze

void setup() {
  Serial.begin(9600);
  pinMode(3, OUTPUT);    // Wiatrak PWM
  pinMode(5, OUTPUT);    // Grzałka PWM
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  pinMode(0, INPUT_PULLUP);  // wyswietlacz
  pinMode(6, INPUT_PULLUP);  // Zmniejsz histereze
  pinMode(7, INPUT_PULLUP);  // Zwiększ histereze
  pinMode(8, INPUT_PULLUP);  // Zmniejsz setpoint
  pinMode(10, INPUT_PULLUP); // Zwiększ setpoint
  for (int i = 0; i < 3; i++) pinMode(digitPin[i], OUTPUT); // Wyświetlacz
}

void updateDisplay(float v) {
  static uint8_t cur = 0;
  int val = int(v * 10 + 0.5);
  int d[3] = { val/100 % 10, val/10 % 10, val % 10 };

  for (int i = 0; i < 3; i++) digitalWrite(digitPin[i], LOW);
  uint8_t patt = ~((cathodeMap[d[cur]]) | (cur == 1 ? DOT : 0));
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, patt);
  digitalWrite(latchPin, HIGH);
  digitalWrite(digitPin[cur], HIGH);
  cur = (cur + 1) % 3;
}

void loop() {
  float temp = analogRead(LM35) * 0.48828125;    // konwersja do °C
  float error = setpoint - temp;
  integral += error;
  float output = Kp * error + Ki * integral + Kd * (error - lastError);

  if (digitalRead(6) == LOW)     { hysteresis = max(0.0, hysteresis - 0.1); delay(150); }
  if (digitalRead(7) == LOW)     { hysteresis += 0.1; delay(150); }
  if (digitalRead(8) == LOW)     { setpoint = max(0.0, setpoint - 0.2); delay(150); }
  if (digitalRead(10) == LOW)    { setpoint += 0.2; delay(150); }

  static bool heating = false, fan = false;
  if (error > hysteresis) {
    analogWrite(5, constrain(output, 15, 255)); // Włącz grzałkę
    analogWrite(3, 0);
    heating = true; fan = false;
  } else if (error < -hysteresis) {
    analogWrite(3, constrain(-output, 100, 255)); // Włącz wiatrak
    analogWrite(5, 0);
    fan = true; heating = false;
  } else {
    if (heating) analogWrite(5, constrain(output, 15, 255));
    else if (fan) analogWrite(3, constrain(-output, 50, 255));
    else { analogWrite(5, 0); analogWrite(3, 0); }
  }

  Serial.print("T:"); Serial.print(temp);
  Serial.print(" SP:"); Serial.print(setpoint);
  Serial.print(" H:"); Serial.println(hysteresis);

  lastError = error;
  if (digitalRead(0) == LOW) {displayed_value = !displayed_value;}
  if (displayed_value) {
    for (int i = 0; i < 2000; i++) updateDisplay(temp);
  } else {
    for (int i = 0; i < 2000; i++) updateDisplay(setpoint);
  }
}
