#include <Arduino.h>
#include <Wire.h>

const int PIN_ENC_A = 23, PIN_ENC_B = 18;
const int PIN_IN3 = 26, PIN_IN4 = 27;
const int PIN_TCRT = 19;
const uint8_t LCD_ADDR = 0x27, INA_ADDR = 0x40;
const int PWM_FREQ = 2000, PWM_RES = 10;          // 0..1023
const int PWM_PRUEBA = 450;                        // ~44 %: lento y seguro

// ---------------- encoder (x4) ----------------
volatile long encCount = 0;
volatile uint8_t encState = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR isrEncoder() {
  static const int8_t tabla[16] = { 0,-1, 1, 0,  1, 0, 0,-1,  -1, 0, 0, 1,  0, 1,-1, 0 };
  uint8_t s = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  portENTER_CRITICAL_ISR(&mux);
  encCount += tabla[(encState << 2) | s];
  encState = s;
  portEXIT_CRITICAL_ISR(&mux);
}
long leerEncoder() { portENTER_CRITICAL(&mux); long c = encCount; portEXIT_CRITICAL(&mux); return c; }
void resetEncoder() { portENTER_CRITICAL(&mux); encCount = 0; portEXIT_CRITICAL(&mux); }

// ---------------- LCD 1602 por PCF8574 ----------------
// P0=RS P1=RW P2=E P3=luz P4..P7=D4..D7
bool lcdOK = false;
void lcdEscribe4(uint8_t v) {
  Wire.beginTransmission(LCD_ADDR); Wire.write(v | 0x04 | 0x08); Wire.endTransmission(); delayMicroseconds(2);
  Wire.beginTransmission(LCD_ADDR); Wire.write((v & ~0x04) | 0x08); Wire.endTransmission(); delayMicroseconds(60);
}
void lcdByte(uint8_t b, bool dato) {
  uint8_t rs = dato ? 0x01 : 0x00;
  lcdEscribe4((b & 0xF0) | rs);
  lcdEscribe4(((b << 4) & 0xF0) | rs);
}
void lcdInit() {
  delay(50);
  lcdEscribe4(0x30); delay(5); lcdEscribe4(0x30); delayMicroseconds(150); lcdEscribe4(0x30);
  lcdEscribe4(0x20);
  lcdByte(0x28, false); lcdByte(0x0C, false); lcdByte(0x06, false); lcdByte(0x01, false); delay(2);
}
void lcdPos(uint8_t col, uint8_t fila) { lcdByte(0x80 | (col + (fila ? 0x40 : 0)), false); }
void lcdTexto(const char *s) { while (*s) lcdByte(*s++, true); }
void lcdLinea(uint8_t fila, const char *s) {
  char buf[17]; snprintf(buf, sizeof(buf), "%-16s", s);
  lcdPos(0, fila); lcdTexto(buf);
}

// ---------------- INA219 (sin calibrar: shunt de 0.1 ohm) ----------------
bool inaOK = false;
int16_t inaReg(uint8_t reg) {
  Wire.beginTransmission(INA_ADDR); Wire.write(reg); Wire.endTransmission();
  Wire.requestFrom((int)INA_ADDR, 2);
  if (Wire.available() < 2) return 0;
  uint8_t hi = Wire.read(), lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}
float inaVoltios() { return ((uint16_t)inaReg(0x02) >> 3) * 0.004f; }
float inaMiliamps() { return inaReg(0x01) * 0.1f; }          // 10 µV / 0.1 Ω

// ---------------- motor ----------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmIni() { ledcAttach(PIN_IN3, PWM_FREQ, PWM_RES); ledcAttach(PIN_IN4, PWM_FREQ, PWM_RES); }
#else
void pwmIni() { ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_IN3, 0); ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_IN4, 1); }
#define ledcWrite(pin, d) ledcWrite((pin) == PIN_IN3 ? 0 : 1, d)
#endif
void motor(int pwm) {            // + adelante, - atrás, 0 parado
  if (pwm > 0)      { ledcWrite(PIN_IN3, pwm); ledcWrite(PIN_IN4, 0); }
  else if (pwm < 0) { ledcWrite(PIN_IN3, 0);   ledcWrite(PIN_IN4, -pwm); }
  else              { ledcWrite(PIN_IN3, 0);   ledcWrite(PIN_IN4, 0); }
}

bool hayDispositivo(uint8_t dir) { Wire.beginTransmission(dir); return Wire.endTransmission() == 0; }
bool pidieronSalir() {
  while (Serial.available()) { char c = Serial.read(); if (c == 'x' || c == 'X') return true; }
  return false;
}

void menu() {
  Serial.println();
  Serial.println("===== PRUEBA POR PARTES =====");
  Serial.println(" 1 Buscar I2C      2 LCD");
  Serial.println(" 3 Encoder (a mano) 4 Sensor TCRT5000");
  Serial.println(" 5 INA219          6 Motor adelante");
  Serial.println(" 7 Motor atras     8 Motor+encoder+INA");
  Serial.println(" (x = salir de una prueba)");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  pwmIni();
  motor(0);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  encState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncoder, CHANGE);
  pinMode(PIN_TCRT, INPUT);
  Wire.begin(21, 22);
  lcdOK = hayDispositivo(LCD_ADDR);
  inaOK = hayDispositivo(INA_ADDR);
  if (lcdOK) { lcdInit(); lcdLinea(0, "Pendulo PID"); lcdLinea(1, "Prueba x partes"); }
  menu();
}

void pruebaI2C() {
  Serial.println("Buscando I2C...");
  int n = 0;
  for (uint8_t d = 1; d < 127; d++) if (hayDispositivo(d)) {
    Serial.printf("  0x%02X", d);
    if (d == 0x27 || d == 0x3F) Serial.print("  <- LCD");
    if (d == 0x40) Serial.print("  <- INA219");
    Serial.println(); n++;
  }
  if (!n) Serial.println("  Nada. Revisa SDA->21, SCL->22, VCC y GND.");
  lcdOK = hayDispositivo(LCD_ADDR); inaOK = hayDispositivo(INA_ADDR);
  Serial.printf("LCD: %s   INA219: %s\n", lcdOK ? "OK" : "NO", inaOK ? "OK" : "NO");
  if (hayDispositivo(0x3F) && !lcdOK) Serial.println("Tu LCD esta en 0x3F: cambia LCD_ADDR a 0x3F.");
}

void pruebaLCD() {
  if (!hayDispositivo(LCD_ADDR)) { Serial.println("No encuentro la LCD en 0x27. Corre la prueba 1."); return; }
  lcdInit(); lcdOK = true;
  lcdLinea(0, "Hola Francisco!");
  lcdLinea(1, "LCD funcionando");
  Serial.println("Mira la LCD. Si se ve la luz pero no letras, gira el tornillito azul (contraste) del modulo.");
}

void pruebaEncoder() {
  resetEncoder();
  Serial.println("Gira el eje A MANO, despacio. Da UNA vuelta completa y anota las cuentas. (x = salir)");
  long ant = 1;
  while (!pidieronSalir()) {
    long c = leerEncoder();
    if (c != ant) {
      Serial.printf("cuentas: %ld   A=%d B=%d\n", c, digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
      if (lcdOK) { char b[17]; snprintf(b, 17, "Enc: %ld", c); lcdLinea(0, b); lcdLinea(1, "gira a mano"); }
      ant = c;
    }
    delay(50);
  }
}

void pruebaTCRT() {
  Serial.println("Pasa el rayo (o tu dedo) frente al sensor. (x = salir)");
  int ant = -1;
  while (!pidieronSalir()) {
    int v = digitalRead(PIN_TCRT);
    if (v != ant) {
      Serial.println(v == LOW ? "DETECTA (el LED del modulo prende)" : "libre");
      if (lcdOK) lcdLinea(1, v == LOW ? "Sensor: DETECTA" : "Sensor: libre");
      ant = v;
    }
    delay(20);
  }
}

void pruebaINA() {
  if (!hayDispositivo(INA_ADDR)) { Serial.println("No encuentro el INA219 en 0x40. Corre la prueba 1."); return; }
  Serial.println("Voltaje y corriente cada 0.3 s. (x = salir)");
  while (!pidieronSalir()) {
    float v = inaVoltios(), ma = inaMiliamps();
    Serial.printf("%.2f V   %.0f mA\n", v, ma);
    if (lcdOK) { char b[17]; snprintf(b, 17, "%.2fV %4.0fmA", v, ma); lcdLinea(1, b); }
    delay(300);
  }
}

void pruebaMotor(int dir, int ms, bool conDatos) {
  Serial.println(dir > 0 ? "Motor ADELANTE..." : "Motor ATRAS...");
  resetEncoder();
  unsigned long t0 = millis();
  motor(dir * PWM_PRUEBA);
  while (millis() - t0 < (unsigned long)ms) {
    if (pidieronSalir()) break;
    if (conDatos) {
      float v = inaOK ? inaVoltios() : 0, ma = inaOK ? inaMiliamps() : 0;
      Serial.printf("enc %ld   %.2f V   %.0f mA\n", leerEncoder(), v, ma);
    }
    delay(250);
  }
  motor(0);
  long c = leerEncoder();
  Serial.printf("Listo. Cuentas del encoder: %ld %s\n", c,
                c == 0 ? "<- el encoder NO cuenta" : (c * dir > 0 ? "(sentido correcto)" : "(sentido invertido: intercambia A y B)"));
  if (lcdOK) { char b[17]; snprintf(b, 17, "Enc: %ld", c); lcdLinea(0, b); lcdLinea(1, "motor parado"); }
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case '1': pruebaI2C(); break;
    case '2': pruebaLCD(); break;
    case '3': pruebaEncoder(); break;
    case '4': pruebaTCRT(); break;
    case '5': pruebaINA(); break;
    case '6': pruebaMotor(+1, 3000, false); break;
    case '7': pruebaMotor(-1, 3000, false); break;
    case '8': pruebaMotor(+1, 4000, true); break;
    default: return;               // ignora Enter y otros
  }
  menu();
}
