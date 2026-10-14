#include <Wire.h>
#include <Preferences.h>   

const float RPM_MAX = 130.0;
float CPR           = 11.0 * 4.0 * 30.0; // cuentas por vuelta del eje de salida
float RAMPA         = 40.0;              // RPM por segundo
int   PWM_MIN_MOV   = 250;               // PWM mínimo para que arranque
float VEL_POS       = 30.0;              // RPM máximas al moverse en modo posición
const float KP_POS  = 1.0;               // RPM por grado de error (encoder)
const float TOL_DEG = 1.5;               // tolerancia para "llegó"

Preferences prefs;
const float CPR_FABRICA = 11.0 * 4.0 * 30.0;
float FACTOR_LA  = 1.0;    // corrección de grados en lazo abierto
float ultimoGiro = 0;      // último giro pedido en modo posición
bool  calManual  = false;  // calibración a mano en curso

const int PIN_ENC_A = 23;
const int PIN_ENC_B = 18;
const int PIN_IN3   = 26;
const int PIN_IN4   = 27;
const int PIN_SDA   = 21;
const int PIN_SCL   = 22;

const int PWM_FREQ = 2000;
const int PWM_RES  = 10;
const int PWM_MAX  = 1023;
const uint32_t TS_MS = 10;
const float ALPHA_RPM = 0.3;

float Kp = 2.0, Ki = 15.0;
float refTeclado = 0;
float refRampa   = 0;
bool  usarEncoder = true;

// Modo posición
bool  modoPos   = false;
bool  moviendo  = false;
float objetivoDeg = 0;   // posición absoluta a la que va
float posEst      = 0;   // posición estimada (lazo abierto)
float vCmd        = 0;   // velocidad con signo que se está pidiendo (RPM)

uint8_t lcdAddr = 0;
bool lcdOk = false;
const uint8_t LCD_RS = 0x01, LCD_EN = 0x04, LCD_BL = 0x08;

void lcdNibble(uint8_t datos, uint8_t modo) {
  uint8_t v = (datos & 0xF0) | modo | LCD_BL;
  Wire.beginTransmission(lcdAddr);
  Wire.write(v | LCD_EN);
  Wire.write(v & ~LCD_EN);
  Wire.endTransmission();
  delayMicroseconds(50);
}
void lcdSend(uint8_t b, uint8_t modo) { lcdNibble(b & 0xF0, modo); lcdNibble((b << 4) & 0xF0, modo); }
void lcdCmd(uint8_t c) { lcdSend(c, 0); if (c <= 0x02) delay(2); }
void lcdSetCursor(uint8_t col, uint8_t fila) { lcdCmd(0x80 | (col + (fila ? 0x40 : 0))); }
void lcdPrint(const char* s) { while (*s) lcdSend(*s++, LCD_RS); }
bool hayDispositivo(uint8_t d) { Wire.beginTransmission(d); return Wire.endTransmission() == 0; }

bool lcdInit() {
  const uint8_t tipicas[] = { 0x27, 0x3F };
  for (uint8_t d : tipicas) if (hayDispositivo(d)) { lcdAddr = d; break; }
  if (!lcdAddr) for (uint8_t d = 0x20; d <= 0x3F; d++) if (hayDispositivo(d)) { lcdAddr = d; break; }
  if (!lcdAddr) return false;
  delay(50);
  lcdNibble(0x30, 0); delay(5);
  lcdNibble(0x30, 0); delay(1);
  lcdNibble(0x30, 0);
  lcdNibble(0x20, 0);
  lcdCmd(0x28); lcdCmd(0x0C); lcdCmd(0x06); lcdCmd(0x01);
  return true;
}

volatile long encCount = 0;
volatile uint8_t encState = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrEncoder() {
  static const int8_t tabla[16] = { 0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0 };
  uint8_t s = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  portENTER_CRITICAL_ISR(&mux);
  encCount += tabla[(encState << 2) | s];
  encState = s;
  portEXIT_CRITICAL_ISR(&mux);
}
long leerEncoder() { portENTER_CRITICAL(&mux); long c = encCount; portEXIT_CRITICAL(&mux); return c; }
float gradosEncoder() { return leerEncoder() / CPR * 360.0f; }

void setMotor(float u) {
  int duty = constrain((int)fabsf(u), 0, PWM_MAX);
  if (u > 0)      { ledcWrite(PIN_IN3, duty); ledcWrite(PIN_IN4, 0); }
  else if (u < 0) { ledcWrite(PIN_IN3, 0);    ledcWrite(PIN_IN4, duty); }
  else            { ledcWrite(PIN_IN3, 0);    ledcWrite(PIN_IN4, 0); }
}

float pwmParaRPM(float rpm) {
  if (rpm <= 0) return 0;
  return PWM_MIN_MOV + rpm / RPM_MAX * (PWM_MAX - PWM_MIN_MOV);
}

// ===================== PID VELOCIDAD =====================
float integ = 0;
float control(float ref, float rpm, float dt) {
  float ff = pwmParaRPM(ref);
  if (!usarEncoder) return ff;
  float e = ref - rpm;
  float iNew = integ + Ki * e * dt;
  float u = ff + Kp * e + iNew;
  if (u > PWM_MAX) { u = PWM_MAX; if (e < 0) integ = iNew; }
  else if (u < 0)  { u = 0;       if (e > 0) integ = iNew; }
  else integ = iNew;
  return u;
}

void imprimirEstado() {
  Serial.printf("# MODO %s | %s | rampa=%.0f | PWM min=%d | vel. posicion=%.0f RPM\n",
                modoPos ? "POSICION" : "VELOCIDAD",
                usarEncoder ? "con encoder" : "LAZO ABIERTO (aproximado)",
                RAMPA, PWM_MIN_MOV, VEL_POS);
  Serial.printf("# [CAL] CPR=%.1f | factor lazo abierto=%.3f\n", CPR, FACTOR_LA);   // [CAL]
}
void imprimirAyuda() {
  Serial.println("# p = modo posicion, v = modo velocidad, s = parar");
  Serial.println("# Velocidad: 1..130 RPM | Posicion: grados (90, 180, -45, 720)");
  Serial.println("# pm <v> = PWM minimo, ra <v> = rampa, vp <v> = vel. en posicion, g = estado");
  Serial.println("# [CAL] re <grados reales>, cal + ok, borrar");   // [CAL]
}

float posicionActual() { return usarEncoder ? gradosEncoder() : posEst; }

// [CAL] Guarda la calibración en la memoria del ESP32
void guardarCal() {
  prefs.putFloat("cpr", CPR);
  prefs.putFloat("fla", FACTOR_LA);
  Serial.printf("# [CAL] Guardado: CPR=%.1f, factor lazo abierto=%.3f\n", CPR, FACTOR_LA);
}

void leerComandos() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  // ---------- [CAL] Comandos de calibración ----------
  if (cmd == "cal") {
    moviendo = false; vCmd = 0; refTeclado = 0; refRampa = 0; integ = 0;
    portENTER_CRITICAL(&mux); encCount = 0; portEXIT_CRITICAL(&mux);
    calManual = true;
    Serial.println("# [CAL] Motor apagado. Marca el eje, gíralo A MANO exactamente 1 vuelta y escribe ok");
    return;
  }
  if (cmd == "ok") {
    if (!calManual) { Serial.println("# [CAL] Primero escribe cal"); return; }
    calManual = false;
    long c = labs(leerEncoder());
    if (c < 20) { Serial.println("# [CAL] El encoder casi no contó: revisa sus cables. No se cambió nada."); return; }
    CPR = c;
    guardarCal();
    posEst = gradosEncoder(); objetivoDeg = posicionActual();
    return;
  }
  if (cmd == "borrar") {
    CPR = CPR_FABRICA; FACTOR_LA = 1.0;
    guardarCal();
    return;
  }
  // ---------------------------------------------------

  // --- Cambio de modo ---
  if (cmd == "p") {
    refTeclado = 0; refRampa = 0; integ = 0;
    modoPos = true; moviendo = false; vCmd = 0;
    posEst = gradosEncoder();
    objetivoDeg = posicionActual();
    Serial.println("# MODO POSICION: escribe los grados a girar (ej. 90, 180, -45)");
    return;
  }
  if (cmd == "v") {
    modoPos = false; moviendo = false; vCmd = 0;
    refTeclado = 0; refRampa = 0; integ = 0;
    Serial.println("# MODO VELOCIDAD: escribe RPM (1..130)");
    return;
  }

  // --- Números ---
  char c0 = cmd.charAt(0);
  if (isDigit(c0) || c0 == '-' || c0 == '+') {
    float v = cmd.toFloat();
    if (modoPos) {
      objetivoDeg = posicionActual() + v;
      moviendo = (fabsf(v) > 0);
      ultimoGiro = v;   // [CAL]
      Serial.printf("# Girando %.1f grados%s\n", v, usarEncoder ? "" : " (aproximado, sin encoder)");
    } else {
      refTeclado = constrain(v, 0, RPM_MAX);
      Serial.printf("# Objetivo: %.0f RPM\n", refTeclado);
    }
    return;
  }

  if (cmd == "s") {
    if (modoPos) { moviendo = false; objetivoDeg = posicionActual(); Serial.println("# Movimiento detenido"); }
    else         { refTeclado = 0; Serial.println("# Parando con rampa"); }
    return;
  }
  if (cmd == "g") { imprimirEstado(); return; }
  if (cmd == "h") { imprimirAyuda(); return; }

  if (cmd.length() > 3 && cmd.charAt(2) == ' ') {
    String k = cmd.substring(0, 2);
    float v = cmd.substring(3).toFloat();
    if      (k == "pm") PWM_MIN_MOV = constrain((int)v, 0, PWM_MAX - 1);
    else if (k == "ra") RAMPA = max(1.0f, v);
    else if (k == "vp") VEL_POS = constrain(v, 1.0f, RPM_MAX);
    else if (k == "kp") Kp = v;
    else if (k == "ki") { Ki = v; integ = 0; }
    else if (k == "re") {   // [CAL] corrección con los grados reales
      if (ultimoGiro == 0 || v == 0) { Serial.println("# [CAL] Primero haz un giro en modo posicion (ej. 360) y luego re <grados reales>"); return; }
      float real = fabsf(v), pedido = fabsf(ultimoGiro);
      if (usarEncoder) CPR = CPR * pedido / real;            // giró de más -> sube CPR
      else             FACTOR_LA = FACTOR_LA * real / pedido; // giró de más -> sube factor
      guardarCal();
      posEst = gradosEncoder(); objetivoDeg = posicionActual();
      ultimoGiro = 0;
      return;
    }
    else { Serial.println("# Comando no reconocido (h = ayuda)"); return; }
    imprimirEstado();
    return;
  }
  Serial.println("# Comando no reconocido (h = ayuda)");
}

void actualizarLCD(float sp, float rpm, float u) {
  if (!lcdOk) return;
  char l1[17], l2[17];
  if (calManual) {   // [CAL]
    snprintf(l1, sizeof(l1), "CAL: 1 vuelta   ");
    snprintf(l2, sizeof(l2), "Cuentas:%8ld", labs(leerEncoder()));
  } else if (modoPos) {
    snprintf(l1, sizeof(l1), "Obj:%8.1f gr ", objetivoDeg);
    snprintf(l2, sizeof(l2), "Pos:%8.1f%s", posicionActual(), usarEncoder ? " gr " : " LA ");
  } else {
    snprintf(l1, sizeof(l1), "SP (rpm):%7.0f", sp);
    if (usarEncoder) snprintf(l2, sizeof(l2), "PV (rpm):%7.1f", rpm);
    else             snprintf(l2, sizeof(l2), "PWM: %4d  L.A. ", (int)u);
  }
  lcdSetCursor(0, 0); lcdPrint(l1);
  lcdSetCursor(0, 1); lcdPrint(l2);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(300);

  // [CAL] Carga la calibración guardada
  prefs.begin("pendulo", false);
  CPR       = prefs.getFloat("cpr", CPR_FABRICA);
  FACTOR_LA = prefs.getFloat("fla", 1.0);

  ledcAttach(PIN_IN3, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_IN4, PWM_FREQ, PWM_RES);
  setMotor(0);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  encState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncoder, CHANGE);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  lcdOk = lcdInit();
  if (lcdOk) Serial.printf("# LCD encontrada en 0x%02X\n", lcdAddr);
  else       Serial.println("# LCD NO encontrada: revisa SDA->21, SCL->22, VCC y GND");

  imprimirAyuda();
  imprimirEstado();
}

// ===================== LOOP =====================
void loop() {
  static uint32_t tPrev = millis(), tPrint = 0, tLcd = 0, tSinSenal = 0;
  static long cuentaPrev = 0;
  static float rpm = 0;

  uint32_t ahora = millis();
  if (ahora - tPrev < TS_MS) return;
  float dt = (ahora - tPrev) / 1000.0f;
  tPrev = ahora;

  leerComandos();

  long cuenta = leerEncoder();
  long delta = cuenta - cuentaPrev;
  cuentaPrev = cuenta;
  rpm += ALPHA_RPM * (fabsf((delta / CPR) / dt * 60.0f) - rpm);

  // [CAL] Durante la calibración a mano, motor apagado
  if (calManual) {
    setMotor(0);
    if (ahora - tLcd >= 250) { tLcd = ahora; actualizarLCD(0, 0, 0); }
    return;
  }

  float u = 0;

  if (modoPos) {
    // ================= CONTROL DE POSICIÓN =================
    float pos = posicionActual();
    float err = objetivoDeg - pos;
    float vDes = 0;

    if (moviendo) {
      if (usarEncoder) {
        if (fabsf(err) < TOL_DEG) {
          moviendo = false;
          Serial.printf("# Llego: %.1f grados\n", pos);
        } else {
          vDes = constrain(KP_POS * err, -VEL_POS, VEL_POS);   // frena al acercarse
        }
      } else {
        if (fabsf(err) < 0.5f) {
          moviendo = false;
          Serial.printf("# Llego (aprox.): %.1f grados\n", pos);
        } else {
          vDes = (err > 0 ? VEL_POS : -VEL_POS);
        }
      }
    }

    if (usarEncoder) {
      float paso = RAMPA * dt;                      // arranque suave
      if (vCmd < vDes) vCmd = min(vCmd + paso, vDes);
      else if (vCmd > vDes) vCmd = max(vCmd - paso, vDes);
      if (!moviendo) vCmd = 0;
    } else {
      vCmd = vDes;                                  // sin encoder: velocidad fija
      float avance = vCmd * 6.0f * dt * FACTOR_LA;  // [CAL] corregido con el factor
      if (fabsf(avance) > fabsf(err)) avance = err; // no pasarse del objetivo
      posEst += avance;
    }

    u = (vCmd > 0 ? 1 : -1) * pwmParaRPM(fabsf(vCmd));
    if (vCmd == 0) u = 0;

  } else {
    // ================= CONTROL DE VELOCIDAD =================
    float paso = RAMPA * dt;
    if (refRampa < refTeclado) refRampa = min(refRampa + paso, refTeclado);
    else if (refRampa > refTeclado) refRampa = max(refRampa - paso, refTeclado);

    u = (refRampa > 0) ? control(refRampa, rpm, dt) : 0;
    if (refRampa <= 0) integ = 0;
  }

  // Encoder sin señal con el motor andando -> lazo abierto (no se para)
  if (usarEncoder && fabsf(u) > PWM_MIN_MOV + 100 && labs(delta) == 0) {
    if (tSinSenal == 0) tSinSenal = ahora;
    if (ahora - tSinSenal > 1500) {
      usarEncoder = false;
      integ = 0;
      posEst = gradosEncoder();
      if (!modoPos) u = pwmParaRPM(refRampa);
      Serial.println("# Encoder sin senal -> sigo en LAZO ABIERTO (aproximado)");
    }
  } else if (labs(delta) > 0) tSinSenal = 0;

  setMotor(u);

  if (ahora - tPrint >= 200) {
    tPrint = ahora;
    if (modoPos) Serial.printf("obj:%.1f,pos:%.1f,vel:%.1f,pwm:%.0f\n", objetivoDeg, posicionActual(), vCmd, u);
    else         Serial.printf("obj:%.0f,rampa:%.1f,rpm:%.1f,pwm:%.0f\n", refTeclado, refRampa, rpm, u);
  }
  if (ahora - tLcd >= 250) {
    tLcd = ahora;
    actualizarLCD(refRampa, rpm, u);
  }
}