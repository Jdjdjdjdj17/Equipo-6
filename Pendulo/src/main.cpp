// Pendulo PID con ESP32
// encoder 23 y 18, L298N 26 y 27, TCRT 19, I2C 21 y 22
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
const int ENC_A = 23, ENC_B = 18, IN3 = 26, IN4 = 27, TCRT = 19, BOOT = 0, PWM_MAX = 1023;
const uint8_t INA = 0x40;
float CPR = 2059.2, RAMPA = 40, VEL_POS = 30, Kp = 2, Ki = 15, I_LIM = 2500, V_BAJO = 9.6, OFFSET = 0;
int PWM_MIN = 250, periodoTx = 0, formato = 0;  // fm 2 = teleplot
const float VEL_CAL = 15, TOL = 1.5;
Preferences prefs;
enum Modo { VEL, POS, PERFIL, TRAY, CAL, CERO } modo = VEL;
float tr0 = 0, trD = 0, trT = 1; uint32_t tTr = 0;  // para mv
float rpm = 0, ref = 0, refObj = 0, integ = 0, objDeg = 0, vCmd = 0, u = 0, refAnt = 0, acel = 0, errMin = 1e9;
float voltios = 0, mA = 0; bool inaOk = false, moviendo = false, monitorOn = false, repetir = false; int sgAnt = 0;
const char *PERF[] = { "const", "rampa", "trap", "seno", "escalon", "exp", "gauss", "sierra", "ruido" };
float pA = 100, pT = 10, pX = 0, ruido = 0; int perfil = 0; uint32_t tPerf = 0, tRuido = 0;
// sensor y calibracion
const float KP_POS = 1.5, VEL_MAX_TR = 110;
int sEst = HIGH, nMarcas = 0; long marca = 0, marcas[3], inicioCal = 0; uint32_t tCal = 0, nPasos = 0;
bool hayMarca = false, marcaNueva = false, sVisto = false;
volatile long tIniC = 0, tMarca = 0, tAncho = 0; volatile bool tDentro = false, tNueva = false;
// encoder
volatile long enc = 0; volatile uint8_t encSt = 0; long cuentaPrev = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR isrEnc() {
  static const int8_t T[16] = { 0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0 };
  uint8_t s = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  portENTER_CRITICAL_ISR(&mux); enc += T[(encSt << 2) | s]; encSt = s; portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR isrTcrt() {
  bool det = digitalRead(TCRT) == LOW; static uint32_t us0 = 0; uint32_t us = micros();
  portENTER_CRITICAL_ISR(&mux);
  if (det && !tDentro) { tDentro = true; tIniC = enc; us0 = us; }
  else if (!det && tDentro) { tDentro = false; if (us - us0 > 300) { tAncho = enc - tIniC; tMarca = (tIniC + enc) / 2; tNueva = true; } }
  portEXIT_CRITICAL_ISR(&mux);
}
long leerEnc() { portENTER_CRITICAL(&mux); long c = enc; portEXIT_CRITICAL(&mux); return c; }
float grados() { return leerEnc() / CPR * 360.0f; }
void moverCero(long c) {
  portENTER_CRITICAL(&mux); enc -= c; tIniC -= c; tMarca -= c; portEXIT_CRITICAL(&mux);
  cuentaPrev -= c; marca -= c; inicioCal -= c;
  for (long &m : marcas) m -= c;
}
// motor
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmIni() { ledcAttach(IN3, 2000, 10); ledcAttach(IN4, 2000, 10); }
void pwm(int pin, int d) { ledcWrite(pin, d); }
#else
void pwmIni() { ledcSetup(0, 2000, 10); ledcAttachPin(IN3, 0); ledcSetup(1, 2000, 10); ledcAttachPin(IN4, 1); }
void pwm(int pin, int d) { ledcWrite(pin == IN3 ? 0 : 1, d); }
#endif
void motor(float v) { int d = constrain((int)fabsf(v), 0, PWM_MAX); pwm(IN3, v > 0 ? d : 0); pwm(IN4, v < 0 ? d : 0); }
float ff(float r) { return r <= 0 ? 0 : PWM_MIN + r / 130.0f * (PWM_MAX - PWM_MIN); }
float pi(float r, float dt) {  // PI con antiwindup
  float e = r - rpm, iN = integ + Ki * e * dt, s = ff(r) + Kp * e + iN;
  if (s > PWM_MAX) { if (e < 0) integ = iN; return PWM_MAX; }  if (s < 0) { if (e > 0) integ = iN; return 0; }
  integ = iN; return s;
}
// lcd e ina219
uint8_t lcdAddr = 0;
bool hay(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }
void lcdNib(uint8_t d, uint8_t m) {
  uint8_t v = (d & 0xF0) | m | 0x08;
  Wire.beginTransmission(lcdAddr); Wire.write(v | 4); Wire.write(v & ~4); Wire.endTransmission(); delayMicroseconds(50);
}
void lcdByte(uint8_t b, uint8_t m) { lcdNib(b & 0xF0, m); lcdNib(b << 4, m); }
void lcdInit() {
  lcdAddr = hay(0x27) ? 0x27 : hay(0x3F) ? 0x3F : 0;
  if (!lcdAddr) return;
  delay(50); lcdNib(0x30, 0); delay(5); lcdNib(0x30, 0); delay(1); lcdNib(0x30, 0); lcdNib(0x20, 0);
  for (uint8_t c : { 0x28, 0x0C, 0x06, 0x01 }) lcdByte(c, 0); delay(2);
}
void lcdLinea(uint8_t f, const char *s) {
  if (!lcdAddr) return;
  char b[17]; snprintf(b, 17, "%-16s", s);
  lcdByte(0x80 | (f ? 0x40 : 0), 0); for (char *p = b; *p; p++) lcdByte(*p, 1);
}
int16_t inaReg(uint8_t r) {
  Wire.beginTransmission(INA); Wire.write(r); Wire.endTransmission();
  Wire.requestFrom((int)INA, 2); if (Wire.available() < 2) return 0;
  uint8_t h = Wire.read(), l = Wire.read(); return (int16_t)((h << 8) | l);
}
void leerINA() { if (inaOk) { voltios = ((uint16_t)inaReg(2) >> 3) * 0.004f; mA = inaReg(1) * 0.1f; } }  // shunt de 0.1
// perfiles
float refPerfil(float t) {
  float r = 0, x = pX;
  switch (perfil) {
    case 0: r = pA; break;
    case 1: r = pA * t / pT; break;
    case 2: { float ta = x > 0 ? x : pT / 4;  // trapecio
              r = t < ta ? pA * t / ta : t < pT - ta ? pA : pA * (pT - t) / ta; break; }
    case 3: r = pA / 2 * (1 - cosf(2 * PI * (x > 0 ? x : 0.2f) * t)); break;
    case 4: { int n = x >= 1 ? (int)x : 5; r = pA * min(n, (int)(t / (pT / n)) + 1) / n; break; }  // escalones
    case 5: r = pA * (1 - expf(-t / (x > 0 ? x : pT / 5))); break;  // exponencial
    case 6: { float s = x > 0 ? x : pT / 8, d = t - pT / 2; r = pA * expf(-d * d / (2 * s * s)); break; }  // gauss
    case 7: { float P = x > 0 ? x : pT / 4; r = pA * fmodf(t, P) / P; break; }  // sierra
    case 8: if (millis() - tRuido >= 100) {  // ruido
              tRuido = millis();
              ruido = (x > 0 ? x : pA * 0.2f) * sqrtf(-2 * logf(random(1, 10001) / 10001.0f)) * cosf(2 * PI * random(10000) / 10000.0f);
            }
            r = pA + ruido; break;
  }
  return constrain(r, 0.0f, 130.0f);
}
// calibracion
void parar() {
  if (modo == PERFIL) Serial.printf("# PERFIL_FIN,%s,parado\n", PERF[perfil]);
  if (modo == TRAY) Serial.println("# PERFIL_FIN,tray,parado");
  modo = VEL; refObj = 0; integ = 0; moviendo = false; vCmd = 0;
}
void iniciarCal(Modo m) {
  parar(); modo = m; nMarcas = 0; tCal = millis(); inicioCal = leerEnc();
  Serial.println(m == CAL ? "# CALIBRANDO: 2 vueltas con el sensor y regreso a donde estaba (ese es tu 0)"
                          : "# Buscando el 0 con el sensor + offset...");
}
void finCal(long c0, const char *q) {
  long k = lround((double)(leerEnc() - c0) / CPR);
  moverCero(c0 + lround(k * (double)CPR));
  modo = POS; objDeg = 0; moviendo = true; vCmd = 0; integ = 0;
  Serial.printf("# CERO listo (%s): estoy en %.1f grados, regreso a 0\n", q, grados());
}
float pasoCal(float dt) {
  if (millis() - tCal > 30000) { modo = VEL; Serial.println("# No vi la marca en 30 s: revisa el TCRT5000"); return 0; }
  if (marcaNueva && !(nMarcas && labs(marca - marcas[nMarcas - 1]) < 300)) {
    marcas[nMarcas++] = marca;
    Serial.printf("# Marca %d en cuenta %ld\n", nMarcas, marca);
    if (modo == CERO) { finCal(marca + lround(OFFSET / 360.0 * CPR), "sensor + offset"); return 0; }
    if (nMarcas == 3) {
      float n = fabsf((float)(marcas[2] - marcas[0])) / 2;
      if (n > 300 && n < 50000) { CPR = n; prefs.putFloat("cpr", CPR); Serial.printf("# CPR = %.1f (guardado)\n", CPR); }
      else Serial.printf("# Calibracion fallo: midio %.0f cuentas por vuelta\n", n);
      finCal(inicioCal, "posicion inicial"); return 0;
    }
  }
  return pi(VEL_CAL, dt);
}
// comandos
void ayuda() {
  Serial.println("# v = velocidad | p = posicion | numero = RPM o grados | s = parar | c / BOOT = calibrar | r = grados a 0");
  Serial.println("# cero = ir al 0 del sensor | of = aprender offset (of 10 = a mano) | m = monitor | g = estado");
  Serial.println("# pf <const|rampa|trap|seno|escalon|exp|gauss|sierra|ruido> [A T extra] | pa pt px = valores | rep = repetir");
  Serial.println("# mv <grados> [segundos] = trayectoria con aceleracion, avance y frenado (ej. mv 360 2)");
  Serial.println("# ajustes: pm ra vp kp ki il vb tx | fm 2 = Teleplot, fm 0 = graficar.py  (ej. tx 50)");
}
void estado() {
  Serial.printf("# modo %d | CPR %.1f | offset %.1f | INA %s %.2f V %.0f mA | LCD %s | sensor %s\n", modo, CPR, OFFSET,
                inaOk ? "OK" : "NO", voltios, mA, lcdAddr ? "OK" : "NO", sEst == LOW ? "DETECTA" : "libre");
  Serial.printf("# perfil A=%.0f T=%.1f extra=%.2f | pm %d ra %.0f vp %.0f kp %.2f ki %.2f il %.0f vb %.1f tx %d fm %d\n",
                pA, pT, pX, PWM_MIN, RAMPA, VEL_POS, Kp, Ki, I_LIM, V_BAJO, periodoTx, formato);
}
bool ajustar(const String &k, float v) {
  if (k == "pa") pA = constrain(v, 0.0f, 130.0f); else if (k == "pt") pT = max(1.0f, v); else if (k == "px") pX = max(0.0f, v);
  else if (k == "pm") PWM_MIN = v; else if (k == "ra") RAMPA = max(1.0f, v); else if (k == "vp") VEL_POS = v;
  else if (k == "kp") Kp = v; else if (k == "ki") { Ki = v; integ = 0; } else if (k == "il") I_LIM = v;
  else if (k == "vb") V_BAJO = v; else if (k == "tx") prefs.putInt("tx", periodoTx = v); else if (k == "fm") prefs.putInt("fm", formato = v); else return false;
  return true;
}
void comando(String c) {
  c.trim(); c.toLowerCase(); if (!c.length()) return;
  int sp = c.indexOf(' '); String k = sp < 0 ? c : c.substring(0, sp);
  float v = sp < 0 ? 0 : c.substring(sp + 1).toFloat();
  if (isDigit(c[0]) || ((c[0] == '-' || c[0] == '+') && c.length() > 1)) {
    if (modo == POS) { objDeg = grados() + c.toFloat(); moviendo = true; Serial.printf("# Girando %.1f grados\n", c.toFloat()); }
    else { if (modo != VEL) parar(); refObj = constrain(c.toFloat(), 0.0f, 130.0f); Serial.printf("# Objetivo %.0f RPM\n", refObj); }
  }
  else if (k == "v") { parar(); Serial.println("# MODO VELOCIDAD (1..130 RPM)"); }
  else if (k == "p") { parar(); modo = POS; ref = 0; objDeg = grados(); Serial.println("# MODO POSICION (grados a girar)"); }
  else if (k == "s") { if (modo == POS) { moviendo = false; objDeg = grados(); } else parar(); Serial.println("# Parar"); }
  else if (k == "c") iniciarCal(CAL);
  else if (k == "cero") iniciarCal(CERO);
  else if (k == "r" || k == "z") { moverCero(leerEnc()); objDeg = 0; moviendo = false; Serial.println("# Este punto ahora es 0 grados"); }
  else if (k == "of") {
    if (sp > 0) OFFSET = v;
    else if (hayMarca) { float m = fmodf(marca / CPR * 360, 360); if (m > 180) m -= 360; if (m < -180) m += 360; OFFSET = -m; }
    else { Serial.println("# Primero haz que el brazo pase por el sensor (p y luego 360)"); return; }
    prefs.putFloat("of", OFFSET); Serial.printf("# Offset del sensor = %.1f grados (guardado)\n", OFFSET);
  }
  else if (k == "pf") {
    String t = sp < 0 ? "" : c.substring(sp + 1);
    int s2 = t.indexOf(' '); String nom = s2 < 0 ? t : t.substring(0, s2);
    int i = 0; while (i < 9 && nom != PERF[i]) i++;
    if (i == 9) { Serial.println("# Perfiles: const rampa trap seno escalon exp gauss sierra ruido"); return; }
    if (s2 > 0) { float a[3] = { pA, pT, 0 }; sscanf(t.substring(s2 + 1).c_str(), "%f %f %f", &a[0], &a[1], &a[2]); pA = a[0]; pT = a[1]; pX = a[2]; }
    parar(); perfil = i; pA = constrain(pA, 0.0f, 130.0f); pT = max(1.0f, pT); modo = PERFIL; tPerf = millis(); integ = 0;
    Serial.printf("# PERFIL_INICIO,%s,%.1f,%.1f,%.2f\n", PERF[i], pA, pT, pX);
  }
  else if (k == "mv") {  // trayectoria
    float a[2] = { 0, 0 }; sscanf(c.substring(sp + 1).c_str(), "%f %f", &a[0], &a[1]);
    float tMin = fabsf(a[0]) / (VEL_MAX_TR * 6) * 4 / 3;
    parar(); modo = TRAY; tr0 = grados(); trD = a[0]; trT = max(max(a[1], tMin), 0.3f); tTr = millis(); integ = 0; sgAnt = 0;
    if (a[1] > 0 && a[1] < tMin) Serial.printf("# Muy rapido para el motor: lo hago en %.2f s\n", trT);
    Serial.printf("# PERFIL_INICIO,tray,%.1f,%.2f,0\n", trD, trT);
  }
  else if (k == "rep") { repetir = !repetir; Serial.println(repetir ? "# Repetir perfil: SI (s = parar)" : "# Repetir perfil: NO"); }
  else if (k == "m") { monitorOn = !monitorOn; Serial.println(monitorOn ? "# Monitor encendido (m = apagar)" : "# Monitor apagado"); }
  else if (k == "g") estado();
  else if (k == "h") ayuda();
  else if (sp > 0 && ajustar(k, v)) Serial.printf("# %s = %.2f\n", k.c_str(), v);
  else Serial.println("# Comando no reconocido (h = ayuda)");
}
void setup() {
  Serial.begin(115200); delay(300);
  prefs.begin("pendulo", false); CPR = prefs.getFloat("cpr", CPR); OFFSET = prefs.getFloat("of", 0);
  periodoTx = prefs.getInt("tx", 0); formato = prefs.getInt("fm", 0);
  pwmIni(); motor(0);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP); pinMode(TCRT, INPUT); pinMode(BOOT, INPUT_PULLUP);
  encSt = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrEnc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(TCRT), isrTcrt, CHANGE);
  Wire.begin(21, 22); Wire.setClock(400000);
  lcdInit(); inaOk = hay(INA); leerINA(); randomSeed(micros());
  lcdLinea(0, "Pendulo PID"); lcdLinea(1, "c o BOOT = cal.");
  Serial.println("\n# ===== PENDULO PID ====="); ayuda(); estado();
  Serial.println("# Pon el brazo en tu 0 y escribe c (o presiona BOOT) para calibrar.");
}
void loop() {
  static uint32_t tPrev = millis(), tTx = 0, tLcd = 0, tMon = 0, tIna = 0, tI = 0, tV = 0;
  static String linea; static bool bootSuelto = true;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (linea.length()) comando(linea); linea = ""; }
    else if (ch == 8 || ch == 127) { if (linea.length()) linea.remove(linea.length() - 1); }
    else if (linea.length() < 60) linea += ch;
  }
  uint32_t ahora = millis();
  if (ahora - tPrev < 10) return;
  float dt = min((ahora - tPrev) / 1000.0f, 0.1f); tPrev = ahora;
  long cuenta = leerEnc(), delta = cuenta - cuentaPrev; cuentaPrev = cuenta;
  rpm += 0.3f * (fabsf(delta / CPR / dt * 60) - rpm);
  if (ahora - tIna >= 20) { tIna = ahora; leerINA(); }
  portENTER_CRITICAL(&mux); bool nueva = tNueva; tNueva = false; long mk = tMarca, an = tAncho; sEst = tDentro ? LOW : HIGH; portEXIT_CRITICAL(&mux);
  marcaNueva = false; if (sEst == LOW) sVisto = true;
  if (nueva && labs(an) < CPR / 4) { marca = mk; hayMarca = marcaNueva = sVisto = true; nPasos++; }
  if (digitalRead(BOOT) == LOW) { if (bootSuelto && ahora - tCal > 1000) { bootSuelto = false; iniciarCal(CAL); } }
  else bootSuelto = true;
  switch (modo) {
    case VEL: {
      float p = RAMPA * dt;
      ref = ref < refObj ? min(ref + p, refObj) : max(ref - p, refObj);
      u = ref > 0 ? pi(ref, dt) : 0; if (ref <= 0) integ = 0; break;
    }
    case PERFIL: {
      float t = (ahora - tPerf) / 1000.0f;
      if (t >= pT && repetir) { tPerf = ahora; t = 0; }
      if (t >= pT) { Serial.printf("# PERFIL_FIN,%s,completo\n", PERF[perfil]); modo = VEL; refObj = 0; u = 0; }
      else { ref = refPerfil(t); u = ref > 0 ? pi(ref, dt) : 0; }
      break;
    }
    case CAL: case CERO: ref = VEL_CAL; u = pasoCal(dt); break;
    case TRAY: {
      float t = (ahora - tTr) / 1000.0f, ta = trT / 4, vm = trD / (trT - ta), ac = vm / ta, sp, vp;
      if (t >= trT) { modo = POS; objDeg = tr0 + trD; moviendo = true; Serial.println("# PERFIL_FIN,tray,completo"); u = 0; break; }
      if (t < ta)            { sp = ac * t * t / 2;                 vp = ac * t; }
      else if (t < trT - ta) { sp = ac * ta * ta / 2 + vm * (t - ta); vp = vm; }
      else                   { float r = trT - t; sp = trD - ac * r * r / 2; vp = ac * r; }
      objDeg = tr0 + sp;
      vCmd = vp / 6 + KP_POS * (objDeg - grados());
      int sg = (vCmd > 0) - (vCmd < 0); if (sg != sgAnt) integ = 0; sgAnt = sg;
      u = sg ? sg * pi(fabsf(vCmd), dt) : 0; ref = vCmd; break;
    }
    case POS: {
      float err = objDeg - grados(), vDes = 0;
      if (moviendo) {
        if (fabsf(err) < TOL) { moviendo = false; Serial.printf("# Llego: %.1f grados\n", grados()); }
        else { vDes = constrain(err, -VEL_POS, VEL_POS); if (fabsf(vDes) < 4) vDes = err > 0 ? 4 : -4; }
        errMin = min(errMin, fabsf(err));
        if (fabsf(err) > errMin + 45) { moviendo = false; vDes = 0; Serial.println("# PARADO: el encoder cuenta al reves (intercambia C1 y C2)"); }
      } else { errMin = 1e9; if (fabsf(err) > 3) moviendo = true; }
      if (!moviendo) vCmd = 0;
      else if (vDes * vCmd >= 0 && fabsf(vDes) < fabsf(vCmd)) vCmd = vDes;
      else vCmd = vCmd < vDes ? min(vCmd + RAMPA * dt, vDes) : max(vCmd - RAMPA * dt, vDes);
      int sg = (vCmd > 0) - (vCmd < 0); if (sg != sgAnt) integ = 0; sgAnt = sg;
      u = sg ? sg * pi(fabsf(vCmd), dt) : 0; ref = vCmd; break;
    }
  }
  acel = (ref - refAnt) / dt; refAnt = ref;
  // proteccion corriente y bateria
  if (inaOk) {
    tI = fabsf(mA) > I_LIM ? (tI ? tI : ahora) : 0;
    tV = (V_BAJO > 0 && voltios > 3 && voltios < V_BAJO && u != 0) ? (tV ? tV : ahora) : 0;
    if ((tI && ahora - tI > 500) || (tV && ahora - tV > 2000)) {
      Serial.printf("# MOTOR PARADO: %s (%.2f V, %.0f mA)\n", tI ? "sobrecorriente" : "bateria baja", voltios, mA);
      parar(); ref = 0; u = 0; tI = tV = 0;
    }
  }
  if (modo == POS && !moviendo) { pwm(IN3, PWM_MAX); pwm(IN4, PWM_MAX); } else motor(u);  // freno
  int per = periodoTx ? ((modo == PERFIL || modo >= TRAY) ? 20 : periodoTx) : 0;
  if (per && ahora - tTx >= (uint32_t)per) {
    tTx = ahora;
    if (formato == 2) Serial.printf(">ref:%.1f\n>rpm:%.1f\n>obj:%.1f\n>pos:%.1f\n>pwm:%.0f\n>mA:%.0f\n>V:%.2f\n>sensor:%d\n>pasos:%lu\n",
                                    ref, rpm, objDeg, grados(), u, mA, voltios, sVisto, (unsigned long)nPasos);
    else Serial.printf("D,%lu,%d,%.2f,%.2f,%.2f,%.2f,%.0f,%.1f,%.3f,%d,%.2f\n", (unsigned long)ahora, modo == TRAY ? 1 : min((int)modo, 3),
                  ref, rpm, objDeg, grados(), u, mA, voltios, sVisto, acel);
    sVisto = false;
  }
  if (ahora - tLcd >= 250) { tLcd = ahora; char l1[17], l2[17];
    if (modo == POS || modo == TRAY) snprintf(l1, 17, "P%7.1f>%7.1f", objDeg, grados());
    else if (modo >= CAL)  snprintf(l1, 17, "CALIBRANDO  m%d", nMarcas);
    else                   snprintf(l1, 17, "%-7s%4.0f>%4.0f", modo == PERFIL ? PERF[perfil] : "VEL", ref, rpm);
    snprintf(l2, 17, "%5.2fV %5.0fmA %c", voltios, mA, sEst == LOW ? '*' : ' ');
    lcdLinea(0, l1); lcdLinea(1, l2);
  }
  if (monitorOn && ahora - tMon >= 300) { tMon = ahora;
    Serial.printf("# RPM %6.1f | Pos %8.1f (obj %7.1f) | Sensor %-7s %4lu pasos | %5.2f V | %5.0f mA | %4.1f W | PWM %5.0f\n",
                  rpm, grados(), objDeg, sEst == LOW ? "DETECTA" : "libre", (unsigned long)nPasos, voltios, mA, voltios * mA / 1000, u);
  }
}