#!/usr/bin/env python3
"""
Gráficas del Péndulo PID (ESP32) para VS Code
=============================================
- Muestra en vivo: velocidad (ref vs medida), posición, PWM y corriente.
- Tú escribes los comandos aquí mismo (los manda al ESP32): 60, p, 90, pf trap 100 10, autocal...
- Cada perfil (pf ...) se GUARDA solo en la carpeta datos/: un .csv y una imagen .png lista para el reporte.

Uso (en la terminal de VS Code, dentro de la carpeta del proyecto):
    python herramientas/graficar.py                 -> busca el ESP32 solo
    python herramientas/graficar.py --puerto COM5   -> si tienes varios puertos
    python herramientas/graficar.py --simular       -> prueba sin el ESP32 (motor simulado)
    python herramientas/graficar.py --archivo datos/trap_....csv   -> vuelve a graficar una prueba guardada

Comandos locales (empiezan con dos puntos):
    :grabar nombre   empieza a guardar lo que llegue (sin perfil)
    :parar           termina y guarda esa grabación
    :salir           cierra el programa
IMPORTANTE: cierra el Monitor Serie de PlatformIO antes de abrir este programa (no pueden usar el puerto a la vez).
"""
import argparse
import csv
import math
import os
import queue
import random
import sys
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np

COLUMNAS = ["t_ms", "modo", "ref_rpm", "rpm", "obj_deg", "pos_deg", "pwm", "mA", "V", "sensor", "acel_ref"]
MODOS = {0: "velocidad", 1: "posicion", 2: "perfil", 3: "calibracion"}


# ----------------------------------------------------------------------------- conexión
def buscar_puerto():
    from serial.tools import list_ports
    puertos = list(list_ports.comports())
    if not puertos:
        return None
    for p in puertos:
        d = f"{p.description} {p.manufacturer} {p.hwid}".lower()
        if any(k in d for k in ("cp210", "ch340", "ch910", "usb serial", "silicon labs", "uart", "esp32")):
            return p.device
    return puertos[0].device


class Simulador:
    """Motor simulado que contesta como el ESP32 (para probar sin hardware)."""

    def __init__(self):
        self.q = queue.Queue()
        self.rpm = 0.0
        self.pos = 0.0
        self.ref = 0.0
        self.perfil = None
        self.t0 = time.time()
        self.vivo = True
        threading.Thread(target=self._ciclo, daemon=True).start()
        self.q.put("# ===== PENDULO PID (SIMULADO) =====\n")

    def write(self, data):
        cmd = data.decode().strip().lower()
        if cmd.startswith("pf "):
            p = cmd.split()
            tipo = p[1]
            A = float(p[2]) if len(p) > 2 else 100
            T = float(p[3]) if len(p) > 3 else 10
            x = float(p[4]) if len(p) > 4 else 0
            self.perfil = (tipo, A, T, x, time.time())
            self.q.put(f"# PERFIL_INICIO,{tipo},{A:.1f},{T:.1f},{x:.2f}\n")
        elif cmd in ("s", "0"):
            self.ref = 0
            if self.perfil:
                self.q.put(f"# PERFIL_FIN,{self.perfil[0]},s\n")
                self.perfil = None
        else:
            try:
                self.ref = max(0, min(130, float(cmd)))
                self.q.put(f"# Objetivo: {self.ref:.0f} RPM\n")
            except ValueError:
                self.q.put(f"# (simulador) comando recibido: {cmd}\n")

    def readline(self):
        try:
            return self.q.get(timeout=0.5).encode()
        except queue.Empty:
            return b""

    def close(self):
        self.vivo = False

    @staticmethod
    def ref_perfil(tipo, A, T, x, t):
        if tipo == "const": r = A
        elif tipo == "rampa": r = A * t / T
        elif tipo == "trap":
            ta = x if x > 0 else T / 4
            r = A * t / ta if t < ta else (A if t < T - ta else A * (T - t) / ta)
        elif tipo == "seno": r = A / 2 * (1 - math.cos(2 * math.pi * (x if x > 0 else 0.2) * t))
        elif tipo == "escalon":
            n = int(x) if x >= 1 else 5
            r = A * min(n, int(t / (T / n)) + 1) / n
        elif tipo == "exp": r = A * (1 - math.exp(-t / (x if x > 0 else T / 5)))
        elif tipo == "gauss":
            sg = x if x > 0 else T / 8
            r = A * math.exp(-((t - T / 2) ** 2) / (2 * sg * sg))
        elif tipo == "sierra":
            P = x if x > 0 else T / 4
            r = A * (t % P) / P
        elif tipo == "ruido": r = A + (x if x > 0 else 0.2 * A) * random.gauss(0, 1)
        else: r = 0
        return max(0, min(130, r))

    def _ciclo(self):
        dt = 0.02
        ref_ant = 0
        while self.vivo:
            if self.perfil:
                tipo, A, T, x, ti = self.perfil
                t = time.time() - ti
                if t >= T:
                    self.q.put(f"# PERFIL_FIN,{tipo},completo\n")
                    self.perfil = None
                    self.ref = 0
                else:
                    self.ref = self.ref_perfil(tipo, A, T, x, t)
            tau = 0.15
            self.rpm += (self.ref - self.rpm) * dt / tau + random.gauss(0, 0.4)
            self.rpm = max(0, self.rpm)
            self.pos += self.rpm * 6 * dt
            pwm = 0 if self.ref <= 0 else 250 + self.ref / 130 * 773
            mA = 120 + 6 * self.rpm + 2.5 * abs(self.ref - self.rpm) * 10 + random.gauss(0, 8)
            acel = (self.ref - ref_ant) / dt
            ref_ant = self.ref
            sensor = 1 if (self.pos % 360) < 4 else 0
            modo = 2 if self.perfil else 0
            self.q.put(f"D,{int((time.time() - self.t0) * 1000)},{modo},{self.ref:.2f},{self.rpm:.2f},0,{self.pos:.2f},"
                       f"{pwm:.0f},{mA:.1f},{12 - mA / 2000:.3f},{sensor},{acel:.2f}\n")
            time.sleep(dt)


# ----------------------------------------------------------------------------- guardado y gráficas de reporte
def guardar_prueba(filas, nombre, carpeta, info=""):
    if len(filas) < 5:
        print("# (muy pocos datos, no guardé nada)")
        return None
    os.makedirs(carpeta, exist_ok=True)
    marca = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(carpeta, f"{nombre}_{marca}")
    with open(base + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        if info:
            w.writerow(["# " + info])
        w.writerow(["t_s"] + COLUMNAS[1:])
        t0 = filas[0][0]
        for r in filas:
            w.writerow([f"{(r[0] - t0) / 1000:.3f}"] + list(r[1:]))
    png = graficar_reporte(base + ".csv", mostrar=False)
    print(f"# GUARDADO: {base}.csv  y  {png}")
    return base


def leer_csv(ruta):
    titulo = ""
    datos = []
    with open(ruta) as f:
        for fila in csv.reader(f):
            if not fila:
                continue
            if fila[0].startswith("#"):
                titulo = fila[0][1:].strip()
                continue
            if fila[0] == "t_s":
                continue
            datos.append([float(x) for x in fila])
    return titulo, np.array(datos)


def metricas(d):
    t, ref, rpm, mA, V = d[:, 0], d[:, 2], d[:, 3], d[:, 7], d[:, 8]
    e = ref - rpm
    dt = np.diff(t, prepend=t[0])
    energia = float(np.sum(V * mA / 1000 * dt))
    return {
        "RMSE (rpm)": float(np.sqrt(np.mean(e ** 2))),
        "Error max (rpm)": float(np.max(np.abs(e))),
        "Corriente media (mA)": float(np.mean(mA)),
        "Corriente pico (mA)": float(np.max(mA)),
        "Energia (J)": energia,
    }


def graficar_reporte(ruta_csv, mostrar=True):
    # Sin pyplot cuando solo se guarda: así no estorba a la ventana en vivo (se puede llamar desde otro hilo)
    if mostrar:
        import matplotlib.pyplot as plt
    else:
        from matplotlib.figure import Figure

    titulo, d = leer_csv(ruta_csv)
    t, ref, rpm, pwm, mA, V, acel = d[:, 0], d[:, 2], d[:, 3], d[:, 6], d[:, 7], d[:, 8], d[:, 10]
    # aceleración medida: derivada de la velocidad suavizada
    k = max(1, int(0.1 / max(1e-3, np.median(np.diff(t)))))       # ventana de ~0.1 s
    rpm_s = np.convolve(np.pad(rpm, (k // 2, k - 1 - k // 2), mode="edge"), np.ones(k) / k, mode="valid")
    acel_med = np.gradient(rpm_s, t)
    m = metricas(d)

    if mostrar:
        fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    else:
        fig = Figure(figsize=(11, 11))
        ax = fig.subplots(4, 1, sharex=True)
    fig.suptitle(f"{os.path.basename(ruta_csv)}\n{titulo}", fontsize=11)
    ax[0].plot(t, ref, label="Referencia", lw=2)
    ax[0].plot(t, rpm, label="Medida (encoder)", lw=1.2)
    ax[0].set_ylabel("Velocidad (RPM)"); ax[0].legend(loc="upper right"); ax[0].grid(alpha=.3)
    ax[0].set_title("  |  ".join(f"{k}: {v:.1f}" for k, v in m.items()), fontsize=9)

    ax[1].plot(t, ref - rpm, color="tab:red", lw=1)
    ax[1].axhline(0, color="k", lw=.6)
    ax[1].set_ylabel("Error (RPM)"); ax[1].grid(alpha=.3)

    ax[2].plot(t, acel, label="Aceleración de la referencia", lw=1.5)
    ax[2].plot(t, acel_med, label="Aceleración medida", lw=1, alpha=.8)
    ax[2].set_ylabel("Aceleración (RPM/s)"); ax[2].legend(loc="upper right"); ax[2].grid(alpha=.3)

    ax[3].plot(t, mA, color="tab:purple", label="Corriente")
    ax[3].set_ylabel("Corriente (mA)"); ax[3].grid(alpha=.3)
    ax3b = ax[3].twinx()
    ax3b.plot(t, pwm, color="tab:gray", lw=.8, alpha=.7, label="PWM")
    ax3b.set_ylabel("PWM (0-1023)")
    ax[3].legend(loc="upper left"); ax3b.legend(loc="upper right")
    ax[3].set_xlabel("Tiempo (s)")
    fig.tight_layout()
    png = os.path.splitext(ruta_csv)[0] + ".png"
    fig.savefig(png, dpi=130)
    print("# Metricas: " + "  ".join(f"{k}={v:.2f}" for k, v in m.items()))
    if mostrar:
        plt.show()
        plt.close(fig)
    return png


# ----------------------------------------------------------------------------- programa en vivo
def en_vivo(args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    if args.simular:
        ser = Simulador()
        print("# Modo SIMULADO (sin ESP32)")
    else:
        import serial
        puerto = args.puerto or buscar_puerto()
        if not puerto:
            print("No encontré ningún puerto. Conecta el ESP32 o usa --puerto COM5")
            sys.exit(1)
        print(f"# Abriendo {puerto} a {args.baud} baud...")
        ser = serial.Serial(puerto, args.baud, timeout=0.5)
        time.sleep(1.2)                       # el ESP32 se reinicia al abrir el puerto
        ser.write(b"fm 0\ntx 50\n")           # prende los datos para las gráficas

    N = int(args.ventana * 60)                      # puntos guardados en pantalla
    buf = {c: deque(maxlen=N) for c in COLUMNAS}
    candado = threading.Lock()
    estado = {"grabando": False, "nombre": "", "filas": [], "info": "", "vivo": True, "ultimo": None}

    def empezar(nombre, info=""):
        estado.update(grabando=True, nombre=nombre, filas=[], info=info)
        print(f"# >>> Grabando '{nombre}'")

    def terminar():
        if estado["grabando"]:
            estado["grabando"] = False
            filas = estado["filas"]
            threading.Thread(target=guardar_prueba, args=(filas, estado["nombre"], args.carpeta, estado["info"]),
                             daemon=True).start()

    def lector():
        while estado["vivo"]:
            try:
                linea = ser.readline().decode(errors="ignore").strip()
            except Exception as e:  # puerto desconectado
                print("# Error leyendo el puerto:", e)
                break
            if not linea:
                continue
            if linea.startswith("D,"):
                p = linea.split(",")
                if len(p) != len(COLUMNAS) + 1:
                    continue
                try:
                    v = [float(x) for x in p[1:]]
                except ValueError:
                    continue
                with candado:
                    for c, x in zip(COLUMNAS, v):
                        buf[c].append(x)
                    estado["ultimo"] = v
                    if estado["grabando"]:
                        estado["filas"].append(v)
            else:
                print(linea)
                if linea.startswith("# PERFIL_INICIO"):
                    p = linea.split(",")
                    nombre = p[1] if len(p) > 1 else "perfil"
                    info = f"perfil={nombre} A={p[2]} rpm T={p[3]} s extra={p[4]}" if len(p) > 4 else ""
                    with candado:
                        empezar(nombre, info)
                elif linea.startswith("# PERFIL_FIN"):
                    with candado:
                        terminar()

    def teclado():
        print("# Escribe comandos para el ESP32 (h = ayuda). Locales: :grabar nombre  :parar  :salir")
        while estado["vivo"]:
            try:
                cmd = input()
            except (EOFError, KeyboardInterrupt):
                break
            cmd = cmd.strip()
            if not cmd:
                continue
            if cmd.startswith(":"):
                partes = cmd[1:].split()
                if partes[0] == "grabar":
                    with candado:
                        empezar(partes[1] if len(partes) > 1 else "grabacion", "grabacion manual")
                elif partes[0] == "parar":
                    with candado:
                        terminar()
                elif partes[0] == "salir":
                    estado["vivo"] = False          # la ventana se cierra sola en la siguiente actualización
                continue
            ser.write((cmd + "\n").encode())

    threading.Thread(target=lector, daemon=True).start()
    threading.Thread(target=teclado, daemon=True).start()

    fig, ax = plt.subplots(5, 1, figsize=(10, 11), sharex=True)
    fig.canvas.manager.set_window_title("Pendulo PID - sensores en vivo")
    l_ref, = ax[0].plot([], [], label="referencia", lw=2)
    l_rpm, = ax[0].plot([], [], label="RPM (encoder)")
    ax[0].set_ylabel("RPM"); ax[0].legend(loc="upper left"); ax[0].grid(alpha=.3)
    l_obj, = ax[1].plot([], [], label="objetivo", lw=2)
    l_pos, = ax[1].plot([], [], label="posición")
    ax[1].set_ylabel("Grados"); ax[1].legend(loc="upper left"); ax[1].grid(alpha=.3)
    l_pwm, = ax[2].plot([], [], color="tab:gray", label="PWM")
    ax[2].set_ylabel("PWM"); ax[2].legend(loc="upper left"); ax[2].grid(alpha=.3)
    l_ma, = ax[3].plot([], [], color="tab:purple", label="corriente (INA219)")
    ax[3].set_ylabel("mA"); ax[3].legend(loc="upper left"); ax[3].grid(alpha=.3)
    l_v, = ax[4].plot([], [], color="tab:red", label="voltaje (INA219)")
    ax[4].set_ylabel("Volts"); ax[4].grid(alpha=.3)
    ax4b = ax[4].twinx()
    l_sen, = ax4b.plot([], [], color="tab:green", lw=1.2, drawstyle="steps-post", label="sensor TCRT (1 = detecta)")
    ax4b.set_ylim(-0.1, 1.3); ax4b.set_yticks([0, 1]); ax4b.set_ylabel("Sensor")
    ax[4].legend(loc="upper left"); ax4b.legend(loc="upper right")
    ax[4].set_xlabel("Tiempo (s)")
    txt = fig.text(0.01, 0.995, "", va="top", fontsize=9, family="monospace")

    def actualizar(_):
        if not estado["vivo"]:
            plt.close(fig)
            return []
        with candado:
            if len(buf["t_ms"]) < 2:
                return []
            t = np.array(buf["t_ms"]) / 1000.0
            cols = {c: np.array(buf[c]) for c in COLUMNAS}
            u = estado["ultimo"]
            grab = estado["grabando"]
        for l, y in ((l_ref, cols["ref_rpm"]), (l_rpm, cols["rpm"]), (l_obj, cols["obj_deg"]), (l_pos, cols["pos_deg"]),
                     (l_pwm, cols["pwm"]), (l_ma, cols["mA"]), (l_v, cols["V"]), (l_sen, cols["sensor"])):
            l.set_data(t, y)
        ax[0].set_xlim(max(t[0], t[-1] - args.ventana), t[-1] + 0.05)
        for a in ax:
            a.relim(); a.autoscale_view(scalex=False)
        if u:
            txt.set_text(f"modo: {MODOS.get(int(u[1]), '?'):11s} ref {u[2]:6.1f}  rpm {u[3]:6.1f}  pos {u[5]:7.1f}°  "
                         f"PWM {u[6]:4.0f}  {u[7]:5.0f} mA  {u[8]:5.2f} V  sensor {'DETECTA' if u[9] else 'libre'}"
                         + ("   ● GRABANDO" if grab else ""))
        return []

    anim = FuncAnimation(fig, actualizar, interval=100, cache_frame_data=False)  # noqa: F841
    plt.tight_layout(rect=(0, 0, 1, 0.97))
    try:
        plt.show()
    finally:
        estado["vivo"] = False
        if estado["grabando"]:
            guardar_prueba(estado["filas"], estado["nombre"], args.carpeta, estado["info"])
        ser.close()


def main():
    ap = argparse.ArgumentParser(description="Gráficas en vivo del Péndulo PID")
    ap.add_argument("--puerto", help="COM5, /dev/ttyUSB0... (si no lo pones, lo busca solo)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--ventana", type=float, default=15, help="segundos visibles en la gráfica en vivo")
    ap.add_argument("--carpeta", default="datos", help="dónde se guardan los .csv y .png")
    ap.add_argument("--simular", action="store_true", help="probar sin ESP32")
    ap.add_argument("--archivo", help="graficar un .csv guardado (no abre el puerto)")
    args = ap.parse_args()
    if args.archivo:
        graficar_reporte(args.archivo, mostrar=True)
    else:
        en_vivo(args)


if __name__ == "__main__":
    main()
