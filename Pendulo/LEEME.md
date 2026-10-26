# Péndulo PID – ESP32

Motor GA25-370 con encoder · L298N · INA219 · TCRT5000 · LCD 16x2 I2C

## Contenido de la carpeta

| Carpeta / archivo | Qué es |
|---|---|
| `src/main.cpp` | Programa del péndulo (se sube con **env:pendulo**) |
| `pruebas/pruebas.cpp` | Programa para probar cada componente (se sube con **env:pruebas**) |
| `herramientas/graficar.py` | Gráficas en Python que guardan CSV y PNG para el reporte |
| `conexiones/` | Imágenes de todas las conexiones |
| `Manual_Pendulo_PID.pdf` | Manual con imágenes |
| `platformio.ini` | Configuración del proyecto (puerto COM3) |

## Instalar y subir

1. Instala **VS Code**, la extensión **PlatformIO IDE** y **Python 3**.
2. **File → Open Folder** y elige esta carpeta (la que tiene `platformio.ini`).
3. En la barra azul de abajo elige **env:pendulo** y presiona **→** (Upload).
4. Si tu ESP32 no está en COM3, cambia `upload_port` y `monitor_port` en `platformio.ini`.
5. Abre el monitor (el enchufe). Escribe un comando y presiona **Enter**.

> Usa los botones de PlatformIO (✓ y →), no el rayo de ESP-IDF.

## Pines

| Componente | Pin | ESP32 |
|---|---|---|
| Encoder | C1 / C2 | D23 / D18 |
| L298N | IN3 / IN4 (jumper ENB puesto) | D26 / D27 |
| TCRT5000 | D0 | D19 |
| INA219 y LCD | SDA / SCL | D21 / D22 |
| Botón BOOT | – | calibrar |

---

## Todos los comandos

Se escriben en el monitor o en la cajita de Teleplot (con fin de línea `\n`). Da igual si usas mayúsculas o minúsculas.

### Moverlo

| Comando | Qué hace |
|---|---|
| `v` | Modo velocidad |
| `60` | En velocidad: gira a 60 RPM (0 a 130) |
| `p` | Modo posición |
| `90` | En posición: gira 90° desde donde está (negativo = al revés) |
| `mv 360 2` | Trayectoria: gira 360° en 2 s con aceleración, avance y frenado suaves (doble lazo). Sin tiempo (`mv 360`) lo calcula solo |
| `s` | Parar |

### Calibrar y cero

| Comando | Qué hace |
|---|---|
| `c` o botón BOOT | Calibra: da 2 vueltas con el sensor, mide el CPR, lo guarda y regresa a donde estaba (ese punto es el 0). Pon antes el brazo en tu 0 |
| `r` o `z` | El punto actual pasa a ser 0° |
| `cero` | Regresa al 0 usando el sensor y el offset |
| `of` | Aprende el offset del sensor: pon el brazo en tu 0, `r`, luego `p` y `360`, y al final `of` |
| `of 10` | Pone el offset a mano, en grados |

### Perfiles de velocidad

Formato: `pf tipo A T extra`

- **A** = RPM máximas (0 a 130)
- **T** = duración en segundos
- **extra** = ajuste de cada perfil (opcional; 0 = automático)

| Perfil | Ejemplo | Qué es el extra |
|---|---|---|
| Constante | `pf const 80 10` | no usa |
| Rampa | `pf rampa 120 15` | no usa |
| Trapezoidal | `pf trap 100 20 4` | tiempo de subida y bajada (s) |
| Senoidal | `pf seno 100 20 0.1` | frecuencia (Hz); más chico = más lento |
| Escalonada | `pf escalon 120 20 5` | número de escalones |
| Exponencial | `pf exp 100 20 3` | τ: qué tan rápido sube (s) |
| Gaussiano | `pf gauss 110 20 3` | σ: ancho de la campana (s) |
| Diente de sierra | `pf sierra 100 20 5` | duración de cada diente (s); más grande = más lento |
| Ruido blanco | `pf ruido 70 20 10` | cuánto varía (± RPM) |

| Comando | Qué hace |
|---|---|
| `pf sierra 100 20 5, trap 100 20 4, seno 100 20 0.1` | **Perfiles en cadena**: al terminar uno empieza el siguiente (hasta 8, separados por coma) |
| `rep` | Repite el perfil o la cadena sin parar hasta `s`. `rep` otra vez lo quita |
| `pa 100` | Cambia A |
| `pt 20` | Cambia T |
| `px 4` | Cambia el extra |
| `pf trap` | Corre el perfil con los últimos valores |

### Sensor de corriente (INA219)

| Comando | Qué hace |
|---|---|
| `ci` | **Cero de corriente**: para el motor, mide la corriente en reposo y la guarda. Después `mA` muestra solo lo que consume el motor |
| `ci 350` | **Ajuste de escala**: con el motor girando, escribe lo que marca el multímetro en serie (en mA) para que el INA219 marque igual |

### Ajustes del control

| Comando | Qué cambia | Inicial |
|---|---|---|
| `ra 40` | Rampa en RPM por segundo (más alto = más brusco) | 40 |
| `vp 30` | Velocidad máxima en modo posición (RPM) | 30 |
| `pm 250` | PWM mínimo para que arranque el motor (0 a 1023) | 250 |
| `kp 2` | Ganancia proporcional del PI | 2 |
| `ki 15` | Ganancia integral del PI | 15 |
| `il 2500` | Límite de corriente (mA); si lo pasa 0.5 s, para el motor | 2500 |
| `vb 9.6` | Voltaje mínimo de batería (`vb 0` lo desactiva; úsalo con el eliminador) | 9.6 |

### Ver datos

| Comando | Qué hace |
|---|---|
| `fm 2` | Datos en formato Teleplot |
| `fm 0` | Datos en formato graficar.py |
| `tx 50` | Manda datos cada 50 ms (`tx 0` = apagado, terminal quieta) |
| `m` | Monitor en texto: RPM, posición, sensor, pasos, RPM del sensor, V, mA, W y PWM (`m` otra vez lo apaga) |
| `g` | Estado: modo, CPR, offset, sensores, calibración de corriente y todos los ajustes |
| `h` | Ayuda corta |

**Se guardan aunque apagues el ESP32:** CPR, offset, `tx`, `fm` y la calibración de corriente (`ci`). Lo demás regresa a su valor inicial.

---

## Gráficas en Teleplot

1. En el monitor: `fm 2` y `tx 50`.
2. Cierra el monitor (solo un programa puede usar el COM a la vez).
3. Teleplot → COM3 → 115200 → **Open**.
4. Para escribir comandos usa la cajita de abajo a la derecha, con fin de línea **`\n`**.

| Gráfica | Qué es |
|---|---|
| `ref` | Velocidad que se pide (RPM) |
| `rpm` | Velocidad medida con el encoder |
| `rpmS` | Velocidad medida con el sensor óptico (tiempo entre pasadas) |
| `obj` | Objetivo de posición. **Empieza en 0 en cada movimiento** |
| `pos` | Posición real. **Empieza en 0 en cada movimiento** |
| `pwm` | PWM que se manda al motor |
| `mA` / `V` | Corriente (ya sin el cero, si hiciste `ci`) y voltaje |
| `sensor` | 1 cuando el brazo pasa frente al TCRT |
| `pasos` | Cuántas veces ha pasado el brazo por el sensor |

Comparaciones útiles: arrastra `ref` sobre `rpm`, `obj` sobre `pos`, y `rpmS` sobre `rpm`.

## Gráficas con Python (para el reporte)

```
python -m pip install -r herramientas/requirements.txt
python herramientas/graficar.py --puerto COM3
```

Cada perfil o `mv` se guarda solo en `datos/` como .csv y .png (RMSE, error máximo, corriente, energía).

| En la terminal de graficar.py | Qué hace |
|---|---|
| `:grabar nombre` / `:parar` | Graba aunque no sea un perfil |
| `:salir` | Cierra |
| `--archivo datos/x.csv` | Vuelve a graficar una prueba guardada |
| `--simular` | Practicar sin el ESP32 |

## Programa de pruebas (env:pruebas)

Elige **env:pruebas**, sube y abre el monitor. Escribe el número y Enter; `x` sale de una prueba continua.

| # | Prueba |
|---|---|
| 1 | Buscar LCD e INA219 en I2C |
| 2 | Probar la LCD |
| 3 | Encoder (gira el eje a mano) |
| 4 | Sensor TCRT5000 |
| 5 | INA219: voltaje y corriente |
| 6 / 7 | Motor adelante / atrás 3 s |
| 8 | Motor + encoder + INA219 juntos |

Para regresar al péndulo elige **env:pendulo** y sube otra vez.

## Pantalla LCD

- **Renglón 1:**
  - Posición: `P objetivo>posición`
  - Velocidad o perfil: `nombre referencia>rpm`
  - Calibrando: `CALIBRANDO m#`
- **Renglón 2:** voltaje, corriente y `*` si el brazo pasó por el sensor.
