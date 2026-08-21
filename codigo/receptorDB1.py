import socket
import time
import math
import csv
import os
import msvcrt

# =========================================================
# CONFIGURACIÓN GENERAL Y CONEXIÓN
# =========================================================

UDP_IP = "0.0.0.0"
UDP_PORT = 5000

CSV_FILE = "ensayos_imu.csv"

ACCEL_SCALE = 0.000061  # g/LSB
GYRO_SCALE = 0.00875    # °/s/LSB

CALIBRATION_TIME = 1.0
PRINT_INTERVAL = 0.15
ALPHA = 0.98

# =========================================================
# GESTIÓN DEL CSV Y PERSISTENCIA DE TRIAL_ID
# =========================================================

HEADERS = [
    "user_id",
    "trial_id",
    "sample_id",
    "a_x", "a_y", "a_z",
    "w_x", "w_y", "w_z",
    "m_x", "m_y", "m_z",
    "Direccion",
    "velocidad_objetivo"
]

def obtener_ultimo_trial_id(filepath):
    """Busca el ultimo trial_id guardado en el archivo CSV."""
    if not os.path.exists(filepath):
        return 0
    
    ultimo_id = 0
    try:
        with open(filepath, mode='r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if "trial_id" in row and row["trial_id"].isdigit():
                    ultimo_id = max(ultimo_id, int(row["trial_id"]))
    except Exception:
        pass
    
    return ultimo_id

# Si no existe el CSV, lo crea con encabezados
if not os.path.exists(CSV_FILE):
    with open(CSV_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(HEADERS)

# Cargar el ultimo trial_id registrado para no sobrescribir la secuencia
trial_id = obtener_ultimo_trial_id(CSV_FILE) + 1

# =========================================================
# VARIABLES GLOBALES DE ESTADO
# =========================================================

user_id = 0
sample_counter = 0
direccion_actual = "Neutra"
velocidad_objetivo = 0.5

grabando_ensayo = False

angle_y = 0.0
angle_z = 0.0
gy_offset = 0.0
gz_offset = 0.0

estado = "MOSTRANDO"
datos_calibracion = []
tiempo_inicio_cal = None
ultimo_timestamp = None

def procesar_paquete(mensaje):
    valores = mensaje.split(",")
    if len(valores) != 10:
        return None

    try:
        timestamp = int(valores[0])
        ax_raw, ay_raw, az_raw = int(valores[1]), int(valores[2]), int(valores[3])
        gx_raw, gy_raw, gz_raw = int(valores[4]), int(valores[5]), int(valores[6])
    except ValueError:
        return None

    return {
        "timestamp": timestamp,
        "ax": ax_raw * ACCEL_SCALE,
        "ay": ay_raw * ACCEL_SCALE,
        "az": az_raw * ACCEL_SCALE,
        "gx": gx_raw * GYRO_SCALE,
        "gy": gy_raw * GYRO_SCALE,
        "gz": gz_raw * GYRO_SCALE
    }

def guardar_muestra_csv(muestra):
    global sample_counter
    sample_counter += 1
    
    mx, my, mz = 0.0, 0.0, 0.0

    wx = math.radians(muestra["gx"])
    wy = math.radians(muestra["gy"])
    wz = math.radians(muestra["gz"])

    fila = [
        user_id,
        trial_id,
        sample_counter,
        f"{muestra['ax']:.5f}", f"{muestra['ay']:.5f}", f"{muestra['az']:.5f}",
        f"{wx:.5f}", f"{wy:.5f}", f"{wz:.5f}",
        f"{mx:.2f}", f"{my:.2f}", f"{mz:.2f}",
        direccion_actual,
        f"{velocidad_objetivo:.2f}"
    ]

    with open(CSV_FILE, mode='a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(fila)

def verificar_teclas():
    global user_id, trial_id, sample_counter, grabando_ensayo
    global direccion_actual, velocidad_objetivo, estado, datos_calibracion, tiempo_inicio_cal

    if msvcrt.kbhit():
        tecla = msvcrt.getch().decode('utf-8', errors='ignore').lower()

        if tecla == 'q':
            return False

        elif tecla == 'u':
            user_id = 1 if user_id == 0 else 0

        elif tecla == 'r':
            if not grabando_ensayo:
                grabando_ensayo = True
                sample_counter = 0
            else:
                grabando_ensayo = False
                trial_id += 1  # Siguiente ID para el proximo ensayo

        elif tecla in ['1', '2', '3', '4', '5']:
            direcciones = {'1': 'Neutra', '2': 'adelante', '3': 'atrás', '4': 'derecha', '5': 'izquierda'}
            direccion_actual = direcciones[tecla]

        elif tecla in ['+', '=']:
            velocidad_objetivo = min(1.0, round(velocidad_objetivo + 0.1, 1))

        elif tecla in ['-', '_']:
            velocidad_objetivo = max(0.0, round(velocidad_objetivo - 0.1, 1))

        elif tecla == 'n' and estado == "MOSTRANDO":
            datos_calibracion = []
            estado = "CALIBRANDO"
            tiempo_inicio_cal = time.time()

    return True

# =========================================================
# BUCLE PRINCIPAL UDP
# =========================================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.05)

ultimo_print = 0

while True:
    if not verificar_teclas():
        break

    if estado == "CALIBRANDO":
        if time.time() - tiempo_inicio_cal >= CALIBRATION_TIME:
            if len(datos_calibracion) > 0:
                gy_offset = sum(d["gy"] for d in datos_calibracion) / len(datos_calibracion)
                gz_offset = sum(d["gz"] for d in datos_calibracion) / len(datos_calibracion)
                
                primera = datos_calibracion[-1]
                angle_y = math.degrees(math.atan2(primera["ax"], math.sqrt(primera["ay"]**2 + primera["az"]**2)))
                angle_z = math.degrees(math.atan2(primera["az"], math.sqrt(primera["ax"]**2 + primera["ay"]**2)))
                estado = "MOSTRANDO"
            else:
                estado = "MOSTRANDO"

    try:
        data, addr = sock.recvfrom(1024)
        mensaje = data.decode().strip()

        if mensaje.startswith("STATUS") or mensaje.startswith("ERROR"):
            continue

        muestra = procesar_paquete(mensaje)
        if muestra is None:
            continue

        if estado == "CALIBRANDO":
            datos_calibracion.append(muestra)
            continue

        if estado == "MOSTRANDO":
            dt = 0.01 if ultimo_timestamp is None else (muestra["timestamp"] - ultimo_timestamp) / 1000.0
            if dt <= 0 or dt > 0.5: dt = 0.01
            ultimo_timestamp = muestra["timestamp"]

            acc_angle_y = math.degrees(math.atan2(muestra["ax"], math.sqrt(muestra["ay"]**2 + muestra["az"]**2)))
            acc_angle_z = math.degrees(math.atan2(muestra["az"], math.sqrt(muestra["ax"]**2 + muestra["ay"]**2)))

            angle_y = ALPHA * (angle_y + (muestra["gy"] - gy_offset) * dt) + (1.0 - ALPHA) * acc_angle_y
            angle_z = ALPHA * (angle_z + (muestra["gz"] - gz_offset) * dt) + (1.0 - ALPHA) * acc_angle_z

            if grabando_ensayo:
                guardar_muestra_csv(muestra)

            ahora = time.time()
            if ahora - ultimo_print >= PRINT_INTERVAL:
                print("\033[H\033[J", end="")
                print("==================================================")
                print("         PANEL DE CONTROL DE ENSAYOS CSV         ")
                print("==================================================")
                print(f" USUARIO ACTUAL      : {user_id}")
                print(f" ID ENSAYO (Trial)   : {trial_id}")
                print(f" N° MUESTRAS (Sample): {sample_counter}")
                print(f" ESTADO GRABACIÓN    : {'[ GRABANDO... ]' if grabando_ensayo else '[ DETENIDO ]'}")
                print("--------------------------------------------------")
                print(f" DIRECCIÓN           : {direccion_actual}")
                print(f" VELOCIDAD OBJETIVO  : {velocidad_objetivo:.1f}")
                print("--------------------------------------------------")
                print(f" ÁNGULO Y: {angle_y:+6.1f}° | ÁNGULO Z: {angle_z:+6.1f}°")
                print("==================================================")
                print(" CONTROLES DE TECLADO DIRECTO (Sin ENTER):")
                print("  R       = Iniciar / Detener Grabación")
                print("  U       = Alternar Usuario (0 / 1)")
                print("  1 a 5   = Dirección (1:Neut, 2:Adel, 3:Atr, 4:Der, 5:Izq)")
                print("  + / -   = Subir / Bajar Velocidad")
                print("  N       = Calibrar Giros en Reposo")
                print("  Q       = Salir")
                ultimo_print = ahora

    except socket.timeout:
        pass
    except KeyboardInterrupt:
        break

sock.close()
print("\nPrograma finalizado. Salida en:", CSV_FILE)