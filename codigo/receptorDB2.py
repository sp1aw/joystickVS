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

TIEMPO_REPOSO_N = 2.0  # Duración en segundos para captura en Neutro

# =========================================================
# ÁNGULOS MÁXIMOS OBTENIDOS EN EL ANÁLISIS PREVIO
# =========================================================
ANGULOS_MAXIMOS = {
    'adelante': 25.0,    # Pitch (Y)
    'atrás': 25.0,       # Pitch (Y)
    'izquierda': 30.0,   # Yaw (Z)
    'derecha': 30.0,     # Yaw (Z)
    'N': 0.0             # Neutro sin movimiento
}

# Secuencia global: 1 Neutro al inicio + 16 variaciones continuas
DIRECCIONES = ['adelante', 'atrás', 'izquierda', 'derecha']
VELOCIDADES = [0.25, 0.5, 0.75, 1.0]

SECUENCIA_ENSAYOS = [('N', 0.0)]  # Ensayo neutro único al inicio del ciclo global

for d in DIRECCIONES:
    for v in VELOCIDADES:
        SECUENCIA_ENSAYOS.append((d, v))  # 16 ensayos restantes del ciclo

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
    if not os.path.exists(filepath):
        return 0
    
    ultimo_id = 0
    try:
        with open(filepath, mode='r', encoding='latin1', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if "trial_id" in row and row["trial_id"].isdigit():
                    ultimo_id = max(ultimo_id, int(row["trial_id"]))
    except Exception:
        pass
    
    return ultimo_id

def eliminar_ultimo_trial_csv(filepath):
    if not os.path.exists(filepath):
        return 0

    filas = []
    ultimo_id = obtener_ultimo_trial_id(filepath)

    if ultimo_id == 0:
        return 0

    try:
        with open(filepath, mode='r', encoding='latin1', newline='') as f:
            reader = list(csv.reader(f))
            if not reader:
                return 0
            
            headers = reader[0]
            filas.append(headers)
            for row in reader[1:]:
                if len(row) > 1 and row[1].isdigit():
                    if int(row[1]) != ultimo_id:
                        filas.append(row)

        with open(filepath, mode='w', encoding='latin1', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(filas)

        return ultimo_id
    except Exception as e:
        print(f"Error al borrar ultimo ensayo: {e}")
        return 0

if not os.path.exists(CSV_FILE):
    with open(CSV_FILE, mode='w', encoding='latin1', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(HEADERS)

trial_id = obtener_ultimo_trial_id(CSV_FILE) + 1

# =========================================================
# VARIABLES GLOBALES DE ESTADO
# =========================================================

user_id = 0
sample_counter = 0
indice_secuencia = 0

grabando_ensayo = False

angle_y = 0.0
angle_z = 0.0
angle_y_inicio = 0.0
angle_z_inicio = 0.0

gy_offset = 0.0
gz_offset = 0.0

estado = "MOSTRANDO"
datos_calibracion = []
tiempo_inicio_cal = None
tiempo_inicio_grab = None
ultimo_timestamp = None
mensaje_alerta = ""

def actualizar_objetivo_actual():
    global direccion_actual, velocidad_objetivo, angulo_objetivo
    direccion_actual, velocidad_objetivo = SECUENCIA_ENSAYOS[indice_secuencia % len(SECUENCIA_ENSAYOS)]
    max_ang = ANGULOS_MAXIMOS.get(direccion_actual, 0.0)
    angulo_objetivo = max_ang * velocidad_objetivo

actualizar_objetivo_actual()

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

    with open(CSV_FILE, mode='a', encoding='latin1', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(fila)

def iniciar_grabacion():
    global grabando_ensayo, sample_counter, angle_y_inicio, angle_z_inicio, mensaje_alerta, tiempo_inicio_grab
    grabando_ensayo = True
    sample_counter = 0
    angle_y_inicio = angle_y
    angle_z_inicio = angle_z
    tiempo_inicio_grab = time.time()
    mensaje_alerta = ""

def detener_y_avanzar():
    global grabando_ensayo, trial_id, indice_secuencia
    grabando_ensayo = False
    trial_id += 1
    indice_secuencia += 1
    actualizar_objetivo_actual()

def verificar_teclas():
    global user_id, trial_id, estado, datos_calibracion, tiempo_inicio_cal, indice_secuencia, mensaje_alerta

    if msvcrt.kbhit():
        tecla = msvcrt.getch().decode('utf-8', errors='ignore').lower()

        if tecla == 'q':
            return False

        elif tecla == 'u':
            user_id = 1 if user_id == 0 else 0

        elif tecla == 'r':
            if not grabando_ensayo:
                iniciar_grabacion()
            else:
                detener_y_avanzar()

        elif tecla == 'd':
            if not grabando_ensayo:
                id_eliminado = eliminar_ultimo_trial_csv(CSV_FILE)
                if id_eliminado > 0:
                    trial_id = id_eliminado
                    if indice_secuencia > 0:
                        indice_secuencia -= 1
                    actualizar_objetivo_actual()
                    mensaje_alerta = f"[ ENSAYO {id_eliminado} ELIMINADO ]"

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

                # EVALUACIÓN DE CRITERIO DE PARADA
                if direccion_actual == 'N':
                    if time.time() - tiempo_inicio_grab >= TIEMPO_REPOSO_N:
                        mensaje_alerta = "[ ENSAYO NEUTRO COMPLETADO ]"
                        detener_y_avanzar()
                else:
                    delta_y = abs(angle_y - angle_y_inicio)
                    delta_z = abs(angle_z - angle_z_inicio)
                    desplazamiento_actual = delta_z if direccion_actual in ['derecha', 'izquierda'] else delta_y

                    if desplazamiento_actual >= angulo_objetivo:
                        mensaje_alerta = f"[ ¡OBJETIVO ALCANZADO! ({desplazamiento_actual:.1f}°) ]"
                        detener_y_avanzar()

            ahora = time.time()
            if ahora - ultimo_print >= PRINT_INTERVAL:
                num_ensayo_ciclo = (indice_secuencia % len(SECUENCIA_ENSAYOS)) + 1
                print("\033[H\033[J", end="")
                print("==================================================")
                print("    PANEL DE CONTROL (CICLO GLOBAL 17 ENSAYOS)   ")
                print("==================================================")
                print(f" USUARIO ACTUAL       : {user_id}")
                print(f" ID ENSAYO (Trial)    : {trial_id}")
                print(f" PASO CICLO ACTUAL    : {num_ensayo_ciclo} / 17")
                print(f" N° MUESTRAS (Sample) : {sample_counter}")
                print(f" ESTADO GRABACIÓN     : {'[ GRABANDO... ]' if grabando_ensayo else '[ DETENIDO ]'}")
                if mensaje_alerta:
                    print(f" ALERTA               : {mensaje_alerta}")
                print("--------------------------------------------------")
                print(f" MOVIMIENTO PEDIDO    : {direccion_actual.upper()}")
                print(f" VELOCIDAD OBJETIVO   : {velocidad_objetivo:.2f}")
                print(f" ÁNGULO MÁX OBJETIVO  : {angulo_objetivo:.1f}°")
                print("--------------------------------------------------")
                print(f" ÁNGULO Y (Pitch)     : {angle_y:+6.1f}°")
                print(f" ÁNGULO Z (Yaw)       : {angle_z:+6.1f}°")
                print("==================================================")
                print(" CONTROLES DE TECLADO:")
                print("  R       = Iniciar / Cancelar Grabación")
                print("  D       = BORRAR ÚLTIMO ENSAYO (Deshacer)")
                print("  U       = Alternar Usuario (0 / 1)")
                print("  N       = Calibrar Giros en Reposo")
                print("  Q       = Salir")
                ultimo_print = ahora

    except socket.timeout:
        pass
    except KeyboardInterrupt:
        break

sock.close()
print("\nPrograma finalizado. Salida en:", CSV_FILE)