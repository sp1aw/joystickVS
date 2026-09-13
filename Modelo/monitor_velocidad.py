import socket
import time

# =========================================================
# CONFIGURACION
# =========================================================

UDP_IP = "0.0.0.0"
UDP_PORT = 5000

# =========================================================
# SOCKET UDP
# =========================================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.5)

print("==================================================")
print("     MONITOR DE VELOCIDAD - JOYSTICK INTELIGENTE   ")
print("==================================================")
print(" Esperando datos del ESP32 ...")
print(" (Conectate primero a la red WiFi 'Guante_IMU')")
print("==================================================")

ultima_velocidad = 0.0
ultimo_angulo = 0.0
ultimo_wx = 0.0
ultimo_wy = 0.0
conectado = False

while True:
    try:
        data, addr = sock.recvfrom(1024)
        mensaje = data.decode().strip()

        # Mensajes de estado / error del ESP32 (arranque, sensores, etc.)
        if mensaje.startswith("STATUS") or mensaje.startswith("ERROR"):
            print(f"[{mensaje}]")
            continue

        # Mensaje de velocidad: "VEL,<velocidad>,<angulo_abs>,<w_x>,<w_y>"
        if mensaje.startswith("VEL,"):
            partes = mensaje.split(",")
            if len(partes) >= 5:
                ultima_velocidad = float(partes[1])
                ultimo_angulo = float(partes[2])
                ultimo_wx = float(partes[3])
                ultimo_wy = float(partes[4])
                conectado = True

                # Barra visual simple para representar la velocidad (0 a 1)
                barra_llena = int(ultima_velocidad * 30)
                barra = "#" * barra_llena + "-" * (30 - barra_llena)

                print("\033[H\033[J", end="")  # limpiar pantalla
                print("==================================================")
                print("     MONITOR DE VELOCIDAD - JOYSTICK INTELIGENTE   ")
                print("==================================================")
                print(f" Angulo de inclinacion : {ultimo_angulo:6.2f} grados")
                print(f" Giroscopio w_x         : {ultimo_wx:6.2f}")
                print(f" Giroscopio w_y         : {ultimo_wy:6.2f}")
                print("--------------------------------------------------")
                print(f" VELOCIDAD CALCULADA   : {ultima_velocidad:.3f}")
                print(f" [{barra}]")
                print("==================================================")

    except socket.timeout:
        if not conectado:
            print(".", end="", flush=True)
        continue
    except KeyboardInterrupt:
        break

sock.close()
print("\nMonitor finalizado.")
