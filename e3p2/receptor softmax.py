import socket
import threading
import math
import sys
import pygame

# ==============================================================================
# CONFIGURACIÓN UDP
# ==============================================================================
ESP_IP = "192.168.4.1"  # IP por defecto de la ESP32 en modo AP
UDP_PORT = 5000

# Variable global para la dirección actual
direccion_actual = "N"
running = True

def escuchar_udp():
    """Hilo secundario para recibir la dirección predicha desde la ESP32"""
    global direccion_actual, running

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(1.0)

    # 1. ENVIAR SALUDO INICIAL A LA ESP32 PARA REGISTRAR NUESTRA IP DINÁMICA
    print(f"[*] Enviando paquete de registro a {ESP_IP}:{UDP_PORT}...")
    try:
        sock.sendto(b"CONNECT_PC", (ESP_IP, UDP_PORT))
    except Exception as e:
        print(f"[!] Error al enviar handshake inicial: {e}")

    print(f"[*] Escuchando predicciones en el puerto {UDP_PORT}...")

    while running:
        try:
            data, addr = sock.recvfrom(1024)
            mensaje = data.decode('utf-8').strip()

            # Procesar mensajes del tipo "PRED,adelante" o datos crudos
            if mensaje.startswith("PRED,"):
                direccion_actual = mensaje.split(",")[1]
            elif mensaje.startswith("STATUS,"):
                print(f"[ESP32 STATUS]: {mensaje}")
            else:
                # Si llega directo el nombre de la clase
                direccion_actual = mensaje

        except socket.timeout:
            continue
        except Exception as e:
            if running:
                print(f"[!] Error en recepción UDP: {e}")

    sock.close()


# ==============================================================================
# VISUALIZACIÓN GRÁFICA (PYGAME)
# ==============================================================================
def main():
    global running

    # Iniciar hilo UDP
    hilo_udp = threading.Thread(target=escuchar_udp, daemon=True)
    hilo_udp.start()

    # Inicializar Pygame
    pygame.init()
    ANCHO, ALTO = 600, 600
    screen = pygame.display.set_mode((ANCHO, ALTO))
    pygame.display.set_caption("Visualizador de Control Guante IMU")
    clock = pygame.time.Clock()

    # Colores
    COLOR_BG = (30, 30, 40)
    COLOR_BASE = (60, 64, 72)
    COLOR_JOYSTICK = (0, 200, 255)
    COLOR_TEXTO = (240, 240, 240)
    COLOR_ACTIVO = (50, 255, 126)

    # Coordenadas del centro
    CENTRO_X, CENTRO_Y = ANCHO // 2, ALTO // 2
    RADIO_MAX = 140

    # Posición objetivo del indicador (para suavizado gráfico)
    target_x, target_y = CENTRO_X, CENTRO_Y
    curr_x, curr_y = CENTRO_X, CENTRO_Y

    # Fuentes
    font_large = pygame.font.SysFont("Arial", 32, bold=True)
    font_small = pygame.font.SysFont("Arial", 18)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # Mapear la dirección recibida a la posición del joystick virtual
        desplazamiento = 110
        dir_clean = direccion_actual.lower().strip()

        if "adelante" in dir_clean or "up" in dir_clean:
            target_x, target_y = CENTRO_X, CENTRO_Y - desplazamiento
        elif "atras" in dir_clean or "back" in dir_clean or "down" in dir_clean:
            target_x, target_y = CENTRO_X, CENTRO_Y + desplazamiento
        elif "izquierda" in dir_clean or "left" in dir_clean:
            target_x, target_y = CENTRO_X - desplazamiento, CENTRO_Y
        elif "derecha" in dir_clean or "right" in dir_clean:
            target_x, target_y = CENTRO_X + desplazamiento, CENTRO_Y
        else:  # Neutral "N"
            target_x, target_y = CENTRO_X, CENTRO_Y

        # Interpolación suave de movimiento (Lerp)
        curr_x += (target_x - curr_x) * 0.25
        curr_y += (target_y - curr_y) * 0.25

        # Renderizado
        screen.fill(COLOR_BG)

        # Dibujar área límite
        pygame.draw.circle(screen, COLOR_BASE, (CENTRO_X, CENTRO_Y), RADIO_MAX, 4)
        pygame.draw.circle(screen, (45, 48, 56), (CENTRO_X, CENTRO_Y), RADIO_MAX - 2)

        # Dibujar líneas de eje
        pygame.draw.line(screen, (70, 75, 85), (CENTRO_X - RADIO_MAX, CENTRO_Y), (CENTRO_X + RADIO_MAX, CENTRO_Y), 2)
        pygame.draw.line(screen, (70, 75, 85), (CENTRO_X, CENTRO_Y - RADIO_MAX), (CENTRO_X, CENTRO_Y + RADIO_MAX), 2)

        # Dibujar brazo o palanca
        pygame.draw.line(screen, COLOR_JOYSTICK, (CENTRO_X, CENTRO_Y), (int(curr_x), int(curr_y)), 6)

        # Dibujar indicador móvil
        color_punto = COLOR_ACTIVO if dir_clean != "n" else COLOR_JOYSTICK
        pygame.draw.circle(screen, color_punto, (int(curr_x), int(curr_y)), 28)
        pygame.draw.circle(screen, (255, 255, 255), (int(curr_x), int(curr_y)), 28, 3)

        # Texto de estado
        txt_estado = font_large.render(f"Estado: {direccion_actual.upper()}", True, COLOR_TEXTO)
        screen.blit(txt_estado, (CENTRO_X - txt_estado.get_width() // 2, 50))

        txt_info = font_small.render("Escuchando UDP en puerto 5000 | [ESC] Salir", True, (150, 150, 160))
        screen.blit(txt_info, (CENTRO_X - txt_info.get_width() // 2, ALTO - 40))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()