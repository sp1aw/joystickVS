# Plataforma de monitoreo: dirección y velocidad comandadas al carrito
# El PC debe estar conectado a la red Wi-Fi "Guante_IMU"
# Teclas: M = cambiar modo (básico/avanzado) | Z = recalibrar guante | G = grabar CSV | ESC = salir
import socket
import threading
import time
import csv
from collections import deque
import pygame

# ---------------- Red ----------------
UDP_PORT = 5000
GUANTE_IP = "192.168.4.1"

# ---------------- Estado compartido ----------------
estado = {"dir": "N", "vel": 0.0, "modo": "BASICO", "t": 0.0, "status": "Esperando guante..."}
historial = deque(maxlen=200)          # últimos 10 s (20 msg/s)
contador = {"n": 0, "tasa": 0.0}
lock = threading.Lock()
corriendo = True
grabando = {"on": False, "archivo": None, "writer": None}


def escuchar(sock):
    # Recibe CMD,dir,vel[,modo] y STATUS,...
    global corriendo
    while corriendo:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue
        except OSError:
            break
        msg = data.decode("utf-8", errors="ignore").strip()
        partes = msg.split(",")
        with lock:
            if partes[0] == "CMD" and len(partes) >= 3:
                try:
                    estado["vel"] = float(partes[2])
                except ValueError:
                    continue
                estado["dir"] = partes[1]
                if len(partes) >= 4:
                    estado["modo"] = partes[3]
                estado["t"] = time.time()
                historial.append((estado["t"], estado["vel"], estado["dir"]))
                contador["n"] += 1
                if grabando["on"]:
                    grabando["writer"].writerow([f"{estado['t']:.3f}", "CMD", estado["dir"], estado["vel"], estado["modo"]])
            elif partes[0] == "RAW" and grabando["on"]:
                grabando["writer"].writerow([f"{time.time():.3f}", "RAW"] + partes[1:])
            elif partes[0] == "STATUS" and len(partes) >= 2:
                estado["status"] = partes[1]


def alternar_grabacion():
    with lock:
        if not grabando["on"]:
            nombre = time.strftime("registro_%Y%m%d_%H%M%S.csv")
            grabando["archivo"] = open(nombre, "w", newline="")
            grabando["writer"] = csv.writer(grabando["archivo"])
            grabando["writer"].writerow(["tiempo", "tipo", "c1", "c2", "c3", "c4", "c5", "c6", "c7", "c8"])
            grabando["on"] = True
        else:
            grabando["on"] = False
            grabando["archivo"].close()


# ---------------- Interfaz ----------------
ANCHO, ALTO = 980, 600
FONDO = (24, 26, 32)
PANEL = (36, 39, 48)
LINEA = (70, 75, 88)
TEXTO = (235, 235, 240)
GRIS = (150, 152, 165)
COLORES = {"N": (150, 152, 165), "adelante": (64, 196, 120), "atras": (235, 110, 90),
           "izquierda": (80, 160, 240), "derecha": (240, 190, 70)}
FLECHA = {"adelante": (0, -1), "atras": (0, 1), "izquierda": (-1, 0), "derecha": (1, 0), "N": (0, 0)}


def texto(sup, fuente, s, color, pos, centro=False):
    img = fuente.render(s, True, color)
    r = img.get_rect()
    if centro:
        r.center = pos
    else:
        r.topleft = pos
    sup.blit(img, r)


def main():
    global corriendo
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.5)
    threading.Thread(target=escuchar, args=(sock,), daemon=True).start()

    pygame.init()
    pantalla = pygame.display.set_mode((ANCHO, ALTO))
    pygame.display.set_caption("Plataforma IoT - Joystick inteligente")
    reloj = pygame.time.Clock()
    f_tit = pygame.font.SysFont("Arial", 26, bold=True)
    f_big = pygame.font.SysFont("Arial", 40, bold=True)
    f_med = pygame.font.SysFont("Arial", 20)
    f_sm = pygame.font.SysFont("Arial", 15)

    cx, cy, R = 250, 330, 150          # joystick virtual
    px, py = float(cx), float(cy)
    t_tasa = time.time()
    t_hola = 0.0

    while corriendo:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                corriendo = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_z:
                sock.sendto(b"ZERO", (GUANTE_IP, UDP_PORT))
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_m:
                sock.sendto(b"MODO", (GUANTE_IP, UDP_PORT))
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_g:
                alternar_grabacion()

        with lock:
            d, v, modo, t_ult, st = estado["dir"], estado["vel"], estado["modo"], estado["t"], estado["status"]
            hist = list(historial)
            graba = grabando["on"]
        ahora = time.time()
        if ahora - t_hola >= 1:   # registro: el guante responde directo a este PC
            t_hola = ahora
            try:
                sock.sendto(b"HOLA", (GUANTE_IP, UDP_PORT))
            except OSError:
                pass
        if ahora - t_tasa >= 1:
            contador["tasa"] = contador["n"] / (ahora - t_tasa)
            contador["n"], t_tasa = 0, ahora
        conectado = ahora - t_ult < 1.0
        color = COLORES.get(d, GRIS)

        pantalla.fill(FONDO)
        texto(pantalla, f_tit, "Plataforma robótica móvil - Joystick inteligente", TEXTO, (30, 20))
        # Conexión
        pygame.draw.circle(pantalla, (64, 196, 120) if conectado else (235, 110, 90), (40, 72), 8)
        texto(pantalla, f_sm, f"{'Conectado' if conectado else 'Sin datos'}  |  {contador['tasa']:.0f} msg/s  |  Guante: {st}",
              GRIS, (56, 63))
        texto(pantalla, f_med, f"Modo: {modo}", (240, 190, 70) if modo == "AVANZADO" else TEXTO, (760, 22))
        texto(pantalla, f_sm, "MLP + MLP" if modo == "AVANZADO" else "Polinómica + Softmax", GRIS, (760, 50))

        # Panel joystick: dirección + magnitud de velocidad
        pygame.draw.rect(pantalla, PANEL, (30, 110, 440, 460), border_radius=12)
        texto(pantalla, f_med, "Dirección comandada", GRIS, (50, 125))
        pygame.draw.circle(pantalla, (45, 48, 58), (cx, cy), R)
        pygame.draw.circle(pantalla, LINEA, (cx, cy), R, 3)
        pygame.draw.line(pantalla, LINEA, (cx - R, cy), (cx + R, cy), 2)
        pygame.draw.line(pantalla, LINEA, (cx, cy - R), (cx, cy + R), 2)
        for nombre, (dx, dy) in FLECHA.items():
            if nombre != "N":
                texto(pantalla, f_sm, nombre, COLORES[nombre], (cx + dx * (R + 30), cy + dy * (R + 18)), centro=True)
        dx, dy = FLECHA.get(d, (0, 0))
        mag = 0.25 + 0.75 * v if d != "N" else 0
        tx, ty = cx + dx * R * mag, cy + dy * R * mag
        px += (tx - px) * 0.3
        py += (ty - py) * 0.3
        pygame.draw.line(pantalla, color, (cx, cy), (int(px), int(py)), 6)
        pygame.draw.circle(pantalla, color, (int(px), int(py)), 24)
        pygame.draw.circle(pantalla, (255, 255, 255), (int(px), int(py)), 24, 2)
        texto(pantalla, f_big, d.upper(), color, (cx, 545), centro=True)

        # Panel velocidad
        pygame.draw.rect(pantalla, PANEL, (500, 110, 450, 180), border_radius=12)
        texto(pantalla, f_med, "Velocidad comandada", GRIS, (520, 125))
        texto(pantalla, f_big, f"{v * 100:5.1f} %", TEXTO, (520, 158))
        pygame.draw.rect(pantalla, (45, 48, 58), (520, 225, 410, 34), border_radius=8)
        pygame.draw.rect(pantalla, color, (520, 225, int(410 * min(max(v, 0), 1)), 34), border_radius=8)
        for i in range(5):
            x = 520 + int(410 * i / 4)
            texto(pantalla, f_sm, f"{i * 25}", GRIS, (x - 8, 263))

        # Historial de velocidad (últimos 10 s), color = dirección
        gx, gy, gw, gh = 500, 310, 450, 260
        pygame.draw.rect(pantalla, PANEL, (gx, gy, gw, gh), border_radius=12)
        texto(pantalla, f_med, "Velocidad en el tiempo (10 s)", GRIS, (gx + 20, gy + 12))
        x0, y0, w0, h0 = gx + 20, gy + 50, gw - 40, gh - 75
        pygame.draw.line(pantalla, LINEA, (x0, y0 + h0), (x0 + w0, y0 + h0), 1)
        pygame.draw.line(pantalla, LINEA, (x0, y0), (x0 + w0, y0), 1)
        texto(pantalla, f_sm, "1.0", GRIS, (x0 - 2, y0 - 18))
        texto(pantalla, f_sm, "0", GRIS, (x0 - 2, y0 + h0 + 2))
        pts = [(x0 + w0 - (ahora - t) / 10 * w0, y0 + h0 - vel * h0, dd) for t, vel, dd in hist if ahora - t <= 10]
        for (x1, y1, d1), (x2, y2, _) in zip(pts, pts[1:]):
            pygame.draw.line(pantalla, COLORES.get(d1, GRIS), (x1, y1), (x2, y2), 3)

        texto(pantalla, f_sm, "M: cambiar modo   Z: recalibrar   G: grabar CSV   ESC: salir", GRIS, (30, 578))
        if graba:
            pygame.draw.circle(pantalla, (235, 60, 60), (ANCHO - 40, 72), 8)
            texto(pantalla, f_sm, "GRABANDO", (235, 60, 60), (ANCHO - 125, 63))

        pygame.display.flip()
        reloj.tick(30)

    if grabando["on"]:
        alternar_grabacion()
    sock.close()
    pygame.quit()


if __name__ == "__main__":
    main()
