import socket

# "0.0.0.0" permite recibir paquetes desde cualquier tarjeta de red activa
UDP_IP = "0.0.0.0"
UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Permitir reutilizar el puerto
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

try:
    sock.bind((UDP_IP, UDP_PORT))
    print(f"=== Escuchando paquetes UDP en el puerto {UDP_PORT} ===")
    print("Mueve el guante o espera las lecturas de velocidad...\n")

    while True:
        data, addr = sock.recvfrom(1024)
        print(f"[{addr[0]}:{addr[1]}] -> {data.decode('utf-8', errors='ignore')}")

except Exception as e:
    print(f"Error en el socket: {e}")
finally:
    sock.close()