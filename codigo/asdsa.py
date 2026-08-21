import csv
import os

CSV_FILE = "ensayos_imu.csv"

def corregir_velocidades_y_direcciones_neutras(file_path):
    if not os.path.exists(file_path):
        print(f"Error: No se encontró el archivo '{file_path}'.")
        return

    filas_modificadas = 0
    filas_totales = 0
    datos_corregidos = []

    # 1. Leer y corregir los datos
    with open(file_path, mode='r', encoding='latin1', newline='') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames

        for row in reader:
            filas_totales += 1
            direccion = str(row.get("Direccion", "")).strip().upper()

            # Evalúa si la dirección es N, Neutra o Neutro
            if direccion in ['N', 'NEUTRA', 'NEUTRO']:
                # Asigna la palabra completa "Neutra" y fija la velocidad en "0.00"
                if row["velocidad_objetivo"] != "0.00" or row["Direccion"] != "Neutra":
                    row["Direccion"] = "Neutra"
                    row["velocidad_objetivo"] = "0.00"
                    filas_modificadas += 1

            datos_corregidos.append(row)

    # 2. Sobrescribir el archivo original
    with open(file_path, mode='w', encoding='latin1', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(datos_corregidos)

    print(f"--- PROCESO COMPLETADO ---")
    print(f"Total de muestras analizadas : {filas_totales}")
    print(f"Filas actualizadas a 'Neutra' con velocidad 0.00: {filas_modificadas}")

if __name__ == "__main__":
    corregir_velocidades_y_direcciones_neutras(CSV_FILE)