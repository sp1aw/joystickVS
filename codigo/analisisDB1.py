import pandas as pd
import numpy as np
import math

FILE_PATH = "ensayos_imu.csv"
ALPHA = 0.98  # Factor del Filtro Complementario
DT = 0.01     # 10 ms por muestra

def analizar_ensayos_consistente(filepath):
    # Cargar CSV manejando posibles problemas de codificacion (acentos/latin1)
    try:
        df = pd.read_csv(filepath, encoding='utf-8')
    except UnicodeDecodeError:
        try:
            df = pd.read_csv(filepath, encoding='latin1')
        except Exception as e:
            print(f"Error al leer el archivo: {e}")
            return
    except FileNotFoundError:
        print(f"Error: No se encontró el archivo {filepath}")
        return

    if df.empty:
        print("El archivo CSV está vacío.")
        return

    # Normalizar encabezados eliminando espacios accidentales
    df.columns = df.columns.str.strip()

    mapa_movimientos = {
        'derecha': ('angle_z', 'Yaw (Z)'),
        'izquierda': ('angle_z', 'Yaw (Z)'),
        'adelante': ('angle_y', 'Pitch (Y)'),
        'atrás': ('angle_y', 'Pitch (Y)'),
        'atras': ('angle_y', 'Pitch (Y)')
    }

    resultados = []
    
    # Normalizar texto de direcciones para evitar fallos por tildes/mayúsculas
    df['Direccion_Clean'] = df['Direccion'].astype(str).str.strip().str.lower()
    df_movs = df[df['Direccion_Clean'].isin(mapa_movimientos.keys())].copy()

    for (direccion, trial_id), grupo in df_movs.groupby(['Direccion', 'trial_id']):
        dir_key = str(direccion).strip().lower()
        col_angulo, nombre_eje = mapa_movimientos[dir_key]

        grupo = grupo.sort_values('sample_id')

        # Convertir velocidades angulares de rad/s a deg/s
        w_y_deg = np.degrees(grupo['w_y'].values)
        w_z_deg = np.degrees(grupo['w_z'].values)

        ax = grupo['a_x'].values
        ay = grupo['a_y'].values
        az = grupo['a_z'].values

        n_samples = len(grupo)
        angles_y = np.zeros(n_samples)
        angles_z = np.zeros(n_samples)

        if n_samples > 0:
            angles_y[0] = math.degrees(math.atan2(ax[0], math.sqrt(ay[0]**2 + az[0]**2)))
            angles_z[0] = math.degrees(math.atan2(az[0], math.sqrt(ax[0]**2 + ay[0]**2)))

        for i in range(1, n_samples):
            acc_y = math.degrees(math.atan2(ax[i], math.sqrt(ay[i]**2 + az[i]**2)))
            acc_z = math.degrees(math.atan2(az[i], math.sqrt(ax[i]**2 + ay[i]**2)))

            angles_y[i] = ALPHA * (angles_y[i-1] + w_y_deg[i] * DT) + (1.0 - ALPHA) * acc_y
            angles_z[i] = ALPHA * (angles_z[i-1] + w_z_deg[i] * DT) + (1.0 - ALPHA) * acc_z

        vector_angulos = angles_z if col_angulo == 'angle_z' else angles_y

        desviaciones = vector_angulos - vector_angulos[0]
        max_cambio = np.max(np.abs(desviaciones))

        resultados.append({
            'Direccion': direccion,
            'trial_id': trial_id,
            'Eje': nombre_eje,
            'Max_Cambio_Deg': max_cambio
        })

    df_res = pd.DataFrame(resultados)

    if df_res.empty:
        print("No se encontraron ensayos válidos.")
        return

    print("==================================================")
    print("  ANÁLISIS CONSISTENTE (FILTRO COMPLEMENTARIO)   ")
    print("==================================================")

    resumen = df_res.groupby(['Direccion', 'Eje']).agg(
        Promedio_Max_Grad=('Max_Cambio_Deg', 'mean'),
        Desviacion_Std=('Max_Cambio_Deg', 'std'),
        Total_Ensayos=('trial_id', 'count')
    ).reset_index()

    for _, row in resumen.iterrows():
        print(f"\nMovimiento: {str(row['Direccion']).upper()} (Eje: {row['Eje']})")
        print(f"  • Ensayos analizados : {int(row['Total_Ensayos'])}")
        print(f"  • Cambio máx promedio : {row['Promedio_Max_Grad']:+.2f}°")
        print(f"  • Desviación estándar : {row['Desviacion_Std']:.2f}°")

    print("\n==================================================")

if __name__ == "__main__":
    analizar_ensayos_consistente(FILE_PATH)