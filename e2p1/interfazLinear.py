import gradio as gr
import joblib
import numpy as np

# --------------------------------------------------
# 1. Cargar los coeficientes theta (.joblib)
# --------------------------------------------------
modelo_cargado = joblib.load("modelo_lineal.joblib")
theta_lineal = np.array(modelo_cargado["theta"]).flatten()


# --------------------------------------------------
# 2. Función de Predicción
# --------------------------------------------------
def predecir_nota_estudiante(medu, fedu, dalc, walc, failures, g1):
    # Reconstruir las características compuestas 'Pedu' y 'Alc'
    pedu = (medu + fedu) / 2
    alc = (dalc + walc) / 2

    # Vector con la columna de ones (1.0) para el intercepto + las 4 características
    # [1.0, Pedu, Alc, failures, G1]
    x_input = np.array([1.0, pedu, alc, failures, g1])

    # Predicción mediante multiplicación matricial con theta: y = x @ theta
    nota_predicha = np.dot(x_input, theta_lineal)

    # Acotar la nota al rango escolar válido (0 a 20)
    nota_final = np.clip(nota_predicha, 0.0, 20.0)

    estado = "Aprobado" if nota_final >= 10 else "Reprobado / En Riesgo"
    return f"{nota_final:.2f} / 20 puntos ({estado})"


# --------------------------------------------------
# 3. Interfaz Gráfica con Gradio
# --------------------------------------------------
demo = gr.Interface(
    fn=predecir_nota_estudiante,
    inputs=[
        gr.Slider(
            minimum=0,
            maximum=4,
            step=1,
            value=2,
            label="Nivel Educativo Madre (Medu: 0=Ninguno, 4=Superior)",
        ),
        gr.Slider(
            minimum=0,
            maximum=4,
            step=1,
            value=2,
            label="Nivel Educativo Padre (Fedu: 0=Ninguno, 4=Superior)",
        ),
        gr.Slider(
            minimum=1,
            maximum=5,
            step=1,
            value=1,
            label="Consumo Alcohol Entre Semana (Dalc: 1=Bajo, 5=Alto)",
        ),
        gr.Slider(
            minimum=1,
            maximum=5,
            step=1,
            value=2,
            label="Consumo Alcohol Fin de Semana (Walc: 1=Bajo, 5=Alto)",
        ),
        gr.Slider(
            minimum=0,
            maximum=4,
            step=1,
            value=0,
            label="Materias Reprobadas Previamente (failures)",
        ),
        gr.Slider(
            minimum=0,
            maximum=20,
            step=1,
            value=12,
            label="Nota Primer Período (G1: Escala 0-20)",
        ),
    ],
    outputs=gr.Textbox(label="Nota Final Predicha (G3)"),
    title="Predicción del Rendimiento Académico (Regresión Lineal)",
    description="Interfaz interactiva cargando el vector theta del modelo lineal preentrenado desde Colab.",
)

if __name__ == "__main__":
    demo.launch()