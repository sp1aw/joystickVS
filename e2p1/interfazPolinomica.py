import gradio as gr
import joblib
import numpy as np
from sklearn.preprocessing import PolynomialFeatures

# --------------------------------------------------
# 1. Cargar el modelo guardado (.joblib)
# --------------------------------------------------
modelo_cargado = joblib.load("modelo_polinomial.joblib")

# Reconstruir poly2 ajustado exactamente para 7 características
poly2 = PolynomialFeatures(degree=2, include_bias=True)
poly2.fit(np.zeros((1, 7)))  # Genera exactamente las 36 columnas requeridas

# Extraer el modelo o los coeficientes
if "model" in modelo_cargado:
    model_poly = modelo_cargado["model"]
    usar_modelo = True
else:
    theta = np.array(modelo_cargado["theta"]).flatten()
    usar_modelo = False


# --------------------------------------------------
# 2. Función de Predicción
# --------------------------------------------------
def predecir_interfaz(
    ram, peso, bateria, pantalla, camara_front, camara_back, marca
):
    mapa_marcas = {"Apple / iPhone": 0.75, "Samsung": 0.50, "Otra Marca": 0.25}
    marca_num = mapa_marcas[marca]

    # Vector de 7 entradas originales
    vector_entrada = np.array(
        [[ram, peso, bateria, pantalla, camara_front, camara_back, marca_num]]
    )

    # Transformación a 36 características polinómicas
    vector_poly = poly2.transform(vector_entrada)

    # Predicción
    if usar_modelo:
        # Si el modelo incluye el bias en los coeficientes o en el intercept
        if (
            hasattr(model_poly, "coef_")
            and model_poly.coef_.shape[-1] == 36
            and model_poly.fit_intercept == False
        ):
            precio_predicho = vector_poly @ model_poly.coef_.T
        else:
            precio_predicho = model_poly.predict(vector_poly)
        precio_val = float(np.ravel(precio_predicho)[0])
    else:
        precio_val = float((vector_poly @ theta)[0])

    precio_final = max(0.0, precio_val)
    return f"${precio_final:,.2f} USD"


# --------------------------------------------------
# 3. Interfaz Gradio
# --------------------------------------------------
demo = gr.Interface(
    fn=predecir_interfaz,
    inputs=[
        gr.Slider(
            minimum=2, maximum=24, step=2, value=8, label="Memoria RAM (GB)"
        ),
        gr.Slider(
            minimum=100,
            maximum=500,
            step=5,
            value=190,
            label="Peso del Celular (g)",
        ),
        gr.Slider(
            minimum=2000,
            maximum=10000,
            step=100,
            value=5000,
            label="Batería (mAh)",
        ),
        gr.Slider(
            minimum=4.0,
            maximum=10.0,
            step=0.1,
            value=6.7,
            label="Pantalla (Pulgadas)",
        ),
        gr.Slider(
            minimum=2, maximum=108, step=1, value=12, label="Cámara Frontal (MP)"
        ),
        gr.Slider(
            minimum=5,
            maximum=200,
            step=1,
            value=50,
            label="Cámara Principal (MP)",
        ),
        gr.Dropdown(
            choices=["Apple / iPhone", "Samsung", "Otra Marca"],
            value="Samsung",
            label="Marca del Dispositivo",
        ),
    ],
    outputs=gr.Textbox(label="Precio Predicho de Lanzamiento"),
    title="Predicción de Precios de Celulares (Ecuación Normal)",
    description="Interfaz interactiva para el modelo de Regresión Polinómica (Grado 2).",
)

if __name__ == "__main__":
    demo.launch()