import numpy as np
import pandas as pd
import joblib
import gradio as gr

# ==========================================
# 1. CARGAR MODELO Y PREPROCESADORES
# ==========================================
poly = joblib.load('poly_features.joblib')
scaler = joblib.load('scaler.joblib')
weights = joblib.load('logistic_regression_weights.joblib')

features = ['cibil_score', 'income_annum', 'loan_amount']

# Tasas de conversión de referencia hacia INR (Rupia India)
USD_TO_INR = 83.0         # 1 USD ≈ 83 INR
COP_TO_INR = 83.0 / 4150.0 # 1 USD ≈ 4150 COP (1 COP ≈ 0.02 INR)

def sigmoid(z):
    return 1 / (1 + np.exp(-np.clip(z, -250, 250)))

# ==========================================
# 2. FUNCIÓN DE PREDICCIÓN MULTI-MONEDA
# ==========================================
# SE CORRIGE EL ORDEN DE LOS PARÁMETROS PARA COINCIDIR CON GRADIO:
def predict_loan_status_multicurrency(cibil_score, currency, income_val, loan_val):
    # Conversión dinámica según la moneda seleccionada
    if currency == "USD ($ Dólares)":
        income_inr = income_val * USD_TO_INR
        loan_inr = loan_val * USD_TO_INR
    elif currency == "COP ($ Pesos Colombianos)":
        income_inr = income_val * COP_TO_INR
        loan_inr = loan_val * COP_TO_INR
    else:  # INR
        income_inr = income_val
        loan_inr = loan_val
    
    # Formatear el DataFrame de entrada
    df_input = pd.DataFrame([[cibil_score, income_inr, loan_inr]], columns=features)
    
    # Preprocesamiento
    X_poly = poly.transform(df_input)
    X_scaled = scaler.transform(X_poly)
    X_b = np.c_[np.ones((X_scaled.shape[0], 1)), X_scaled]
    
    # Inferencia
    probability = sigmoid(np.dot(X_b, weights))[0]
    
    # Formatear la respuesta visual
    if probability >= 0.5:
        resultado = f"🟢 APROBADO (Probabilidad de Aprobación: {probability * 100:.2f}%)"
    else:
        resultado = f"🔴 RECHAZADO (Probabilidad de Aprobación: {probability * 100:.2f}%)"
        
    return resultado

# ==========================================
# 3. INTERFAZ DE GRADIO CON SELECCIÓN DE MONEDA
# ==========================================
demo = gr.Interface(
    fn=predict_loan_status_multicurrency,
    inputs=[
        gr.Slider(
            minimum=300, 
            maximum=900, 
            step=1, 
            value=720, 
            label="Puntaje Crediticio CIBIL (300 - 900)"
        ),
        gr.Dropdown(
            choices=["USD ($ Dólares)", "COP ($ Pesos Colombianos)", "INR (Rupias Indias)"],
            value="COP ($ Pesos Colombianos)",
            label="Moneda de Ingreso"
        ),
        gr.Number(
            value=300000000, 
            label="Ingreso Anual del Solicitante"
        ),
        gr.Number(
            value=600000000, 
            label="Monto del Préstamo Solicitado"
        )
    ],
    outputs=gr.Textbox(
        label="Dictamen Final de la Solicitud", 
        interactive=False
    ),
    title="🏦 Clasificador Binario para Aprobación de Préstamos",
    description="Seleccione la moneda preferida (USD o COP) e ingrese los datos para verificar la viabilidad del préstamo.",
    examples=[
        [780, "COP ($ Pesos Colombianos)", 480000000, 1500000000], # Ejemplo en COP (Aprobado)
        [780, "USD ($ Dólares)", 115000, 360000],                 # Ejemplo en USD (Aprobado)
        [417, "COP ($ Pesos Colombianos)", 200000000, 600000000]   # Ejemplo en COP (Rechazado)
    ],
    theme=gr.themes.Soft()
)

# ==========================================
# 4. LANZAMIENTO
# ==========================================
demo.launch(share=True, debug=True)