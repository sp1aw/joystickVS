import gradio as gr
import joblib
import numpy as np
import pandas as pd

# ==========================================
# 1. CARGA DEL MODELO Y LISTA DE SÍNTOMAS
# ==========================================
# Asegúrate de haber ejecutado el script anterior para generar estos dos archivos
clf = joblib.load('dt_model.joblib')
symptoms_list = joblib.load('symptoms_list.joblib')

# ==========================================
# 2. FUNCIÓN DE PREDICCIÓN
# ==========================================
def predict_disease(selected_symptoms):
    if not selected_symptoms:
        return "⚠️ **Por favor, seleccione al menos un síntoma para realizar el diagnóstico.**"
    
    # Crear el vector binario (1 si el síntoma fue seleccionado, 0 si no)
    input_vector = [1 if sym in selected_symptoms else 0 for sym in symptoms_list]
    df_input = pd.DataFrame([input_vector], columns=symptoms_list)
    
    # Realizar predicción y obtener probabilidades
    prediction = clf.predict(df_input)[0]
    probabilities = clf.predict_proba(df_input)[0]
    
    # Obtener el Top 3 de diagnósticos más probables
    top_indices = np.argsort(probabilities)[::-1][:3]
    classes = clf.classes_
    
    # Formatear el resultado en Markdown
    result = f"🩺 **Diagnóstico Principal Sugerido:**\n## {prediction}\n\n"
    result += "---\n"
    result += "📊 **Probabilidades Estimadas por el Modelo:**\n"
    
    for idx in top_indices:
        prob = probabilities[idx]
        if prob > 0:
            result += f"* **{classes[idx]}**: {prob * 100:.1f}%\n"
            
    return result

# ==========================================
# 3. INTERFAZ GRÁFICA CON GRADIO
# ==========================================
with gr.Blocks(theme=gr.themes.Soft(), title="Sistema de Diagnóstico Médico") as demo:
    gr.Markdown(
        """
        # 🩺 Clasificador Médico con Árboles de Decisión
        Seleccione los síntomas que presenta el paciente. Cada casilla marcada equivale a **1** (presente) y desmarcada a **0** (ausente).
        """
    )
    
    with gr.Row():
        with gr.Column(scale=2):
            symptom_checkboxes = gr.CheckboxGroup(
                choices=symptoms_list,
                label="Lista de Síntomas Disponibles"
            )
            btn_predict = gr.Button("🔍 Analizar Síntomas", variant="primary")
            btn_clear = gr.Button("🧹 Limpiar Selección", variant="secondary")
            
        with gr.Column(scale=1):
            output_markdown = gr.Markdown(
                value="*Los resultados de la evaluación aparecerán aquí.*",
                label="Resultado del Diagnóstico"
            )
    
    # Eventos de los botones
    btn_predict.click(
        fn=predict_disease,
        inputs=symptom_checkboxes,
        outputs=output_markdown
    )
    
    btn_clear.click(
        fn=lambda: ([], "*Los resultados de la evaluación aparecerán aquí.*"),
        inputs=None,
        outputs=[symptom_checkboxes, output_markdown]
    )

# Lanzar servidor local (o enlace público en Colab)
if __name__ == '__main__':
    demo.launch(share=True)