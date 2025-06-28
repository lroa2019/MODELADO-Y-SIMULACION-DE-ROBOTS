import matplotlib.pyplot as plt
import pandas as pd

# Cargar el CSV
csv_filename = "Fase3_Laura_Roa.csv"
data = pd.read_csv(csv_filename)

# Extraer datos
x_position = data["Tiempo"]
g_parcial = data["G_parcial"]

# Cálculos requeridos
g_total = g_parcial.sum()
g_std = g_parcial.std()

# Crear el gráfico
plt.figure(figsize=(10, 5))
plt.plot(x_position, g_parcial, marker='o', linestyle='-', color='b', label="G_parcial")

# Etiquetas y título
plt.xlabel("Tiempo (s)")
plt.ylabel("Sumatorio de Fuerzas de cada link (G_parcial)")
plt.title(f"G_parcial vs. Tiempo\nG_total = {g_total:.2f} | Desviación estándar = {g_std:.2f}")
plt.legend()
plt.grid(True)

# Guardar como PDF
plt.savefig("Fase3_Laura_Roa.pdf")
plt.show()
