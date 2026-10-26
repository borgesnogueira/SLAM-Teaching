import matplotlib.pyplot as plt
import os

# Gravação feita pelo robo.record()
arquivo = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dados", "recording.csv")

# Listas para guardar valores do ground truth
x_vals = []
y_vals = []
theta_vals = []

with open(arquivo, "r") as f:
    for linha in f:
        partes = linha.strip().split(",")
        if partes[0] == "ground_truth":
            # converte os números depois do rótulo
            valores = list(map(float, partes[1:]))
            x_vals.append(valores[0])
            y_vals.append(valores[1])
            theta_vals.append(valores[2])

# --- Plotando ---
plt.figure(figsize=(8, 6))

plt.plot(x_vals, y_vals, 'r.', label="Ground Truth")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Trajetória real")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.show()