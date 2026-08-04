import control as ct
import matplotlib.pyplot as plt
import numpy as np

# 1. Função de Transferência da Planta (30 graus)
# G(s) = 4.33 / (s^2 + 4.11s + 36.12)
num_planta = [4.33]
den_planta = [1, 4.11, 36.12]
G = ct.tf(num_planta, den_planta)

# Aproximação de Padé para o Atraso de Transporte (L = 0.15s)
atraso = ct.pade(0.15, 3) 
G_atraso = ct.tf(atraso[0], atraso[1])

# Planta completa = G(s) * Atraso
Planta = ct.series(G, G_atraso)

# 2. O Controlador PID Universal (Robusto / Pior Caso)
Kp, Ki, Kd = 2.1, 16.0, 0.8
# C(s) = (Kd*s^2 + Kp*s + Ki) / s
num_pid = [Kd, Kp, Ki]
den_pid = [1, 0]
C = ct.tf(num_pid, den_pid)

# 3. Malha Aberta Total: G_ma(s) = C(s) * Planta(s)
G_ma = ct.series(C, Planta)

# 4. Plotando o Diagrama de Nyquist
plt.figure(figsize=(8, 8))
ct.nyquist_plot(G_ma, omega=np.logspace(-2, 2, 1000))
plt.title("Diagrama de Nyquist - PID Robusto (Ganhos Universais)")
plt.plot(-1, 0, 'ro', label='Ponto Crítico (-1, j0)')
plt.legend()
plt.grid(True)
plt.show()