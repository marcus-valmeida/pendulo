import serial
import matplotlib.pyplot as plt
import csv
from datetime import datetime

PORTA = '/dev/ttyUSB0'
BAUD_RATE = 115200

rotulo = input("Rotulo deste ensaio (ex: 15graus, deixe vazio se nao quiser): ").strip()
rotulo = "".join(c if c.isalnum() else "_" for c in rotulo)

tempos = []
angulos_reais = []
alvos = []

print(f"Abrindo conexão em {PORTA}...")
print("Ouvindo a placa... (Pressione Ctrl+C para parar a gravação)")

try:
    with serial.Serial(PORTA, BAUD_RATE, timeout=1) as ser:
        ser.reset_input_buffer()
        while True:
            if ser.in_waiting > 0:
                linha = ser.readline().decode('utf-8', errors='ignore').strip()
                partes = linha.split(',')
                if len(partes) == 3:
                    try:
                        t = int(partes[0])
                        ang = float(partes[1])
                        alvo = float(partes[2])
                        
                        tempos.append(t)
                        angulos_reais.append(ang)
                        alvos.append(alvo)
                        
                        print(f"Tempo: {t} ms | Real: {ang}° | Alvo: {alvo}°")
                    except ValueError:
                        pass 
except KeyboardInterrupt:
    print("\nGravação finalizada pelo usuário!")

if tempos:
    # 1. Salva os dados (nome unico por ensaio, evita sobrescrever dados anteriores)
    sufixo = f"_{rotulo}" if rotulo else ""
    nome_arquivo = f"telemetria{sufixo}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    with open(nome_arquivo, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Tempo_ms", "Angulo_Real_graus", "Alvo_graus"])
        for t, a, alv in zip(tempos, angulos_reais, alvos):
            writer.writerow([t, a, alv])
    
    # 2. Prepara os vetores para o gráfico
    tempo_inicial = tempos[0]
    tempos_ajustados = [(t - tempo_inicial) / 1000.0 for t in tempos] 
    
    plt.figure(figsize=(10, 6))
    plt.plot(tempos_ajustados, angulos_reais, 'b-', linewidth=2, label="Resposta do Hardware")
    
    # 3. Inteligência para diferenciar Malha Aberta vs Fechada
    modo_malha_fechada = any(a != 0.0 for a in alvos)
    
    if modo_malha_fechada:
        # É Malha Fechada: plota a linha do Setpoint (Alvo)
        plt.plot(tempos_ajustados, alvos, 'r--', linewidth=2, label="Alvo (Setpoint)")
        plt.title("Ensaio de Malha Fechada - Controle PID Robusto", fontsize=14, fontweight='bold')
    else:
        # É Malha Aberta: Calcula e plota onde estabilizou (ganho K do Ogata)
        valor_final = sum(angulos_reais[-10:]) / 10 if len(angulos_reais) > 10 else angulos_reais[-1]
        plt.axhline(y=valor_final, color='r', linestyle='--', label=f'Estabilização (~{valor_final:.1f}°)')
        plt.title("Ensaio de Malha Aberta - Resposta ao Degrau (Ogata)", fontsize=14, fontweight='bold')

    plt.xlabel("Tempo (segundos)", fontsize=12)
    plt.ylabel("Ângulo (graus)", fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend()
    plt.show()
else:
    print("Nenhum dado válido foi capturado.")