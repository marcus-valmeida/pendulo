"""
Identificacao de planta (K, T, L) a partir de ensaio de malha aberta (degrau)
e calculo dos ganhos PID pelo metodo de Ziegler-Nichols (Metodo 1 - curva de
reacao / metodo da tangente), seguindo exatamente as formulas do material da
disciplina (4_AcoesBasicasControle-2.pdf, slides 123-131).

Modelo da planta: H(s) = K/(Ts+1) * e^(-Ls)

Uso:
    python3 identifica_zn.py telemetria_XXXXXXXX_HHMMSS.csv --pwm 317

--pwm e o valor de PWM aplicado no degrau (deve bater com o que foi escrito
em angulo_m_a / PWM_TESTE_MALHA_ABERTA no main.c para esse ensaio).
"""

import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks


def carregar_csv(caminho):
    tempos, angulos = [], []
    with open(caminho, newline='') as f:
        leitor = csv.reader(f)
        next(leitor)  # cabecalho
        for linha in leitor:
            if len(linha) < 2:
                continue
            tempos.append(float(linha[0]))
            angulos.append(float(linha[1]))
    tempos = np.array(tempos)
    tempos = (tempos - tempos[0]) / 1000.0  # ms -> s, relativo ao inicio
    angulos = np.array(angulos)
    return tempos, angulos


def metodo_tangente(t, y, U):
    """Metodo da tangente (slides 123-125): K, L, T a partir do ponto de
    inflexao (derivada maxima)."""
    y_inf = np.mean(y[-max(5, len(y)//20):])  # y(infinito): media do trecho final
    K = y_inf / U

    dy = np.gradient(y, t)
    i_max = np.argmax(dy)
    alpha_max = dy[i_max]
    t_max = t[i_max]
    y_max = y[i_max]

    if alpha_max <= 0:
        raise ValueError("Derivada maxima <= 0 — curva nao parece um degrau valido.")

    L = t_max - y_max / alpha_max
    L = max(L, 0.0)

    alvo_63 = (1 - np.exp(-1)) * y_inf
    idx_63 = np.argmin(np.abs(y - alvo_63))
    T = t[idx_63] - L

    if T <= 0:
        print("  AVISO: T <= 0 pelo metodo da tangente — a subida foi mais "
              "rapida do que a amostragem consegue resolver (T real menor "
              "que o intervalo entre amostras). L ainda e valido; T deve ser "
              "tratado como 'muito pequeno / nao resolvido', nao como esse "
              "valor negativo.")

    return K, T, L, y_inf


def metodo_minimos_quadrados(t, y, U, n_pontos=25):
    """Metodo dos minimos quadrados (slides 128-131), eq. 190-203.

    Regride A(Tf) = U*Tf*K - U*(L*K) - T*y(Tf) usando varios Tf crescentes,
    resolvendo para theta = [K, L*K, T] por minimos quadrados.
    """
    A = np.concatenate(([0.0], np.cumsum(
        (y[1:] + y[:-1]) / 2.0 * np.diff(t)
    )))  # integral acumulada (trapezio), A(t)

    # Comeca cedo (logo apos os primeiros pontos) para garantir uma mistura
    # de pontos dentro do transiente e em regime — comecar tarde demais (ex:
    # so em regime) deixa as colunas U (constante) e -y(Tf) quase colineares
    # e a regressao fica mal condicionada.
    idx = np.linspace(3, len(t) - 1, n_pontos).astype(int)
    idx = np.unique(idx)

    Psi = np.column_stack([
        U * t[idx],
        -U * np.ones(len(idx)),
        -y[idx],
    ])
    Gamma = A[idx]

    theta, *_ = np.linalg.lstsq(Psi, Gamma, rcond=None)
    K, LK, T = theta
    L = LK / K
    return K, T, L


def metodo_2a_ordem_decremento_log(t, y, U, y0=None, prominencia=1.0):
    """2a ordem subamortecida + atraso, via decremento logaritmico entre
    extremos sucessivos (slides 73-75: M = exp(-zeta*pi/sqrt(1-zeta^2))).

    Usa a diferenca pico-vale (nao o valor final) para achar zeta e omega_n,
    o que torna o metodo imune a uma deriva lenta de regime (uma segunda
    dinamica, bem mais lenta, sobreposta a oscilacao mecanica rapida) — o
    metodo da tangente e o dos minimos quadrados assumem 1a ordem pura e
    quebram quando a curva tem esse tipo de oscilacao.
    """
    y0 = y[0] if y0 is None else y0
    limiar = max(0.4, 0.02 * (np.max(y) - y0))
    idx_inicio = int(np.argmax(np.abs(y - y0) > limiar))
    if idx_inicio == 0:
        raise ValueError("Nao foi possivel detectar o inicio do movimento (L).")
    L = (t[idx_inicio - 1] + t[idx_inicio]) / 2.0

    t_mov = t[idx_inicio:]
    y_mov = y[idx_inicio:]

    picos, _ = find_peaks(y_mov, prominence=prominencia)
    vales, _ = find_peaks(-y_mov, prominence=prominencia)

    if len(picos) < 2 or len(vales) < 2:
        raise ValueError("Menos de 2 picos/vales — curva nao parece ter "
                          "oscilacao subamortecida visivel (pode ser 1a "
                          "ordem, ou T menor que a amostragem).")

    p1, p2 = picos[0], picos[1]
    v1, v2 = vales[0], vales[1]

    span1 = y_mov[p1] - y_mov[v1]
    span2 = y_mov[p2] - y_mov[v2]
    if span1 <= 0 or span2 <= 0 or span2 >= span1:
        raise ValueError("Amplitudes pico-vale nao decaem como esperado de "
                          "uma oscilacao amortecida — revise o grafico.")

    T_periodo = t_mov[p2] - t_mov[p1]  # um periodo completo entre picos

    delta = np.log(span1 / span2)
    x = delta / (2 * np.pi)
    zeta = x / np.sqrt(1 + x ** 2)
    omega_d = 2 * np.pi / T_periodo
    omega_n = omega_d / np.sqrt(1 - zeta ** 2)

    return zeta, omega_n, L, T_periodo


def ganhos_zn_metodo1(K, T, L):
    """Tabela de Ziegler-Nichols (Metodo 1), slides 125-127:
    PID: Kp = 1.2*T/(K*L), Ti = 2L, Td = 0.5L
    """
    Kp = 1.2 * T / (K * L)
    Ti = 2 * L
    Td = 0.5 * L
    Ki = Kp / Ti
    Kd = Kp * Td
    return Kp, Ki, Kd, Ti, Td


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="arquivo de telemetria (malha aberta)")
    parser.add_argument("--pwm", type=float, required=True,
                         help="PWM aplicado no degrau (0-1000)")
    args = parser.parse_args()

    t, y = carregar_csv(args.csv)
    U = args.pwm

    print(f"Arquivo: {args.csv}  |  Degrau PWM = {U}  |  {len(t)} amostras, {t[-1]:.2f}s\n")

    K_tan, T_tan, L_tan, y_inf = metodo_tangente(t, y, U)
    print("== Metodo da tangente ==")
    print(f"  y(inf) = {y_inf:.3f} graus")
    print(f"  K = {K_tan:.5f} graus/PWM")
    print(f"  L = {L_tan:.4f} s")
    print(f"  T = {T_tan:.4f} s")

    K_mq, T_mq, L_mq = metodo_minimos_quadrados(t, y, U)
    print("\n== Metodo dos minimos quadrados ==")
    print(f"  K = {K_mq:.5f} graus/PWM")
    print(f"  L = {L_mq:.4f} s")
    print(f"  T = {T_mq:.4f} s")

    diff_K = abs(K_tan - K_mq) / K_mq * 100
    diff_T = abs(T_tan - T_mq) / T_mq * 100
    diff_L = abs(L_tan - L_mq) / max(L_mq, 1e-6) * 100
    print(f"\n  Divergencia tangente vs MQ: K={diff_K:.1f}%  T={diff_T:.1f}%  L={diff_L:.1f}%")
    if max(diff_K, diff_T, diff_L) > 30:
        print("  ATENCAO: os dois metodos divergem bastante — inspecione o "
              "grafico, pode haver ruido ou o degrau nao ter atingido regime.")

    print("\n== Ganhos Ziegler-Nichols Metodo 1 (usando K,T,L dos minimos quadrados) ==")
    Kp, Ki, Kd, Ti, Td = ganhos_zn_metodo1(K_mq, T_mq, L_mq)
    print(f"  Kp = {Kp:.4f}")
    print(f"  Ki = {Ki:.4f}   (Ti = {Ti:.4f}s)")
    print(f"  Kd = {Kd:.4f}   (Td = {Td:.4f}s)")

    # ---- deteccao de deriva lenta (segunda dinamica) na cauda ----
    n_cauda = max(10, len(y) // 10)
    metade1 = y[-n_cauda:-n_cauda // 2].mean()
    metade2 = y[-n_cauda // 2:].mean()
    deriva = metade2 - metade1
    if abs(deriva) > 0.3:
        print(f"\n  AVISO: a cauda do sinal ainda varia {deriva:+.2f} graus "
              "entre a primeira e a segunda metade do trecho final — o "
              "sinal pode nao ter atingido o regime permanente de verdade "
              "(dinamica lenta secundaria sobreposta a oscilacao rapida). "
              "K acima pode estar subestimado/superestimado.")

    print("\n== Metodo 2a ordem subamortecida (decremento logaritmico, slides 73-75) ==")
    try:
        zeta, omega_n, L_2a, T_periodo = metodo_2a_ordem_decremento_log(t, y, U)
        print(f"  zeta  = {zeta:.4f}")
        print(f"  omega_n = {omega_n:.4f} rad/s")
        print(f"  L (atraso) = {L_2a:.4f} s")
        print(f"  periodo de oscilacao = {T_periodo:.4f} s")
        print(f"  (K usado para os ganhos abaixo: {K_tan:.5f} graus/PWM, "
              "da media da cauda — ver aviso de deriva acima se houver)")
    except ValueError as e:
        print(f"  Nao aplicavel: {e}")

    # ---- grafico de validacao ----
    t_fit = np.linspace(0, t[-1], 500)
    y_fit_tan = np.where(t_fit < L_tan, 0.0,
                          U * K_tan * (1 - np.exp(-(t_fit - L_tan) / T_tan)))
    y_fit_mq = np.where(t_fit < L_mq, 0.0,
                         U * K_mq * (1 - np.exp(-(t_fit - L_mq) / T_mq)))

    plt.figure(figsize=(10, 6))
    plt.plot(t, y, 'k.', markersize=3, label="Dados reais (bancada)")
    plt.plot(t_fit, y_fit_tan, 'b--', label=f"Ajuste tangente (K={K_tan:.3f}, T={T_tan:.2f}, L={L_tan:.2f})")
    plt.plot(t_fit, y_fit_mq, 'r-', label=f"Ajuste minimos quadrados (K={K_mq:.3f}, T={T_mq:.2f}, L={L_mq:.2f})")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Angulo (graus)")
    plt.title(f"Identificacao da planta — degrau PWM={U}")
    plt.legend()
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.show()


if __name__ == "__main__":
    main()
