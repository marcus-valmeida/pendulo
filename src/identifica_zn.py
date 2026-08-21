"""Identificacao da planta e projeto do PID por Ziegler-Nichols.

    python3 identifica_zn.py identificar <csv> --pwm 450
    python3 identifica_zn.py projetar
"""

import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

# Pontos identificados na bancada por decremento logaritmico.
# K em graus/PWM, wn em rad/s. Ver docs/projeto_pid.md, secao 2.
PONTOS = {
    30: dict(K=0.136, zeta=0.130, wn=8.38),
    45: dict(K=0.129, zeta=0.146, wn=8.14),
    60: dict(K=0.139, zeta=0.171, wn=7.70),
    75: dict(K=0.156, zeta=0.144, wn=6.32),
    90: dict(K=0.198, zeta=0.165, wn=5.83),
}

# Atraso de laco: ZOH da amostragem de 50 Hz + atraso de empuxo da helice.
# NAO usar o L dos CSVs (0,7 a 1,5 s) — aquilo e o tempo ate ligar a fonte.
L_LACO = 0.11

# (nome, Kp/Ku, Ti/Tu, Td/Tu)
LINHAS_ZN = [
    ("ZN classica",       0.60, 0.50, 0.125),
    ("ZN some overshoot", 0.33, 0.50, 1.0 / 3.0),
    ("ZN no overshoot",   0.20, 0.50, 1.0 / 3.0),
    ("Tyreus-Luyben",     0.45, 2.20, 0.159),
]


def carregar_csv(caminho):
    tempos, angulos = [], []
    with open(caminho, newline='') as f:
        leitor = csv.reader(f)
        next(leitor)
        for linha in leitor:
            if len(linha) < 2:
                continue
            tempos.append(float(linha[0]))
            angulos.append(float(linha[1]))
    tempos = np.array(tempos)
    tempos = (tempos - tempos[0]) / 1000.0  # ms -> s, relativo ao inicio
    return tempos, np.array(angulos)


# =============================== IDENTIFICACAO ==============================

def metodo_tangente(t, y, U):
    """Metodo da tangente: K, L e T a partir do ponto de inflexao."""
    y_inf = np.mean(y[-max(5, len(y) // 20):])
    K = y_inf / U

    dy = np.gradient(y, t)
    i_max = np.argmax(dy)
    if dy[i_max] <= 0:
        raise ValueError("Derivada maxima <= 0 — a curva nao parece um degrau.")

    L = max(t[i_max] - y[i_max] / dy[i_max], 0.0)

    alvo_63 = (1 - np.exp(-1)) * y_inf
    T = t[np.argmin(np.abs(y - alvo_63))] - L
    if T <= 0:
        print("  AVISO: T <= 0 — a subida foi mais rapida que a amostragem. "
              "L continua valido; T deve ser lido como 'nao resolvido'.")
    return K, T, L, y_inf


def metodo_minimos_quadrados(t, y, U, n_pontos=25):
    """Regride A(Tf) = U*Tf*K - U*L*K - T*y(Tf) para theta = [K, L*K, T]."""
    A = np.concatenate(([0.0], np.cumsum((y[1:] + y[:-1]) / 2.0 * np.diff(t))))

    # Mistura pontos do transiente e do regime: so em regime as colunas ficam
    # quase colineares e a regressao sai mal condicionada.
    idx = np.unique(np.linspace(3, len(t) - 1, n_pontos).astype(int))

    Psi = np.column_stack([U * t[idx], -U * np.ones(len(idx)), -y[idx]])
    theta, *_ = np.linalg.lstsq(Psi, A[idx], rcond=None)
    K, LK, T = theta
    return K, T, LK / K


def metodo_2a_ordem_decremento_log(t, y, U, y0=None, prominencia=1.0):
    """2a ordem subamortecida via decremento logaritmico entre picos e vales.

    Usa a diferenca pico-vale, nao o valor final, o que torna o metodo imune
    a deriva lenta de regime — e por isso e o metodo usado no projeto.
    """
    y0 = y[0] if y0 is None else y0
    limiar = max(0.4, 0.02 * (np.max(y) - y0))
    idx_inicio = int(np.argmax(np.abs(y - y0) > limiar))
    if idx_inicio == 0:
        raise ValueError("Nao foi possivel detectar o inicio do movimento.")
    L = (t[idx_inicio - 1] + t[idx_inicio]) / 2.0

    t_mov, y_mov = t[idx_inicio:], y[idx_inicio:]
    picos, _ = find_peaks(y_mov, prominence=prominencia)
    vales, _ = find_peaks(-y_mov, prominence=prominencia)
    if len(picos) < 2 or len(vales) < 2:
        raise ValueError("Menos de 2 picos/vales — sem oscilacao visivel.")

    span1 = y_mov[picos[0]] - y_mov[vales[0]]
    span2 = y_mov[picos[1]] - y_mov[vales[1]]
    if span1 <= 0 or span2 <= 0 or span2 >= span1:
        raise ValueError("As amplitudes nao decaem — revise o grafico.")

    T_periodo = t_mov[picos[1]] - t_mov[picos[0]]
    x = np.log(span1 / span2) / (2 * np.pi)
    zeta = x / np.sqrt(1 + x ** 2)
    omega_n = (2 * np.pi / T_periodo) / np.sqrt(1 - zeta ** 2)
    return zeta, omega_n, L, T_periodo


def ganhos_zn_metodo1(K, T, L):
    """Tabela ZN Metodo 1: Kp = 1.2*T/(K*L), Ti = 2L, Td = 0.5L."""
    Kp = 1.2 * T / (K * L)
    Ti, Td = 2 * L, 0.5 * L
    return Kp, Kp / Ti, Kp * Td, Ti, Td


# ================================ ZN METODO 2 ===============================

def ganho_critico(K, zeta, wn, L):
    """Ku = 1/|G(jwu)| na frequencia wu onde a fase de G cruza -180 graus.

    G(s) = K*wn^2/(s^2 + 2*zeta*wn*s + wn^2) * e^(-L*s)
    """
    def fase(w):
        return -np.arctan2(2 * zeta * wn * w, wn ** 2 - w ** 2) - w * L

    def mag(w):
        return K * wn ** 2 / np.hypot(wn ** 2 - w ** 2, 2 * zeta * wn * w)

    w_lo, w_hi, w = 1e-3, 0.0, 1e-3
    while w < 200:
        if fase(w) <= -np.pi:
            w_hi = w
            break
        w_lo, w = w, w * 1.01
    if w_hi == 0.0:
        raise ValueError("A fase nunca cruza -180 graus.")

    for _ in range(200):
        wm = 0.5 * (w_lo + w_hi)
        w_lo, w_hi = (wm, w_hi) if fase(wm) > -np.pi else (w_lo, wm)
    wu = 0.5 * (w_lo + w_hi)
    return 1.0 / mag(wu), 2 * np.pi / wu, wu


def ganhos_da_linha(linha, Ku, Tu):
    _, fKp, fTi, fTd = linha
    Kp, Ti, Td = fKp * Ku, fTi * Tu, fTd * Tu
    return Kp, Kp / Ti, Kp * Td, Ti, Td


# =================================== CLI ====================================

def cmd_identificar(args):
    t, y = carregar_csv(args.csv)
    U = args.pwm
    print(f"{args.csv} | degrau PWM={U} | {len(t)} amostras, {t[-1]:.2f}s\n")

    K_tan, T_tan, L_tan, y_inf = metodo_tangente(t, y, U)
    print("== Metodo da tangente ==")
    print(f"  y(inf) = {y_inf:.3f} graus")
    print(f"  K = {K_tan:.5f} graus/PWM   L = {L_tan:.4f}s   T = {T_tan:.4f}s")

    K_mq, T_mq, L_mq = metodo_minimos_quadrados(t, y, U)
    print("\n== Metodo dos minimos quadrados ==")
    print(f"  K = {K_mq:.5f} graus/PWM   L = {L_mq:.4f}s   T = {T_mq:.4f}s")

    print("\n== Ganhos ZN Metodo 1 (a partir dos minimos quadrados) ==")
    Kp, Ki, Kd, Ti, Td = ganhos_zn_metodo1(K_mq, T_mq, L_mq)
    print(f"  Kp = {Kp:.4f}   Ki = {Ki:.4f} (Ti={Ti:.4f}s)   Kd = {Kd:.4f} (Td={Td:.4f}s)")
    if Kp < 0 or L_mq < 0:
        print("  L NEGATIVO -> Metodo 1 fora da faixa de validade nesta planta.")
        print("  Use os parametros de 2a ordem abaixo e o subcomando 'projetar'.")

    print("\n== 2a ordem subamortecida (decremento logaritmico) ==")
    try:
        zeta, omega_n, L_2a, T_periodo = metodo_2a_ordem_decremento_log(t, y, U)
        print(f"  zeta = {zeta:.4f}   omega_n = {omega_n:.4f} rad/s")
        print(f"  periodo = {T_periodo:.4f}s   inicio do movimento = {L_2a:.4f}s")
        print(f"  K (media da cauda) = {K_tan:.5f} graus/PWM")
    except ValueError as e:
        print(f"  Nao aplicavel: {e}")

    if args.sem_grafico:
        return

    t_fit = np.linspace(0, t[-1], 500)
    plt.figure(figsize=(10, 6))
    plt.plot(t, y, 'k.', markersize=3, label="Dados reais (bancada)")
    plt.plot(t_fit, np.where(t_fit < L_tan, 0.0,
             U * K_tan * (1 - np.exp(-(t_fit - L_tan) / T_tan))),
             'b--', label=f"Tangente (K={K_tan:.3f} T={T_tan:.2f} L={L_tan:.2f})")
    plt.plot(t_fit, np.where(t_fit < L_mq, 0.0,
             U * K_mq * (1 - np.exp(-(t_fit - L_mq) / T_mq))),
             'r-', label=f"Min. quadrados (K={K_mq:.3f} T={T_mq:.2f} L={L_mq:.2f})")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Angulo (graus)")
    plt.title(f"Identificacao da planta — degrau PWM={U}")
    plt.legend()
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.show()


def cmd_projetar(args):
    L = args.atraso
    print(f"ZIEGLER-NICHOLS METODO 2  (atraso de laco L = {L:.3f}s)\n")
    print(f"{'ang':>5} {'Ku':>8} {'Tu (s)':>9} {'wu (rad/s)':>11}   modelo")

    kus = {}
    for a in sorted(PONTOS):
        p = PONTOS[a]
        Ku, Tu, wu = ganho_critico(p["K"], p["zeta"], p["wn"], L)
        kus[a] = (Ku, Tu)
        print(f"{a:>5} {Ku:>8.2f} {Tu:>9.3f} {wu:>11.3f}   "
              f"K={p['K']:.3f} zeta={p['zeta']:.3f} wn={p['wn']:.2f}")

    ang_proj = min(kus, key=lambda a: kus[a][0])
    Ku_proj, Tu_proj = kus[ang_proj]
    print(f"\n  PIOR CASO (menor Ku): {ang_proj} graus -> Ku={Ku_proj:.4f}, Tu={Tu_proj:.4f}s")
    print("  Projetar aqui cobre toda a faixa com um unico jogo de ganhos.\n")

    print(f"{'linha':<20} {'Kp':>8} {'Ki':>9} {'Kd':>8} {'Ti(s)':>8} {'Td(s)':>8}")
    for linha in LINHAS_ZN:
        Kp, Ki, Kd, Ti, Td = ganhos_da_linha(linha, Ku_proj, Tu_proj)
        marca = "  <- em uso no pid.c" if linha[0] == "ZN some overshoot" else ""
        print(f"{linha[0]:<20} {Kp:>8.4f} {Ki:>9.4f} {Kd:>8.4f} "
              f"{Ti:>8.3f} {Td:>8.3f}{marca}")

    print("\nPara validar um candidato em malha fechada: simula_partida_degrau.py")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p1 = sub.add_parser("identificar", help="identifica a planta de um CSV de malha aberta")
    p1.add_argument("csv", help="telemetria do ensaio de degrau")
    p1.add_argument("--pwm", type=float, required=True, help="PWM aplicado (0-1000)")
    p1.add_argument("--sem-grafico", action="store_true", help="nao abre o grafico")
    p1.set_defaults(func=cmd_identificar)

    p2 = sub.add_parser("projetar", help="Ku/Tu e ganhos por ZN Metodo 2")
    p2.add_argument("--atraso", type=float, default=L_LACO,
                    help=f"atraso de laco em segundos (default: {L_LACO})")
    p2.set_defaults(func=cmd_projetar)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
