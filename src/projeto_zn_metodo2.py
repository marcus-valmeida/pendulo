"""
Projeto do PID por ZIEGLER-NICHOLS METODO 2 (ganho critico / oscilacao
sustentada), com validacao em malha fechada nao-linear.

Por que METODO 2 e nao METODO 1:
--------------------------------
O metodo 1 (curva de reacao / tangente) exige uma resposta ao degrau em S,
monotonica, de um processo essencialmente de 1a ordem + atraso. A planta do
aeropendulo NAO e assim: a resposta ao degrau em malha aberta e claramente
subamortecida (zeta ~ 0.13-0.17, periodo ~0.8-1.0 s). Rodando
identifica_zn.py o ajuste por minimos quadrados devolve L NEGATIVO em todos
os pontos (-1.16 s a 30 graus, -0.15 s a 60 e 75), o que produz ganhos ZN
sem sentido fisico (Kp negativo). Ou seja: o metodo 1 esta fora de faixa de
validade para esta planta. O metodo 2 nao tem essa restricao — ele so precisa
do ponto onde a fase do processo cruza -180 graus, que existe e e bem
definido aqui.

O que este script faz:
    1. calcula Ku e Tu ANALITICAMENTE em cada ponto de operacao, a partir do
       modelo identificado na bancada (2a ordem + atraso de transporte);
    2. aplica as linhas da tabela de Ziegler-Nichols (classica, "some
       overshoot" e "no overshoot") sobre esses Ku/Tu;
    3. simula cada conjunto de ganhos em malha fechada nao-linear (a mesma
       discretizacao de pid.c, com saturacao de PWM e atraso real), no degrau
       de partida e num disturbio tipo "peteleco", em 30/45/60/75/90 graus;
    4. imprime overshoot, tempo de acomodacao e recuperacao de disturbio para
       cada candidato.

Uso:
    python3 projeto_zn_metodo2.py                  # tabela ZN + varredura
    python3 projeto_zn_metodo2.py --b 0.3 --rampa 40
"""

import argparse
import numpy as np

DT = 0.020
PWM_MIN, PWM_MAX = 0.0, 1000.0
N_FILTRO = 10.0

# Pontos identificados na bancada (docs/diagnostico_oscilacao_pid.md, 6.2/6.3)
PONTOS = {
    30: dict(K=0.136, zeta=0.130, wn=8.38, L=0.70),
    45: dict(K=0.129, zeta=0.146, wn=8.14, L=1.00),
    60: dict(K=0.139, zeta=0.171, wn=7.70, L=0.85),
    75: dict(K=0.156, zeta=0.144, wn=6.32, L=1.07),
    90: dict(K=0.198, zeta=0.165, wn=5.83, L=1.53),
}


def saturar(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------- Ku e Tu --
def ganho_critico(K, zeta, wn, L):
    """Acha wu tal que fase(G(jwu)) = -180 graus e devolve Ku = 1/|G(jwu)|.

    G(s) = K*wn^2/(s^2 + 2*zeta*wn*s + wn^2) * e^(-L*s)
    fase = -atan2(2*zeta*wn*w, wn^2 - w^2) - w*L
    """
    def fase(w):
        return -np.arctan2(2 * zeta * wn * w, wn ** 2 - w ** 2) - w * L

    def mag(w):
        num = K * wn ** 2
        den = np.hypot(wn ** 2 - w ** 2, 2 * zeta * wn * w)
        return num / den

    # busca a primeira raiz de fase(w) = -pi por bissecao
    w_lo, w_hi = 1e-3, 0.0
    w = 1e-3
    while w < 200:
        if fase(w) <= -np.pi:
            w_hi = w
            break
        w_lo = w
        w *= 1.01
    if w_hi == 0.0:
        raise ValueError("fase nunca cruza -180 graus")
    for _ in range(200):
        wm = 0.5 * (w_lo + w_hi)
        if fase(wm) > -np.pi:
            w_lo = wm
        else:
            w_hi = wm
    wu = 0.5 * (w_lo + w_hi)
    Ku = 1.0 / mag(wu)
    Tu = 2 * np.pi / wu
    return Ku, Tu, wu


# ------------------------------------------------------- tabelas de ZN -----
# (nome, Kp/Ku, Ti/Tu, Td/Tu)
LINHAS_ZN = [
    ("ZN classica",      0.60,  0.50,   0.125),
    ("ZN some overshoot", 0.33, 0.50,   1.0 / 3.0),
    ("ZN no overshoot",   0.20, 0.50,   1.0 / 3.0),
    ("Tyreus-Luyben",     0.45, 2.20,   0.159),
]


def ganhos_da_linha(linha, Ku, Tu):
    _, fKp, fTi, fTd = linha
    Kp = fKp * Ku
    Ti = fTi * Tu
    Td = fTd * Tu
    return Kp, Kp / Ti, Kp * Td, Ti, Td


# ------------------------------------------------- simulacao nao-linear ----
def planta_ode(theta, dtheta, pwm, zeta, wn, K):
    return dtheta, K * wn ** 2 * pwm - wn ** 2 * theta - 2 * zeta * wn * dtheta


def rk4(theta, dtheta, pwm, zeta, wn, K, h):
    k1 = planta_ode(theta, dtheta, pwm, zeta, wn, K)
    k2 = planta_ode(theta + h / 2 * k1[0], dtheta + h / 2 * k1[1], pwm, zeta, wn, K)
    k3 = planta_ode(theta + h / 2 * k2[0], dtheta + h / 2 * k2[1], pwm, zeta, wn, K)
    k4 = planta_ode(theta + h * k3[0], dtheta + h * k3[1], pwm, zeta, wn, K)
    return (theta + h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
            dtheta + h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]))


def simular(ponto, alvo, Kp, Ki, Kd, b=1.0, rampa=0.0, t_final=30.0,
            t_peteleco=None, peteleco=40.0, n_sub=10):
    """Malha fechada com a mesma logica discreta de pid.c.

    b  = peso do setpoint no termo proporcional (forma ISA: P = Kp*(b*r - y)).
         b=1 -> PID classico; b=0 -> I-PD. O termo D ja e sobre -y (c=0).
    rampa = graus/s de variacao maxima do setpoint interno (0 = degrau puro).
    peteleco = impulso de velocidade angular (graus/s) aplicado em t_peteleco.
    """
    p = PONTOS[ponto]
    zeta, wn, K, L = p["zeta"], p["wn"], p["K"], p["L"]
    atraso = max(1, round(L / DT))

    theta, dtheta = 0.0, 0.0
    hist_pwm = []
    integral = 0.0
    ang_ant = 0.0
    deriv_estado = 0.0
    primeiro = True
    r_interno = 0.0

    n = int(t_final / DT)
    t_h, ang_h, pwm_h = np.zeros(n), np.zeros(n), np.zeros(n)

    for k in range(n):
        t = k * DT
        ang = theta
        t_h[k], ang_h[k] = t, ang

        # rampa de setpoint (0 = degrau puro)
        if rampa > 0:
            passo = rampa * DT
            r_interno += saturar(alvo - r_interno, -passo, passo)
        else:
            r_interno = alvo

        erro = r_interno - ang
        P = Kp * (b * r_interno - ang)

        I_tent = integral + Ki * erro * DT

        D = 0.0
        if not primeiro:
            deriv = (ang - ang_ant) / DT
            deriv_estado += N_FILTRO * DT * (deriv - deriv_estado)
            D = -Kd * deriv_estado
        ang_ant = ang
        primeiro = False

        pre_sat = P + I_tent + D
        saida = saturar(pre_sat, PWM_MIN, PWM_MAX)
        if PWM_MIN < pre_sat < PWM_MAX:
            integral = I_tent

        pwm_h[k] = saida
        hist_pwm.append(saida)

        idx = k - atraso
        pwm_aplicado = hist_pwm[idx] if idx >= 0 else 0.0

        h = DT / n_sub
        for _ in range(n_sub):
            theta, dtheta = rk4(theta, dtheta, pwm_aplicado, zeta, wn, K, h)

        if t_peteleco is not None and abs(t - t_peteleco) < DT / 2:
            dtheta += peteleco

    return t_h, ang_h, pwm_h


def metricas(t, ang, alvo, t_ini=0.0):
    m = t >= t_ini
    tt, aa = t[m], ang[m]
    pico = aa.max()
    over = (pico - alvo) / alvo * 100
    # acomodacao em +-5% do alvo
    faixa = 0.05 * alvo
    idx_fora = np.where(np.abs(aa - alvo) > faixa)[0]
    t_acom = tt[idx_fora[-1]] - t_ini if len(idx_fora) and idx_fora[-1] < len(tt) - 1 else np.inf
    # ripple no ultimo terco
    cauda = aa[int(len(aa) * 0.66):]
    return dict(pico=pico, over=over, t_acom=t_acom,
                erro_final=alvo - cauda.mean(), ripple=cauda.max() - cauda.min())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--b", type=float, default=1.0, help="peso do setpoint em P")
    ap.add_argument("--rampa", type=float, default=0.0, help="graus/s da rampa de setpoint")
    ap.add_argument("--ganhos", nargs=3, type=float, metavar=("KP", "KI", "KD"),
                    help="testa um trio de ganhos direto, sem a tabela ZN")
    args = ap.parse_args()

    print("=" * 78)
    print("1) GANHO CRITICO (Ku) E PERIODO CRITICO (Tu) POR PONTO DE OPERACAO")
    print("=" * 78)
    print(f"{'ang':>5} {'Ku':>10} {'Tu (s)':>9} {'wu (rad/s)':>11}   modelo")
    kus = {}
    for a in sorted(PONTOS):
        p = PONTOS[a]
        Ku, Tu, wu = ganho_critico(p["K"], p["zeta"], p["wn"], p["L"])
        kus[a] = (Ku, Tu)
        print(f"{a:>5} {Ku:>10.2f} {Tu:>9.3f} {wu:>11.3f}   "
              f"K={p['K']:.3f} zeta={p['zeta']:.3f} wn={p['wn']:.2f} L={p['L']:.2f}")

    Ku_proj = min(v[0] for v in kus.values())
    ang_proj = min(kus, key=lambda a: kus[a][0])
    Tu_proj = kus[ang_proj][1]
    print(f"\n  -> PIOR CASO (menor Ku): {ang_proj} graus  ->  Ku={Ku_proj:.2f}, Tu={Tu_proj:.3f}s")
    print("     (projetar no pior caso garante estabilidade em toda a faixa "
          "com UM unico jogo de ganhos)")

    if args.ganhos:
        candidatos = [("manual", *args.ganhos, None, None)]
    else:
        candidatos = []
        for linha in LINHAS_ZN:
            Kp, Ki, Kd, Ti, Td = ganhos_da_linha(linha, Ku_proj, Tu_proj)
            candidatos.append((linha[0], Kp, Ki, Kd, Ti, Td))

    print()
    print("=" * 78)
    print("2) GANHOS PELA TABELA DE ZIEGLER-NICHOLS (sobre o pior caso)")
    print("=" * 78)
    print(f"{'linha':<20} {'Kp':>8} {'Ki':>9} {'Kd':>8} {'Ti(s)':>8} {'Td(s)':>8}")
    for c in candidatos:
        nome, Kp, Ki, Kd = c[0], c[1], c[2], c[3]
        Ti = c[4] if c[4] else float('nan')
        Td = c[5] if c[5] else float('nan')
        print(f"{nome:<20} {Kp:>8.4f} {Ki:>9.4f} {Kd:>8.4f} {Ti:>8.3f} {Td:>8.3f}")

    print()
    print("=" * 78)
    print(f"3) SIMULACAO EM MALHA FECHADA  (b={args.b}, rampa={args.rampa} graus/s)")
    print("=" * 78)
    for c in candidatos:
        nome, Kp, Ki, Kd = c[0], c[1], c[2], c[3]
        print(f"\n-- {nome}:  Kp={Kp:.4f}  Ki={Ki:.4f}  Kd={Kd:.4f}")
        print(f"   {'alvo':>5} {'pico':>8} {'over%':>8} {'t5%(s)':>8} "
              f"{'err_fim':>8} {'ripple':>8} | {'pico apos peteleco':>18}")
        for alvo in (30, 45, 60, 75, 90):
            t, ang, pwm = simular(alvo, alvo, Kp, Ki, Kd, b=args.b,
                                  rampa=args.rampa, t_final=30.0)
            m = metricas(t, ang, alvo)
            t2, ang2, _ = simular(alvo, alvo, Kp, Ki, Kd, b=args.b,
                                  rampa=args.rampa, t_final=40.0,
                                  t_peteleco=25.0, peteleco=120.0)
            pos = ang2[t2 >= 25.0]
            print(f"   {alvo:>5} {m['pico']:>8.1f} {m['over']:>8.1f} "
                  f"{m['t_acom']:>8.2f} {m['erro_final']:>8.2f} "
                  f"{m['ripple']:>8.2f} | {pos.max():>8.1f} / {pos.min():>8.1f}")


if __name__ == "__main__":
    main()
