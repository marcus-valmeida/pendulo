"""Simula o degrau de partida (repouso -> alvo) replicando a logica discreta
de pid.c: mesmas formulas, anti-windup por clamp e estagios de seguranca.
Aqui a saturacao de PWM entra, o que analise_margem_multiponto.py nao cobre.

    python3 simula_partida_degrau.py --alvo 60 --ponto 60
"""

import argparse
import numpy as np
import control as ct
import matplotlib.pyplot as plt

# K/zeta/wn vem de identifica_zn.py (fonte unica). O L aqui e o do ENSAIO,
# nao o de projeto: inclui o tempo ate a fonte ser ligada, e e ele que
# reproduz o pico de 117 graus medido na bancada. Ver docs/projeto_pid.md 2.3.
from identifica_zn import PONTOS as _P
PONTOS = {
    30: dict(**_P[30], L=0.70),
    45: dict(**_P[45], L=1.00),
    60: dict(**_P[60], L=0.85),
    75: dict(**_P[75], L=1.07),
    90: dict(**_P[90], L=1.53),
}

# Espelho de src/pid.c — se mudar la, mude aqui.
KP, KI, KD, N = 0.8138, 2.3589, 0.1870, 50.0
DT = 0.020
PWM_MIN, PWM_MAX = 0.0, 1000.0
INTEGRAL_MAX = 620.0
LIM_REDUZIR1, LIM_REDUZIR2, LIM_DESLIGAR, LIM_TRAVAR = 130.0, 150.0, 160.0, 170.0
FATOR_REDUCAO1, FATOR_REDUCAO2 = 0.90, 0.80


def saturar(v, lo, hi):
    return max(lo, min(hi, v))


# Pontos com zeta/wn resolvidos (15 grau fica de fora — zeta/wn nao
# resolvidos na bancada, ver docs secao 6.2). K e L sao interpolados
# separadamente pois estao definidos em todos os pontos, incluindo 15.
_ANG_ZW = sorted(a for a in PONTOS if a != 15)
_ANG_KL = sorted(PONTOS)


def interpolar_planta(theta_abs):
    """Interpola zeta, wn, K e L entre os pontos da bancada (constante fora
    da faixa medida). E o modelo da PLANTA, nao do controlador: o PID
    continua com ganho unico e fixo."""
    ta = saturar(theta_abs, _ANG_ZW[0], _ANG_ZW[-1])
    zeta = np.interp(ta, _ANG_ZW, [PONTOS[a]["zeta"] for a in _ANG_ZW])
    wn = np.interp(ta, _ANG_ZW, [PONTOS[a]["wn"] for a in _ANG_ZW])
    tk = saturar(theta_abs, _ANG_KL[0], _ANG_KL[-1])
    K = np.interp(tk, _ANG_KL, [PONTOS[a]["K"] for a in _ANG_KL])
    L = np.interp(tk, _ANG_KL, [PONTOS[a]["L"] for a in _ANG_KL])
    return zeta, wn, K, L


def planta_ode(theta, dtheta, pwm, zeta, wn, K):
    """theta'' = K*wn^2*pwm - wn^2*theta - 2*zeta*wn*dtheta"""
    d2theta = K * wn ** 2 * pwm - wn ** 2 * theta - 2 * zeta * wn * dtheta
    return dtheta, d2theta


def rk4_passo(theta, dtheta, pwm, zeta, wn, K, h):
    k1 = planta_ode(theta, dtheta, pwm, zeta, wn, K)
    k2 = planta_ode(theta + h / 2 * k1[0], dtheta + h / 2 * k1[1], pwm, zeta, wn, K)
    k3 = planta_ode(theta + h / 2 * k2[0], dtheta + h / 2 * k2[1], pwm, zeta, wn, K)
    k4 = planta_ode(theta + h * k3[0], dtheta + h * k3[1], pwm, zeta, wn, K)
    theta_novo = theta + h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
    dtheta_novo = dtheta + h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
    return theta_novo, dtheta_novo


def simular(ponto, alvo, integral_max=INTEGRAL_MAX, t_final=15.0,
            planta_variavel=False, n_sub=10):
    p = PONTOS[ponto]

    if not planta_variavel:
        Gc = ct.tf([p["K"] * p["wn"] ** 2], [1, 2 * p["zeta"] * p["wn"], p["wn"] ** 2])
        Gd = ct.c2d(Gc, DT, method="zoh")
        A, B, C, D = ct.ssdata(Gd)
        x = np.zeros((A.shape[0], 1))
        atraso_amostras = max(1, round(p["L"] / DT))
        buffer_pwm = [0.0] * atraso_amostras
    else:
        theta_pl, dtheta_pl = 0.0, 0.0
        pwm_historico = []

    n = int(t_final / DT)
    t_hist, ang_hist, pwm_hist, integ_hist = [], [], [], []

    integral = 0.0
    angulo_anterior = 0.0
    deriv_estado = 0.0
    travado = False
    primeiro_ciclo = True

    for k in range(n):
        if not planta_variavel:
            angulo_atual = float((C @ x + D * 0.0).item())
        else:
            angulo_atual = theta_pl
        ang_abs = abs(angulo_atual)
        t_hist.append(k * DT)
        ang_hist.append(angulo_atual)

        if travado:
            saida = 0.0
        elif ang_abs >= LIM_TRAVAR:
            saida = 0.0
            integral = 0.0
            travado = True
        elif ang_abs >= LIM_DESLIGAR:
            saida = 0.0
            integral = 0.0
        else:
            erro = alvo - angulo_atual
            P = KP * erro

            D = 0.0
            if not primeiro_ciclo:
                deriv_angulo = (angulo_atual - angulo_anterior) / DT
                deriv_estado = deriv_estado + N * DT * (deriv_angulo - deriv_estado)
                D = -KD * deriv_estado
            angulo_anterior = angulo_atual
            primeiro_ciclo = False

            # Mesma ordem de pid.c: clamp dinamico e depois o teto absoluto.
            integral = integral + KI * erro * DT
            integral = saturar(integral, PWM_MIN - (P + D), PWM_MAX - (P + D))
            integral = saturar(integral, -integral_max, integral_max)

            saida = saturar(P + integral + D, PWM_MIN, PWM_MAX)

            if ang_abs >= LIM_REDUZIR1:
                saida *= FATOR_REDUCAO1
            if ang_abs >= LIM_REDUZIR2:
                saida *= FATOR_REDUCAO2

        pwm_hist.append(saida)
        integ_hist.append(integral)

        if not planta_variavel:
            buffer_pwm.append(saida)
            pwm_aplicado = buffer_pwm.pop(0)
            x = A @ x + B * pwm_aplicado
        else:
            pwm_historico.append(saida)
            zeta_th, wn_th, K_th, L_th = interpolar_planta(abs(theta_pl))
            atraso_amostras = max(1, round(L_th / DT))
            idx_atrasado = k - atraso_amostras
            pwm_aplicado = pwm_historico[idx_atrasado] if idx_atrasado >= 0 else 0.0
            h = DT / n_sub
            for _ in range(n_sub):
                theta_pl, dtheta_pl = rk4_passo(theta_pl, dtheta_pl, pwm_aplicado,
                                                 zeta_th, wn_th, K_th, h)

    return dict(t=np.array(t_hist), ang=np.array(ang_hist),
                pwm=np.array(pwm_hist), integ=np.array(integ_hist),
                ponto=ponto, alvo=alvo, integral_max=integral_max)


def resumo(r):
    pico = r["ang"].max()
    t_pico = r["t"][np.argmax(r["ang"])]
    travou = pico >= LIM_TRAVAR
    print(f"  ponto={r['ponto']}°  INTEGRAL_MAX={r['integral_max']:.0f}"
          f"  -> pico={pico:.1f}° em t={t_pico:.2f}s"
          + ("  *** TRAVA DE SEGURANCA ACIONADA (>=170) ***" if travou else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--alvo", type=float, default=60.0)
    ap.add_argument("--ponto", type=int, choices=list(PONTOS.keys()), default=60,
                     help="ponto identificado (graus) usado como modelo de planta")
    ap.add_argument("--integral-max", type=float, default=INTEGRAL_MAX,
                    help=f"teto do integral (default: {INTEGRAL_MAX:.0f}, como em pid.c)")
    ap.add_argument("--comparar", type=float,
                    help="simula tambem com outro INTEGRAL_MAX e sobrepoe as curvas")
    ap.add_argument("--t-final", type=float, default=15.0)
    ap.add_argument("--planta-variavel", action="store_true",
                     help="planta muda de zeta/wn/K/L com o angulo atual (interpolado "
                          "dos pontos da bancada), em vez do modelo fixo de --ponto. "
                          "Simula a subida real do repouso ate o alvo cruzando "
                          "varias regioes de dinamica diferente.")
    args = ap.parse_args()

    if args.planta_variavel:
        print(f"Simulando degrau de partida (repouso -> {args.alvo}°) com PLANTA "
              f"VARIAVEL (zeta/wn/K/L interpolados por angulo, pontos da bancada "
              f"15-90°)\n")
    else:
        print(f"Simulando degrau de partida (repouso -> {args.alvo}°) com o modelo "
              f"FIXO identificado em {args.ponto}° (K={PONTOS[args.ponto]['K']}, "
              f"zeta={PONTOS[args.ponto]['zeta']}, wn={PONTOS[args.ponto]['wn']}, "
              f"L={PONTOS[args.ponto]['L']})\n")

    print("Configuracao de pid.c:")
    r_atual = simular(args.ponto, args.alvo, args.integral_max, args.t_final,
                      planta_variavel=args.planta_variavel)
    resumo(r_atual)

    r_novo = None
    if args.comparar is not None:
        print(f"\nComparacao com INTEGRAL_MAX={args.comparar:.0f}:")
        r_novo = simular(args.ponto, args.alvo, args.comparar, args.t_final,
                         planta_variavel=args.planta_variavel)
        resumo(r_novo)

    fig, axs = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
    axs[0].plot(r_atual["t"], r_atual["ang"],
                label=f"pid.c (INTEGRAL_MAX={args.integral_max:.0f})", color="tab:red")
    if r_novo:
        axs[0].plot(r_novo["t"], r_novo["ang"],
                    label=f"INTEGRAL_MAX={args.comparar:.0f}", color="tab:blue")
    axs[0].axhline(args.alvo, color="gray", linestyle="--", label="Alvo")
    axs[0].axhline(LIM_TRAVAR, color="black", linestyle=":", label="LIM_TRAVAR (170°)")
    axs[0].set_ylabel("Angulo (graus)")
    axs[0].set_title(f"Degrau de partida simulado: repouso -> {args.alvo}° "
                      f"(modelo do ponto {args.ponto}°)")
    axs[0].legend()
    axs[0].grid(True, linestyle=":", alpha=0.6)

    axs[1].plot(r_atual["t"], r_atual["pwm"], label="PWM", color="tab:red")
    axs[1].plot(r_atual["t"], r_atual["integ"], label="Integral",
                color="tab:red", linestyle="--", alpha=0.6)
    if r_novo:
        axs[1].plot(r_novo["t"], r_novo["pwm"], label="PWM (comparacao)", color="tab:blue")
        axs[1].plot(r_novo["t"], r_novo["integ"], label="Integral (comparacao)",
                    color="tab:blue", linestyle="--", alpha=0.6)
    axs[1].set_xlabel("Tempo (s)")
    axs[1].set_ylabel("PWM / Integral")
    axs[1].legend()
    axs[1].grid(True, linestyle=":", alpha=0.6)

    plt.tight_layout()
    out = f"simulacao_partida_degrau_{args.ponto}graus.png"
    plt.savefig(out, dpi=120)
    print(f"\nGrafico salvo em {out}")


if __name__ == "__main__":
    main()
