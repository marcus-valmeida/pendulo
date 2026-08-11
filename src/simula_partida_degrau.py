"""
Simulacao em malha fechada do DEGRAU DE PARTIDA (repouso -> alvo), replicando
EXATAMENTE a logica discreta de pid.c (mesmas formulas, mesma ordem de
operacoes, mesmo anti-windup por clamp, mesmos estagios de seguranca) contra
o modelo de planta identificado na bancada (docs/diagnostico_oscilacao_pid.md,
secao 6.2 — decremento logaritmico).

Motivacao: no ensaio real com alvo=60 graus, o braco "levou um susto" ao
ligar a fonte, passando de 180 graus antes de estabilizar. Kp e pequeno
(0.0953) e nao explica isso sozinho — a hipotese e windup do integral
combinado com o degrau de setpoint aplicado sem rampa e o atraso de
transporte identificado (L). Este script testa a hipotese em malha fechada
não-linear (com saturacao de PWM e os estagios de seguranca reais), o que
analise_margem_multiponto.py NAO cobre (aquele so avalia margem de fase
linearizada em pequenos sinais).

Planta: G(s) = K*wn^2/(s^2+2*zeta*wn*s+wn^2), discretizada (ZOH, DT=0.02s).
O atraso de transporte L e simulado por um buffer de amostras atrasando o
PWM aplicado a planta (equivalente a um atraso puro, mais fiel no tempo do
que a aproximacao de Pade usada na analise em frequencia).

Uso:
    python3 simula_partida_degrau.py --alvo 60 --ponto 60
    python3 simula_partida_degrau.py --alvo 60 --ponto 60 --integral-max 550 --anti-windup-condicional
"""

import argparse
import numpy as np
import control as ct
import matplotlib.pyplot as plt

# Pontos identificados na bancada (docs/diagnostico_oscilacao_pid.md, 6.2/6.3)
PONTOS = {
    30: dict(K=0.136, zeta=0.130, wn=8.38, L=0.70),
    45: dict(K=0.129, zeta=0.146, wn=8.14, L=1.00),
    60: dict(K=0.139, zeta=0.171, wn=7.70, L=0.85),
    75: dict(K=0.156, zeta=0.144, wn=6.32, L=1.07),
    90: dict(K=0.198, zeta=0.165, wn=5.83, L=1.53),
}

# Ganhos gravados hoje em src/pid.c
KP, KI, KD, N = 0.0953, 2.0912, 0.0524, 15.0
DT = 0.020
PWM_MIN, PWM_MAX = 0.0, 1000.0
LIM_REDUZIR, LIM_DESLIGAR, LIM_TRAVAR = 130.0, 150.0, 170.0
FATOR_REDUCAO = 0.80


def saturar(v, lo, hi):
    return max(lo, min(hi, v))


# Pontos com zeta/wn resolvidos (15 grau fica de fora — zeta/wn nao
# resolvidos na bancada, ver docs secao 6.2). K e L sao interpolados
# separadamente pois estao definidos em todos os pontos, incluindo 15.
_ANG_ZW = sorted(a for a in PONTOS if a != 15)
_ANG_KL = sorted(PONTOS)


def interpolar_planta(theta_abs):
    """Interpola zeta(theta), wn(theta), K(theta), L(theta) a partir dos
    pontos identificados na bancada (mantem constante fora da faixa medida:
    <=30 usa o ponto de 30, >=90 usa o ponto de 90). Isso NAO e escalonamento
    do controlador — e so o modelo (nao-linear, quasi-LPV) usado para
    SIMULAR a planta real, cujos parametros fisicos mudam com o angulo
    (efeito mola gravitacional). O PID em si continua ganho unico e fixo."""
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


def simular(ponto, alvo, integral_max, anti_windup_condicional, t_final=15.0,
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

            I_tentativo = integral + KI * erro * DT
            I_tentativo = saturar(I_tentativo, -integral_max, integral_max)

            D = 0.0
            if not primeiro_ciclo:
                deriv_angulo = (angulo_atual - angulo_anterior) / DT
                deriv_estado = deriv_estado + N * DT * (deriv_angulo - deriv_estado)
                D = -KD * deriv_estado
            angulo_anterior = angulo_atual
            primeiro_ciclo = False

            saida_pre_sat = P + I_tentativo + D
            saida = saturar(saida_pre_sat, PWM_MIN, PWM_MAX)

            # anti-windup condicional (candidato): so integra se a saida
            # antes da saturacao ainda estava dentro dos limites do atuador
            if anti_windup_condicional:
                if PWM_MIN < saida_pre_sat < PWM_MAX:
                    integral = I_tentativo
                # senao: mantem o integral anterior (nao acumula mais)
            else:
                integral = I_tentativo  # comportamento atual do pid.c

            if ang_abs >= LIM_REDUZIR:
                saida *= FATOR_REDUCAO

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
                ponto=ponto, alvo=alvo, integral_max=integral_max,
                anti_windup_condicional=anti_windup_condicional)


def resumo(r):
    pico = r["ang"].max()
    t_pico = r["t"][np.argmax(r["ang"])]
    travou = pico >= LIM_TRAVAR
    print(f"  ponto={r['ponto']}°  INTEGRAL_MAX={r['integral_max']}  "
          f"anti-windup-cond={r['anti_windup_condicional']}  "
          f"-> pico={pico:.1f}° em t={t_pico:.2f}s"
          + ("  *** TRAVA DE SEGURANCA ACIONADA (>=170) ***" if travou else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--alvo", type=float, default=60.0)
    ap.add_argument("--ponto", type=int, choices=list(PONTOS.keys()), default=60,
                     help="ponto identificado (graus) usado como modelo de planta")
    ap.add_argument("--integral-max", type=float, default=900.0)
    ap.add_argument("--anti-windup-condicional", action="store_true")
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

    print("Caso atual (pid.c como esta gravado hoje):")
    r_atual = simular(args.ponto, args.alvo, 900.0, False, args.t_final,
                       planta_variavel=args.planta_variavel)
    resumo(r_atual)

    print("\nCandidato (INTEGRAL_MAX menor + anti-windup condicional):")
    r_novo = simular(args.ponto, args.alvo, args.integral_max, True, args.t_final,
                      planta_variavel=args.planta_variavel)
    resumo(r_novo)

    fig, axs = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
    axs[0].plot(r_atual["t"], r_atual["ang"], label="Atual (INTEGRAL_MAX=900)", color="tab:red")
    axs[0].plot(r_novo["t"], r_novo["ang"],
                label=f"Candidato (INTEGRAL_MAX={args.integral_max}, anti-windup cond.)",
                color="tab:blue")
    axs[0].axhline(args.alvo, color="gray", linestyle="--", label="Alvo")
    axs[0].axhline(LIM_TRAVAR, color="black", linestyle=":", label="LIM_TRAVAR (170°)")
    axs[0].set_ylabel("Angulo (graus)")
    axs[0].set_title(f"Degrau de partida simulado: repouso -> {args.alvo}° "
                      f"(modelo do ponto {args.ponto}°)")
    axs[0].legend()
    axs[0].grid(True, linestyle=":", alpha=0.6)

    axs[1].plot(r_atual["t"], r_atual["pwm"], label="PWM atual", color="tab:red")
    axs[1].plot(r_novo["t"], r_novo["pwm"], label="PWM candidato", color="tab:blue")
    axs[1].plot(r_atual["t"], r_atual["integ"], label="Integral atual", color="tab:red", linestyle="--", alpha=0.6)
    axs[1].plot(r_novo["t"], r_novo["integ"], label="Integral candidato", color="tab:blue", linestyle="--", alpha=0.6)
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
