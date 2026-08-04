"""
Checagem de margem de estabilidade (Bode/Nyquist) de um candidato de PID
contra TODOS os pontos de operacao identificados na bancada (15 a 90 graus).

Cada ponto foi identificado por decremento logaritmico (metodo_2a_ordem_
decremento_log em identifica_zn.py), a partir de ensaios reais de malha
aberta — ver docs/diagnostico_oscilacao_pid.md, secao 6.2.

Modelo de cada ponto: G(s) = K*wn^2 / (s^2 + 2*zeta*wn*s + wn^2) * e^(-Ls)

O candidato de PID e calculado por cancelamento de polos (zeros do PID
cancelam o par de polos complexos da planta escolhida como ponto de
projeto), visando uma margem de fase alvo. Depois, o MESMO candidato e
testado contra TODOS os pontos, para achar o pior caso real em malha
fechada — e nao so o pior caso "no papel" olhando zeta/wn isolados.

Uso:
    python3 analise_margem_multiponto.py --projeto 90 --mf-alvo 60
    python3 analise_margem_multiponto.py --kp 2.5 --ki 3.0 --kd 0.4  (testa ganhos manuais)
"""

import argparse
import numpy as np
import control as ct
import matplotlib.pyplot as plt


# ============================================================================
# Pontos identificados na bancada (docs/diagnostico_oscilacao_pid.md, 6.2)
# K: graus/PWM | zeta: adimensional | wn: rad/s | L: segundos
# ============================================================================
PONTOS = {
    30: dict(K=0.136, zeta=0.130, wn=8.38, L=0.70),
    45: dict(K=0.129, zeta=0.146, wn=8.14, L=1.00),
    60: dict(K=0.139, zeta=0.171, wn=7.70, L=0.85),
    75: dict(K=0.156, zeta=0.144, wn=6.32, L=1.07),  # re-ensaiado (substitui o zeta=0.073 suspeito)
    90: dict(K=0.198, zeta=0.165, wn=5.83, L=1.53),  # excluido por padrao — ensaio pouco confiavel (ver docs)
}
PONTOS_EXCLUIDOS_PADRAO = [90]


def planta(K, zeta, wn, L, ordem_pade=3):
    """G(s) = K*wn^2/(s^2+2*zeta*wn*s+wn^2) * e^(-Ls), atraso via Pade."""
    G2 = ct.tf([K * wn ** 2], [1, 2 * zeta * wn, wn ** 2])
    num_atraso, den_atraso = ct.pade(L, ordem_pade)
    G_atraso = ct.tf(num_atraso, den_atraso)
    return ct.series(G2, G_atraso)


def pid_por_cancelamento_polos(zeta, wn, L, K, mf_alvo_graus=60.0):
    """Cancela o par de polos complexos da planta com os zeros do PID e
    escolhe a frequencia de cruzamento para bater a margem de fase alvo,
    considerando so o atraso de transporte (a planta de 2a ordem + o
    integrador do PID somam exatamente 90 graus quando os polos sao
    cancelados, entao a fase que resta a perder e so a do atraso).

    C(s) = (Kd*s^2 + Kp*s + Ki)/s
    Cancelamento:  Kp/Kd = 2*zeta*wn   e   Ki/Kd = wn^2
    """
    mf_alvo = np.radians(mf_alvo_graus)
    # fase disponivel para o atraso: PM = 90 - L*wc (em graus/rad)
    wc = (np.pi / 2 - mf_alvo) / L
    Kd = wc / (K * wn ** 2)
    Kp = Kd * 2 * zeta * wn
    Ki = Kd * wn ** 2
    return Kp, Ki, Kd, wc


def avaliar_candidato(Kp, Ki, Kd, pontos=PONTOS):
    """Roda margin() do candidato contra cada ponto identificado."""
    resultados = []
    num_pid = [Kd, Kp, Ki]
    den_pid = [1, 0]
    C = ct.tf(num_pid, den_pid)

    for angulo, p in sorted(pontos.items()):
        G = planta(**p)
        G_ma = ct.series(C, G)
        gm, pm, wg, wp = ct.margin(G_ma)
        gm_db = 20 * np.log10(gm) if np.isfinite(gm) and gm > 0 else float('inf')
        resultados.append(dict(angulo=angulo, gm_db=gm_db, pm=pm, wg=wg, wp=wp))
    return resultados


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--projeto", type=int, choices=list(PONTOS.keys()), default=75,
                         help="angulo (grau) usado como ponto de projeto para o cancelamento de polos (default: 75 — pior caso de wn entre os pontos confiaveis, com o 90 excluido)")
    parser.add_argument("--mf-alvo", type=float, default=60.0,
                         help="margem de fase alvo em graus para o cancelamento de polos (default: 60)")
    parser.add_argument("--kp", type=float, help="usar Kp manual em vez de calcular")
    parser.add_argument("--ki", type=float, help="usar Ki manual em vez de calcular")
    parser.add_argument("--kd", type=float, help="usar Kd manual em vez de calcular")
    parser.add_argument("--excluir", type=int, nargs="*", default=PONTOS_EXCLUIDOS_PADRAO,
                         help=f"angulos a excluir da avaliacao (default: {PONTOS_EXCLUIDOS_PADRAO})")
    parser.add_argument("--incluir-todos", action="store_true",
                         help="ignora --excluir e usa todos os pontos, incluindo o 90 graus")
    args = parser.parse_args()

    pontos = dict(PONTOS)
    if not args.incluir_todos:
        for ang in args.excluir:
            pontos.pop(ang, None)

    if args.kp is not None and args.ki is not None and args.kd is not None:
        Kp, Ki, Kd = args.kp, args.ki, args.kd
        print(f"Candidato manual: Kp={Kp}  Ki={Ki}  Kd={Kd}\n")
    else:
        p = PONTOS[args.projeto]
        Kp, Ki, Kd, wc = pid_por_cancelamento_polos(
            p["zeta"], p["wn"], p["L"], p["K"], args.mf_alvo)
        print(f"Ponto de projeto: {args.projeto}° (zeta={p['zeta']}, wn={p['wn']}, "
              f"L={p['L']}, K={p['K']})")
        print(f"Cancelamento de polos, MF alvo={args.mf_alvo}°  ->  wc={wc:.3f} rad/s")
        print(f"Candidato: Kp={Kp:.4f}  Ki={Ki:.4f}  Kd={Kd:.4f}\n")

    resultados = avaliar_candidato(Kp, Ki, Kd, pontos)

    print(f"{'Angulo':>7} | {'GM (dB)':>9} | {'PM (graus)':>11} | {'wg (rad/s)':>11} | {'wp (rad/s)':>11}")
    print("-" * 62)
    for r in resultados:
        print(f"{r['angulo']:>6}° | {r['gm_db']:>9.2f} | {r['pm']:>11.2f} | "
              f"{r['wg']:>11.3f} | {r['wp']:>11.3f}")

    pior = min(resultados, key=lambda r: r['pm'])
    print(f"\nPior caso: {pior['angulo']}° com margem de fase = {pior['pm']:.2f}°")
    if pior['pm'] < 30:
        print("ATENCAO: margem de fase do pior caso < 30 graus — candidato "
              "provavelmente vai oscilar bastante nesse ponto de operacao.")
    elif pior['pm'] < 45:
        print("AVISO: margem de fase do pior caso < 45 graus — pouca folga, "
              "considere um Kp/Ki mais conservador.")

    # ---- grafico: margem de fase por angulo ----
    angulos = [r['angulo'] for r in resultados]
    pms = [r['pm'] for r in resultados]
    plt.figure(figsize=(8, 5))
    plt.plot(angulos, pms, 'o-', color='tab:blue')
    plt.axhline(45, color='orange', linestyle='--', label='45° (aviso)')
    plt.axhline(30, color='red', linestyle='--', label='30° (risco)')
    plt.xlabel("Ponto de operacao (graus)")
    plt.ylabel("Margem de fase (graus)")
    plt.title(f"Margem de fase do candidato PID (Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}) por angulo")
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend()
    plt.savefig("margem_por_angulo.png", dpi=120)
    print("\nGrafico salvo em margem_por_angulo.png")


if __name__ == "__main__":
    main()
