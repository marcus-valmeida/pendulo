"""Margem de ganho e de fase de um PID contra TODOS os pontos identificados,
para achar o pior caso real e nao o pior caso "no papel" olhando zeta/wn.
Planta: G(s) = K*wn^2/(s^2 + 2*zeta*wn*s + wn^2) * e^(-Ls).

    python3 analise_margem_multiponto.py --pid-do-firmware
"""

import argparse
import numpy as np
import control as ct
import matplotlib.pyplot as plt


from identifica_zn import PONTOS, L_LACO

# Ganhos gravados em src/pid.c — se mudar la, mude aqui.
PID_FIRMWARE = (0.8138, 2.3589, 0.1870)


def planta(K, zeta, wn, L=L_LACO, ordem_pade=3):
    """G(s) = K*wn^2/(s^2+2*zeta*wn*s+wn^2) * e^(-Ls), atraso via Pade."""
    G2 = ct.tf([K * wn ** 2], [1, 2 * zeta * wn, wn ** 2])
    num_atraso, den_atraso = ct.pade(L, ordem_pade)
    G_atraso = ct.tf(num_atraso, den_atraso)
    return ct.series(G2, G_atraso)


def pid_por_cancelamento_polos(zeta, wn, L, K, mf_alvo_graus=60.0):
    """Zeros do PID cancelam os polos da planta; wc e escolhida para bater a
    margem de fase alvo. Cancelados os polos, planta + integrador somam 90
    graus, entao a fase que resta a perder e so a do atraso.
    C(s) = (Kd*s^2 + Kp*s + Ki)/s, com Kp/Kd = 2*zeta*wn e Ki/Kd = wn^2."""
    mf_alvo = np.radians(mf_alvo_graus)
    # fase disponivel para o atraso: PM = 90 - L*wc (em graus/rad)
    wc = (np.pi / 2 - mf_alvo) / L
    Kd = wc / (K * wn ** 2)
    Kp = Kd * 2 * zeta * wn
    Ki = Kd * wn ** 2
    return Kp, Ki, Kd, wc


def avaliar_candidato(Kp, Ki, Kd, pontos=PONTOS, L=L_LACO):
    """Roda margin() do candidato contra cada ponto identificado."""
    resultados = []
    C = ct.tf([Kd, Kp, Ki], [1, 0])

    for angulo, p in sorted(pontos.items()):
        G = planta(L=L, **p)
        G_ma = ct.series(C, G)
        gm, pm, wg, wp = ct.margin(G_ma)
        gm_db = 20 * np.log10(gm) if np.isfinite(gm) and gm > 0 else float('inf')
        resultados.append(dict(angulo=angulo, gm_db=gm_db, pm=pm, wg=wg, wp=wp))
    return resultados


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pid-do-firmware", action="store_true",
                    help="avalia os ganhos que estao em pid.c")
    ap.add_argument("--kp", type=float, help="Kp manual")
    ap.add_argument("--ki", type=float, help="Ki manual")
    ap.add_argument("--kd", type=float, help="Kd manual")
    ap.add_argument("--projeto", type=int, choices=list(PONTOS.keys()), default=30,
                    help="ponto de projeto do cancelamento de polos (default: 30)")
    ap.add_argument("--mf-alvo", type=float, default=60.0,
                    help="margem de fase alvo do cancelamento de polos, em graus")
    ap.add_argument("--atraso", type=float, default=L_LACO,
                    help=f"atraso de laco em segundos (default: {L_LACO})")
    ap.add_argument("--excluir", type=int, nargs="*", default=[],
                    help="angulos a excluir da avaliacao")
    args = ap.parse_args()

    pontos = {a: p for a, p in PONTOS.items() if a not in args.excluir}

    if args.pid_do_firmware:
        Kp, Ki, Kd = PID_FIRMWARE
        print(f"Ganhos de pid.c: Kp={Kp}  Ki={Ki}  Kd={Kd}\n")
    elif None not in (args.kp, args.ki, args.kd):
        Kp, Ki, Kd = args.kp, args.ki, args.kd
        print(f"Candidato manual: Kp={Kp}  Ki={Ki}  Kd={Kd}\n")
    else:
        p = PONTOS[args.projeto]
        Kp, Ki, Kd, wc = pid_por_cancelamento_polos(
            p["zeta"], p["wn"], args.atraso, p["K"], args.mf_alvo)
        print(f"Ponto de projeto: {args.projeto}° "
              f"(zeta={p['zeta']}, wn={p['wn']}, K={p['K']}, L={args.atraso})")
        print(f"Cancelamento de polos, MF alvo={args.mf_alvo}° -> wc={wc:.3f} rad/s")
        print(f"Candidato: Kp={Kp:.4f}  Ki={Ki:.4f}  Kd={Kd:.4f}\n")

    resultados = avaliar_candidato(Kp, Ki, Kd, pontos, args.atraso)

    print(f"{'Angulo':>7} | {'GM (dB)':>9} | {'PM (graus)':>11} | "
          f"{'wg (rad/s)':>11} | {'wp (rad/s)':>11}")
    print("-" * 62)
    for r in resultados:
        print(f"{r['angulo']:>6}° | {r['gm_db']:>9.2f} | {r['pm']:>11.2f} | "
              f"{r['wg']:>11.3f} | {r['wp']:>11.3f}")

    pior = min(resultados, key=lambda r: r['pm'])
    print(f"\nPior caso: {pior['angulo']}° com margem de fase = {pior['pm']:.2f}°")
    if pior['pm'] < 30:
        print("ATENCAO: MF < 30 graus — deve oscilar bastante nesse ponto.")
    elif pior['pm'] < 45:
        print("AVISO: MF < 45 graus — pouca folga, considere ganhos menores.")

    plt.figure(figsize=(8, 5))
    plt.plot([r['angulo'] for r in resultados], [r['pm'] for r in resultados],
             'o-', color='tab:blue')
    plt.axhline(45, color='orange', linestyle='--', label='45° (aviso)')
    plt.axhline(30, color='red', linestyle='--', label='30° (risco)')
    plt.xlabel("Ponto de operacao (graus)")
    plt.ylabel("Margem de fase (graus)")
    plt.title(f"Margem de fase por angulo — "
              f"Kp={Kp:.3f}, Ki={Ki:.3f}, Kd={Kd:.3f}, L={args.atraso}s")
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend()
    plt.savefig("margem_por_angulo.png", dpi=120)
    print("\nGrafico salvo em margem_por_angulo.png")


if __name__ == "__main__":
    main()
