"""
Exemplo 1 - Odometria
---------------------
O robô executa uma trajetória em malha aberta (só comandos de roda) e a
animação mostra, ao vivo, o que a odometria acha que aconteceu:

    - mapa com a trajetória estimada e o robô em escala real
    - velocidades medidas das rodas esquerda e direita
    - orientação theta ao longo do tempo

    python3 exemplos/01_odometria.py                       # oito, robô real
    python3 exemplos/01_odometria.py --trajetoria quadrado
    python3 exemplos/01_odometria.py --sim                 # sem robô
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import _comum as c

V = 0.12                          # m/s, velocidade de cruzeiro


def circulo(robo, esperar, sentido=1):
    """Uma volta completa: roda externa mais rápida que a interna."""
    v_int, v_ext = 0.08, 0.16
    w = (v_ext - v_int) / c.WHEEL_BASE
    duracao = 2 * np.pi / w
    if sentido > 0:
        return c.rodas(robo, v_int, v_ext, duracao, esperar)
    return c.rodas(robo, v_ext, v_int, duracao, esperar)


def oito(robo, esperar):
    return circulo(robo, esperar, +1) and circulo(robo, esperar, -1)


def quadrado(robo, esperar, lado=0.5):
    v_giro = 1.0 * c.WHEEL_BASE / 2       # 1 rad/s girando no lugar
    for _ in range(4):
        if not c.rodas(robo, V, V, lado / V, esperar):
            return False
        if not c.rodas(robo, -v_giro, v_giro, np.pi / 2, esperar):
            return False
    return True


TRAJETORIAS = {"oito": oito, "circulo": circulo, "quadrado": quadrado}


def main():
    parser = c.argumentos("Odometria em malha aberta com animação.")
    parser.add_argument("--trajetoria", choices=TRAJETORIAS, default="oito")
    args = parser.parse_args()

    robo, _ = c.carregar(args.sim)
    c.conectar(robo)
    robo.set_odometry(0, 0, 0)
    time.sleep(0.3)
    if args.gravar:
        robo.record(True)

    # ---------- figura ----------
    c.estilo()
    fig = plt.figure(figsize=(12, 6.5))
    grade = fig.add_gridspec(2, 2, width_ratios=[1.25, 1], hspace=0.45, wspace=0.22)
    ax_mapa = fig.add_subplot(grade[:, 0])
    ax_rodas = fig.add_subplot(grade[0, 1])
    ax_theta = fig.add_subplot(grade[1, 1], sharex=ax_rodas)

    titulo = fig.suptitle("", x=0.06, ha="left", fontsize=13, fontweight="bold", color=c.TEXTO)
    subtitulo = fig.text(0.06, 0.915, "", color=c.TEXTO_2, fontsize=9)

    ax_mapa.set_title("Trajetória estimada pela odometria")
    ax_mapa.set_aspect("equal")
    ax_mapa.set_xlabel("x [m]")
    ax_mapa.set_ylabel("y [m]")
    ax_mapa.plot([0], [0], "o", color=c.TEXTO_2, markersize=8, label="início")
    linha_mapa, = ax_mapa.plot([], [], color=c.COR_1, label="odometria")
    robo_desenho = c.Robo(ax_mapa, c.COR_1)
    ax_mapa.legend(loc="upper right")

    ax_rodas.set_title("Velocidade das rodas")
    ax_rodas.set_ylabel("m/s")
    linha_esq, = ax_rodas.plot([], [], color=c.COR_1, label="esquerda")
    linha_dir, = ax_rodas.plot([], [], color=c.COR_2, label="direita")
    ax_rodas.legend(loc="upper left", ncols=2)
    ax_rodas.set_ylim(-0.25, 0.25)
    plt.setp(ax_rodas.get_xticklabels(), visible=False)

    ax_theta.set_title("Orientação θ")
    ax_theta.set_ylabel("graus")
    ax_theta.set_xlabel("tempo [s]")
    ax_theta.set_ylim(-190, 190)
    ax_theta.set_yticks([-180, -90, 0, 90, 180])
    linha_theta, = ax_theta.plot([], [], color=c.COR_1)

    # ---------- dados e animação ----------
    t, xs, ys, th, vl, vr = [], [], [], [], [], []
    t0 = time.time()

    movimento = c.Movimento(robo, lambda esperar: TRAJETORIAS[args.trajetoria](robo, esperar))
    movimento.iniciar()

    def atualizar(_):
        x, y, theta = robo.get_odometry()
        esq, dir_ = robo.get_wheel_speeds()
        t.append(time.time() - t0)
        xs.append(x); ys.append(y); th.append(np.degrees(theta))
        vl.append(esq); vr.append(dir_)

        linha_mapa.set_data(xs, ys)
        robo_desenho.mover(x, y, theta)
        c.enquadrar(ax_mapa, xs + [0], ys + [0])

        linha_esq.set_data(t, vl)
        linha_dir.set_data(t, vr)
        linha_theta.set_data(*c.sem_saltos(t, th))
        ax_rodas.set_xlim(0, max(10, t[-1]))

        distancia = np.sum(np.hypot(np.diff(xs), np.diff(ys))) if len(xs) > 1 else 0.0
        estado = "concluída" if movimento.terminou else "executando"
        titulo.set_text(f"Odometria · {args.trajetoria}" + (" (simulação)" if args.sim else ""))
        subtitulo.set_text(f"Trajetória {estado}  ·  {t[-1]:.1f} s  ·  "
                           f"{distancia:.2f} m percorridos  ·  "
                           f"pose x={x:.2f} m, y={y:.2f} m, θ={np.degrees(theta):.0f}°")

    anim = FuncAnimation(fig, atualizar, interval=50, cache_frame_data=False)

    try:
        plt.show()
    finally:
        movimento.parar()
        robo.stop()
        robo.record(False)
        robo.disconnect()
        c.salvar_figura(fig, f"01_odometria_{args.trajetoria}.png")


if __name__ == "__main__":
    main()
