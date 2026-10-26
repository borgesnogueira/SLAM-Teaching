"""
Exemplo 3 - Tudo junto: controle, odometria e ground truth
----------------------------------------------------------
O robô percorre um quadrado de waypoints com o controle de posição do
controle.py. A odometria começa na pose da câmera e, ao vivo, a animação mostra:

    - mapa com a referência, a trajetória real (GT) e a estimada (odometria)
    - erro de posição da odometria em relação à GT
    - erro de orientação da odometria em relação à GT

Por padrão o controle fecha a malha na odometria: o robô "acha" que fez o
quadrado, e a GT mostra o quanto ele realmente desviou. Com --malha gt o
controle usa a câmera e quem deriva é só a odometria.

    python3 exemplos/03_odometria_vs_gt.py
    python3 exemplos/03_odometria_vs_gt.py --malha gt --voltas 3
    python3 exemplos/03_odometria_vs_gt.py --sim
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import _comum as c
import controle

TOLERANCIA = 0.04     # m, distância para considerar o waypoint atingido


def waypoints(gt, lado):
    """Quadrado centrado na área vista pela câmera."""
    cx, cy = gt.area_visivel().mean(axis=0)
    m = lado / 2
    return [(cx - m, cy - m), (cx + m, cy - m), (cx + m, cy + m), (cx - m, cy + m)]


def percorrer(robo, pose, refs, voltas, esperar, estado):
    """Leva o robô por todos os waypoints usando a lei de controle do controle.py."""
    for volta in range(voltas):
        for i, ref in enumerate(refs):
            estado["alvo"] = i
            estado["volta"] = volta + 1
            while True:
                q = pose()
                if q is None:                       # GT perdida: espera parado
                    robo.send_wheel_speed(0, 0)
                elif np.hypot(ref[0] - q[0], ref[1] - q[1]) < TOLERANCIA:
                    break
                else:
                    v, w = controle.positioning_control(q, ref)
                    robo.send_wheel_speed(*controle.unicycle_to_wheels(v, w))
                if not esperar(controle.PERIODO):
                    return
    estado["alvo"] = None


def main():
    parser = c.argumentos("Odometria x ground truth seguindo waypoints.")
    parser.add_argument("--malha", choices=["odom", "gt"], default="odom",
                        help="pose usada pelo controle (padrão: odom)")
    parser.add_argument("--lado", type=float, default=0.7, help="lado do quadrado [m]")
    parser.add_argument("--voltas", type=int, default=2)
    args = parser.parse_args()

    robo, gt = c.carregar(args.sim)
    c.conectar(robo)
    if not gt.start(on_pose=robo.log_ground_truth):
        print("Câmera indisponível.")
        robo.disconnect()
        return

    # Espera a câmera e inicia a odometria na pose real
    inicio = time.time()
    while gt.get_timestamp() is None and time.time() - inicio < 10:
        time.sleep(0.1)
    if gt.get_timestamp() is None:
        print("Robô não visto pela câmera.")
        gt.stop(); robo.disconnect()
        return
    robo.set_odometry(*gt.get_pose())
    time.sleep(0.3)
    if args.gravar:
        robo.record(True)

    refs = waypoints(gt, args.lado)
    pose = controle.pose_odom if args.malha == "odom" else controle.pose_gt

    # ---------- figura ----------
    c.estilo()
    fig = plt.figure(figsize=(13, 7))
    grade = fig.add_gridspec(2, 2, width_ratios=[1.3, 1], hspace=0.45, wspace=0.2)
    ax_mapa = fig.add_subplot(grade[:, 0])
    ax_pos = fig.add_subplot(grade[0, 1])
    ax_ang = fig.add_subplot(grade[1, 1], sharex=ax_pos)

    malha = "odometria" if args.malha == "odom" else "câmera (GT)"
    fig.suptitle("Odometria × ground truth" + (" (simulação)" if args.sim else ""),
                 x=0.06, ha="left", fontsize=13, fontweight="bold", color=c.TEXTO)
    subtitulo = fig.text(0.06, 0.915, "", color=c.TEXTO_2, fontsize=9)

    ax_mapa.set_title(f"Trajetórias · controle em malha fechada na {malha}")
    ax_mapa.set_aspect("equal")
    ax_mapa.set_xlabel("x [m]")
    ax_mapa.set_ylabel("y [m]")
    c.desenhar_area(ax_mapa, gt)
    rx = [r[0] for r in refs] + [refs[0][0]]
    ry = [r[1] for r in refs] + [refs[0][1]]
    ax_mapa.plot(rx, ry, "--", color=c.COR_3, linewidth=1.5, label="referência")
    ax_mapa.plot(rx[:-1], ry[:-1], "s", color=c.COR_3, markersize=8,
                 markeredgecolor=c.FUNDO, markeredgewidth=2)
    alvo, = ax_mapa.plot([], [], "s", color=c.COR_3, markersize=16, markerfacecolor="none",
                         markeredgewidth=2)
    linha_gt, = ax_mapa.plot([], [], color=c.COR_2, label="ground truth (real)")
    linha_odom, = ax_mapa.plot([], [], color=c.COR_1, label="odometria (estimada)")
    robo_gt = c.Robo(ax_mapa, c.COR_2)
    robo_odom = c.Robo(ax_mapa, c.COR_1)
    ax_mapa.legend(loc="upper left", bbox_to_anchor=(0, -0.08), ncols=4)
    c.enquadrar(ax_mapa, rx, ry, margem=0.35)

    ax_pos.set_title("Erro de posição da odometria")
    ax_pos.set_ylabel("cm")
    linha_pos, = ax_pos.plot([], [], color=c.COR_1)
    plt.setp(ax_pos.get_xticklabels(), visible=False)

    ax_ang.set_title("Erro de orientação da odometria")
    ax_ang.set_ylabel("graus")
    ax_ang.set_xlabel("tempo [s]")
    ax_ang.axhline(0, color=c.TEXTO_2, linewidth=0.8)
    linha_ang, = ax_ang.plot([], [], color=c.COR_1)

    # ---------- dados e animação ----------
    t0 = time.time()
    ox, oy, gx, gy = [], [], [], []
    t_err, e_pos, e_ang = [], [], []
    estado = {"alvo": 0, "volta": 1}

    movimento = c.Movimento(
        robo, lambda esperar: percorrer(robo, pose, refs, args.voltas, esperar, estado))
    movimento.iniciar()

    def atualizar(_):
        agora = time.time()
        x, y, theta = robo.get_odometry()
        ox.append(x); oy.append(y)
        linha_odom.set_data(ox, oy)
        robo_odom.mover(x, y, theta)

        q_gt = controle.pose_gt()
        if q_gt is not None:
            gx.append(q_gt[0]); gy.append(q_gt[1])
            linha_gt.set_data(gx, gy)
            robo_gt.mover(*q_gt)
            t_err.append(agora - t0)
            e_pos.append(100 * np.hypot(x - q_gt[0], y - q_gt[1]))
            e_ang.append(np.degrees(controle.angle_wrap(theta - q_gt[2])))
            linha_pos.set_data(t_err, e_pos)
            linha_ang.set_data(t_err, e_ang)
            ax_pos.set_xlim(0, max(10, t_err[-1]))
            ax_pos.set_ylim(0, max(5, max(e_pos) * 1.15))
            lim = max(5, max(abs(a) for a in e_ang) * 1.15)
            ax_ang.set_ylim(-lim, lim)

        if estado["alvo"] is not None:
            r = refs[estado["alvo"]]
            alvo.set_data([r[0]], [r[1]])
        else:
            alvo.set_data([], [])

        if movimento.terminou:
            progresso = "percurso concluído"
        else:
            progresso = f"volta {estado['volta']}/{args.voltas}, waypoint {estado['alvo'] + 1}/4"
        erro = f"erro atual {e_pos[-1]:.1f} cm / {e_ang[-1]:.1f}°" if e_pos else "sem GT"
        gt_ok = "" if q_gt is not None else "  ·  GT perdida"
        subtitulo.set_text(f"{progresso}  ·  {agora - t0:.1f} s  ·  {erro}{gt_ok}")

    anim = FuncAnimation(fig, atualizar, interval=50, cache_frame_data=False)

    try:
        plt.show()
    finally:
        movimento.parar()
        robo.stop()
        robo.record(False)
        gt.stop()
        robo.disconnect()
        c.salvar_figura(fig, f"03_odometria_vs_gt_{args.malha}.png")


if __name__ == "__main__":
    main()
