"""
Exemplo 2 - Ground truth pela câmera do teto
--------------------------------------------
Mostra, ao vivo, como a pose de referência é obtida:

    - imagem da câmera com o caminho do marcador ArUco desenhado em pixels
    - mesma trajetória convertida para metros (homografia), dentro da área vista
    - taxa de detecção do marcador (cai quando o robô sai da imagem ou some)

Mova o robô com a mão, com o joystick (python3 controle.py joystick, em outro
terminal) ou peça um círculo:

    python3 exemplos/02_ground_truth.py
    python3 exemplos/02_ground_truth.py --circulo    # robô anda em círculos
    python3 exemplos/02_ground_truth.py --sim
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import _comum as c


def circulos(robo, esperar):
    while c.rodas(robo, 0.06, 0.12, 1.0, esperar):
        pass


def main():
    parser = c.argumentos("Ground truth pela câmera com animação.")
    parser.add_argument("--circulo", action="store_true",
                        help="faz o robô andar em círculos (precisa do MQTT)")
    args = parser.parse_args()
    mover = args.circulo or args.sim

    robo, gt = c.carregar(args.sim)
    if mover or args.gravar:
        c.conectar(robo)
    if args.gravar:
        robo.record(True)
    if not gt.start(on_pose=robo.log_ground_truth if args.gravar else None):
        print("Câmera indisponível.")
        return

    # ---------- figura ----------
    c.estilo()
    fig = plt.figure(figsize=(13, 6.5))
    grade = fig.add_gridspec(2, 2, width_ratios=[1, 1], height_ratios=[2.2, 1],
                             hspace=0.4, wspace=0.18)
    ax_img = fig.add_subplot(grade[:, 0])
    ax_mapa = fig.add_subplot(grade[0, 1])
    ax_taxa = fig.add_subplot(grade[1, 1])

    titulo = fig.suptitle("Ground truth" + (" (simulação)" if args.sim else ""),
                          x=0.06, ha="left", fontsize=13, fontweight="bold", color=c.TEXTO)
    subtitulo = fig.text(0.06, 0.915, "", color=c.TEXTO_2, fontsize=9)

    ax_img.set_title("Câmera do teto (pixels)")
    ax_img.grid(False)
    ax_img.set_xticks([]); ax_img.set_yticks([])
    for lado in ax_img.spines.values():
        lado.set_visible(False)
    imagem = None
    aviso_img = ax_img.text(0.5, 0.5, "aguardando imagem...", transform=ax_img.transAxes,
                            ha="center", va="center", color=c.TEXTO_2)
    trilha_px, = ax_img.plot([], [], color=c.COR_2, linewidth=2)
    ponto_px, = ax_img.plot([], [], "o", color=c.COR_2, markersize=10,
                            markeredgecolor=c.FUNDO, markeredgewidth=2)

    ax_mapa.set_title("Mesma trajetória no chão (metros)")
    ax_mapa.set_aspect("equal")
    ax_mapa.set_xlabel("x [m]")
    ax_mapa.set_ylabel("y [m]")
    cantos = c.desenhar_area(ax_mapa, gt)
    if cantos is not None:
        c.enquadrar(ax_mapa, list(cantos[:, 0]), list(cantos[:, 1]), margem=0.1)
    trilha, = ax_mapa.plot([], [], color=c.COR_2, label="ground truth")
    robo_desenho = c.Robo(ax_mapa, c.COR_2)
    robo_desenho.patch.set_visible(False)
    ax_mapa.legend(loc="upper right")

    ax_taxa.set_title("Taxa de detecção do marcador")
    ax_taxa.set_ylabel("poses/s")
    ax_taxa.set_xlabel("tempo [s]")
    ax_taxa.set_ylim(0, 25)
    linha_taxa, = ax_taxa.plot([], [], color=c.COR_2)

    # ---------- dados e animação ----------
    t0 = time.time()
    xs, ys, us, vs = [], [], [], []
    deteccoes = []                     # instantes em que chegou pose nova
    t_taxa, taxa = [], []
    ultimo_ts = [None]

    movimento = None
    if mover:
        movimento = c.Movimento(robo, lambda esperar: circulos(robo, esperar))
        movimento.iniciar()

    def atualizar(_):
        nonlocal imagem
        agora = time.time()
        ts = gt.get_timestamp()

        if ts is not None and ts != ultimo_ts[0]:
            ultimo_ts[0] = ts
            x, y, theta = gt.get_pose()
            xs.append(x); ys.append(y)
            deteccoes.append(ts)
            trilha.set_data(xs, ys)
            robo_desenho.mover(x, y, theta)
            robo_desenho.patch.set_visible(True)
            px = gt.get_pixel()
            if px is not None:
                us.append(px[0]); vs.append(px[1])
                trilha_px.set_data(us, vs)
                ponto_px.set_data([px[0]], [px[1]])

        frame = gt.get_frame()
        if frame is not None:
            if imagem is None:
                imagem = ax_img.imshow(frame, zorder=0)
                aviso_img.set_visible(False)
            else:
                imagem.set_data(frame)
        elif args.sim:
            aviso_img.set_text("sem imagem na simulação")

        recentes = sum(1 for d in deteccoes[-40:] if agora - d <= 1.0)
        t_taxa.append(agora - t0)
        taxa.append(recentes)
        linha_taxa.set_data(t_taxa, taxa)
        ax_taxa.set_xlim(max(0, t_taxa[-1] - 30), max(30, t_taxa[-1]))

        if ts is None:
            estado = "robô ainda não visto"
        elif agora - ts > 0.5:
            estado = f"robô perdido há {agora - ts:.1f} s"
        else:
            x, y, theta = gt.get_pose()
            estado = f"x={x:.3f} m, y={y:.3f} m, θ={np.degrees(theta):.1f}°"
        subtitulo.set_text(f"{estado}  ·  {recentes} poses/s  ·  {len(xs)} poses recebidas")

    anim = FuncAnimation(fig, atualizar, interval=50, cache_frame_data=False)

    try:
        plt.show()
    finally:
        if movimento:
            movimento.parar()
            robo.stop()
        robo.record(False)
        gt.stop()
        if mover or args.gravar:
            robo.disconnect()
        c.salvar_figura(fig, "02_ground_truth.png")


if __name__ == "__main__":
    main()
