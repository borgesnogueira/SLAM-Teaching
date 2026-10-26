"""
Exemplo 5 - Câmera embarcada no robô
------------------------------------
Liga o vídeo da câmera do robô (robo.start_camera) e mostra, ao vivo e no mesmo instante:

    - o que o robô está vendo
    - onde ele está: ground truth da câmera do teto e odometria no mapa
    - qualidade do vídeo: frames por segundo e atraso captura -> chegada

Mova o robô com o joystick (python3 controle.py joystick, em outro terminal)
ou peça círculos. Com --gravar, robo.record() salva também os frames em
dados/camera_<data>/, com o timestamp da captura no nome (mesmo relógio do
recording.csv, onde cada frame vira uma linha "camera,frame_id,timestamp").

    python3 exemplos/05_camera_robo.py
    python3 exemplos/05_camera_robo.py --circulo --gravar
    python3 exemplos/05_camera_robo.py --sem-gt          # sem a câmera do teto
    python3 exemplos/05_camera_robo.py --sim
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
    parser = c.argumentos("Câmera do robô com mapa e qualidade do vídeo.")
    parser.add_argument("--circulo", action="store_true", help="faz o robô andar em círculos")
    parser.add_argument("--sem-gt", action="store_true", help="não usa a câmera do teto")
    args = parser.parse_args()
    mover = args.circulo or args.sim

    robo, gt = c.carregar(args.sim)
    c.conectar(robo)
    if not robo.start_camera():
        robo.disconnect()
        return

    usa_gt = not args.sem_gt and gt.start(on_pose=robo.log_ground_truth)
    if usa_gt:
        inicio = time.time()
        while gt.get_timestamp() is None and time.time() - inicio < 5:
            time.sleep(0.1)
        if gt.get_timestamp() is not None:
            robo.set_odometry(*gt.get_pose())     # odometria no referencial da GT
    if args.gravar:
        robo.record(True)

    # ---------- figura ----------
    c.estilo()
    fig = plt.figure(figsize=(13, 6.8))
    grade = fig.add_gridspec(2, 2, width_ratios=[1.35, 1], height_ratios=[2.2, 1],
                             hspace=0.42, wspace=0.18)
    ax_img = fig.add_subplot(grade[:, 0])
    ax_mapa = fig.add_subplot(grade[0, 1])
    ax_fps = fig.add_subplot(grade[1, 1])

    fig.suptitle("Câmera do robô" + (" (simulação)" if args.sim else ""),
                 x=0.06, ha="left", fontsize=13, fontweight="bold", color=c.TEXTO)
    subtitulo = fig.text(0.06, 0.915, "", color=c.TEXTO_2, fontsize=9)

    ax_img.set_title("O que o robô vê")
    ax_img.grid(False)
    ax_img.set_xticks([]); ax_img.set_yticks([])
    for lado in ax_img.spines.values():
        lado.set_visible(False)
    aviso = ax_img.text(0.5, 0.5, "aguardando vídeo do robô...\n(porta UDP 5001)",
                        transform=ax_img.transAxes, ha="center", va="center", color=c.TEXTO_2)
    imagem = None

    ax_mapa.set_title("Onde ele está")
    ax_mapa.set_aspect("equal")
    ax_mapa.set_xlabel("x [m]")
    ax_mapa.set_ylabel("y [m]")
    if usa_gt:
        cantos = c.desenhar_area(ax_mapa, gt)
        if cantos is not None:
            c.enquadrar(ax_mapa, list(cantos[:, 0]), list(cantos[:, 1]), margem=0.1)
        linha_gt, = ax_mapa.plot([], [], color=c.COR_2, label="ground truth")
        robo_gt = c.Robo(ax_mapa, c.COR_2)
    linha_odom, = ax_mapa.plot([], [], color=c.COR_1, label="odometria")
    robo_odom = c.Robo(ax_mapa, c.COR_1)
    ax_mapa.legend(loc="upper right")

    ax_fps.set_title("Frames por segundo recebidos")
    ax_fps.set_ylabel("fps")
    ax_fps.set_xlabel("tempo [s]")
    ax_fps.set_ylim(0, 15)
    linha_fps, = ax_fps.plot([], [], color=c.COR_1)

    # ---------- dados e animação ----------
    t0 = time.time()
    ox, oy, gx, gy, t_fps, fps = [], [], [], [], [], []
    ultimo_gt = [None]

    movimento = None
    if mover:
        movimento = c.Movimento(robo, lambda esperar: circulos(robo, esperar))
        movimento.iniciar()

    def atualizar(_):
        nonlocal imagem
        agora = time.time()

        frame = robo.get_camera_frame()
        if frame is not None:
            if imagem is None:
                imagem = ax_img.imshow(frame)
                aviso.set_visible(False)
            else:
                imagem.set_data(frame)

        x, y, theta = robo.get_odometry()
        ox.append(x); oy.append(y)
        linha_odom.set_data(ox, oy)
        robo_odom.mover(x, y, theta)
        if usa_gt:
            ts = gt.get_timestamp()
            if ts is not None and ts != ultimo_gt[0]:
                ultimo_gt[0] = ts
                q = gt.get_pose()
                gx.append(q[0]); gy.append(q[1])
                linha_gt.set_data(gx, gy)
                robo_gt.mover(*q)
        else:
            c.enquadrar(ax_mapa, ox, oy)

        s = robo.get_camera_stats()
        t_fps.append(agora - t0)
        fps.append(s["fps"])
        linha_fps.set_data(t_fps, fps)
        ax_fps.set_xlim(max(0, t_fps[-1] - 30), max(30, t_fps[-1]))

        ts_cam = robo.get_camera_timestamp()
        if ts_cam is None:
            estado = "sem vídeo ainda (robô ligado? firewall liberando UDP 5001?)"
        else:
            atraso = f"{s['atraso'] * 1000:.0f} ms" if s["atraso"] is not None else "-"
            estado = (f"{s['fps']:.1f} fps  ·  atraso {atraso}  ·  "
                      f"{s['frames']} frames, {s['perdidos']} perdidos")
            if agora - ts_cam > 1.0:
                estado = f"vídeo parado há {agora - ts_cam:.1f} s  ·  " + estado
        subtitulo.set_text(estado + ("  ·  gravando" if args.gravar else ""))

    anim = FuncAnimation(fig, atualizar, interval=50, cache_frame_data=False)

    try:
        plt.show()
    finally:
        if movimento:
            movimento.parar()
        robo.stop()
        robo.record(False)
        if usa_gt:
            gt.stop()
        robo.disconnect()
        c.salvar_figura(fig, "05_camera_robo.png")


if __name__ == "__main__":
    main()
