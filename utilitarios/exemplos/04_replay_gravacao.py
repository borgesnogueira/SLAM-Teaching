"""
Exemplo 4 - Replay de uma gravação
----------------------------------
Reproduz uma sessão do recording.csv (entre "Inicio" e "Fim") como animação,
sem precisar do robô nem da câmera:

    - mapa com odometria e ground truth (se a sessão tiver as duas)
    - com as duas: erros de posição e orientação da odometria
    - só com odometria: velocidades das rodas e orientação

    python3 exemplos/04_replay_gravacao.py --listar           # mostra as sessões
    python3 exemplos/04_replay_gravacao.py                    # última sessão com dados
    python3 exemplos/04_replay_gravacao.py --sessao 3 --velocidade 4
    python3 exemplos/04_replay_gravacao.py --alinhar          # põe a odometria no referencial da GT
    python3 exemplos/04_replay_gravacao.py --salvar replay.gif
    python3 exemplos/04_replay_gravacao.py --arquivo dados/simulacao.csv
"""

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

import _comum as c

HZ_SEM_TEMPO = 20     # gravações antigas sem timestamp: assume odometria a 20 Hz
PASSO = 0.05          # s de tela entre quadros


def ler_sessoes(caminho):
    """Retorna uma lista de sessões, cada uma {'odometria': [...], 'ground_truth': [...], ...}."""
    sessoes, atual = [], None
    t_falso = 0.0
    with open(caminho) as f:
        for linha in f:
            partes = linha.strip().split(",")
            if partes[0] == "Inicio":
                atual = {"odometria": [], "ground_truth": [], "velocidades": []}
                t_falso = 0.0
            elif partes[0] == "Fim":
                if atual is not None:
                    sessoes.append(atual)
                atual = None
            elif atual is not None and partes[0] in atual:
                valores = [float(v) for v in partes[1:]]
                n = 2 if partes[0] == "velocidades" else 3
                if len(valores) > n:
                    t = valores[n]                   # timestamp gravado pelo robo.py
                else:
                    t = t_falso
                    if partes[0] == "odometria":
                        t_falso += 1 / HZ_SEM_TEMPO
                atual[partes[0]].append([t] + valores[:n])
    if atual is not None:                            # sessão sem "Fim"
        sessoes.append(atual)
    return [{k: np.array(v).reshape(-1, 3 if k == "velocidades" else 4) for k, v in s.items()}
            for s in sessoes]


def resumo(s):
    partes = [f"{len(s[k])} {k}" for k in ("odometria", "ground_truth", "velocidades") if len(s[k])]
    duracao = max((s[k][-1, 0] - s[k][0, 0] for k in s if len(s[k]) > 1), default=0)
    return f"{duracao:6.1f} s  ·  " + (", ".join(partes) or "vazia")


def alinhar(odom, gt):
    """Transformação rígida que leva a primeira pose da odometria à GT do mesmo instante."""
    xg = np.interp(odom[0, 0], gt[:, 0], gt[:, 1])
    yg = np.interp(odom[0, 0], gt[:, 0], gt[:, 2])
    thg = np.interp(odom[0, 0], gt[:, 0], np.unwrap(gt[:, 3]))
    d = thg - odom[0, 3]
    cs, sn = np.cos(d), np.sin(d)
    dx, dy = odom[:, 1] - odom[0, 1], odom[:, 2] - odom[0, 2]
    novo = odom.copy()
    novo[:, 1] = xg + cs * dx - sn * dy
    novo[:, 2] = yg + sn * dx + cs * dy
    novo[:, 3] = np.arctan2(np.sin(odom[:, 3] + d), np.cos(odom[:, 3] + d))
    return novo


def main():
    parser = argparse.ArgumentParser(description="Replay animado de uma gravação.")
    parser.add_argument("--arquivo", default=os.path.join(c.PASTA_DADOS, "recording.csv"))
    parser.add_argument("--sessao", type=int, help="índice da sessão (ver --listar)")
    parser.add_argument("--listar", action="store_true")
    parser.add_argument("--velocidade", type=float, default=1.0, help="multiplicador do tempo")
    parser.add_argument("--alinhar", action="store_true",
                        help="leva a odometria para o referencial da ground truth")
    parser.add_argument("--salvar", help="salva a animação (.gif) em vez de mostrar")
    args = parser.parse_args()

    sessoes = ler_sessoes(args.arquivo)
    com_dados = [i for i, s in enumerate(sessoes) if len(s["odometria"]) + len(s["ground_truth"]) > 1]
    if args.listar:
        for i in com_dados:
            print(f"[{i:3d}] {resumo(sessoes[i])}")
        print(f"{len(sessoes) - len(com_dados)} sessões vazias omitidas.")
        return
    if not com_dados:
        print("Nenhuma sessão com dados em", args.arquivo)
        return
    indice = args.sessao if args.sessao is not None else com_dados[-1]
    s = sessoes[indice]
    print(f"Sessão {indice}: {resumo(s)}")

    odom, gt, vel = s["odometria"], s["ground_truth"], s["velocidades"]
    tem_odom, tem_gt = len(odom) > 1, len(gt) > 1
    if args.alinhar and tem_odom and tem_gt:
        odom = alinhar(odom, gt)
    t0 = min(a[0, 0] for a in (odom, gt, vel) if len(a))
    for a in (odom, gt, vel):
        if len(a):
            a[:, 0] -= t0
    duracao = max(a[-1, 0] for a in (odom, gt, vel) if len(a))

    # Erros da odometria nos instantes da GT
    comparar = tem_odom and tem_gt
    if comparar:
        ox = np.interp(gt[:, 0], odom[:, 0], odom[:, 1])
        oy = np.interp(gt[:, 0], odom[:, 0], odom[:, 2])
        oth = np.interp(gt[:, 0], odom[:, 0], np.unwrap(odom[:, 3]))
        e_pos = 100 * np.hypot(ox - gt[:, 1], oy - gt[:, 2])
        e_ang = np.degrees(np.arctan2(np.sin(oth - gt[:, 3]), np.cos(oth - gt[:, 3])))

    # ---------- figura ----------
    c.estilo()
    fig = plt.figure(figsize=(13, 7))
    grade = fig.add_gridspec(2, 2, width_ratios=[1.3, 1], hspace=0.45, wspace=0.2)
    ax_mapa = fig.add_subplot(grade[:, 0])
    ax_a = fig.add_subplot(grade[0, 1])
    ax_b = fig.add_subplot(grade[1, 1], sharex=ax_a)

    nome = os.path.basename(args.arquivo)
    fig.suptitle(f"Replay · {nome} · sessão {indice}", x=0.06, ha="left",
                 fontsize=13, fontweight="bold", color=c.TEXTO)
    subtitulo = fig.text(0.06, 0.915, "", color=c.TEXTO_2, fontsize=9)

    ax_mapa.set_title("Trajetórias" + (" (odometria alinhada à GT)" if args.alinhar and comparar else ""))
    ax_mapa.set_aspect("equal")
    ax_mapa.set_xlabel("x [m]")
    ax_mapa.set_ylabel("y [m]")
    if tem_gt:
        import ground_truth
        c.desenhar_area(ax_mapa, ground_truth)
    xs = list(odom[:, 1]) + list(gt[:, 1]) if tem_odom or tem_gt else [0]
    ys = list(odom[:, 2]) + list(gt[:, 2]) if tem_odom or tem_gt else [0]
    c.enquadrar(ax_mapa, xs, ys, margem=0.2)

    # Trajetória completa clarinha ao fundo, parte já percorrida forte por cima
    linhas = []
    for dados, cor, rotulo, ok in ((gt, c.COR_2, "ground truth (real)", tem_gt),
                                   (odom, c.COR_1, "odometria (estimada)", tem_odom)):
        if not ok:
            continue
        ax_mapa.plot(dados[:, 1], dados[:, 2], color=cor, alpha=0.18, linewidth=2)
        linha, = ax_mapa.plot([], [], color=cor, label=rotulo)
        linhas.append((dados, linha, c.Robo(ax_mapa, cor)))
    ax_mapa.legend(loc="upper left", bbox_to_anchor=(0, -0.08), ncols=3)

    cursores = []
    if comparar:
        ax_a.set_title("Erro de posição da odometria")
        ax_a.set_ylabel("cm")
        ax_a.plot(gt[:, 0], e_pos, color=c.COR_1, alpha=0.18)
        serie_a, = ax_a.plot([], [], color=c.COR_1)
        ax_a.set_ylim(0, max(5, e_pos.max() * 1.15))
        ax_b.set_title("Erro de orientação da odometria")
        ax_b.set_ylabel("graus")
        ax_b.axhline(0, color=c.TEXTO_2, linewidth=0.8)
        ax_b.plot(gt[:, 0], e_ang, color=c.COR_1, alpha=0.18)
        serie_b, = ax_b.plot([], [], color=c.COR_1)
        lim = max(5, np.abs(e_ang).max() * 1.15)
        ax_b.set_ylim(-lim, lim)
        series = [(gt[:, 0], e_pos, serie_a), (gt[:, 0], e_ang, serie_b)]
    else:
        base = odom if tem_odom else gt
        ax_a.set_title("Velocidade das rodas")
        ax_a.set_ylabel("m/s")
        series = []
        if len(vel):
            for col, cor, rotulo in ((1, c.COR_1, "esquerda"), (2, c.COR_2, "direita")):
                ax_a.plot(vel[:, 0], vel[:, col], color=cor, alpha=0.18)
                linha, = ax_a.plot([], [], color=cor, label=rotulo)
                series.append((vel[:, 0], vel[:, col], linha))
            ax_a.legend(loc="upper left", ncols=2)
        ax_b.set_title("Orientação θ")
        ax_b.set_ylabel("graus")
        ax_b.set_ylim(-190, 190)
        ax_b.set_yticks([-180, -90, 0, 90, 180])
        cor = c.COR_1 if tem_odom else c.COR_2
        t_th, th = c.sem_saltos(base[:, 0], np.degrees(base[:, 3]))
        ax_b.plot(t_th, th, color=cor, alpha=0.18)
        linha, = ax_b.plot([], [], color=cor)
        # searchsorted precisa de tempos crescentes: NaN de quebra herda o tempo anterior
        series.append((np.fmax.accumulate(np.nan_to_num(t_th, nan=-np.inf)), th, linha))
    plt.setp(ax_a.get_xticklabels(), visible=False)
    ax_b.set_xlabel("tempo [s]")
    ax_a.set_xlim(0, max(duracao, 1))
    for ax in (ax_a, ax_b):
        cursores.append(ax.axvline(0, color=c.TEXTO_2, linewidth=0.8, alpha=0.6))

    # ---------- animação ----------
    n_quadros = int(duracao / (PASSO * args.velocidade)) + 2

    def atualizar(quadro):
        t = min(duracao, quadro * PASSO * args.velocidade)
        for dados, linha, desenho in linhas:
            k = np.searchsorted(dados[:, 0], t, side="right")
            linha.set_data(dados[:k, 1], dados[:k, 2])
            if k:
                desenho.mover(*dados[k - 1, 1:4])
        for ts, ys_, linha in series:
            k = np.searchsorted(ts, t, side="right")
            linha.set_data(ts[:k], ys_[:k])
        for cursor in cursores:
            cursor.set_xdata([t, t])
        texto = f"t = {t:5.1f} / {duracao:.1f} s  ·  {args.velocidade:g}×"
        if comparar:
            k = np.searchsorted(gt[:, 0], t, side="right")
            if k:
                texto += f"  ·  erro {e_pos[k - 1]:.1f} cm / {e_ang[k - 1]:.1f}°"
        subtitulo.set_text(texto)

    anim = FuncAnimation(fig, atualizar, frames=n_quadros, interval=PASSO * 1000,
                         repeat=False, cache_frame_data=False)

    if args.salvar:
        print(f"Salvando {n_quadros} quadros em {args.salvar}...")
        anim.save(args.salvar, writer=PillowWriter(fps=int(1 / PASSO)), dpi=80)
        print("Pronto.")
    else:
        plt.show()


if __name__ == "__main__":
    main()
