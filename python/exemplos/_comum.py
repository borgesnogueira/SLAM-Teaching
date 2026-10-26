"""
Peças comuns dos exemplos: estilo dos gráficos, desenho do robô, argumentos
de linha de comando e escolha entre robô real e simulado (--sim).
"""

import os
import sys
import time
import argparse
import threading

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

PASTA_EXEMPLOS = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(PASTA_EXEMPLOS)
PASTA_DADOS = os.path.join(RAIZ, "dados")
sys.path.insert(0, RAIZ)            # para "import robo", "import ground_truth" etc.

BROKER = "192.168.0.125"
WHEEL_BASE = 0.143                  # m, igual ao firmware

# ===============================
# Cores (paleta categórica, sempre na mesma ordem)
# ===============================
COR_1 = "#2a78d6"    # azul    -> odometria
COR_2 = "#eb6834"    # laranja -> ground truth
COR_3 = "#1baf7a"    # verde-água -> referência / waypoints
FUNDO = "#fcfcfb"
TEXTO = "#0b0b0b"
TEXTO_2 = "#52514e"
GRADE = "#e4e3df"
AREA = "#efeeea"     # área vista pela câmera


def estilo():
    """Estilo único para todos os exemplos: fundo claro, grade discreta, linhas finas."""
    plt.rcParams.update({
        "figure.facecolor": FUNDO,
        "figure.subplot.top": 0.84,          # espaço para título + subtítulo
        "axes.facecolor": FUNDO,
        "savefig.facecolor": FUNDO,
        "axes.edgecolor": GRADE,
        "axes.labelcolor": TEXTO_2,
        "axes.titlecolor": TEXTO,
        "axes.titlesize": 11,
        "axes.titleweight": "bold",
        "axes.titlelocation": "left",
        "axes.labelsize": 9,
        "axes.grid": True,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "grid.color": GRADE,
        "grid.linewidth": 0.8,
        "xtick.color": TEXTO_2,
        "ytick.color": TEXTO_2,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.frameon": False,
        "legend.fontsize": 9,
        "legend.labelcolor": TEXTO,
        "lines.linewidth": 2,
        "font.size": 9,
    })


# ===============================
# Linha de comando e robô real/simulado
# ===============================
def argumentos(descricao):
    parser = argparse.ArgumentParser(description=descricao)
    parser.add_argument("--sim", action="store_true",
                        help="usa o simulador em vez do robô/câmera reais")
    parser.add_argument("--gravar", action="store_true",
                        help="grava em dados/recording.csv (ou dados/simulacao.csv com --sim)")
    return parser


def carregar(sim):
    """
    Retorna (robo, gt): os módulos reais ou o simulador com a mesma interface.
    Também aponta o controle.py para eles.
    """
    import controle
    if sim:
        import simulador
        robo, gt = simulador.criar()
    else:
        import robo
        import ground_truth as gt
    controle.robo, controle.gt = robo, gt
    return robo, gt


def conectar(robo):
    """Conecta com client_id único: dois clientes com o mesmo id derrubam um ao outro no broker."""
    robo.connect(BROKER, 1883, client_id=f"exemplo-{os.getpid()}")


class Movimento:
    """
    Roda a função de movimento numa thread, enquanto a animação roda na principal.
    A função recebe `esperar(segundos)`, que devolve False se a janela foi fechada.
    """

    def __init__(self, robo, funcao):
        self.robo = robo
        self._parar = threading.Event()
        self.terminou = False
        self._thread = threading.Thread(target=self._rodar, args=(funcao,), daemon=True)

    def esperar(self, segundos):
        return not self._parar.wait(segundos)

    def _rodar(self, funcao):
        try:
            funcao(self.esperar)
        finally:
            self.robo.send_wheel_speed(0, 0)
            self.terminou = True

    def iniciar(self):
        self._thread.start()

    def parar(self):
        self._parar.set()
        self._thread.join(timeout=2)


def rodas(robo, v_esq, v_dir, duracao, esperar):
    """Mantém uma velocidade nas rodas por `duracao` segundos (reenvia a cada 0,2 s)."""
    fim = time.time() + duracao
    while time.time() < fim:
        robo.send_wheel_speed(v_esq, v_dir)
        if not esperar(min(0.2, max(0.0, fim - time.time()))):
            return False
    return True


# ===============================
# Desenho
# ===============================
class Robo:
    """Robô desenhado em escala real: corpo triangular apontando para theta."""

    FORMA = np.array([[0.09, 0.0], [-0.06, 0.065], [-0.035, 0.0], [-0.06, -0.065]])

    def __init__(self, ax, cor, rotulo=None):
        self.patch = Polygon(self.FORMA, closed=True, facecolor=cor, edgecolor=FUNDO,
                             linewidth=1.5, zorder=5, label=rotulo)
        ax.add_patch(self.patch)

    def mover(self, x, y, theta):
        c, s = np.cos(theta), np.sin(theta)
        pts = self.FORMA @ np.array([[c, s], [-s, c]]) + [x, y]
        self.patch.set_xy(pts)


def desenhar_area(ax, gt):
    """Contorno claro da área do chão vista pela câmera."""
    try:
        cantos = gt.area_visivel()
    except Exception:
        return None
    ax.add_patch(Polygon(cantos, closed=True, facecolor=AREA, edgecolor=GRADE,
                         linewidth=1, zorder=0, label="área da câmera"))
    return cantos


def enquadrar(ax, xs, ys, margem=0.25, minimo=0.6):
    """Ajusta os limites de um mapa (aspecto 1:1) para conter os pontos."""
    if len(xs) == 0:
        return
    cx = (max(xs) + min(xs)) / 2
    cy = (max(ys) + min(ys)) / 2
    meio = max(max(xs) - min(xs), max(ys) - min(ys), minimo) / 2 + margem
    ax.set_xlim(cx - meio, cx + meio)
    ax.set_ylim(cy - meio, cy + meio)


def sem_saltos(t, graus):
    """Quebra a linha (NaN) onde o ângulo dá a volta de +180 para -180, sem riscos verticais."""
    t = np.asarray(t, dtype=float)
    graus = np.asarray(graus, dtype=float)
    saltos = np.where(np.abs(np.diff(graus)) > 180)[0] + 1
    return np.insert(t, saltos, np.nan), np.insert(graus, saltos, np.nan)


def salvar_figura(fig, nome):
    os.makedirs(PASTA_DADOS, exist_ok=True)
    caminho = os.path.join(PASTA_DADOS, nome)
    fig.savefig(caminho, dpi=150, bbox_inches="tight")
    print(f"[EXEMPLO] Figura salva em {caminho}")
