"""
Simulador com a mesma interface de robo.py (incluindo a câmera) e ground_truth.py, para rodar os
exemplos sem o robô e sem a câmera (opção --sim).

    - O robô real é um uniciclo com atraso de primeira ordem nas rodas.
    - A odometria integra as rodas com erros de calibração (raio e base), então
      deriva com o tempo como no robô de verdade.
    - A ground truth é a pose real com ruído de poucos mm, a 20 Hz, e só existe
      dentro da área vista pela câmera.
"""

import os
import time
import threading
import numpy as np

WHEEL_BASE = 0.143
TAU_RODA = 0.08          # s, atraso das rodas
ERRO_RAIO_DIR = 1.02     # roda direita "parece" 2% maior para a odometria
ERRO_BASE = 1.04         # base estimada 4% maior que a real
RUIDO_GT_POS = 0.003     # m
RUIDO_GT_ANG = 0.01      # rad
DT = 0.005
PERIODO_GT = 0.05
POSE_INICIAL = (1.0, 0.8, 0.0)


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _dentro(p, poligono):
    """Ponto dentro de polígono convexo (cantos em ordem)."""
    sinais = []
    for a, b in zip(poligono, np.roll(poligono, -1, axis=0)):
        sinais.append(np.cross(b - a, p - a) >= 0)
    return all(sinais) or not any(sinais)


class _Mundo:
    def __init__(self):
        self.lock = threading.Lock()
        self.q = np.array(POSE_INICIAL, dtype=float)       # pose real
        self.odom = np.zeros(3)                              # pose da odometria
        self.cmd = np.zeros(2)                               # referência das rodas
        self.rodas = np.zeros(2)                             # velocidade real das rodas
        self.gt_pose = None
        self.gt_ts = None
        self.on_pose = None
        self.area = self._area()
        self.gravacao = None
        self.rng = np.random.default_rng(0)
        threading.Thread(target=self._loop, daemon=True).start()

    @staticmethod
    def _area():
        try:
            import ground_truth
            return ground_truth.area_visivel()
        except Exception:
            return np.array([[2.4, 2.2], [2.4, -0.5], [-0.2, -0.4], [-0.2, 2.0]])

    def _loop(self):
        prox_gt = time.time()
        anterior = time.time()
        while True:
            time.sleep(DT)
            agora = time.time()
            dt = agora - anterior
            anterior = agora
            with self.lock:
                self.rodas += (self.cmd - self.rodas) * min(1.0, dt / TAU_RODA)
                vl, vr = self.rodas

                v, w = (vl + vr) / 2, (vr - vl) / WHEEL_BASE
                self.q += [v * np.cos(self.q[2]) * dt, v * np.sin(self.q[2]) * dt, w * dt]
                self.q[2] = _wrap(self.q[2])

                vl_o, vr_o = vl, vr * ERRO_RAIO_DIR
                v_o, w_o = (vl_o + vr_o) / 2, (vr_o - vl_o) / (WHEEL_BASE * ERRO_BASE)
                self.odom += [v_o * np.cos(self.odom[2]) * dt, v_o * np.sin(self.odom[2]) * dt, w_o * dt]
                self.odom[2] = _wrap(self.odom[2])
                q = self.q.copy()
                odom = self.odom.copy()

            if agora >= prox_gt:
                self._log(f"odometria,{odom[0]},{odom[1]},{odom[2]}")
                prox_gt = agora + PERIODO_GT
                if _dentro(q[:2], self.area):
                    pose = (q[0] + self.rng.normal(0, RUIDO_GT_POS),
                            q[1] + self.rng.normal(0, RUIDO_GT_POS),
                            _wrap(q[2] + self.rng.normal(0, RUIDO_GT_ANG)))
                    self.gt_pose, self.gt_ts = pose, time.time()
                    if self.on_pose:
                        self.on_pose(*pose)

    def _log(self, linha):
        if self.gravacao:
            self.gravacao.write(f"{linha},{time.time():.3f}\n")


class RoboSim:
    """Mesma interface de robo.py."""

    def __init__(self, mundo):
        self._m = mundo
        self._cam = _CameraSim(mundo)

    def connect(self, *args, **kwargs):
        print("[SIM] Robô simulado conectado.")

    def disconnect(self):
        print("[SIM] Robô simulado desconectado.")

    def get_odometry(self):
        with self._m.lock:
            return tuple(float(v) for v in self._m.odom)

    def get_wheel_speeds(self):
        with self._m.lock:
            return tuple(float(v) for v in self._m.rodas)

    def get_observation(self):
        return 0.0, 0.0, -1

    def get_status(self):
        return "Online"

    def send_wheel_speed(self, v_left, v_right):
        with self._m.lock:
            self._m.cmd[:] = (v_left, v_right)

    def set_odometry(self, x=0.0, y=0.0, theta=0.0):
        with self._m.lock:
            self._m.odom[:] = (x, y, theta)

    def stop(self):
        self.send_wheel_speed(0, 0)

    def set_camera_udp(self, ip="auto", port=5001):
        pass

    def start_camera(self, show=False, port=5001):
        print("[SIM] Câmera do robô simulada ligada.")
        return True

    def stop_camera(self):
        pass

    def show_camera(self, show=True):
        pass

    def get_camera_frame(self):
        return self._cam.get_frame()

    def get_camera_timestamp(self):
        return self._cam.get_timestamp()

    def get_camera_frame_id(self):
        return self._cam.get_frame_id()

    def get_camera_stats(self):
        return self._cam.get_stats()

    def record(self, cmd):
        m = self._m
        if cmd and not m.gravacao:
            pasta = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dados")
            m.gravacao = open(os.path.join(pasta, "simulacao.csv"), "a")
            m.gravacao.write("Inicio\n")
        elif not cmd and m.gravacao:
            arq, m.gravacao = m.gravacao, None
            arq.write("Fim\n")
            arq.close()

    def log_ground_truth(self, x, y, theta):
        self._m._log(f"ground_truth,{x},{y},{theta}")


class GroundTruthSim:
    """Mesma interface de ground_truth.py."""

    THETA_OFFSET = 0.0

    def __init__(self, mundo):
        self._m = mundo

    def start(self, show=False, on_pose=None, timeout=15):
        self._m.on_pose = on_pose
        print("[SIM] Câmera simulada iniciada.")
        return True

    def stop(self):
        self._m.on_pose = None

    def show_image(self, show=True):
        pass

    def get_pose(self):
        return self._m.gt_pose or (0.0, 0.0, 0.0)

    def get_timestamp(self):
        return self._m.gt_ts

    def get_frame(self):
        return None

    def get_pixel(self):
        return None

    def area_visivel(self):
        return self._m.area


class _CameraSim:
    """
    Câmera do robô simulada (usada pelo RoboSim). Em vez de vídeo real, recorta
    a vista de cima do chão logo à frente do robô (lajotas + marcos coloridos).
    """

    ESCALA = 200                     # px/m da textura do chão
    X0, Y1 = -0.5, 2.5               # canto superior esquerdo da textura (m)
    LARG, ALT = 320, 240
    PERTO, LONGE, LADO = 0.15, 0.75, 0.4   # m: faixa vista à frente do robô

    def __init__(self, mundo):
        self._m = mundo
        self._chao = self._textura()
        self._ultimo = None
        self._t_ultimo = 0.0
        self._frames = 0

    def _textura(self):
        import cv2
        n = int(3.0 * self.ESCALA)
        lado = int(0.25 * self.ESCALA)
        yy, xx = np.mgrid[0:n, 0:n]
        img = np.where(((xx // lado) + (yy // lado)) % 2 == 0, 225, 200).astype(np.uint8)
        img = np.dstack([img, img, img])
        rng = np.random.default_rng(1)
        cores = [(42, 120, 214), (235, 104, 52), (27, 175, 122), (237, 161, 0)]
        for i in range(14):
            cx, cy = rng.integers(40, n - 40, size=2)
            cv2.circle(img, (int(cx), int(cy)), 14, cores[i % 4], -1)
        return img

    def get_frame(self):
        import cv2
        agora = time.time()
        if self._ultimo is not None and agora - self._t_ultimo < 0.1:   # ~10 fps
            return self._ultimo
        with self._m.lock:
            px, py, th = self._m.q
        c, s = np.cos(th), np.sin(th)
        k = (self.LONGE - self.PERTO) / self.ALT                  # m por pixel
        # pixel da saída (u, v) -> ponto do chão (m) -> pixel da textura
        wx = [k * s, -k * c, px + c * self.LONGE - s * self.LADO]
        wy = [-k * c, -k * s, py + s * self.LONGE + c * self.LADO]
        M = np.array([[self.ESCALA * wx[0], self.ESCALA * wx[1], self.ESCALA * (wx[2] - self.X0)],
                      [-self.ESCALA * wy[0], -self.ESCALA * wy[1], self.ESCALA * (self.Y1 - wy[2])]])
        self._ultimo = cv2.warpAffine(self._chao, M, (self.LARG, self.ALT),
                                      flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR,
                                      borderValue=(120, 120, 120))
        self._t_ultimo = agora
        self._frames += 1
        return self._ultimo

    def get_timestamp(self):
        return self._t_ultimo or None

    def get_frame_id(self):
        return self._frames

    def get_stats(self):
        return {"frames": self._frames, "perdidos": 0, "fps": 10.0, "atraso": 0.08}


def criar():
    """Retorna (robo, gt) simulados que compartilham o mesmo mundo."""
    mundo = _Mundo()
    return RoboSim(mundo), GroundTruthSim(mundo)
