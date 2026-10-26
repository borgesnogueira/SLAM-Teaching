"""
Módulo ground_truth.py
----------------------
Ground truth da pose do robô pela câmera do teto (ArUco 4x4), em segundo plano.
Mesma lógica de gt.py, mas como biblioteca e independente do robo.py.

Funções principais:
    - start()         : conecta na Raspberry, abre o stream e começa o rastreamento
    - stop()          : encerra rastreamento, stream e SSH
    - get_pose()      : retorna (x, y, theta) em m, m, rad
    - get_timestamp() : time.time() da última pose (None se o robô nunca foi visto)
    - show()          : mostra/esconde a janela com a imagem da câmera

Uso típico:
    import ground_truth as gt

    gt.start(show=True)
    x, y, theta = gt.get_pose()
    gt.stop()
"""

import numpy as np
import threading
import time
import os

# ===============================
# Configuração
# ===============================
_DIR = os.path.dirname(os.path.abspath(__file__))
ENV_FILE = os.path.join(_DIR, "config", "env.txt")   # GT_IP, GT_USER, GT_PORT, GT_PASSWORD
H_FILE = os.path.join(_DIR, "config", "H.npy")     # homografia gerada por comp_homo.py
ROBOT_ID = 2       # ID do ArUco 4x4 em cima do robô (robot2 -> ID 2)
WIDTH = 720        # resolução em que H foi calibrada
HEIGHT = 720
CAM_X = 0.8        # posição X da câmera
CAM_Y = 0.8        # posição Y da câmera
CAM_Z = 2.79       # altura da câmera em relação ao chão
ROBO_H = 0.08      # altura do marcador em relação ao chão
THETA_OFFSET = 0.0 # rad, somado ao ângulo do marcador p/ apontar p/ a frente do robô
                   # (medir com: python3 controle.py calibrar)
WINDOW = "Ground truth"

# ===============================
# Variáveis internas
# ===============================
_pose = (0.0, 0.0, 0.0)
_timestamp = None
_thread = None
_running = False
_show = False       # janela com a imagem da câmera (ligável a qualquer momento)
_on_pose = None     # callback(x, y, theta) chamado a cada nova pose
_frame = None       # último frame (RGB) para quem quiser desenhar a imagem
_pixel = None       # (u, v) do centro do marcador no último frame em que foi visto


def _set_pose(x, y, theta):
    global _pose, _timestamp
    theta = np.arctan2(np.sin(theta + THETA_OFFSET), np.cos(theta + THETA_OFFSET))
    _pose = (float(x), float(y), float(theta))
    _timestamp = time.time()
    if _on_pose:
        _on_pose(*_pose)


def _px_to_world(u, v, H):
    pw = H @ np.array([u, v, 1.0], dtype=np.float64)
    x_ground = pw[0] / pw[2]
    y_ground = pw[1] / pw[2]

    # Corrige a paralaxe: o marcador está ROBO_H acima do plano calibrado
    scale = (CAM_Z - ROBO_H) / CAM_Z
    return (CAM_X + scale * (x_ground - CAM_X),
            CAM_Y + scale * (y_ground - CAM_Y))


def _read_env(path):
    """Lê um arquivo CHAVE=valor (formato .env)."""
    env = {}
    with open(path) as f:
        for line in f:
            if "=" in line and not line.lstrip().startswith("#"):
                key, value = line.split("=", 1)
                env[key.strip()] = value.strip()
    return env


def _loop(ready):
    global _running, _frame, _pixel
    import cv2
    import paramiko

    ssh = cap = None
    try:
        env = _read_env(ENV_FILE)
        ip, port = env["GT_IP"], int(env["GT_PORT"])
        H = np.load(H_FILE)

        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(ip, username=env["GT_USER"], password=env["GT_PASSWORD"])
        ssh.exec_command("pkill -f rpicam-vid")
        time.sleep(1)
        ssh.exec_command(
            f"rpicam-vid -t 0 --nopreview --mode 3280:2464 "
            f"--width {WIDTH} --height {HEIGHT} --framerate 20 "
            f"--codec mjpeg --inline --listen -o tcp://0.0.0.0:{port}"
        )
        time.sleep(3)

        cap = cv2.VideoCapture(f"tcp://{ip}:{port}", cv2.CAP_FFMPEG)
        if not cap.isOpened():
            raise RuntimeError("stream da câmera não abriu")

        detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters(),
        )
        print("[GT] Stream da câmera conectado.")
    except Exception as e:
        print(f"[GT] Erro ao iniciar: {e}")
        _running = False
    ready.set()

    window_open = False
    warned = False
    while _running:
        ret, frame = cap.read()
        if not ret:
            print("[GT] Falha ao ler frame.")
            break
        frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
        _frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids is not None and ROBOT_ID not in ids and not warned:
            warned = True
            print(f"[GT] Aviso: vendo ArUco {ids.flatten().tolist()}, mas não o ID {ROBOT_ID} (ROBOT_ID).")
        if ids is not None:
            for corner, mid in zip(corners, ids.flatten()):
                if mid != ROBOT_ID:
                    continue
                pts = corner[0]
                _pixel = (float(pts[:, 0].mean()), float(pts[:, 1].mean()))
                x, y = _px_to_world(*_pixel, H)
                fx, fy = _px_to_world(*pts[0], H)
                rx, ry = _px_to_world(*pts[1], H)
                _set_pose(x, y, np.arctan2(ry - fy, rx - fx))

        if _show:
            window_open = True
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            gx, gy, gth = _pose
            cv2.putText(frame, f"GT x={gx:.3f} y={gy:.3f} th={np.degrees(gth):.1f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 80), 2)
            cv2.imshow(WINDOW, frame)
            cv2.waitKey(1)
        elif window_open:
            window_open = False
            cv2.destroyWindow(WINDOW)
            cv2.waitKey(1)

    if cap is not None:
        cap.release()
    if window_open:
        cv2.destroyWindow(WINDOW)
        cv2.waitKey(1)
    if ssh is not None:
        ssh.exec_command("pkill -f rpicam-vid")
        ssh.close()
    _running = False

# ===============================
# Funções públicas
# ===============================
def start(show=False, on_pose=None, timeout=15):
    """
    Inicia o rastreamento do ArUco do robô em segundo plano.
    Bloqueia até o stream abrir (ou falhar / estourar o timeout).

    Args:
        show (bool): abre a janela com a imagem da câmera.
        on_pose (callable): chamada como on_pose(x, y, theta) a cada nova pose
                            (ex.: robo.log_ground_truth para gravar no recording.csv).

    Returns:
        True se o rastreamento está rodando.
    """
    global _thread, _running, _on_pose
    show_image(show)
    _on_pose = on_pose
    if _running:
        return True
    _running = True
    ready = threading.Event()
    _thread = threading.Thread(target=_loop, args=(ready,), daemon=True)
    _thread.start()
    ready.wait(timeout)
    return _running


def stop():
    """Encerra o rastreamento, o stream da câmera e o SSH."""
    global _running
    _running = False
    if _thread is not None:
        _thread.join(timeout=5)


def show_image(show=True):
    """Mostra (True) ou esconde (False) a janela com a imagem da câmera."""
    global _show
    _show = bool(show)


def get_pose():
    """
    Retorna a última pose medida pela câmera.

    Returns:
        tuple (x, y, theta) em m, m, rad
    """
    return _pose


def get_timestamp():
    """Retorna o time.time() da última pose (None se o robô nunca foi visto)."""
    return _timestamp


def get_frame():
    """Retorna o último frame da câmera (RGB, WIDTH x HEIGHT), ou None."""
    return _frame


def get_pixel():
    """Retorna (u, v) do marcador no último frame em que foi visto, ou None."""
    return _pixel


def area_visivel():
    """
    Retorna os 4 cantos (x, y) da área do chão vista pela câmera, em metros,
    na ordem da imagem: sup-esq, sup-dir, inf-dir, inf-esq.
    """
    H = np.load(H_FILE)
    cantos = [(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT)]
    return np.array([_px_to_world(u, v, H) for u, v in cantos])


if __name__ == "__main__":
    # Teste rápido: mostra a imagem e imprime a pose até Ctrl+C
    if start(show=True):
        try:
            while True:
                x, y, theta = get_pose()
                print(f"x={x:.3f}  y={y:.3f}  θ={np.degrees(theta):.1f}°")
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
    stop()
