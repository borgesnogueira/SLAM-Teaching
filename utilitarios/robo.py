"""
Módulo robo.py
--------------
Biblioteca de alto nível para comunicação com o robô via MQTT.

Funções principais:
    - connect()           : conecta ao broker e inicializa comunicação
    - disconnect()        : encerra conexão
    - get_odometry()      : retorna (x, y, theta)
    - get_observation()   : retorna (r, theta, label)
    - log_ground_truth()  : grava uma pose de ground truth no recording.csv
    - send_wheel_speed()  : envia velocidades para as rodas
    - stop()              : envia comando de parada
    - start_camera()      : liga o vídeo da câmera do robô (UDP)
    - get_camera_frame()  : último frame da câmera do robô (RGB)
    - record()            : grava recording.csv (+ frames da câmera, se ligada)

Uso típico:
    import robo

    robo.connect()
    x, y, theta = robo.get_odometry()
    print("Posição:", x, y, theta)
    robo.send_wheel_speed(0.1, 0.1)
    robo.start_camera()
    frame = robo.get_camera_frame()
    robo.stop()
    robo.disconnect()
"""

import paho.mqtt.client as mqtt
import simplejson as json
import numpy as np
import threading
import socket
import struct
import time
import os

# ===============================
# Configuração (deve casar com firmware/main/main.c)
# ===============================
ROBOT_PREFIX = "swarm/robot2"
TOPIC_ODOM = f"{ROBOT_PREFIX}/telemetry/odometry"     # {"odometria": "x, y, theta", "timestamp_us": N}
TOPIC_WHEELS = f"{ROBOT_PREFIX}/telemetry/wheels"     # {"wheels": "wl, wr"} em rad/s
TOPIC_STATUS = f"{ROBOT_PREFIX}/telemetry/status"     # "Online" (e LWT)
TOPIC_OBS = f"{ROBOT_PREFIX}/observation"
TOPIC_CMD_WHEELS = f"{ROBOT_PREFIX}/cmd/wheels"       # {"l": wl, "r": wr} em rad/s
TOPIC_CMD_ODOM = f"{ROBOT_PREFIX}/cmd/odometry"       # {"x", "y", "theta"}
TOPIC_CMD_MODE = f"{ROBOT_PREFIX}/cmd/mode"           # qualquer JSON -> parada de emergência
TOPIC_CMD_CAMERA = f"{ROBOT_PREFIX}/cmd/camera_udp"   # {"ip", "port"}
TOPIC_GROUND_TRUTH = "ground_truth/data"

WHEEL_RADIUS = 0.0823  # m, igual a WHEEL_RADIUS no firmware
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dados")
RECORDING_FILE = os.path.join(DATA_DIR, "recording.csv")

# Câmera do robô: JPEG (VGA, ~10 fps) por UDP, em fragmentos de até 1200 bytes.
# Cabeçalho = CameraUdpFragHeader_t (packed, little-endian):
#   frame_id u32 | timestamp_us u64 | total_frags u16 | frag_idx u16 | payload_len u16
# timestamp_us é o instante da captura em tempo Unix (robô sincroniza por SNTP).
CAMERA_PORT = 5001     # CAMERA_UDP_DEFAULT_PORT
CAMERA_HEADER = struct.Struct("<IQHHH")
CAMERA_WINDOW = "Camera do robo"

# ===============================
# Variáveis internas
# ===============================
_odom = np.array([[0.0], [0.0], [0.0]])
_vels = np.array([[0.0], [0.0]])
_obs = np.array([[0.0], [0.0], [-1]])
_ground_truth = np.array([[0.0], [0.0], [0.0]])
_client = None
_connected = False
_recording = False
_recording_file = None
_status = None
_odom_timestamp_us = None
_broker = None
_file_lock = threading.Lock()
_cam_frame = None
_cam_jpeg = None
_cam_timestamp = None
_cam_frame_id = None
_cam_thread = None
_cam_running = False
_cam_show = False
_cam_dir = None        # pasta dos frames enquanto record(True)
_cam_stats = {"frames": 0, "perdidos": 0, "fps": 0.0, "atraso": None}


def _write(line):
    """
    Escreve uma linha na gravação, com o time.time() como última coluna
    (thread do MQTT e log_ground_truth escrevem concorrentemente).
    """
    with _file_lock:
        if _recording and _recording_file:
            _recording_file.write(f"{line.rstrip()},{time.time():.3f}\n")

# ===============================
# Callbacks internos do MQTT
# ===============================
def _on_connect(client, userdata, flags, reason_code, properties):
    global _connected
    if reason_code == 0:
        _connected = True
        client.subscribe(TOPIC_ODOM)
        client.subscribe(TOPIC_WHEELS)
        client.subscribe(TOPIC_STATUS)
        client.subscribe(TOPIC_OBS)
        client.subscribe(TOPIC_GROUND_TRUTH)
        print("[ROBO] Conectado ao broker.")
    else:
        print(f"[ROBO] Falha na conexão. Código: {reason_code}")


def _on_message(client, userdata, msg):
    global _odom, _vels, _obs, _ground_truth, _recording, _recording_file, _status, _odom_timestamp_us

    if msg.topic == TOPIC_STATUS:
        _status = msg.payload.decode(errors="replace")
        print(f"[ROBO] Status do robô: {_status}")

    elif msg.topic == TOPIC_ODOM:
        try:
            data = json.loads(msg.payload.decode())
            values = data["odometria"].split(', ')
            _odom = np.array(
                [[values[0]], [values[1]], [values[2]]], dtype="float32"
            )
            _odom_timestamp_us = data.get("timestamp_us")
            # Notificar que a odometria foi atualizada
            _write(f"odometria,{float(_odom[0])},{float(_odom[1])},{float(_odom[2])}\n")
        except Exception as e:
            print(f"[ROBO] Erro ao processar odometria: {e}")

    elif msg.topic == TOPIC_WHEELS:
        data = None
        try:
            data = json.loads(msg.payload.decode())
            values = data["wheels"].split(', ')
            # Firmware publica rad/s; a interface do módulo é em m/s
            _vels = np.array(
                [[values[0]], [values[1]]], dtype="float32"
            ) * WHEEL_RADIUS
            _write(f"velocidades,{float(_vels[0])},{float(_vels[1])}\n")
        except Exception as e:
            print(f"[ROBO] Erro ao processar velocidades: {e}, {data}")

    elif msg.topic == TOPIC_OBS:
        try:
            data = json.loads(msg.payload.decode())
            _obs = np.array([[data["r"]], [data["theta"]], [data["label"]]])
            _write(f"observacao,{float(_obs[0])},{float(_obs[1])},{int(_obs[2])}\n")
        except Exception as e:
            print(f"[ROBO] Erro ao processar observação: {e}")

    elif msg.topic == TOPIC_GROUND_TRUTH:
        try:
            data = json.loads(msg.payload.decode())
            _ground_truth = np.array(
                [[data["x"]], [data["y"]], [data["theta_rad"]]], dtype="float32"
            )
            # print(f"Ground Truth na saída: x={_ground_truth[0]:.2f}, y={_ground_truth[1]:.2f}, θ={_ground_truth[2]:.2f}")

            _write(f"ground_truth,{float(_ground_truth[0])},{float(_ground_truth[1])},{float(_ground_truth[2])}\n")
        except Exception as e:
            print(f"[ROBO] Erro ao processar ground truth: {e}")

# ===============================
# Recepção da câmera do robô
# ===============================
def _camera_loop(sock):
    global _cam_running, _cam_frame, _cam_jpeg, _cam_timestamp, _cam_frame_id
    import cv2

    montando = {}           # frame_id -> {"total", "frags": {idx: bytes}}
    chegadas = []           # instantes dos últimos frames completos (para o fps)
    ultimo_completo = None
    window_open = False

    while _cam_running:
        try:
            pacote, _ = sock.recvfrom(2048)
        except socket.timeout:
            pacote = None

        if pacote and len(pacote) >= CAMERA_HEADER.size:
            fid, ts_us, total, idx, tam = CAMERA_HEADER.unpack_from(pacote)
            item = montando.setdefault(fid, {"total": total, "frags": {}})
            item["frags"][idx] = pacote[CAMERA_HEADER.size:CAMERA_HEADER.size + tam]

            if len(item["frags"]) == item["total"]:
                jpeg = b"".join(item["frags"][i] for i in range(item["total"]))
                del montando[fid]
                # Frames mais antigos que não completaram não vão mais completar
                for antigo in [f for f in montando if f < fid]:
                    del montando[antigo]
                if ultimo_completo is not None and fid > ultimo_completo + 1:
                    _cam_stats["perdidos"] += fid - ultimo_completo - 1
                ultimo_completo = fid

                bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if bgr is None:
                    _cam_stats["perdidos"] += 1
                    continue
                agora = time.time()
                _cam_frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                _cam_jpeg = jpeg
                _cam_timestamp = ts_us / 1e6
                _cam_frame_id = fid
                _cam_stats["frames"] += 1
                _cam_stats["atraso"] = agora - _cam_timestamp
                chegadas = [t for t in chegadas if agora - t <= 2.0] + [agora]
                if len(chegadas) > 1:
                    _cam_stats["fps"] = (len(chegadas) - 1) / (chegadas[-1] - chegadas[0])

                pasta = _cam_dir
                if pasta:
                    os.makedirs(pasta, exist_ok=True)
                    with open(os.path.join(pasta, f"{ts_us}.jpg"), "wb") as f:
                        f.write(jpeg)
                    _write(f"camera,{fid},{_cam_timestamp:.6f}\n")

                if _cam_show:
                    window_open = True
                    cv2.imshow(CAMERA_WINDOW, bgr)
                    cv2.waitKey(1)

        if not _cam_show and window_open:
            window_open = False
            cv2.destroyWindow(CAMERA_WINDOW)
            cv2.waitKey(1)

    if window_open:
        cv2.destroyWindow(CAMERA_WINDOW)
        cv2.waitKey(1)
    sock.close()
    _cam_running = False

# ===============================
# Funções públicas
# ===============================
def connect(broker="192.168.0.125", port=1883, client_id="aluno", timeout=5):
    """
    Conecta ao broker MQTT e espera até estar conectado.
    """
    global _client, _connected, _broker
    _broker = broker
    _client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    _client.on_connect = _on_connect
    _client.on_message = _on_message

    try:
        _client.connect(broker, port)
        _client.loop_start()

        # Espera até _connected ficar True ou estourar o timeout
        start = time.time()
        while not _connected and (time.time() - start) < timeout:
            time.sleep(0.1)

        if not _connected:
            raise TimeoutError("[ROBO] Timeout ao conectar no broker.")

    except Exception as e:
        print(f"[ROBO] Erro ao conectar: {e}")
        _connected = False

def disconnect():
    """Encerra a comunicação com o robô."""
    global _client, _connected
    stop_camera()
    if _client:
        _client.loop_stop()
        _client.disconnect()
        _connected = False
        print("[ROBO] Desconectado.")


def get_odometry():
    """
    Retorna a odometria atual.

    Returns:
        tuple (x, y, theta)
    """
    return float(_odom[0]), float(_odom[1]), float(_odom[2])

def get_wheel_speeds():
    """
    Retorna as velocidades atuais das rodas.

    Returns:
        tuple (v_left, v_right)
    """
    return float(_vels[0]), float(_vels[1])


def get_observation():
    """
    Retorna a última observação do sensor.

    Returns:
        tuple (r, theta, label)
    """
    return float(_obs[0]), float(_obs[1]), int(_obs[2])


def send_wheel_speed(v_left, v_right):
    """
    Envia velocidades para as rodas.

    Args:
        v_left (float): velocidade da roda esquerda (m/s).
        v_right (float): velocidade da roda direita (m/s).
    """
    if not _client or not _connected:
        print("[ROBO] Cliente não conectado. Use connect() antes.")
        return

    # Firmware espera referência angular da roda (rad/s): w = v / R
    msg = {"l": float(v_left) / WHEEL_RADIUS, "r": float(v_right) / WHEEL_RADIUS}
    _client.publish(TOPIC_CMD_WHEELS, json.dumps(msg))


def set_odometry(x=0.0, y=0.0, theta=0.0):
    """
    Redefine a odometria do robô (ex.: zerar no início de um experimento).
    """
    if not _client or not _connected:
        print("[ROBO] Cliente não conectado. Use connect() antes.")
        return

    msg = {"x": float(x), "y": float(y), "theta": float(theta)}
    _client.publish(TOPIC_CMD_ODOM, json.dumps(msg))


def set_camera_udp(ip="auto", port=5001):
    """
    Define o destino do stream UDP da câmera do robô (baixo nível; prefira start_camera()).
    ip="auto" usa o IP deste PC na rede do broker; ip=None desabilita o envio.
    """
    if not _client or not _connected:
        print("[ROBO] Cliente não conectado. Use connect() antes.")
        return

    if ip == "auto":
        # Descobre por qual interface este PC alcança o broker (não envia nada)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect((_broker, 1))
            ip = s.getsockname()[0]
        print(f"[ROBO] Câmera do robô enviando para {ip}:{port}")

    payload = json.dumps({"ip": ip, "port": int(port)}) if ip else "off"
    _client.publish(TOPIC_CMD_CAMERA, payload)


def start_camera(show=False, port=CAMERA_PORT):
    """
    Liga o vídeo da câmera do robô: abre a porta UDP deste PC, recebe em
    segundo plano e pede ao robô para enviar para cá.

    Args:
        show (bool): abre uma janela com o vídeo.

    Returns:
        True se a recepção está rodando.
    """
    global _cam_thread, _cam_running
    show_camera(show)
    if _cam_running:
        return True
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        sock.bind(("0.0.0.0", port))
        sock.settimeout(0.5)
    except OSError as e:
        print(f"[ROBO] Não foi possível abrir a porta UDP {port}: {e}")
        return False
    _cam_running = True
    _cam_thread = threading.Thread(target=_camera_loop, args=(sock,), daemon=True)
    _cam_thread.start()
    set_camera_udp("auto", port)
    return True


def stop_camera():
    """Pede ao robô para parar o vídeo e encerra a recepção."""
    global _cam_running
    if not _cam_running:
        return
    if _client and _connected:
        set_camera_udp(None)
    _cam_running = False
    if _cam_thread is not None:
        _cam_thread.join(timeout=2)


def show_camera(show=True):
    """Mostra (True) ou esconde (False) a janela com o vídeo do robô."""
    global _cam_show
    _cam_show = bool(show)


def get_camera_frame():
    """Último frame completo da câmera do robô (RGB, 640x480), ou None."""
    return _cam_frame


def get_camera_jpeg():
    """Bytes JPEG do último frame, como vieram do robô, ou None."""
    return _cam_jpeg


def get_camera_timestamp():
    """Instante da captura do último frame (s, tempo Unix, mesmo relógio de time.time()), ou None."""
    return _cam_timestamp


def get_camera_frame_id():
    return _cam_frame_id


def get_camera_stats():
    """
    Returns:
        dict com fps (últimos 2 s), frames (recebidos), perdidos e
        atraso (s entre a captura no robô e a chegada aqui).
    """
    return dict(_cam_stats)


def get_status():
    """Retorna o último status publicado pelo robô ("Online", LWT, ou None)."""
    return _status


def get_odometry_timestamp_us():
    """Retorna o timestamp (µs) da última odometria recebida, ou None."""
    return _odom_timestamp_us

def get_ground_truth():
    """
    Retorna a ground truth atual.

    Returns:
        tuple (x, y, theta)
    """
    gnd = float(_ground_truth[0]), float(_ground_truth[1]), float(_ground_truth[2])
    # print(f"Ground Truth na ponta da faca: x={gnd[0]:.2f}, y={gnd[1]:.2f}, θ={gnd[2]:.2f}")
    return gnd


def log_ground_truth(x, y, theta):
    """
    Grava uma pose de ground truth no recording.csv (se estiver gravando).
    Ex.: ground_truth.start(on_pose=robo.log_ground_truth)
    """
    _write(f"ground_truth,{float(x)},{float(y)},{float(theta)}\n")

def record(cmd):
    """
    Liga/desliga a gravação em dados/recording.csv. Com a câmera ligada, os
    frames também são salvos em dados/camera_AAAAMMDD_HHMMSS/<timestamp_us>.jpg
    e cada um gera uma linha "camera,frame_id,timestamp" no CSV.
    """
    global _recording, _recording_file, _cam_dir
    with _file_lock:
        if cmd and not _recording:
            os.makedirs(DATA_DIR, exist_ok=True)
            _recording_file = open(RECORDING_FILE, "a")
            _recording_file.write(f"Inicio\n")
            _recording = True
            _cam_dir = os.path.join(DATA_DIR, time.strftime("camera_%Y%m%d_%H%M%S"))
        elif not cmd and _recording:
            _recording = False
            _cam_dir = None
            _recording_file.write(f"Fim\n")
            _recording_file.close()
    
def stop():
    """
    Envia comando de parada para o robô.
    """
    if not _client or not _connected:
        print("[ROBO] Cliente não conectado. Use connect() antes.")
        return
    
    send_wheel_speed(0, 0)
    # cmd/mode zera referências e reseta os PIDs no firmware
    _client.publish(TOPIC_CMD_MODE, json.dumps({"mode": "stop"}))
    print("[ROBO] Comando STOP enviado.")
