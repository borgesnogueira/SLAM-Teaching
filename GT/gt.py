import cv2
import time
import paramiko
import numpy as np
from pathlib import Path
from dotenv import load_dotenv
import os
import csv

# ============================================================
# CONFIGURAÇÕES DE CONEXÃO E STREAM
# ============================================================

load_dotenv()

RASPBERRY_IP = os.getenv("GT_IP")
USERNAME     = os.getenv("GT_USER")
STREAM_PORT  = int(os.getenv("GT_PORT"))
PASSWORD     = os.getenv("GT_PASSWORD")

STREAM_URL  = f"tcp://{RASPBERRY_IP}:{STREAM_PORT}"

# ============================================================
# CONFIGURAÇÃO DA RESOLUÇÃO E FPS
# ============================================================

STREAM_PRESETS = {
    "LOW":    {"WIDTH": 320,  "HEIGHT": 240,  "FPS": 15},
    "MEDIUM": {"WIDTH": 640,  "HEIGHT": 480,  "FPS": 30},
    "HIGH":   {"WIDTH": 1280, "HEIGHT": 720,  "FPS": 30},
    "FULLHD": {"WIDTH": 1920, "HEIGHT": 1080, "FPS": 30},
    "FAST":   {"WIDTH": 640,  "HEIGHT": 480,  "FPS": 60},
    "EQUAL":  {"WIDTH": 720,  "HEIGHT": 720,  "FPS": 30},
}

PROFILE = "EQUAL"
# PROFILE = "MEDIUM"

WIDTH  = STREAM_PRESETS[PROFILE]["WIDTH"]
HEIGHT = STREAM_PRESETS[PROFILE]["HEIGHT"]
FPS    = STREAM_PRESETS[PROFILE]["FPS"]


# ============================================================
# CONFIGURAÇÃO DE RASTREAMENTO E GRAVAÇÃO
# ============================================================

SHOW_INFOS = True  # Exibe informações de posição e orientação na tela


# ============================================================
# CONFIGURAÇÃO DA CÂMERA (POSIÇÃO FIXA)
# ============================================================
CAM_X = 0.8  # Posição X da câmera
CAM_Y = 0.8  # Posição Y da câmera
CAM_Z = 2.79 # Altura da câmera em relação ao chão

# ============================================================
# DIMENSÃO REAL DO ROBÔ (PARA AJUSTE DE ESCALA)
# ============================================================

ROBO_H = 0.08 # Ex: 8 cm

# ============================================================
# ID DO ROBÔ E VARIÁVEIS DE GRAVAÇÃO
# ============================================================

ROBOT_ID = 0   # ID do ArUco 4x4 em cima do robô

# ============================================================
# VARIÁVEIS DE CONTROLE DE GRAVAÇÃO
# ============================================================

is_recording = False
trajectory_data = []      # Armazena [timestamp, x, y, theta]
trajectory_pixels = []    # Armazena (cx, cy) para plotar na imagem
initial_frame = None
final_frame = None

# ============================================================
# CARREGA H
# ============================================================

H_FILE = Path("C:\\Users\\igojo\\Desktop\\SLAM-Teaching\\GT\\H.npy")

try:
    H = np.load(str(H_FILE))
    print(f"[H] Carregada de {H_FILE}")
except FileNotFoundError:
    print(f"[ERRO] {H_FILE} nao encontrada.")
    print("      Rode comp_homo.py antes e pressione [s].")
    exit()

# ============================================================
# DETECTOR 4x4 E HOMOGRAFIA
# ============================================================

aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)



def px_to_world(u, v, H):
    p  = np.array([u, v, 1.0], dtype=np.float64)
    pw = H @ p
    x_ground = pw[0] / pw[2]
    y_ground = pw[1] / pw[2]

    scale = (CAM_Z - ROBO_H) / CAM_Z
    
    x_real = CAM_X + scale * (x_ground - CAM_X)
    y_real = CAM_Y + scale * (y_ground - CAM_Y)
    
    return x_real, y_real

# ============================================================
# SSH + STREAM
# ============================================================

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

print("[INFO] Conectando via SSH...")
ssh.connect(RASPBERRY_IP, username=USERNAME, password=PASSWORD)
ssh.exec_command("pkill -f rpicam-vid")
time.sleep(1)


cmd = (
    f"rpicam-vid -t 0 "
    f"--nopreview "
    f"--mode 3280:2464 "
    f"--width 720 "
    f"--height 720 "
    f"--framerate 20 "
    f"--codec mjpeg "
    f"--inline "
    f"--listen "
    f"-o tcp://0.0.0.0:{STREAM_PORT}"
)


print(f"[INFO] Iniciando stream com comando:\n{cmd}")

ssh.exec_command(cmd)
time.sleep(3)

cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)
if not cap.isOpened():
    print("[ERRO] Stream não abriu.")
    ssh.close()
    exit()

print("[INFO] Stream conectado!")
print("=" * 55)
print(f"  RASTREAMENTO ArUco 4x4  |  ID alvo = {ROBOT_ID}")
print("=" * 55)
print("  [s] INICIAR gravação da trajetória")
print("  [f] FINALIZAR gravação, salvar CSV e imagens")
print("  [q] SAIR sem salvar")
print("=" * 55)
print()

fps_time    = time.time()
fps_counter = 0
fps_value   = 0.0

# ============================================================
# LOOP PRINCIPAL
# ============================================================

while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERRO] Frame falhou.")
        break

    clean_frame = frame.copy()
    # print(f"[DEBUG] Frame recebido: {frame.shape[1]}x{frame.shape[0]}")
    frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners_all, ids_raw, _ = detector.detectMarkers(gray)

    ids_all = ids_raw.flatten().tolist() if ids_raw is not None else []

    if ids_raw is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners_all, ids_raw)

    found = ROBOT_ID in ids_all

    for corner, mid in zip(corners_all, ids_all):
        pts = corner[0]
        cx  = pts[:, 0].mean()
        cy  = pts[:, 1].mean()

        wx, wy = px_to_world(cx, cy, H)

        fx, fy = px_to_world(*pts[0], H)
        rx, ry = px_to_world(*pts[1], H)
        theta  = np.arctan2(ry - fy, rx - fx)
        theta_deg = np.degrees(theta)

        cxi, cyi = int(cx), int(cy)

        cv2.circle(frame, (cxi, cyi), 6, (0, 220, 80), -1)
        cv2.arrowedLine(frame, tuple(pts[0].astype(int)), tuple(pts[1].astype(int)),
                        (0, 80, 255), 3, tipLength=0.3)

        line1 = f"ID {mid} | x={wx:.3f}m  y={wy:.3f}m"
        line2 = f"theta={theta_deg:.1f} deg"
        cv2.putText(frame, line1, (cxi + 10, cyi - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 80), 2)
        cv2.putText(frame, line2, (cxi + 10, cyi + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 180, 255), 2)

        if is_recording and mid == ROBOT_ID:
            timestamp = time.time()
            trajectory_data.append([timestamp, wx, wy, theta_deg])
            trajectory_pixels.append((cxi, cyi))

    
    if SHOW_INFOS:
        if is_recording and len(trajectory_pixels) > 1:
            for i in range(1, len(trajectory_pixels)):
                cv2.line(frame, trajectory_pixels[i-1], trajectory_pixels[i], (255, 0, 255), 2)

    fps_counter += 1
    now = time.time()
    if now - fps_time >= 1.0:
        fps_value   = fps_counter / (now - fps_time)
        fps_counter = 0
        fps_time    = now

    if(SHOW_INFOS):
        # HUD
        status       = f"ID {ROBOT_ID}: DETECTADO" if found else f"ID {ROBOT_ID}: nao visto"
        status_color = (0, 220, 80) if found else (0, 100, 255)
        
        rec_status   = "GRAVANDO" if is_recording else "AGUARDANDO INICIO"
        rec_color    = (0, 0, 255) if is_recording else (0, 255, 255)

        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(frame, f"FPS: {fps_value:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, rec_status, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, rec_color, 2)

    cv2.imshow("Rastreamento 4x4", frame)

    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('s') and not is_recording:
        is_recording = True
        initial_frame = clean_frame # Salva a foto inicial limpa
        print("[INFO] Gravação INICIADA.")
        
    elif key == ord('f'):
        if is_recording:
            final_frame = clean_frame # Salva a foto final limpa
            print("[INFO] Gravação FINALIZADA pelo usuário.")
        break
        
    elif key == ord('q'):
        # Força saída sem salvar
        is_recording = False
        break

# ============================================================
# SALVAMENTO DE DADOS (CSV e IMAGEM)
# ============================================================

if is_recording and final_frame is not None and initial_frame is not None:
    csv_filename = "trajetoria_robo.csv"
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["timestamp", "x", "y", "theta_deg"])
        writer.writerows(trajectory_data)
    print(f"[SUCESSO] Dados salvos em '{csv_filename}'. Total de pontos: {len(trajectory_data)}")

    # 2. Mesclar e Salvar Imagem
    blended_image = cv2.addWeighted(initial_frame, 0.5, final_frame, 0.5, 0)
    
    if len(trajectory_pixels) > 1:
        for i in range(1, len(trajectory_pixels)):
            cv2.line(blended_image, trajectory_pixels[i-1], trajectory_pixels[i], (0, 0, 255), 2)
            
    img_filename = "resultado_trajetoria.jpg"
    cv2.imwrite(img_filename, blended_image)
    print(f"[SUCESSO] Imagem mesclada salva em '{img_filename}'.")

# ============================================================
# FINALIZAÇÃO GERAL
# ============================================================

print("[INFO] Encerrando processos e conexões...")
cap.release()
cv2.destroyAllWindows()
ssh.exec_command("pkill -f rpicam-vid")
ssh.close()
print("[INFO] Finalizado.")