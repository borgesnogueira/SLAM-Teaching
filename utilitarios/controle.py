"""
Módulo controle.py
------------------
Controle do robô em cima de robo.py (MQTT) e ground_truth.py (câmera do teto).

Funções principais:
    - positioning_control() : lei de controle (v, w) para ir até um ponto
    - unicycle_to_wheels()  : converte (v, w) em velocidades das rodas
    - go_to()               : leva o robô até (x, y) em malha fechada
    - joystick()            : teleoperação com gamepad (pygame)
    - calibrar_theta()      : mede o THETA_OFFSET do marcador da GT

Uso pela linha de comando:
    python3 controle.py goto 1.0 0.8          # vai até (1.0, 0.8) usando a GT
    python3 controle.py goto 1.0 0.8 --odom   # idem, usando a odometria
    python3 controle.py joystick              # segure X para andar, quadrado encerra
    python3 controle.py calibrar              # anda reto ~15 cm e mede o offset do marcador
"""

import numpy as np
import time
import os

import robo
import ground_truth as gt

# ===============================
# Configuração
# ===============================
WHEEL_BASE = 0.143    # m, igual a WHEEL_BASE no firmware
V_MAX = 0.2           # m/s, velocidade linear máxima
W_MAX = 2.0           # rad/s, velocidade angular máxima
V_RODA_MAX = 0.25     # m/s, limite de cada roda
K1 = 0.8              # ganho de distância
K2 = 1.6              # ganho de ângulo
PERIODO = 0.05        # s, período do laço de controle (20 Hz)
GT_MAX_IDADE = 0.5    # s, GT mais velha que isso é considerada perdida

# Gamepad (numeração do pygame/SDL para controle de PS4/PS5)
BTN_HABILITA = 0      # X: segurar para andar
BTN_FIM = 2           # quadrado: encerra
EIXO_W = 0            # analógico esquerdo, horizontal
EIXO_V = 1            # analógico esquerdo, vertical
ZONA_MORTA = 0.08


# ===============================
# Lei de controle
# ===============================
def angle_wrap(ang):
    """Leva o ângulo para [-pi, pi]."""
    return np.arctan2(np.sin(ang), np.cos(ang))


def positioning_control(q, ref, k1=K1, k2=K2):
    """
    Controle de posição: q = (x, y, theta), ref = (x, y).

    Returns:
        tuple (v, w)
    """
    dx = ref[0] - q[0]
    dy = ref[1] - q[1]

    erro_theta = angle_wrap(np.arctan2(dy, dx) - q[2])
    dl = np.hypot(dx, dy) * np.cos(erro_theta)

    v = np.clip(k1 * dl, -V_MAX, V_MAX)
    w = np.clip(k2 * erro_theta, -W_MAX, W_MAX)
    return v, w


def unicycle_to_wheels(v, w):
    """
    Converte (v, w) do uniciclo em velocidades das rodas (m/s), saturando
    proporcionalmente para não distorcer a curva.

    Returns:
        tuple (v_left, v_right)
    """
    v_left = v - w * WHEEL_BASE / 2
    v_right = v + w * WHEEL_BASE / 2
    maior = max(abs(v_left), abs(v_right))
    if maior > V_RODA_MAX:
        v_left *= V_RODA_MAX / maior
        v_right *= V_RODA_MAX / maior
    return v_left, v_right


# ===============================
# Fontes de pose
# ===============================
def pose_gt():
    """Pose da câmera, ou None se estiver desatualizada."""
    ts = gt.get_timestamp()
    if ts is None or time.time() - ts > GT_MAX_IDADE:
        return None
    return gt.get_pose()


def pose_odom():
    return robo.get_odometry()


# ===============================
# Comportamentos
# ===============================
def go_to(ref, get_pose=pose_gt, tol=0.05, timeout=60):
    """
    Leva o robô até ref = (x, y). Para quando chega a menos de tol metros,
    estoura o timeout ou a pose some (ex.: robô fora da câmera).

    Returns:
        True se chegou.
    """
    inicio = time.time()
    while time.time() - inicio < timeout:
        q = get_pose()
        if q is None:
            robo.send_wheel_speed(0, 0)
            print("[CTRL] Pose indisponível, robô parado.")
            time.sleep(PERIODO)
            continue

        dist = np.hypot(ref[0] - q[0], ref[1] - q[1])
        if dist < tol:
            robo.send_wheel_speed(0, 0)
            print(f"[CTRL] Chegou em ({q[0]:.3f}, {q[1]:.3f}), erro {dist*100:.1f} cm.")
            return True

        v, w = positioning_control(q, ref)
        robo.send_wheel_speed(*unicycle_to_wheels(v, w))
        time.sleep(PERIODO)

    robo.send_wheel_speed(0, 0)
    print("[CTRL] Timeout antes de chegar.")
    return False


def joystick():
    """
    Teleoperação: segure BTN_HABILITA e use o analógico; BTN_FIM encerra.
    """
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    import pygame

    pygame.display.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[CTRL] Nenhum gamepad encontrado.")
        return
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"[CTRL] Gamepad: {js.get_name()}. Segure X para andar, quadrado encerra.")

    def eixo(i):
        a = js.get_axis(i)
        return 0.0 if abs(a) < ZONA_MORTA else a

    andando = False
    try:
        while True:
            pygame.event.pump()
            if js.get_button(BTN_FIM):
                break

            if js.get_button(BTN_HABILITA):
                # Eixos do pygame: para cima/esquerda é negativo
                v = -eixo(EIXO_V) * V_MAX
                w = -eixo(EIXO_W) * W_MAX
                robo.send_wheel_speed(*unicycle_to_wheels(v, w))
                andando = True
            elif andando:
                robo.send_wheel_speed(0, 0)
                andando = False
            time.sleep(PERIODO)
    finally:
        robo.send_wheel_speed(0, 0)
        pygame.quit()
    print("[CTRL] Joystick encerrado.")


def calibrar_theta(v=0.1, duracao=1.5):
    """
    Anda reto para a frente e compara a direção do deslocamento (GT) com o
    ângulo do marcador. A diferença é o valor de ground_truth.THETA_OFFSET.
    """
    inicio_pose = pose_gt()
    if inicio_pose is None:
        print("[CTRL] Robô não visto pela câmera.")
        return None

    robo.send_wheel_speed(v, v)
    time.sleep(duracao)
    robo.send_wheel_speed(0, 0)
    time.sleep(0.5)

    fim_pose = pose_gt()
    if fim_pose is None:
        print("[CTRL] Robô saiu da câmera.")
        return None

    dx = fim_pose[0] - inicio_pose[0]
    dy = fim_pose[1] - inicio_pose[1]
    if np.hypot(dx, dy) < 0.05:
        print("[CTRL] Robô quase não andou, calibração inválida.")
        return None

    # O marcador mede theta - THETA_OFFSET; a direção real é atan2(dy, dx)
    media = inicio_pose[2] + angle_wrap(fim_pose[2] - inicio_pose[2]) / 2
    marcador = angle_wrap(media - gt.THETA_OFFSET)
    offset = angle_wrap(np.arctan2(dy, dx) - marcador)
    print(f"[CTRL] THETA_OFFSET = {offset:.3f} rad ({np.degrees(offset):.1f}°). "
          f"Coloque esse valor em ground_truth.py.")
    return offset


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Controle do robô")
    sub = parser.add_subparsers(dest="modo", required=True)
    p_goto = sub.add_parser("goto", help="vai até um ponto")
    p_goto.add_argument("x", type=float)
    p_goto.add_argument("y", type=float)
    p_goto.add_argument("--odom", action="store_true", help="usa odometria em vez da GT")
    sub.add_parser("joystick", help="teleoperação com gamepad")
    sub.add_parser("calibrar", help="mede o THETA_OFFSET do marcador")
    parser.add_argument("--imagem", action="store_true", help="mostra a imagem da GT")
    parser.add_argument("--gravar", action="store_true", help="grava no recording.csv")
    args = parser.parse_args()

    robo.connect("192.168.0.125", 1883, client_id="controle")
    gt.start(show=args.imagem, on_pose=robo.log_ground_truth)
    if args.gravar:
        robo.record(True)

    try:
        if args.modo == "goto":
            go_to((args.x, args.y), get_pose=pose_odom if args.odom else pose_gt)
        elif args.modo == "joystick":
            joystick()
        elif args.modo == "calibrar":
            time.sleep(1)  # espera a primeira pose da câmera
            calibrar_theta()

    except KeyboardInterrupt: #CTRL + C
        pass

    finally:
        robo.stop()
        robo.record(False)
        gt.stop()
        robo.disconnect()
