import robo
import ground_truth as gt
import time

USAR_GT = True             # rastreamento pela câmera do teto
MOSTRAR_IMAGEM_GT = True   # janela com a imagem da câmera da ground truth

robo.connect("192.168.0.125", 1883)
if USAR_GT:
    # Cada pose da câmera também vai para o recording.csv
    gt.start(show=MOSTRAR_IMAGEM_GT, on_pose=robo.log_ground_truth)

try:
    # Espera a câmera ver o robô pela primeira vez
    start = time.time()
    while USAR_GT and gt.get_timestamp() is None and time.time() - start < 10:
        time.sleep(0.1)

    if gt.get_timestamp() is None:
        print("Ground truth indisponível: robô não visto pela câmera.")
    else:
        # Começa a odometria na mesma pose da ground truth
        gx, gy, gtheta = gt.get_pose()
        robo.set_odometry(gx, gy, gtheta)
        print(f"Pose inicial: x={gx:.3f}, y={gy:.3f}, θ={gtheta:.2f}")
        time.sleep(0.2)

    robo.record(True)
    for _ in range(10):
        x, y, theta = robo.get_odometry()
        l, d = robo.get_wheel_speeds()
        gx, gy, gtheta = gt.get_pose()
        gt_ts = gt.get_timestamp()
        gt_age = time.time() - gt_ts if gt_ts is not None else float("inf")

        print(f"Velocidades: esquerda={l:.2f}, direita={d:.2f}")
        print(f"Posição: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")
        print(f"Ground Truth: x={gx:.2f}, y={gy:.2f}, θ={gtheta:.2f}"
              + ("" if gt_age < 0.5 else f"  (desatualizada: {gt_age:.1f} s)"))
        robo.send_wheel_speed(0.08, 0.2)
        time.sleep(1)

    robo.stop()
    robo.record(False)

except KeyboardInterrupt: #CTRL + C
    robo.stop()
    robo.record(False)

finally:
    gt.stop()
    robo.disconnect()
