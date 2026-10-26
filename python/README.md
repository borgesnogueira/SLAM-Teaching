# SLAM: robô diferencial + ground truth por câmera

Bibliotecas e exemplos em Python para fazer experimentos de odometria e SLAM
com o robô `swarm/robot2`. Esta pasta (`python/`) fica dentro do repositório
SLAM-Teaching, cuja raiz é o firmware ESP-IDF do robô. Os comandos abaixo
assumem que você está **dentro de `python/`**:

```bash
cd SLAM-Teaching/python
```

- **`robo.py`** conversa com o robô pela rede: odometria, velocidade das rodas,
  comandos de movimento, observações e o vídeo da câmera embarcada.
- **`ground_truth.py`** mede a pose *real* do robô com a câmera do teto (marcador ArUco).
- **`controle.py`** leva o robô até pontos, faz teleoperação por joystick e calibra
  o ângulo do marcador.
- **`exemplos/`** tem cinco experimentos prontos com gráficos animados. Todos rodam
  também **sem hardware** com `--sim`.

---

## Sumário

1. [Visão geral do sistema](#1-visão-geral-do-sistema)
2. [Estrutura do diretório](#2-estrutura-do-diretório)
3. [Instalação](#3-instalação)
4. [Primeiros passos](#4-primeiros-passos)
5. [Exemplos animados](#5-exemplos-animados)
6. [Referência: `robo.py`](#6-referência-robopy)
7. [Referência: `ground_truth.py`](#7-referência-ground_truthpy)
8. [Referência: `controle.py`](#8-referência-controlepy)
9. [Gravação e formato do `recording.csv`](#9-gravação-e-formato-do-recordingcsv)
10. [Convenções: unidades, referenciais e tempo](#10-convenções-unidades-referenciais-e-tempo)
11. [Escrevendo seu próprio experimento](#11-escrevendo-seu-próprio-experimento)
12. [Solução de problemas](#12-solução-de-problemas)

---

## 1. Visão geral do sistema

```
                 MQTT (broker 192.168.0.125:1883)
   ┌──────────┐  odometria, rodas, status, observação  ┌──────────────┐
   │  Robô    │ ──────────────────────────────────────▶│              │
   │ ESP32-S3 │ ◀────────────────────────────────────── │   Este PC    │
   │          │  comandos de roda, odometria, câmera    │              │
   │ câmera   │                                         │  robo.py     │
   │ OV2640   │ ──── UDP 5001: vídeo JPEG ─────────────▶│  ground_     │
   └──────────┘                                         │  truth.py    │
                                                        │  controle.py │
   ┌──────────────┐  SSH (liga a câmera)                │              │
   │ Raspberry Pi │ ◀────────────────────────────────── │              │
   │ câmera teto  │ ──── TCP 8888: vídeo MJPEG ────────▶│              │
   └──────────────┘                                     └──────────────┘
```

| Fonte | O que entrega | Taxa aprox. | Módulo |
|---|---|---|---|
| Odometria do robô | pose estimada `(x, y, θ)` pelos encoders | 10 Hz | `robo.py` |
| Rodas | velocidade de cada roda (m/s) | 10 Hz | `robo.py` |
| Câmera do robô | imagem 640×480 do que está à frente | ~10 fps | `robo.py` |
| Câmera do teto | pose **real** `(x, y, θ)` pelo ArUco | ~20 Hz | `ground_truth.py` |

A odometria acumula erro com o tempo. A ground truth (GT) não acumula erro,
mas só existe enquanto o robô está dentro da imagem da câmera do teto.
Comparar as duas é o ponto central dos exemplos.

---

## 2. Estrutura do diretório

```
SLAM-Teaching/
├── main/main.c, CMakeLists.txt, ...   # firmware ESP-IDF do robô (idf.py build flash)
└── python/
    ├── main.py            # experimento simples: anda 10 s lendo odometria + GT e grava
    ├── robo.py            # robô: MQTT, câmera embarcada (UDP) e gravação
    ├── ground_truth.py    # pose real pela câmera do teto (ArUco), em segundo plano
    ├── controle.py        # controle de posição, joystick e calibração
    ├── config/
    │   ├── env.example    # modelo: copie para env.txt e coloque a senha
    │   ├── env.txt        # IP, usuário, porta e senha da Raspberry (fora do git)
    │   └── H.npy          # homografia: pixel da câmera do teto -> metros no chão
    ├── exemplos/
    │   ├── 01_odometria.py        # trajetória em malha aberta: mapa, rodas, θ
    │   ├── 02_ground_truth.py     # câmera do teto: imagem, mapa em metros, taxa de detecção
    │   ├── 03_odometria_vs_gt.py  # quadrado de waypoints: odometria × GT e erros
    │   ├── 04_replay_gravacao.py  # reproduz uma sessão gravada (pode salvar .gif)
    │   ├── 05_camera_robo.py      # vídeo do robô + mapa + fps/atraso
    │   ├── simulador.py           # robô, câmera do teto e câmera do robô simulados
    │   └── _comum.py              # estilo dos gráficos, cores, desenho do robô
    ├── dados/             # fora do git (só a pasta vai para o repositório)
    │   ├── recording.csv  # tudo que robo.record() grava
    │   └── camera_*/      # frames da câmera do robô (nome = instante da captura em µs)
    └── scripts/
        ├── gt.py              # rastreador antigo, standalone (teclas s/f/q)
        └── exemplo_leitura.py # plota a ground truth do recording.csv
```

Os módulos de `python/` encontram `config/` e `dados/` pelo próprio caminho. Por
isso você pode rodar os scripts de qualquer pasta.

---

## 3. Instalação

Python 3.10+ e as bibliotecas:

```bash
pip install paho-mqtt simplejson numpy opencv-contrib-python paramiko matplotlib pillow pygame
```

| Biblioteca | Usada para |
|---|---|
| `paho-mqtt`, `simplejson` | comunicação com o robô |
| `numpy` | contas |
| `opencv-contrib-python` | ArUco, decodificação do vídeo, janelas de imagem |
| `paramiko` | ligar a câmera do teto via SSH |
| `matplotlib`, `pillow` | gráficos animados e exportação de `.gif` |
| `pygame` | joystick (só para `controle.py joystick`) |

**Rede:** o PC precisa estar na mesma rede do broker (`192.168.0.x`). Para
receber o vídeo do robô, a porta **UDP 5001** precisa estar liberada no firewall:

```bash
sudo ufw allow 5001/udp
```

**Configuração da câmera do teto:** fica em `config/env.txt`. Esse arquivo
tem a senha e por isso **não vai para o git**. Depois de clonar, crie o seu a
partir do modelo:

```bash
cp config/env.example config/env.txt   # e edite GT_PASSWORD
```

```
GT_IP=192.168.0.125
GT_USER=gt
GT_PORT=8888
GT_PASSWORD=...
```

---

## 4. Primeiros passos

### 4.1 Sem hardware (simulação)

Comece por aqui para conhecer os gráficos:

```bash
python3 exemplos/03_odometria_vs_gt.py --sim
```

O simulador tem erros de calibração de propósito, então a odometria vai
se afastando da posição real como acontece no robô de verdade.

### 4.2 Com o robô

1. Ligue o robô e espere ele conectar no Wi-Fi/MQTT.
2. Coloque-o no chão, **dentro da área vista pela câmera do teto**.
3. Teste cada parte isoladamente:

```bash
python3 ground_truth.py            # abre a imagem do teto e imprime a pose do robô
python3 controle.py joystick       # dirige o robô com o gamepad (segure X)
python3 exemplos/05_camera_robo.py # vídeo da câmera do robô
```

4. **Calibre o ângulo do marcador** (uma vez, ou sempre que colar o marcador de novo):

```bash
python3 controle.py calibrar
# [CTRL] THETA_OFFSET = 1.571 rad (90.0°). Coloque esse valor em ground_truth.py.
```

Copie o valor para `THETA_OFFSET` no início do `ground_truth.py`. Sem isso o
`θ` da câmera pode estar girado em relação à frente do robô, e o controle por
GT anda para o lado errado.

5. Rode o experimento completo:

```bash
python3 exemplos/03_odometria_vs_gt.py --gravar
```

---

## 5. Exemplos animados

Todos os exemplos:

- aceitam `--sim` (sem hardware) e `--gravar` (grava em `dados/recording.csv`);
- abrem uma janela animada que se atualiza a 20 Hz;
- **param o robô quando a janela é fechada** (ou com Ctrl+C);
- salvam a figura final em `dados/<nome_do_exemplo>.png`.

Cores fixas em todos eles: **azul = odometria**, **laranja = ground truth**,
**verde-água = referência/waypoints**. O robô é desenhado em escala real.

### 5.1 `01_odometria.py`: só odometria

O robô executa uma trajetória **em malha aberta**: só manda velocidades para
as rodas, sem olhar onde está. A animação mostra o que a odometria acha que
aconteceu.

| Painel | Conteúdo |
|---|---|
| Esquerda | mapa com a trajetória estimada e o robô |
| Direita, em cima | velocidade medida das rodas esquerda e direita |
| Direita, embaixo | orientação θ ao longo do tempo |

```bash
python3 exemplos/01_odometria.py                        # oito (padrão)
python3 exemplos/01_odometria.py --trajetoria circulo
python3 exemplos/01_odometria.py --trajetoria quadrado  # 4 lados de 0,5 m
```

A odometria começa zerada em `(0, 0, 0)`. **Experimente:** faça o quadrado e
compare onde a odometria diz que o robô terminou com onde ele realmente parou.

### 5.2 `02_ground_truth.py`: obtenção da ground truth

Mostra como a pose real é medida.

| Painel | Conteúdo |
|---|---|
| Esquerda | imagem da câmera do teto com o caminho do marcador em pixels |
| Direita, em cima | o mesmo caminho em **metros**, dentro da área vista (cinza) |
| Direita, embaixo | poses por segundo: cai quando o marcador some |

```bash
python3 exemplos/02_ground_truth.py              # mova o robô com a mão ou com o joystick
python3 exemplos/02_ground_truth.py --circulo    # o robô anda em círculos sozinho
```

Este exemplo só conecta no robô se você usar `--circulo` ou `--gravar`. Para
dirigir com o joystick ao mesmo tempo, rode `python3 controle.py joystick` em
outro terminal.

### 5.3 `03_odometria_vs_gt.py`: tudo junto

O robô percorre um quadrado de waypoints centrado na área da câmera, usando o
controle de posição do `controle.py`. A odometria começa na pose da GT.

| Painel | Conteúdo |
|---|---|
| Esquerda | referência, trajetória real (GT) e estimada (odometria), waypoint atual destacado |
| Direita, em cima | erro de posição da odometria (cm) |
| Direita, embaixo | erro de orientação da odometria (graus) |

```bash
python3 exemplos/03_odometria_vs_gt.py                      # controle usa a odometria
python3 exemplos/03_odometria_vs_gt.py --malha gt           # controle usa a câmera
python3 exemplos/03_odometria_vs_gt.py --lado 0.5 --voltas 3
```

As duas malhas mostram coisas diferentes:

- **`--malha odom`** (padrão): o robô *acha* que fez o quadrado perfeito, e
  a linha laranja mostra onde ele realmente foi. O erro cresce a cada volta.
- **`--malha gt`**: o robô faz o quadrado de verdade (laranja em cima da
  referência) e quem deriva é só a odometria (azul).

Se a GT sumir (robô fora da imagem), o controle por GT para o robô e espera.

### 5.4 `04_replay_gravacao.py`: replay de uma gravação

Reproduz uma sessão gravada como animação, **sem robô e sem câmera**.

```bash
python3 exemplos/04_replay_gravacao.py --listar            # lista as sessões com dados
python3 exemplos/04_replay_gravacao.py                     # última sessão
python3 exemplos/04_replay_gravacao.py --sessao 12 --velocidade 4
python3 exemplos/04_replay_gravacao.py --alinhar           # leva a odometria ao referencial da GT
python3 exemplos/04_replay_gravacao.py --salvar replay.gif # exporta em vez de mostrar
python3 exemplos/04_replay_gravacao.py --arquivo dados/simulacao.csv
```

- Se a sessão tem odometria **e** GT, os painéis da direita mostram os erros.
  Se tem só uma delas, mostram as rodas e θ.
- A trajetória completa aparece clarinha ao fundo, e a parte já percorrida em cor forte.
- `--alinhar` serve para gravações em que a odometria não foi iniciada na pose
  da GT: gira e translada a odometria para começar no mesmo ponto.
- Gravações antigas (sem a coluna de tempo) têm o tempo estimado, supondo 20 Hz
  de odometria. Isso pode gerar um pico falso de erro no início.

### 5.5 `05_camera_robo.py`: câmera embarcada

| Painel | Conteúdo |
|---|---|
| Esquerda | o que o robô está vendo |
| Direita, em cima | onde ele está (GT e odometria) |
| Direita, embaixo | frames por segundo recebidos |

O subtítulo mostra o fps, o atraso entre a captura no robô e a chegada no PC,
e quantos frames foram perdidos.

```bash
python3 exemplos/05_camera_robo.py
python3 exemplos/05_camera_robo.py --circulo --gravar  # grava CSV + frames JPEG
python3 exemplos/05_camera_robo.py --sem-gt            # sem a câmera do teto
```

Com `--gravar`, os frames vão para `dados/camera_AAAAMMDD_HHMMSS/`. Na
simulação, a "câmera" mostra o piso logo à frente do robô.

### 5.6 `main.py` e scripts avulsos

- **`main.py`**: experimento mínimo, sem gráficos. Espera a GT, inicia a
  odometria nela, anda 10 s fazendo uma curva e imprime odometria × GT a cada
  segundo. Grava tudo. Bom ponto de partida para copiar.
  - `USAR_GT` liga ou desliga a câmera do teto.
  - `MOSTRAR_IMAGEM_GT` abre a janela da câmera do teto.
- **`scripts/exemplo_leitura.py`**: plota (estático) os pontos de GT do `recording.csv`.
- **`scripts/gt.py`**: o rastreador original, standalone, com teclas `s`/`f`/`q`.
  Hoje ele aponta para um `H.npy` em caminho do Windows e usa `python-dotenv`.
  Prefira o `ground_truth.py`.

---

## 6. Referência: `robo.py`

```python
import robo
robo.connect()                       # broker 192.168.0.125:1883
```

### Conexão

| Função | Descrição |
|---|---|
| `connect(broker="192.168.0.125", port=1883, client_id="aluno", timeout=5)` | Conecta ao broker e espera até 5 s. |
| `disconnect()` | Desliga a câmera (se ligada) e desconecta. |
| `get_status()` | Último status do robô (`"Online"`, mensagem de queda, ou `None`). |

> **`client_id` precisa ser único.** Se dois programas conectarem com o mesmo
> id, o broker derruba o primeiro. Os exemplos usam `exemplo-<pid>`,
> o `controle.py` usa `controle`.

### Movimento

| Função | Descrição |
|---|---|
| `send_wheel_speed(v_left, v_right)` | Velocidade de cada roda em **m/s** (o firmware tem PID por roda). |
| `stop()` | Zera as rodas e reseta os PIDs no firmware. |

O robô mantém a última velocidade recebida. **Sempre** termine com `stop()`,
de preferência num `finally`.

### Sensores

| Função | Retorna |
|---|---|
| `get_odometry()` | `(x, y, theta)`: m, m, rad |
| `set_odometry(x=0, y=0, theta=0)` | redefine a odometria no robô |
| `get_odometry_timestamp_us()` | instante da última odometria (µs, relógio do robô) |
| `get_wheel_speeds()` | `(v_left, v_right)` em m/s |
| `get_observation()` | `(r, theta, label)` da última observação publicada pelo robô |

Os valores são atualizados em segundo plano. As funções `get_*` só devolvem
o último valor recebido, sem esperar por um novo.

### Câmera do robô

| Função | Descrição |
|---|---|
| `start_camera(show=False, port=5001)` | Abre a porta UDP, pede ao robô para enviar para este PC (IP detectado sozinho) e recebe em segundo plano. |
| `stop_camera()` | Pede ao robô para parar e encerra. |
| `show_camera(show=True)` | Abre ou fecha uma janela com o vídeo. |
| `get_camera_frame()` | Último frame completo: `numpy` RGB 480×640×3, ou `None`. |
| `get_camera_jpeg()` | O mesmo frame em bytes JPEG, como veio do robô. |
| `get_camera_timestamp()` | Instante da **captura** (s, tempo Unix, comparável com `time.time()`). |
| `get_camera_frame_id()` | Número sequencial do frame. |
| `get_camera_stats()` | `{"fps", "frames", "perdidos", "atraso"}`. O atraso é em segundos, da captura até a chegada. |
| `set_camera_udp(ip="auto", port=5001)` | Baixo nível: só manda o comando ao robô (`ip=None` desliga). |

### Gravação

| Função | Descrição |
|---|---|
| `record(True)` / `record(False)` | Começa/termina uma sessão em `dados/recording.csv` (e frames, se a câmera estiver ligada). |
| `log_ground_truth(x, y, theta)` | Escreve uma linha de GT na gravação. Use como callback: `gt.start(on_pose=robo.log_ground_truth)`. |

---

## 7. Referência: `ground_truth.py`

```python
import ground_truth as gt
gt.start(show=True)        # SSH na Raspberry, liga o stream, começa a rastrear
x, y, theta = gt.get_pose()
gt.stop()
```

| Função | Descrição |
|---|---|
| `start(show=False, on_pose=None, timeout=15)` | Liga a câmera e rastreia em segundo plano. Retorna `True` se deu certo. `on_pose(x, y, theta)` é chamada a cada pose nova. |
| `stop()` | Encerra o rastreamento, o stream e o SSH. |
| `get_pose()` | Última pose `(x, y, theta)` em m, m, rad. |
| `get_timestamp()` | `time.time()` da última pose, ou `None` se o robô nunca foi visto. |
| `show_image(show=True)` | Abre ou fecha a janela com a imagem do teto. |
| `get_frame()` | Último frame do teto (RGB 720×720), ou `None`. |
| `get_pixel()` | `(u, v)` do marcador no último frame em que foi visto. |
| `area_visivel()` | 4 cantos `(x, y)` da área do chão vista pela câmera. |
| `python3 ground_truth.py` | Teste rápido: mostra a imagem e imprime a pose. |

> **A pose não "some" quando o robô sai da imagem.** `get_pose()` continua
> devolvendo a última pose vista. Para saber se ela é atual, confira a idade:
> `time.time() - gt.get_timestamp()`. O `controle.py` considera perdida acima de 0,5 s.

**Constantes importantes** (início do arquivo):

| Constante | Valor | Significado |
|---|---|---|
| `ROBOT_ID` | `2` | ID do ArUco 4×4 colado no robô |
| `THETA_OFFSET` | `0.0` | correção do ângulo do marcador (medir com `controle.py calibrar`) |
| `CAM_X`, `CAM_Y`, `CAM_Z` | 0,8 / 0,8 / 2,79 m | posição da câmera do teto |
| `ROBO_H` | 0,08 m | altura do marcador (corrige a paralaxe) |
| `WIDTH`, `HEIGHT` | 720 | resolução em que `H.npy` foi calibrada |

---

## 8. Referência: `controle.py`

### Linha de comando

```bash
python3 controle.py goto 1.0 0.8           # vai até (1,0; 0,8) usando a GT
python3 controle.py goto 1.0 0.8 --odom    # idem, usando a odometria
python3 controle.py joystick               # teleoperação
python3 controle.py calibrar               # mede THETA_OFFSET (anda ~15 cm reto)
python3 controle.py --imagem --gravar goto 1.0 0.8   # opções globais vêm ANTES do modo
```

### Joystick

- **Segure X** e use o analógico esquerdo (para frente/trás = `v`, lados = `w`).
- Soltou o X, o robô para. **Quadrado** encerra.
- O mapeamento foi pensado para controles PS4/PS5. Para outro gamepad, ajuste
  `BTN_HABILITA`, `BTN_FIM`, `EIXO_V` e `EIXO_W`.

### Funções para usar no seu código

| Função | Descrição |
|---|---|
| `positioning_control(q, ref)` | Lei de controle: `q=(x,y,θ)`, `ref=(x,y)` → `(v, w)`, com `v = K1·d·cos(e_θ)` e `w = K2·e_θ`. |
| `unicycle_to_wheels(v, w)` | `(v, w)` → `(v_left, v_right)`. Satura mantendo a curvatura. |
| `go_to(ref, get_pose=pose_gt, tol=0.05, timeout=60)` | Laço completo até o ponto. Retorna `True` se chegou. |
| `pose_gt()` / `pose_odom()` | Fontes de pose para o `go_to`. `pose_gt()` devolve `None` se a GT estiver velha. |
| `angle_wrap(a)` | Leva o ângulo para [-π, π]. |
| `calibrar_theta()` | Mede e imprime o `THETA_OFFSET`. |

**Constantes:** `V_MAX = 0.2` m/s, `W_MAX = 2.0` rad/s, `V_RODA_MAX = 0.25` m/s,
`K1 = 0.8`, `K2 = 1.6`, `WHEEL_BASE = 0.143` m (igual ao firmware), laço a 20 Hz.

---

## 9. Gravação e formato do `recording.csv`

Cada `robo.record(True)` … `robo.record(False)` vira uma **sessão** entre
`Inicio` e `Fim`, sempre acrescentada no fim do arquivo (nada é apagado).
Cada linha é um evento. A **última coluna é o `time.time()` do PC** no
momento em que o dado chegou:

```
Inicio
odometria,0.5129,0.6701,3.1294,1793041982.440
velocidades,0.0967,0.1935,1793041982.441
ground_truth,0.5573,0.6941,2.9222,1793041982.452
observacao,0.84,0.12,3,1793041982.460
camera,1532,1793041982.398211,1793041982.470
Fim
```

| Tipo | Colunas (antes do tempo) | Unidades |
|---|---|---|
| `odometria` | `x, y, theta` | m, m, rad |
| `velocidades` | `v_left, v_right` | m/s |
| `ground_truth` | `x, y, theta` | m, m, rad |
| `observacao` | `r, theta, label` | como publicado pelo robô |
| `camera` | `frame_id, instante_da_captura` | –, s (Unix) |

Frames da câmera do robô ficam em `dados/camera_AAAAMMDD_HHMMSS/<instante_us>.jpg`.
O nome é o instante da captura em microssegundos. Para achar a pose no momento
de uma foto, procure no CSV as linhas com o tempo mais próximo.

Para ler em Python:

```python
import numpy as np

odom, gt = [], []
with open("dados/recording.csv") as f:
    for linha in f:
        p = linha.strip().split(",")
        if p[0] == "odometria":
            odom.append([float(v) for v in p[1:]])     # x, y, theta, t
        elif p[0] == "ground_truth":
            gt.append([float(v) for v in p[1:]])
odom, gt = np.array(odom), np.array(gt)
```

O `04_replay_gravacao.py` já faz isso separando as sessões (`ler_sessoes()`).
Gravações antigas não têm a coluna de tempo.

---

## 10. Convenções: unidades, referenciais e tempo

- **Unidades:** metros, radianos, segundos. Só os gráficos mostram ângulos em graus.
- **θ** fica sempre em [-π, π], medido a partir do eixo x, positivo no sentido anti-horário.
- **Referencial da GT:** o chão, definido pela homografia `H.npy`. A área vista
  vai de aproximadamente x ∈ [-0,2; 2,4] m e y ∈ [-0,5; 2,2] m (veja `gt.area_visivel()`).
- **Referencial da odometria:** começa onde o robô estava no último
  `set_odometry()`. Para comparar com a GT, faça
  `robo.set_odometry(*gt.get_pose())` no início, como os exemplos 3 e 5 fazem.
- **Tempo:** o robô sincroniza o relógio por SNTP. Os instantes do robô
  (`timestamp_us` da odometria e da câmera) e o `time.time()` do PC estão na
  mesma escala, se o PC também estiver com o relógio sincronizado.
- **Robô:** raio da roda 0,0823 m e distância entre rodas 0,143 m.

---

## 11. Escrevendo seu próprio experimento

Modelo mínimo, seguro (sempre para o robô) e já gravando:

```python
import time
import robo
import ground_truth as gt
import controle

robo.connect(client_id="meu_experimento")
gt.start(on_pose=robo.log_ground_truth)       # GT também vai para o CSV

try:
    # 1. espera a câmera do teto ver o robô e alinha a odometria
    while gt.get_timestamp() is None:
        time.sleep(0.1)
    robo.set_odometry(*gt.get_pose())
    time.sleep(0.2)

    # 2. grava e se move
    robo.record(True)
    controle.go_to((1.2, 0.6))                    # malha fechada na GT
    controle.go_to((0.8, 1.0), get_pose=controle.pose_odom)  # ou na odometria

    # 3. lê o que quiser
    print("odometria:", robo.get_odometry())
    print("real:     ", gt.get_pose())

finally:
    robo.stop()
    robo.record(False)
    gt.stop()
    robo.disconnect()
```

Para testar sem hardware, use o simulador (mesma interface):

```python
import sys; sys.path.insert(0, "exemplos")
import simulador
import controle

robo, gt = simulador.criar()
controle.robo, controle.gt = robo, gt     # o go_to também passa a usar o simulador
```

Para gráficos animados no mesmo estilo dos exemplos, copie um exemplo e use
`exemplos/_comum.py`. Ele traz o estilo (`estilo()`), as cores (`COR_1/2/3`),
o desenho do robô (`Robo`), a área da câmera (`desenhar_area`) e o movimento
em segundo plano (`Movimento`).

---

## 12. Solução de problemas

| Sintoma | Causa provável / solução |
|---|---|
| `Ground truth indisponível: robô não visto` | O robô está fora da imagem, ou o marcador não é o `ROBOT_ID`. O `ground_truth.py` avisa quando vê outro ID (`[GT] Aviso: vendo ArUco [..]`). |
| Controle por GT anda para o lado errado | Falta calibrar: `python3 controle.py calibrar` e ajuste `THETA_OFFSET`. |
| Odometria/rodas sempre zeradas | O robô está desligado ou fora do Wi-Fi. Confira `robo.get_status()` e se o PC alcança o broker. |
| Um programa "desconecta" quando outro abre | Mesmo `client_id` nos dois. Passe ids diferentes em `robo.connect(client_id=...)`. |
| Vídeo do robô não chega (`sem vídeo ainda`) | Firewall: `sudo ufw allow 5001/udp`. Confira também se o IP impresso em `[ROBO] Câmera do robô enviando para ...` é o da rede `192.168.0.x`. |
| Muitos frames perdidos | Wi-Fi fraco. O UDP não reenvia: um pedaço perdido descarta o frame inteiro. |
| `Não foi possível abrir a porta UDP 5001` | Outro programa já está recebendo o vídeo. Feche-o, ou use `start_camera(port=...)`. |
| `Erro ao iniciar: ... Authentication` (GT) | Credenciais em `config/env.txt`. |
| Avisos `QObject::killTimer` ao sair | Inofensivos: vêm das janelas do OpenCV abertas em outra thread. |
| `Erro ao iniciar: [Errno 2] ... env.txt` (GT) | Falta criar `config/env.txt`: `cp config/env.example config/env.txt`. |
| Compilar o firmware após mover a pasta | Rode `idf.py fullclean` uma vez: a pasta `build/` guarda o caminho antigo. |
