# SLAM-Teaching

Plataforma de ensino de odometria e SLAM com um robô diferencial (ESP32-S3
com câmera) e uma câmera no teto que mede a pose real do robô (ground truth).

O repositório tem **duas pastas**:

| Pasta | O que é | Linguagem | Documentação |
|---|---|---|---|
| [`firmware/`](firmware) | programa que roda no robô: motores, encoders, odometria, Wi-Fi, MQTT e câmera | C (ESP-IDF) | [seção 3](#3-firmware) deste arquivo |
| [`utilitarios/`](utilitarios) | bibliotecas e exemplos que rodam no PC: falar com o robô, medir a ground truth, controlar, gravar e visualizar | Python | [seção 4](#4-utilitários) e o [README completo](utilitarios/README.md) |

---

## Sumário

1. [Como as peças se conectam](#1-como-as-peças-se-conectam)
2. [Início rápido](#2-início-rápido)
3. [Firmware](#3-firmware)
4. [Utilitários](#4-utilitários)
5. [Protocolo de comunicação](#5-protocolo-de-comunicação)
6. [Parâmetros que precisam bater entre firmware e utilitários](#6-parâmetros-que-precisam-bater-entre-firmware-e-utilitários)
7. [Estrutura completa do repositório](#7-estrutura-completa-do-repositório)
8. [Solução de problemas](#8-solução-de-problemas)

---

## 1. Como as peças se conectam

```
                       broker MQTT 192.168.0.125:1883
   ┌───────────────┐   telemetria (odometria, rodas, status)   ┌──────────────────┐
   │ Robô          │ ────────────────────────────────────────▶ │ PC               │
   │ firmware/     │ ◀──────────────────────────────────────── │ utilitarios/     │
   │ ESP32-S3      │   comandos (rodas, odometria, parada,     │                  │
   │ OV2640        │             destino do vídeo)             │  robo.py         │
   │               │ ── UDP 5001: vídeo JPEG da câmera ──────▶ │  controle.py     │
   └───────────────┘                                           │  ground_truth.py │
                                                               │  exemplos/       │
   ┌───────────────┐   SSH: liga a câmera                      │                  │
   │ Raspberry Pi  │ ◀──────────────────────────────────────── │                  │
   │ câmera do teto│ ── TCP 8888: vídeo MJPEG ───────────────▶ │                  │
   └───────────────┘   (marcador ArUco ID 2 em cima do robô)   └──────────────────┘
```

- **O robô** controla as rodas por PID, calcula a própria odometria e publica
  tudo por MQTT. Também transmite o vídeo da câmera dele por UDP.
- **O PC** manda comandos de velocidade, lê a odometria, recebe o vídeo, e
  mede onde o robô *realmente* está pela câmera do teto.
- A odometria acumula erro, e a ground truth não. Comparar as duas é o ponto
  central dos exemplos.

---

## 2. Início rápido

### 2.1 Sem nada de hardware

```bash
cd utilitarios
pip install paho-mqtt simplejson numpy opencv-contrib-python paramiko matplotlib pillow pygame
python3 exemplos/03_odometria_vs_gt.py --sim
```

Abre uma animação com um robô simulado percorrendo um quadrado. A odometria
(azul) vai se afastando da posição real (laranja).

### 2.2 Com o robô

1. **Grave o firmware** no robô ([seção 3.3](#33-compilar-e-gravar)).
2. **Conecte o robô no Wi-Fi** do laboratório ([seção 3.4](#34-primeira-conexão-wi-fi)).
   O LED fica **verde** quando ele está pronto.
3. **Configure a câmera do teto** no PC:
   ```bash
   cd utilitarios
   cp config/env.example config/env.txt   # e coloque a senha da Raspberry
   ```
4. **Teste:**
   ```bash
   python3 ground_truth.py                    # imagem do teto + pose do robô
   python3 controle.py joystick               # dirigir com gamepad (segure X)
   python3 controle.py calibrar               # mede o ângulo do marcador (uma vez)
   python3 exemplos/03_odometria_vs_gt.py --gravar
   ```

---

## 3. Firmware

Pasta [`firmware/`](firmware). É um projeto ESP-IDF com um único arquivo-fonte,
[`firmware/main/main.c`](firmware/main/main.c).

### 3.1 Hardware

| Item | Detalhe |
|---|---|
| Placa | ESP32-S3 com PSRAM octal (formato ESP32-S3-CAM) |
| Câmera | OV2640, JPEG VGA 640×480, 10 fps, qualidade 18 |
| Motores | 2 motores DC com ponte H, PWM 20 kHz, 8 bits |
| Encoders | quadratura, 1496 bordas por volta da roda |
| LED | 1 WS2812 (indica o estado) |
| Geometria | raio da roda **0,0823 m**, distância entre rodas **0,143 m** |

**Pinos:**

| Função | GPIO |
|---|---|
| Motor esquerdo A / B | 39 / 40 |
| Motor direito A / B | 42 / 41 |
| Encoder esquerdo A / B | 1 / 2 |
| Encoder direito A / B | 47 / 38 |
| LED WS2812 | 48 |
| Câmera XCLK / SIOD / SIOC | 15 / 4 / 5 |
| Câmera D0–D7 (Y2–Y9) | 11, 9, 8, 10, 12, 18, 17, 16 |
| Câmera VSYNC / HREF / PCLK | 6 / 7 / 13 |

### 3.2 Como o firmware funciona

**Máquina de estados** (a cor do LED indica o estado):

| Estado | LED | O que acontece |
|---|---|---|
| **INIT** | azul fixo | Inicializa encoders, PWM e câmera. Conecta no Wi-Fi salvo, sincroniza o relógio por NTP (`pool.ntp.org`, **precisa de internet**) e conecta no MQTT. |
| INIT (configuração) | azul piscando | Não há Wi-Fi salvo, ou a conexão falhou. O robô vira um ponto de acesso para você configurar ([3.4](#34-primeira-conexão-wi-fi)). |
| **STANDBY** | verde | Conectado e esperando. Sai daqui ao receber **qualquer** mensagem MQTT. |
| **RUNNING** | magenta | **Zera a odometria**, liga o controle das rodas (a cada 25 ms) e começa a publicar a telemetria (a cada 100 ms). |

> A primeira mensagem MQTT (qualquer comando) faz o robô passar de STANDBY
> para RUNNING e zera a odometria. Se a primeira coisa que você mandar for
> `set_odometry`, ela pode ser sobrescrita pelo zero. Mande um comando
> qualquer antes, por exemplo `robo.stop()`.

**Controle das rodas (a cada 25 ms):**

1. Lê os encoders e calcula a velocidade de cada roda, `ω` em rad/s.
2. PID por roda (`Kp = 37`, `Ki = 370`, `Kd = 0,01`, saída ±255 de PWM),
   seguindo a referência recebida por MQTT, em rad/s.
3. Atualiza a odometria por cinemática diferencial:
   - `v = R·(ωr + ωl)/2`
   - `w = R·(ωr − ωl)/L`
   - `θ += w·dt` (mantido em [-π, π])
   - `x += v·cos θ·dt`
   - `y += v·sin θ·dt`

**Vídeo:**

- Uma tarefa captura a 10 fps. Cada frame carrega o instante da captura
  (relógio sincronizado por NTP).
- Outra tarefa envia o frame por UDP, em pedaços, para o IP/porta definidos
  pelo tópico `cmd/camera_udp`.
- Sem destino definido, os frames são descartados.
- A janela do sensor é recortada em 64 px na horizontal
  (`camera_apply_hwindow_correction`) para corrigir o enquadramento.

**Segurança:**

- O robô mantém a última velocidade recebida. Não há parada automática se o
  PC sumir, então os utilitários sempre mandam `stop()` ao terminar.
- Se o robô cair, o broker publica `Offline` em `telemetry/status` (mensagem
  de "last will").

### 3.3 Compilar e gravar

Requer **ESP-IDF v5.x** (testado com **v5.4.2**), alvo **esp32s3**.

```bash
cd firmware
idf.py set-target esp32s3      # só na primeira vez
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor   # Ctrl+] sai do monitor
```

- As dependências (`espressif/esp32-camera` e `espressif/led_strip`) são
  baixadas sozinhas pelo gerenciador de componentes, para `managed_components/`.
- O `sdkconfig.defaults` liga a PSRAM octal (onde ficam os frames da câmera) e
  a tabela de partição padrão "single app large" do ESP-IDF (app de ~1,5 MB).
  O `partitions.csv` da pasta **não é usado** com essa configuração. Ele só vale
  se você escolher "Custom partition table" no `idf.py menuconfig`, e nesse caso
  precisa de flash de 16 MB.
- `build/`, `sdkconfig` e `managed_components/` são gerados na compilação e
  ficam fora do git.
- Também há um **devcontainer** em `firmware/.devcontainer/`, com a imagem
  oficial do ESP-IDF, para compilar sem instalar nada (VS Code → *Reopen in Container*).

### 3.4 Primeira conexão Wi-Fi

As credenciais ficam gravadas na memória do robô (NVS), não no código.

1. Ligue o robô sem Wi-Fi salvo (ou longe da rede salva): o LED **pisca azul**.
2. Conecte o celular ou o PC na rede **`Robot_Config`**, senha **`12345678`**.
3. Abra **http://192.168.4.1**, digite o SSID e a senha da rede do laboratório
   e salve.
4. O robô salva, reinicia sozinho e conecta na rede. LED **verde** = pronto.

A rede precisa ter **internet** (para o NTP) e alcançar o broker `192.168.0.125`.

### 3.5 O que mudar para outro robô ou laboratório

Tudo isso está em `firmware/main/main.c`:

| O quê | Onde | Também mudar em |
|---|---|---|
| Nome do robô nos tópicos (`swarm/robot2/...`) | strings dos tópicos em `mqtt_event_handler`, `mqtt_start` e `vTaskPublishOdometry` | `ROBOT_PREFIX` em `utilitarios/robo.py` |
| Endereço do broker | `.broker.address.uri` em `mqtt_start` | `BROKER` e `connect()` nos utilitários |
| Raio da roda / distância entre rodas | `WHEEL_RADIUS`, `WHEEL_BASE` | `robo.py` e `controle.py` ([seção 6](#6-parâmetros-que-precisam-bater-entre-firmware-e-utilitários)) |
| Ganhos do PID | `PID_MOTOR_KP/KI/KD` | – |
| Taxa de publicação | `PUBLISH_PERIOD_MS` (100 ms) | – |
| Fps / qualidade / porta do vídeo | `CAMERA_STREAM_FPS`, `CAMERA_JPEG_QUALITY`, `CAMERA_UDP_DEFAULT_PORT` | `CAMERA_PORT` em `robo.py` |
| Marcador ArUco do robô | (adesivo físico) | `ROBOT_ID` em `utilitarios/ground_truth.py` |

---

## 4. Utilitários

Pasta [`utilitarios/`](utilitarios). Referência completa, com todas as funções,
opções e formatos: **[utilitarios/README.md](utilitarios/README.md)**.

### 4.1 Módulos

| Arquivo | Para que serve |
|---|---|
| `robo.py` | Conecta no robô: `send_wheel_speed()` (m/s), `get_odometry()`, `get_wheel_speeds()`, `set_odometry()`, `stop()`, câmera do robô (`start_camera()`, `get_camera_frame()`) e gravação (`record()`). |
| `ground_truth.py` | Pose real pela câmera do teto: `start()`, `get_pose()`, `get_timestamp()`, `area_visivel()`. |
| `controle.py` | `go_to()` (controle de posição), teleoperação com joystick, calibração do ângulo do marcador. Também roda pela linha de comando. |
| `main.py` | Experimento mínimo sem gráficos: bom para copiar e adaptar. |

### 4.2 Exemplos animados

Todos aceitam `--sim` (sem hardware) e `--gravar`, param o robô ao fechar a
janela e salvam a figura final em `dados/`.

| Exemplo | Mostra |
|---|---|
| `exemplos/01_odometria.py` | trajetória em malha aberta (oito, círculo, quadrado): mapa, rodas, θ |
| `exemplos/02_ground_truth.py` | câmera do teto: imagem, trajetória em metros, taxa de detecção |
| `exemplos/03_odometria_vs_gt.py` | quadrado de waypoints com controle: odometria × GT e os erros |
| `exemplos/04_replay_gravacao.py` | reproduz uma sessão gravada, sem hardware (pode salvar `.gif`) |
| `exemplos/05_camera_robo.py` | vídeo da câmera do robô + mapa + fps e atraso |

### 4.3 Dados

- `robo.record(True)` grava em `utilitarios/dados/recording.csv`. Cada linha é
  `tipo,valores...,tempo_do_PC`.
- Com a câmera do robô ligada, os frames vão para `dados/camera_<data>/`.
- `config/env.txt` (senha da Raspberry) e `dados/` **não vão para o git**.
  Quem clona cria o `env.txt` a partir do `config/env.example`.

---

## 5. Protocolo de comunicação

### 5.1 MQTT (broker `192.168.0.125:1883`)

Robô → PC:

| Tópico | Conteúdo | Taxa |
|---|---|---|
| `swarm/robot2/telemetry/odometry` | `{"odometria": "x, y, theta", "timestamp_us": N}` em m, m, rad. `timestamp_us` = instante da amostra (tempo Unix, µs) | 10 Hz |
| `swarm/robot2/telemetry/wheels` | `{"wheels": "wl, wr"}` em **rad/s** | 10 Hz |
| `swarm/robot2/telemetry/status` | `"Online"` ao conectar, `"Offline"` (last will) ao cair | eventos |

PC → robô:

| Tópico | Conteúdo | Efeito |
|---|---|---|
| `swarm/robot2/cmd/wheels` | `{"l": wl, "r": wr}` em **rad/s** | referência dos PIDs (o `robo.py` converte de m/s: `ω = v/R`) |
| `swarm/robot2/cmd/odometry` | `{"x": .., "y": .., "theta": ..}` | redefine a odometria |
| `swarm/robot2/cmd/mode` | qualquer JSON | parada: zera referências e reseta os PIDs |
| `swarm/robot2/cmd/camera_udp` | `{"ip": "a.b.c.d", "port": 5001}` (ou `"ip:porta"`) | destino do vídeo. Algo inválido (ex.: `"off"`) desliga. |

O `robo.py` também escuta `swarm/robot2/observation` (`{"r", "theta", "label"}`)
e `ground_truth/data`. **O firmware atual não publica nenhum dos dois.** Eles
existem para observações e ground truth vindas de outras fontes.

### 5.2 Vídeo UDP (robô → PC, porta 5001)

Cada frame JPEG é dividido em pedaços de até 1200 bytes. Cada pacote UDP tem
um cabeçalho de **18 bytes** (little-endian, sem preenchimento) seguido do pedaço:

| Campo | Tipo | Significado |
|---|---|---|
| `frame_id` | uint32 | número do frame (cresce de 1 em 1) |
| `timestamp_us` | uint64 | instante da captura, tempo Unix em µs |
| `total_frags` | uint16 | quantos pedaços tem o frame |
| `frag_idx` | uint16 | índice deste pedaço (0 … total−1) |
| `payload_len` | uint16 | bytes de JPEG neste pacote |

Em Python: `struct.Struct("<IQHHH")`. O receptor junta os pedaços por
`frame_id`. Se faltar algum pedaço, o frame inteiro é perdido (o UDP não
reenvia). O `robo.py` já faz tudo isso em `start_camera()`.

### 5.3 Câmera do teto (Raspberry Pi → PC)

O `ground_truth.py` entra por SSH na Raspberry (credenciais em
`utilitarios/config/env.txt`) e inicia:

```
rpicam-vid --mode 3280:2464 --width 720 --height 720 --framerate 20 --codec mjpeg --listen -o tcp://0.0.0.0:8888
```

Depois lê o stream por TCP e detecta o ArUco 4×4 **ID 2**. A posição em pixels
vira metros pela homografia `config/H.npy`, com correção de paralaxe pela altura
do marcador (0,08 m) e da câmera (2,79 m).

---

## 6. Parâmetros que precisam bater entre firmware e utilitários

Se mudar um lado, mude o outro:

| Parâmetro | Firmware (`main.c`) | Utilitários |
|---|---|---|
| Raio da roda (0,0823 m) | `WHEEL_RADIUS` | `WHEEL_RADIUS` em `robo.py` |
| Distância entre rodas (0,143 m) | `WHEEL_BASE` | `WHEEL_BASE` em `controle.py` e `exemplos/_comum.py` |
| Prefixo dos tópicos (`swarm/robot2`) | strings dos tópicos | `ROBOT_PREFIX` em `robo.py` |
| Broker (`192.168.0.125:1883`) | `mqtt_start()` | `connect()` em `robo.py`, `BROKER` em `exemplos/_comum.py` |
| Porta do vídeo (5001) | `CAMERA_UDP_DEFAULT_PORT` | `CAMERA_PORT` em `robo.py` |
| Cabeçalho UDP | `CameraUdpFragHeader_t` | `CAMERA_HEADER` em `robo.py` |
| Unidade das rodas no MQTT (rad/s) | `wl_ref`, `wl_meas` | conversão em `send_wheel_speed()` e no callback de `telemetry/wheels` |

---

## 7. Estrutura completa do repositório

```
SLAM-Teaching/
├── README.md                  # este arquivo
├── LICENSE
├── .gitignore
├── firmware/                  # ── programa do robô (ESP-IDF) ──
│   ├── CMakeLists.txt         # projeto ESP-IDF
│   ├── sdkconfig.defaults     # PSRAM octal, partição "single app large"
│   ├── partitions.csv         # tabela alternativa (não usada por padrão)
│   ├── .devcontainer/         # ambiente Docker com ESP-IDF
│   └── main/
│       ├── main.c             # todo o firmware
│       ├── CMakeLists.txt     # componentes usados
│       └── idf_component.yml  # dependências (esp32-camera, led_strip)
└── utilitarios/               # ── ferramentas do PC (Python) ──
    ├── README.md              # referência completa dos utilitários
    ├── robo.py                # MQTT + câmera do robô + gravação
    ├── ground_truth.py        # pose real pela câmera do teto
    ├── controle.py            # go_to, joystick, calibração
    ├── main.py                # experimento mínimo
    ├── config/
    │   ├── env.example        # modelo das credenciais da Raspberry
    │   ├── env.txt            # (fora do git) credenciais reais
    │   └── H.npy              # homografia da câmera do teto
    ├── exemplos/              # 5 exemplos animados + simulador
    ├── scripts/               # gt.py (rastreador antigo), exemplo_leitura.py
    └── dados/                 # (fora do git) gravações e frames
```

---

## 8. Solução de problemas

| Sintoma | O que verificar |
|---|---|
| LED fica **azul fixo** e não muda | Sem internet para o NTP (o robô tenta de novo a cada 5 s) ou falha na câmera. Veja `idf.py monitor`. |
| LED **pisca azul** | Sem Wi-Fi configurado ou a rede salva não conectou. Veja a [seção 3.4](#34-primeira-conexão-wi-fi). |
| LED verde, mas a odometria não chega | O robô só publica em RUNNING: mande qualquer comando (ex.: `robo.stop()`). |
| Odometria volta a zero logo no começo | Efeito da entrada em RUNNING ([seção 3.2](#32-como-o-firmware-funciona)). Chame `set_odometry()` depois do primeiro comando. |
| Robô não para quando o programa trava | O firmware mantém a última velocidade. Mande `robo.stop()` (ou qualquer JSON em `cmd/mode`). |
| Vídeo do robô não chega no PC | Firewall do PC: `sudo ufw allow 5001/udp`. O IP impresso por `start_camera()` precisa ser da rede `192.168.0.x`. |
| Ground truth não acha o robô | Robô fora da imagem do teto, ou o adesivo não é o ArUco **ID 2**. |
| Dois programas Python se derrubam | Mesmo `client_id` MQTT nos dois. Use ids diferentes em `robo.connect(client_id=...)`. |
| Compilação do firmware reclama do caminho | A pasta mudou de lugar: `idf.py fullclean` dentro de `firmware/`. |

Mais casos dos utilitários: [utilitarios/README.md §12](utilitarios/README.md#12-solução-de-problemas).
