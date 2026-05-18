# Rastreamento de Robô com ArUco + Raspberry Pi

Sistema de rastreamento visual utilizando marcadores ArUco, OpenCV e Raspberry Pi Camera.

O programa realiza:

- Streaming de vídeo da Raspberry Pi via TCP;
- Detecção de marcadores ArUco 4x4;
- Estimativa de posição `(x, y)` e orientação `(θ)` do robô;
- Conversão de coordenadas da imagem para coordenadas reais usando homografia;
- Gravação da trajetória do robô em CSV;
- Geração de imagem final contendo o trajeto percorrido.

---

# Tecnologias Utilizadas

- Python
- OpenCV
- NumPy
- Paramiko (SSH)
- Raspberry Pi Camera (`rpicam-vid`)
- ArUco Markers

---

# Estrutura do Projeto

```text
.
├── gt.py
├── H.npy
├── .env
├── trajetoria_robo.csv
└── resultado_trajetoria.jpg
```

---

# Configuração do `.env`

Crie um arquivo `.env` na raiz do projeto:

```env
GT_IP=192.168.0.125
GT_USER=usuario
GT_PASSWORD=senha
GT_PORT=8888
```

---

# Instalação das Dependências

```bash
pip install opencv-python numpy paramiko python-dotenv
```

---

# Arquivo de Homografia

O sistema utiliza um arquivo chamado:

```text
H.npy
```

Esse arquivo contém a matriz de homografia usada para converter coordenadas da imagem em coordenadas reais do ambiente.

Caso o arquivo não exista, execute previamente o script de calibração:

```bash
python comp_homo.py
```

---

# Funcionamento

## 1. Conexão SSH

O programa conecta na Raspberry Pi via SSH utilizando o Paramiko.

---

## 2. Inicialização do Stream

Após conectar, o programa executa:

```bash
rpicam-vid
```

para iniciar o streaming MJPEG via TCP.

---

## 3. Recepção do Vídeo

O OpenCV recebe o stream usando:

```python
cv2.VideoCapture()
```

---

## 4. Detecção ArUco

O sistema detecta marcadores ArUco 4x4 utilizando:

```python
cv2.aruco.ArucoDetector()
```

---

## 5. Estimativa de Pose

A partir dos cantos do marcador:

- calcula o centro do robô;
- converte pixels para coordenadas reais;
- estima o ângulo de orientação do robô.

---

## 6. Gravação da Trajetória

Durante a gravação, o sistema salva:

- timestamp;
- posição X;
- posição Y;
- orientação θ.

Os dados são armazenados em:

```text
trajetoria_robo.csv
```

---

## 7. Geração da Imagem Final

Ao finalizar:

- a imagem inicial e final são mescladas;
- a trajetória do robô é desenhada;
- o resultado é salvo em:

```text
resultado_trajetoria.jpg
```

---

# Controles do Teclado

| Tecla | Função |
|---|---|
| `s` | Inicia gravação |
| `f` | Finaliza e salva |
| `q` | Sai sem salvar |

---

# Configurações de Resolução

O sistema possui presets de resolução:

```python
LOW
MEDIUM
HIGH
FULLHD
FAST
EQUAL
```

A seleção é feita pela variável:

```python
PROFILE = "EQUAL"
```

---

# Saídas Geradas

## CSV da trajetória

```text
trajetoria_robo.csv
```

Contém:

| timestamp | x | y | theta_deg |
|---|---|---|---|

---

## Imagem da trajetória

```text
resultado_trajetoria.jpg
```

Imagem contendo o trajeto percorrido pelo robô.

---

# Observações

- O marcador ArUco deve estar visível para a câmera;
- A homografia deve estar corretamente calibrada;
- O Raspberry Pi deve estar acessível na rede;
- O `rpicam-vid` deve estar instalado na Raspberry Pi.

---

# Exemplo de Aplicações

- Rastreamento de robôs móveis;
- Odômetria visual;
- Navegação autônoma;
- SLAM visual;
- Controle e monitoramento em laboratório.
