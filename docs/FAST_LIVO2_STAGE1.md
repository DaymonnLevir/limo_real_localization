# FAST-LIVO2 Etapa 1 — LIMO Pro / MID-360 / ROS 2 Humble

Status: **Etapa 1 validada com MID-360 físico e movimento real no chão**. Data: 2026-10-04.

Validação funcional de um deslocamento curto no laboratório; não é uma avaliação
de precisão absoluta, percurso longo, curvas, autonomia ou segurança de navegação.

## Arquitetura e proveniência

MID-360 físico (192.168.1.179) → **único** `livox_ros_driver2` existente →
`/livox/lidar` + `/livox/imu` → `fast_livo` (`img_en=0`, `lidar_en=1`) →
`/aft_mapped_to_init`, `/path`, `/cloud_registered` → RViz.

Tudo executa no Docker Ubuntu 22.04 / ROS 2 Humble, compilado na Jetson ARM64.
Nenhum processo deste projeto usa ROS 1 Noetic ou ROS 2 Foxy do host.
O pacote `ros2_ws/src/fast_lio2` e o driver existente são preservados.
Os dois algoritmos não são iniciados simultaneamente nesta etapa.

- Fork: https://github.com/U-AMC/FAST-LIVO2-ROS2
- Commit-base: `a09599a288c56b832114d7ad9e898248a4111d9e`.
- Referência conceitual: https://github.com/hku-mars/FAST-LIVO2.
- Dependência Vikit: https://github.com/U-AMC/rpg_vikit_rational_polynomial,
  commit `6f213c7fe6be1aff1fc79c7bd6103e3e9bc4a02d`.
- Código importado com `git archive`, sem `.git` aninhado.
- Diretório novo: `ros2_ws/src/fast_livo2`; nome ROS preservado: `fast_livo`.
- Dockerfile efetivamente utilizado: `docker/Dockerfile` na raiz do repositório.
  O Dockerfile dentro do pacote importado é somente referência upstream.

## Alterações locais

- `src/LIVMapper.cpp`: condiciona carregamento/initialização de câmera e VIO a
  `img_en`; não cria gerenciador visual, assinatura nem publicação de imagem
  quando desativada. Isso também evita destruir um VIO não inicializado. O upstream fazia
  carregamento incondicional de câmera através de `parameter_blackboard`, mesmo
  no modo LiDAR-inertial. Não é preciso uma câmera fictícia ou esse servidor.
- `config/limo_mid360_only.yaml`: derivado de `mid360_only.yaml`, mantendo o exemplo
  original. Define os tópicos reais, imagem desativada, LiDAR/IMU ativados,
  extrínseca do LIMO, sem gravação de PCD/imagens, rolling shutter ou estimação
  de exposição. Parâmetros de processamento LiDAR são os do exemplo upstream.
- `launch/limo_livox.launch.py`: inicia o driver existente, separadamente, com
  `xfer_format=0`, `multi_topic=0`, 10 Hz e JSON existente do MID-360.
- `launch/limo_mid360_only.launch.py`: inicia apenas o estimador e, opcionalmente,
  RViz. Sem câmera, republisher de imagem, parameter_blackboard ou respawn.
- `rviz_cfg/limo_mid360_only.rviz`: nuvem, trajetória e pose no frame `camera_init`.
- `docker/Dockerfile`: preserva a seleção de pacotes preexistente e adiciona
  somente `vikit_ros` e `fast_livo`; build sequencial, dois jobs por compilação.

Não houve tuning do filtro, alteração de EKF, sincronização ou buffers.

## Formato e extrínseca verificados no código

No fork escolhido, `preprocess.lidar_type=8` (MID360) usa **PointCloud2**;
`CustomMsg` é escolhido pelo ramo AVIA. Portanto esta sessão mantém
`xfer_format=0` no mesmo driver, sem cópias nem alteração de seus arquivos.
Os campos são x/y/z/intensity FLOAT32, tag/line UINT8 e timestamp FLOAT64,
26 bytes por ponto. O timestamp do ponto é absoluto em nanossegundos,
convertido para diferença em milissegundos em `mid360_handler`.

A transformação em `LIVMapper.cpp::pointBodyToWorld` e no processamento IMU é
`p_IMU = R * p_LiDAR + T`, a mesma encontrada no FAST-LIO2 existente.
Assim usamos `T=[-0.011,-0.02329,0.04412]`, `R=I`. Não é extrínseca de câmera.
O driver publica aceleração em g; o processamento upstream normaliza pela
média de inicialização e por `G_m_s2`. Nenhuma conversão adicional foi inserida.

## Dependências adicionadas

`libopencv-dev`, `libfmt-dev`, `ros-humble-sophus`, `ros-humble-cv-bridge`,
`ros-humble-image-transport`, Vikit fixado no commit acima. Vikit common é
compilado com CMake e instalado em `/usr/local`; Vikit ROS é compilado por
colcon. São dependências de compilação do port completo, mesmo sem câmera em
runtime. Eigen, PCL, Boost, SDK Livox e driver já existiam na imagem.

## Reconstruir

No robô, somente nesta branch/workspace:

```bash
cd /home/agilex/levir_limo_real/FAST_LIVO2_2026-10-04
git branch --show-current  # fast-livo2/2026-10-04
docker build -t limo-real-fast-livo2:dev -f docker/Dockerfile .
```

## Rede e container

Sensor ligado e cabo conectado. Não modificar wlan0:

```bash
ip -4 -br addr
# Apenas se 192.168.1.50/24 não existir:
sudo ip addr add 192.168.1.50/24 dev eth0
ping -c 3 192.168.1.179
```

O endereço foi adicionado em runtime (não persiste após reinicialização).
Para a sessão gráfica local atual da Jetson, DISPLAY=:0 e cookie X11 em
`/run/user/1000/gdm/Xauthority`. Se a sessão de desktop mudar, verificar esses
valores. Não é necessário liberar X11 globalmente com `xhost +`.

```bash
docker run -d --name limo-fast-livo2-stage1 --network host --ipc host \
  --hostname "$(hostname)" \
  -e DISPLAY=:0 -e XAUTHORITY=/tmp/host.xauthority \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /run/user/1000/gdm/Xauthority:/tmp/host.xauthority:ro \
  limo-real-fast-livo2:dev sleep infinity
```

A execução normal da odometria não monta `/dev/ttyTHS0`. No teste físico
realizado nesta sessão, o usuário autorizou controle pela base; a serial foi
montada somente em um container temporário separado, encerrado ao final.
Não monte o workspace do host sobre `/workspace`, pois isso esconderia o build.
Antes de repetir, encerre a sessão anterior para não ter dois drivers na rede.

## Executar driver e estimador separadamente

Terminal 1:

```bash
docker exec -it limo-fast-livo2-stage1 bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 launch fast_livo limo_livox.launch.py
```

Terminal 2, primeiro conferir os dados:

```bash
docker exec -it limo-fast-livo2-stage1 bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/lidar
# Ctrl-C, depois:
ros2 topic hz /livox/imu
# Ctrl-C; mantenha o robô parado durante inicialização:
ros2 launch fast_livo limo_mid360_only.launch.py use_rviz:=true
```

Terminal 3:

```bash
docker exec -it limo-fast-livo2-stage1 bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 node info /laserMapping
ros2 param get /laserMapping common.img_en
ros2 param get /laserMapping common.lidar_en
ros2 topic hz /aft_mapped_to_init
```

Para encerrar esta sessão: Ctrl-C nos launches e
`docker stop limo-fast-livo2-stage1`. Não parar containers de outros trabalhos.

## Evidências e problemas

1. Sensor inicialmente desligado: eth0 NO-CARRIER, ping sem resposta. O usuário
   ligou o Livox; ping passou (3/3), sem alterar Wi-Fi.
2. eth0 possuía 192.168.1.5/24; adicionado 192.168.1.50/24, destino do JSON Livox.
3. Pré-validação com imagem baseline e sensor físico, janela de 20 s:
   LiDAR 10.043 Hz (201 mensagens), IMU 200.073 Hz (4002 mensagens).
   Nenhum timestamp repetido/regressivo. Scan ~0.09979 s, timestamp inicial do
   ponto coincide com header. Não foram usados rosbag, simulação ou dados fake.
4. Inicialização incondicional da câmera no upstream: corrigida conforme acima.
5. Build do Vikit não encontrava `sophus/se3.hpp`: o fork usa variáveis CMake
   legadas, enquanto o pacote Sophus do ROS exporta um target. Adicionado
   `CPATH=/opt/ros/humble/include`, conforme o Docker upstream.
6. `ros2 topic hz` durante instalação das dependências indicou LiDAR ~9.69 Hz
   e IMU ~199.99 Hz. É uma medição do subscriber CLI sob carga de build; a
   janela dedicada de 20 s acima registrou ~10 e ~200 Hz.

7. RViz inicialmente apresentava corrupção gráfica no Docker. Resolvido em
   runtime com `--ipc host` e `QT_X11_NO_MITSHM=1`, mantendo X11 autenticado e
   renderização por software. Não exigiu mudança no algoritmo ou novo build.
8. Depois de o usuário retirar o robô da mesa e colocá-lo no chão, o estimador
   e o RViz foram reiniciados. A validação de movimento refere-se **somente à
   nova origem criada no chão**, não ao transporte manual anterior.

## Build validado

Imagem ARM64: `limo-real-fast-livo2:dev`.
ID: `sha256:4324ca2fd03a0dd8c4b1197a9df3bdf196cfc951bdfac185d368562adf53dad8`.
O colcon terminou com **6 packages finished [8min 46s]**, exit code 0:
`limo_msgs`, `livox_ros_driver2`, `vikit_ros`, `fast_lio`, `fast_livo`, `limo_base`.
Há avisos CMake, Eigen/NEON e de headers obsoletos; não houve erro no build final.

A integração exclui o PDF suplementar upstream e duas imagens de depuração
não utilizadas, sem alteração de fontes, headers, CMake, launch ou YAML
compilados/testados. A figura do README upstream foi preservada.

## Resultados medidos

| Tópico | Tipo ROS 2 | Frequência observada |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 9,999 Hz antes do estimador; 9,771 Hz no CLI com RViz ativo |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 200,065 Hz antes do estimador; 200,001 Hz no CLI com RViz ativo |
| `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | **9,974 Hz** durante a janela do teste físico; CLI posterior 9,932 Hz |
| `/path` | `nav_msgs/msg/Path` | 9,971 Hz na janela de 20 s com IPC corrigido |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | publicação contínua confirmada no observador e RViz |

`/laserMapping` foi confirmado assinando **ambos** os tópicos Livox, sem assinatura
ou publicação de imagem. Parâmetros efetivos: `common.img_en=0`, `common.lidar_en=1`.
O único publisher de entrada é `/livox_lidar_publisher`; a odometria é publicada
por `/laserMapping`, não pela base. Frame da odometria: `camera_init`; filho:
`aft_mapped`. `/Odometry` permanece associado ao FAST-LIO2, que não foi iniciado.

O log da inicialização no chão registrou `IMU Initials` e
`Gravity Alignment Finished`; gravidade inicial aproximada
`[-0.1852, 0.1674, -9.8068]` m/s². Sem crash na sessão de validação.

Antes do teste no chão, uma janela estacionária de 180 s registrou 1796 poses,
9,977 Hz, todas finitas, timestamps crescentes e desvio máximo de 3,55 mm em
relação à primeira amostra. Ela comprova continuidade, não movimento físico.

## Teste físico executado pelo agente

O usuário informou área disponível de 1 × 1 m e autorizou o agente a mover o
robô. O estimador foi reiniciado no chão antes de qualquer comando.
Foi usado o `limo_base` **existente na mesma imagem Humble**, em container
separado com `/dev/ttyTHS0`, `pub_odom_tf=false`, sem alterar seus arquivos.
Comandos e feedback foram remapeados para `/stage1_test/*`, evitando outros
publishers de `/cmd_vel`. O FAST-LIVO2 continuou usando exclusivamente LiDAR/IMU.

- Estado da base: normal, comando serial, modo diferencial, sem erros.
- Teste: avanço reto a 0,02 m/s por 2 s; duração efetiva enviada 1,999 s.
- Processo de guarda separado: limita velocidade, corta lease após 0,12 s,
  verifica frescor de LiDAR/pose/roda/status, distância máxima 7 cm e afastamento
  observado mínimo 0,55 m. Os retornos iniciais mais próximos nas faixas
  inspecionadas estavam a aproximadamente 0,69 m. Esses controles foram usados
  apenas neste teste; não constituem um sistema de navegação/evitação certificado.
- Feedback de roda confirmou velocidade aproximadamente 0,016–0,017 m/s nas
  amostras em movimento; após comando zero, confirmou velocidade zero.
- FAST-LIVO2: posição inicial `[0.000228, -0.000213, 0.000049]` m e final
  `[0.032442, -0.001010, 0.000585]` m; deslocamento líquido **3,22 cm**.
- Pico de deslocamento: 3,74 cm; maior passo entre poses: 3,54 mm.
- Janela de 35,044 s: **350 poses, 9,974 Hz**, sem NaN/Inf nem timestamp repetido
  ou regressivo; maior intervalo de recepção 0,208 s.
- A trajetória avançou no eixo X e estabilizou após parar, coerente com o comando.
  Não foi testada curva: o avanço curto foi suficiente para validar resposta
  física da odometria dentro do espaço reduzido disponível.
- Nenhum limite de guarda foi acionado. Enviados comandos zero ao terminar;
  guarda finalizada e container de controle da base **parado**.

Não houve medição externa com régua ou ground truth. Os 3,22 cm são a estimativa
FAST-LIVO2, corroborada pela direção do comando e pelo feedback de rodas.

## RViz e limites observados

RViz renderizou corretamente a nuvem do laboratório e a trajetória, com
`Global Status: Ok`. Uma segunda vista temporária com grade de 1 cm confirmou
visualmente o avanço reto e os agrupamentos de poses antes/depois. Capturas
foram entregues separadamente ao usuário e não versionadas no repositório.

A vista densa por software, acumulando 30 s de nuvem, mostrou cerca de 1–2 FPS;
a vista de trajetória isolada mostrou 10 FPS. A odometria se manteve em ~10 Hz.
Não foram feitos tuning, alteração de EKF, buffers ou otimização do algoritmo.
O observador Python multi-tópico recebe menos mensagens de IMU/nuvem sob carga;
por isso os valores finais de sensor acima usam medições dedicadas do CLI.

O upstream mantém o timestamp do header de `/path` fixo na inicialização,
enquanto acumula poses; a odometria tem timestamps crescentes. A visualização
no frame fixo funciona. Essa particularidade foi documentada, não modificada.

## Evidências compactas e repetição de medidas

- `validation/stage1_results.json`: resultados estruturados e amostras de pose.
- `validation/floor-startup.txt`: inicialização de IMU/gravidade e RViz.
- `validation/graph-and-parameters.txt`: assinaturas, tipos e parâmetros efetivos.
- `validation/topic-hz.txt`: últimas linhas das três medições CLI.
- `measure_stage1.py`: observador somente leitura; não publica nem move o robô.

Para medir novamente, sem enviar movimento:

```bash
cd /home/agilex/levir_limo_real/FAST_LIVO2_2026-10-04
docker cp docs/measure_stage1.py limo-fast-livo2-stage1:/tmp/measure_stage1.py
docker exec limo-fast-livo2-stage1 bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspace/ros2_ws/install/setup.bash
  python3 /tmp/measure_stage1.py --seconds 30 --topics /aft_mapped_to_init
'
```

Logs completos e scripts temporários de atuação permanecem fora do Git, em
`/tmp/limo-stage1-evidence` e containers de teste parados. Nenhuma credencial,
build local, rosbag, dump de nuvem ou screenshot foi adicionado ao commit.

## Proteção do baseline e Git

Branch: `fast-livo2/2026-10-04`; base do projeto:
`a15dca575fe924ac89f28d251e5ddb093d12e68e`.
`git diff -- ros2_ws/src/fast_lio2` permaneceu vazio e todos os arquivos desse
pacote passaram na verificação SHA-256 contra o registro anterior à tarefa.
Nenhum arquivo de outros pacotes ROS ou do driver Livox foi modificado.
A modificação preexistente de seleção de pacotes no Dockerfile foi preservada.

O commit de conclusão pode ser identificado com:

```bash
git log -1 --format='%H %s' -- docs/FAST_LIVO2_STAGE1.md
git status --short
git diff a15dca575fe924ac89f28d251e5ddb093d12e68e HEAD -- ros2_ws/src/fast_lio2
```

Não houve merge, push ou reescrita de histórico. A sessão final mantém driver,
FAST-LIVO2 e RViz ativos, sem processo de controle de movimento.

