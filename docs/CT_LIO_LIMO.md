# CT-LIO no LIMO Pro — Fases 1 e 2

Fase 1: integração/build de software. Fase 2: validação física concluída em
05/10/2026 com o MID-360 real, 90 s parado e **uma única** passagem reta de
aproximadamente 1 m. Nenhuma curva, ré ou segunda tentativa foi executada.

## Identidade

- Algoritmo original: https://github.com/chengwei0427/ct-lio
- Port ROS 2: https://github.com/inkccc/ct_lio_ros2
- Branch do port: `main`
- Commit fixado: `bdc683e9fe74de8fc7501086f7ae105d96bc5db1`
- Worktree: `/home/agilex/levir_limo_real/CT_LIO_2026-10-05`, branch
  `ct-lio/2026-10-05`.

O GitHub marca `inkccc/ct_lio_ros2` como fork de `chengwei0427/ct-lio`.
README e fontes do port mantêm CT-ICP contínuo, ESKF, transformação por tempo
de ponto e mapa voxel. O port faz alterações de engenharia em filas, LRU e
sincronização; não se afirma identidade byte a byte com o ROS 1 original.
Fonte vendorizada em `ros2_ws/src/ct_lio`, sem `.git` aninhado.

## Dependências e imagem

O pacote é `ct_lio_ros2`; executável `ct_lio_node`. Depende de ROS 2 Humble
(`rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2`, `tf2_ros`,
`pcl_conversions`, `eigen3_cmake_module`), Ceres 2, PCL, Eigen 3 e headers
Sophus/tsl incluídos pelo upstream e `fmt`. O port usa `fmt` mas não o ligava
explicitamente; `CMakeLists.txt` foi ajustado para `fmt::fmt` após erro de
link no primeiro build ARM64. Não depende de runtime ROS 1. A imagem
herda a stack ARM64 validada `limo-real-fast-livo2:stage2`, ID
`sha256:bba1ef40cc6efaa3ed6e540cff9eda97d9140c77dafd6a04a0532b2fc7981ce1`.
Ela já contém Ceres, PCL e Eigen; não houve nova instalação apt. A imagem
derivada é `limo-real-ct-lio:humble`. A compilação seleciona somente
`ct_lio_ros2`, sequencial, `-j1`, preservando as outras imagens.
Build ARM64 concluído em 05/10/2026: imagem ID
`sha256:cf3cb9915093054dbb3162e7c7b13fb1aa13c59ae0d5b31d3956426805cc212b`.

```bash
cd /home/agilex/levir_limo_real/CT_LIO_2026-10-05
bash docker/build_ct_lio.sh
docker image inspect limo-real-ct-lio:humble --format '{{.Id}}'
```

## MID-360: tipo e tempo de ponto

O driver oficial já existente, em `xfer_format=0`, publica
`/livox/lidar` como `sensor_msgs/msg/PointCloud2`. O layout XYZRTL
tem 26 bytes por ponto: `x/y/z/intensity` FLOAT32 nos deslocamentos
0/4/8/12, `tag/line` UINT8 em 16/17 e `timestamp` FLOAT64 em 18.
O cabeçalho recebe `pkg.base_time`; cada `timestamp` recebe
`pkg.points[i].offset_time`. Seguindo `pub_handler.cpp`, este valor contém
**tempo absoluto do ponto em nanossegundos**, apesar do nome da variável.
Não é deslocamento relativo ao quadro no PointCloud2. O pré-processador do
CT-LIO subtrai o tempo do primeiro ponto e divide por 1e9, obtendo segundos
relativos; `alpha_time = relative_time/timespan` alimenta a interpolação
contínua de pose. Nenhum tempo é sintetizado a partir do índice do ponto.

Modificação mínima em `src/sensor/point_cloud_preprocessor.cpp`: `timestamp`
é reconhecido explicitamente como nanossegundos, inclusive se a primeira
amostra valer zero; a subtração é feita antes da divisão para preservar
precisão. Nuvens sem campo temporal FLOAT64 ou sem intervalo temporal positivo
são rejeitadas, em vez de receber uma duração artificial de 0,1 s.
`test/timestamp_smoke.cpp` reproduz o layout do driver e verifica 0/0,05/0,1 s,
`alpha` 0/0,5/1 e rejeição sem `timestamp`. Algoritmo CT-ICP/ESKF intacto.

## IMU e extrínseca

O driver do MID-360 neste LIMO publica `/livox/imu` como
`sensor_msgs/msg/Imu` a cerca de 200 Hz; medições reais anteriores neste
conjunto mostraram norma parada próxima de 1,0. O valor está em **g**.
`LioNode::ImuCallback` do port multiplica cada componente da aceleração por
`eskf.gravity_norm` antes de enviar à inicialização e ESKF, cujos estados
esperam **m/s²**. O perfil LIMO mantém o valor do port `9.7880` m/s² e inicia
no modo `static`. O giroscópio é copiado sem conversão: o driver entrega
**rad/s**, mesma unidade esperada por `ImuMeasurement`. A detecção de
saturação opera após a conversão, usando `satu_acc=29.4` m/s² e
`satu_gyro=35.0` rad/s do upstream. Os valores serão avaliados com dados
físicos na Fase 2.

`LidarOdometry::TransformPoint` aplica
`p_world = R_world_imu * (R_imu_lidar * p_lidar + T_imu_lidar) + t_world_imu`.
Portanto a extrínseca configura diretamente a transformação LiDAR → IMU
validada neste LIMO: `T=[-0.011,-0.02329,0.04412]` m, `R=I`.
O port não possui estimativa online da extrínseca; ela permanece fixa.

## Perfil e saídas esperadas

`config/limo_mid360.yaml` preserva `config/params.yaml` upstream.
Usa `/livox/lidar`, `/livox/imu`, `use_sim_time=false`, frame global
`ct_lio_odom` e frame móvel `livox_frame`. Saídas esperadas pelo código:
`/ct_lio/odom` (`nav_msgs/msg/Odometry`),
`/ct_lio/odometry_path` (`nav_msgs/msg/Path`),
`/ct_lio/map_incremental` (`sensor_msgs/msg/PointCloud2`) e
TF `ct_lio_odom → livox_frame`. A gravação de PCD fica inicialmente desligada.
Launch dedicado: `launch/limo_mid360.launch.py`. RViz ROS 2 dedicado:
`config/rviz/limo_mid360.rviz`, com nuvem, trajetória e TF.

## Comandos para reproduzir a Fase 2

Antes de iniciar o Livox, verifique que nenhum driver Livox, de qualquer
domínio ROS, usa o sensor. Não encerre processos de outras tarefas sem
coordenação. Se necessário após reboot, adicione o IP de forma **temporária**:

```bash
ip -4 addr show eth0
sudo ip addr add 192.168.1.50/24 dev eth0   # somente se ausente
ping -c 2 192.168.1.179
cd /home/agilex/levir_limo_real/CT_LIO_2026-10-05
bash docker/run_ct_lio.sh
```

O helper cria `limo-ct-lio` no domínio ROS 46. Se já existir parado, use
`docker start limo-ct-lio`. Em terminais diferentes:

```bash
# Driver oficial, um só, PointCloud2/xfer_format=0; não inicia RViz duplicado.
docker exec -it limo-ct-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame -p user_config_path:=/workspace/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json -p cmdline_input_bd_code:=livox0000000001'

# Alternativa via launch oficial: ros2 launch livox_ros_driver2 rviz_MID360_launch.py
# (também abre o RViz genérico do driver; não execute junto com o comando acima).

# CT-LIO. Deixe o robô parado na inicialização da IMU.
docker exec -it limo-ct-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; ros2 launch ct_lio_ros2 limo_mid360.launch.py rviz:=false'

# Visualização LIMO.
docker exec -it limo-ct-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; rviz2 -d /workspace/ros2_ws/install/ct_lio_ros2/share/ct_lio_ros2/config/rviz/limo_mid360.rviz'
```

Ao terminar, Ctrl+C nos processos e `docker stop -t 15 limo-ct-lio`.

## Verificações da Fase 1 e pendências

O build ARM64, teste sintético de timestamp, descoberta do pacote,
executável, resolução do launch, instalação de YAML/RViz e `ldd` sem
bibliotecas ausentes passaram. Smoke sem driver no domínio ROS 46 iniciou
`ct_lio_node`, carregou `lidar_topic=/livox/lidar`, `imu_topic=/livox/imu`,
`odometry.extrinsic_t=[-0.011,-0.02329,0.04412]`,
`eskf.gravity_norm=9.788` e `save_map.enabled=false`. O nó criou uma
assinatura RELIABLE de `PointCloud2` em `/livox/lidar` e uma de `Imu` em
`/livox/imu`. Nenhum publisher de sensor estava ativo. O container de smoke
foi encerrado; não há container CT-LIO rodando ao fim da Fase 1.
O worktree CT-LIO foi comparado antes/depois: FAST-LIO2, FAST-LIVO2,
driver Livox e Dockerfile raiz sem alterações. Outras branches e worktrees
não foram modificados. Sem commit, push, merge ou tag.

Os resultados físicos da Fase 2 estão abaixo.

## Fase 2 — validação física em 05/10/2026

O LIMO estava no chão, com corredor reto de pelo menos 1,5 m confirmado pelo
operador, que permaneceu junto ao robô para parada de emergência. O Ethernet
`eth0` tinha `192.168.1.5/24`; foi acrescentado apenas para esta sessão o IP
secundário `192.168.1.50/24`. O MID-360 em `192.168.1.179` respondeu a
3/3 pings. Não havia driver Livox concorrente. Foram usados a imagem da Fase 1
sem rebuild, o domínio ROS 46 e um único driver oficial com `xfer_format=0`,
`multi_topic=0` e `publish_freq=10.0`.

### Sensor, tempo e saídas

`/livox/lidar` publicou `sensor_msgs/msg/PointCloud2` com os campos reais
`x,y,z,intensity,tag,line,timestamp`, entre 16.224 e 23.424 pontos/quadro
na janela parada (cerca de 20 mil em regime estável). O teste leve antes de
iniciar CT-LIO mediu 10,19 Hz; após o movimento, 9,50 Hz. Os primeiros dez
quadros inspecionados em cada janela tinham `timestamp` FLOAT64 finito,
intervalo positivo por quadro, normalmente 0,098–0,101 s, sem regressão do
timestamp de cabeçalho entre quadros. Os pontos **não estão estritamente
ordenados** por tempo no vetor: cerca de 130–140 pares adjacentes por quadro
retrocedem tipicamente 0,25 ms (pior recuo observado em três quadros:
0,358 ms). Isso é uma inversão local de ordem de amostras, não um retorno do
relógio do sensor: primeiro e último pontos cobrem ~0,10 s, e o
pré-processador CT-LIO usa o tempo de cada ponto e o máximo do intervalo,
sem exigir ordenação. A limitação fica registrada para qualquer consumidor
que suponha ordem temporal estrita.

`/livox/imu` publicou `sensor_msgs/msg/Imu` a 200,66 Hz antes do CT-LIO e
200,20 Hz no contador CDR leve após o movimento. Parado, a norma média da
aceleração foi 0,9911 g e a norma média do giro 0,0106 rad/s. Não houve
regressão de cabeçalho nem amostra IMU não finita. A sonda completa, que
desserializa nuvens e `Path`, perdeu mensagens sob carga; por isso as taxas
reais de IMU acima vêm da pré-verificação e do contador CDR `rates_raw.py`.

Após inicialização estática da IMU, publicaram mensagens reais:

| Saída | Tipo | Taxa medida em 90 s parados | Taxa no contador leve após movimento |
| --- | --- | ---: | ---: |
| `/ct_lio/odom` | `nav_msgs/msg/Odometry` | 10,00 Hz | 9,07 Hz |
| `/ct_lio/odometry_path` | `nav_msgs/msg/Path` | 9,99 Hz | 8,77 Hz |
| `/ct_lio/map_incremental` | `sensor_msgs/msg/PointCloud2` | 9,97 Hz | 8,77 Hz |
| `/tf` | `tf2_msgs/msg/TFMessage` | 9,98 Hz | 9,07 Hz |

TF contém `ct_lio_odom → livox_frame`. Um quadro do mapa após o movimento
tinha 13.255 pontos, todos com XYZ finitos. O `Path` em `ct_lio_odom` tinha
6.184 poses finitas, sem lacuna maior que 8,6 mm entre poses consecutivas.

### Teste estacionário de 90 s

`validation/ct_lio/probe.py` coletou por 90,016 s, com robô parado. A posição
inicial CT-LIO foi `[-0,002259, 0,000257, 0,000849]` m; a final,
`[-0,001080, -0,000826, 0,000020]` m. Deslocamento final **1,80 mm** e
deslocamento máximo **5,78 mm**. Zero mensagens não finitas e zero regressões
de cabeçalho em LiDAR, IMU, odometria, trajetória e mapa. O processo CT-LIO
permaneceu ativo, sem erro de sincronização, reset ou crash nos logs. RViz
abriu com OpenGL 4.5 em software, status global **Ok**, fixed frame
`ct_lio_odom`, nuvem, trajetória e TF ativos. Paredes e objetos permaneceram
alinhados visualmente antes e depois dos 90 s.

### Única passagem reta de aproximadamente 1 m

O `limo_base` foi iniciado em container separado com `/dev/ttyTHS0` e tópicos
isolados em `/ct_lio_test/*`, sem alterar o código de controle. Antes de
comandar, reportou `vehicle_state=0`, `control_mode=1`, `error_code=0`,
`motion_mode=0`, odometria fresca e velocidade zero. Uma faixa frontal de
`x=0,25…1,5 m`, `|y|<0,35 m`, `z=-0,15…0,8 m` do MID-360 não tinha retornos
nas dez nuvens de pré-checagem; o operador também confirmou o corredor livre.
Essa checagem não substitui proteção anticolisão certificada.

`validation/ct_lio/motion_1m.py` comandou somente `linear.x=0,045 m/s`,
`angular.z=0`, durante **22,924 s**; o limite rígido era 30 s. O critério de
parada foi a **odometria de roda**, independente do CT-LIO. A distância de
roda ao cortar o comando foi **1,0038 m** (máximo 1,0042 m). O script enviou
zero por ~1,8 s e saiu. Uma leitura posterior mostrou velocidades linear e
angular zero e odometria de roda inalterada; havia **zero publicadores** no
tópico `/ct_lio_test/cmd_vel`. O container da base foi então encerrado.

No instante do corte, CT-LIO estimava deslocamento de **0,9468 m**, com
`ΔXYZ=[+0,9464, -0,0238, +0,0131]` m. A estimativa continuou a se ajustar
após a parada; depois de assentar ficou em **1,0583 m** desde o início da
passagem, com `ΔXYZ=[+1,0578, -0,0255, +0,0182]` m e mudança total de
orientação de **3,65°**. Portanto, a diferença final em relação à referência
operacional de roda é **+0,0545 m** (+5,4%). Na janela posterior de 15,18 s,
o deslocamento CT-LIO entre início e fim foi 2,82 mm (máximo 5,31 mm).
Nenhum NaN/Inf, regressão de cabeçalho, reset ou crash foi observado. A
direção estimada foi +X, coerente com o avanço físico; a pequena mudança
lateral/angular deve ser lida como comportamento estimado, sem metrologia
externa. O mapa não apresentou cisalhamento ou explosão visual durante ou
após a passagem, e o `Path` permaneceu contínuo.

RViz manteve **Global Status: Ok** durante e após o movimento; há capturas em
`validation/ct_lio/ct_rviz_stationary_end.png`, `ct_rviz_motion.png` e
`ct_rviz_after_motion.png`. Sem avisos de RViz além de estéreo não suportado,
que não afeta a visualização. Ao final, os containers da base e do CT-LIO
(incluindo driver e RViz) foram encerrados. Nenhum outro teste de movimento
deve ser executado nesta etapa.

### Limitações e reprodução

A referência de roda não é ground truth externo e pode deslizar. O recuo
local na ordenação dos pontos Livox deve ser respeitado por ferramentas que
esperem timestamp monotônico dentro do vetor. A sonda Python completa perdeu
amostras durante o movimento porque desserializa nuvens grandes e um `Path`
crescente; seus ~3,9 Hz são taxa **recebida pela sonda**, não taxa confiável
dos publicadores. O contador CDR e a guarda mediram ~9–10 Hz de CT-LIO com
menor carga. O CT-LIO também mostrou ajuste transitório de ~11 cm entre a
leitura no corte e a pose assentada; avaliar latência/correções e precisão
métrica requer experimento separado, com referência externa, sem repetir
movimento nesta fase.

Os arquivos pequenos em `validation/ct_lio/` incluem `sensor_preflight.json`,
`stationary.json`, `motion_1m.json`, `motion_observer.json`, `post_stop.json`,
`rates_after_motion.json`, `path_check.json`, `map_check.json`,
`phase2_summary.json`, scripts e logs.
`raw/` permanece ignorado pelo Git. **Nenhum commit, push, merge ou tag foi
feito.**
