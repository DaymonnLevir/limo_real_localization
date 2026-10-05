# Point-LIO no LIMO Pro — ROS 2 Humble / ARM64

## Proveniência e isolamento

Worktree exclusivo: `/home/agilex/levir_limo_real/POINT_LIO_2026-10-05`, branch
`point-lio/2026-10-05`, base Git `a2b13d7355af0b67b680c4f00612bec80229beeb`.

Upstream: https://github.com/LihanChen2004/Point-LIO, branch `grid_map_ros2`,
SHA `155f46da0b1eb3fc7c7facf43b00cb124631c807` (verificado em 05/10/2026).
Fonte vendorizada em `ros2_ws/src/point_lio`; sem `.git` aninhado.
Algoritmo C++ e configurações genéricas preservados. Adicionados perfil LIMO,
launch e RViz nativo ROS 2. O RViz genérico do upstream usa classes ROS 1 e
seu launch divide incorretamente o nome do arquivo; o launch dedicado evita
esses problemas sem alterar os arquivos genéricos.

Imagem derivada: `limo-real-point-lio:humble`. Base local validada:
`limo-real-fast-livo2:stage2`, ID
`sha256:bba1ef40cc6efaa3ed6e540cff9eda97d9140c77dafd6a04a0532b2fc7981ce1`.
O script de build exige esse ID. A base deve estar disponível no Jetson;
não é uma imagem pública. Não sobrescreve imagens validadas nem exige novas
dependências apt. Compila apenas `point_lio`, sequencialmente, com `-j1`.
Bibliotecas ROS, Eigen, PCL, Python, OpenMP e driver Livox vêm da base.

## Build

```bash
cd /home/agilex/levir_limo_real/POINT_LIO_2026-10-05
bash docker/build_point_lio.sh
```

## Inicialização

Verifique `docker ps` e processos ROS antes de iniciar o driver. Deve existir
somente um driver Livox usando o sensor, inclusive entre domínios DDS diferentes.
Não encerre automaticamente processos de outras tarefas.

```bash
ip -4 addr show eth0
ping -c 2 192.168.1.179
# Somente se o IP 192.168.1.50/24 estiver ausente:
sudo ip addr add 192.168.1.50/24 dev eth0

cd /home/agilex/levir_limo_real/POINT_LIO_2026-10-05
bash docker/run_point_lio.sh
```

O script cria o container `limo-point-lio`, domínio ROS 45, X11/software OpenGL
e acesso à serial existente. Se um container com esse nome estiver parado,
retome-o com `docker start limo-point-lio`. Mantenha o robô parado durante a
inicialização da IMU. Reinicie Point-LIO após reposicionar o robô manualmente.

Em terminais separados:

```bash
# 1. MID-360: launch oficial já existente, xfer_format=1, 10 Hz.
docker exec -it limo-point-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; ros2 launch livox_ros_driver2 msg_MID360_launch.py'

# 2. Point-LIO e RViz.
docker exec -it limo-point-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; ros2 launch point_lio limo_mid360.launch.py rviz:=true'

# 3. Base existente. Só iniciar quando o robô puder receber comandos.
docker exec -it limo-point-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; ros2 run limo_base limo_base --ros-args -p port_name:=ttyTHS0 -p pub_odom_tf:=false'

# 4. Teleop com velocidades iniciais baixas; mantenha controle de parada disponível.
docker exec -it limo-point-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.02 -p turn:=0.1'
```

Para abrir somente RViz com Point-LIO já ativo:

```bash
docker exec -it limo-point-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; rviz2 -d /workspace/ros2_ws/install/point_lio/share/point_lio/rviz_cfg/limo_mid360.rviz'
```

Não execute os comandos acima duplicando processos já ativos. Ao finalizar,
pare teleop/base após enviar comando zero, encerre launches com Ctrl+C e use
`docker stop -t 15 limo-point-lio` quando não precisar da visualização.

## Entradas, unidades e transformações

- `/livox/lidar`: `livox_ros_driver2/msg/CustomMsg`, alvo 10 Hz. Aproximadamente
  20 mil pontos/quadro; `offset_time` em nanossegundos, convertido pelo
  preprocessamento para milissegundos. Quatro linhas, `lidar_type=1`.
- `/livox/imu`: `sensor_msgs/msg/Imu`, alvo 200 Hz. **Este driver publica
  aceleração em g**, embora o tipo ROS convencionalmente espere m/s². Medição
  parada: norma média 0,9951 g, desvio 0,0040 g; giro médio 0,00396 rad/s.
- `acc_norm=1.0`; Point-LIO multiplica por `9.81/acc_norm`. Limites conservadores
  mantidos do perfil MID-360: `satu_acc=3.0` g e `satu_gyro=35.0` rad/s.
  São limiares de rejeição do algoritmo, não medições de saturação física.
  Em `Estimator.cpp`, a comparação ocorre antes da normalização, nas unidades
  brutas; o comentário genérico sobre independência de unidades é impreciso.
- `pointBodyLidarToIMU`, em `src/laserMapping.cpp`, implementa
  `p_imu = R * p_lidar + T`. Portanto `T=[-0.011,-0.02329,0.04412]` m,
  `R=I`, sem inversão. `extrinsic_est_en=false`.
- `use_sim_time=false`, câmera não utilizada; não existe calibração de câmera
  envolvida neste teste.

Referência do protocolo Livox para unidades:
https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html

## Saídas e RViz

- `/aft_mapped_to_init`: `nav_msgs/msg/Odometry`.
- `/cloud_registered`: `sensor_msgs/msg/PointCloud2`, coordenadas `camera_init`.
- `/path`: `nav_msgs/msg/Path`.
- TF: `camera_init -> aft_mapped`. O upstream identifica `body` como child da
  mensagem Odometry; isso difere do nome publicado em TF. RViz usa o TF real.
- `/Laser_map`, `/cloud_effected` e `/cloud_registered_body` podem aparecer
  no grafo sem publicar; não confundir presença de publisher com dados reais.
- RViz usa fixed frame `camera_init`, nuvens registradas acumuladas por 10 s,
  trajetória e TF. Isso é uma visualização de nuvens registradas; não um
  serviço de exportação persistente do mapa interno.

## Validação reproduzível

```bash
docker exec -it limo-point-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; python3 /validation/measure.py --seconds 90 --output /validation/stationary.json --cloud-output /validation/raw/stationary_cloud.npz'
```

O medidor conta CustomMsg por CDR bruto para evitar que desserializar 20 mil
objetos Python por quadro falseie as frequências. Uma amostra é decodificada
para verificar número de pontos e tempos. As saídas incluem frequências,
intervalos, valores não finitos, regressões temporais e deslocamento. Arquivos
grandes ficam em `validation/point_lio/raw/`, ignorados pelo Git.

O teste automatizado de movimento usa `motion_guard.py` e `motion_pulse.py`,
com base remapeada para `/point_lio_test/*`, separada do `/cmd_vel` normal.
Exige feedback fresco, estado normal da base, velocidade <= 0,02 m/s,
deslocamento < 7 cm e afastamento >= 55 cm nos retornos filtrados. A guarda
independente envia zero quando a autorização temporária de comando expira
(0,12 s), ao atingir limites e ao encerrar. O pulso dura no máximo 2 s.
Esses scripts são instrumentos deste teste supervisionado, não navegação
autônoma nem um sistema certificado de prevenção de colisões.

## Resultados desta execução (05/10/2026, após reboot)

- Commit da branch já existente: `f051f32306bbe2e802eb26afcd5367b01db137e9`.
  Sem recompilação nesta retomada. Imagem ARM64 válida:
  `limo-real-point-lio:humble`, ID
  `sha256:534396c26c32c27dd986fa3a9488277f046ca34ff60f9004880a8bff44353acb`.
  `ros2 pkg executables point_lio` encontrou `pointlio_mapping`; launch e
  configuração instalados; `ldd` sem bibliotecas ausentes.
- Após reboot, `eth0` começou `NO-CARRIER`. Com o sensor energizado, o link
  retornou em `192.168.1.5/24`; foi adicionado *temporariamente*
  `192.168.1.50/24` e ping de `192.168.1.179` respondeu. O IP secundário
  desaparece no próximo reboot. O driver único iniciou com `xfer_format=1`.
- Janela parada: `validation/point_lio/stationary.json`, **90,03 s**. LiDAR
  **9,997 Hz**, aproximadamente **20.064 pontos por quadro** e intervalo de
  tempo por ponto de ~0 a 99,9 ms. Odometria e nuvem **10,007 Hz**;
  trajetória **9,997 Hz**. IMU: aceleração média **0,9968 g** (desvio
  0,0033 g), giro médio **0,0052 rad/s**. O medidor Python completo contou
  145,5 Hz de IMU, pois a inspeção de nuvens e trajetórias ocupou seu laço.
  A medição independente com `ros2 topic hz` confirmou **200,02 Hz**.
  Deslocamento estimado final **6,1 mm**; desvio máximo **20,3 mm**. Sem
  NaN/Inf, regressão de timestamps ou crash.
- Pulso de movimento: base em modo normal/serial, erro zero, comando reto
  limitado a **0,02 m/s** por até 2 s. A guarda encaminhou movimento por
  **1,784 s**; roda observada em **0,017–0,018 m/s**, seguida de zero.
  Janela de 35,09 s em `validation/point_lio/motion.json`: Point-LIO avançou
  **+30,2 mm no eixo X** entre primeira e última amostra; máximo afastamento
  da primeira pose **47,0 mm**. Escala e direção coerentes com um avanço
  físico de poucos centímetros. Sem NaN/Inf, regressão de timestamps, crash
  ou explosão de trajetória. Não houve medição externa de distância.
  Os `6,5 Hz` observados pelo medidor completo durante esse ensaio refletiram
  sobrecarga do próprio processo Python junto ao RViz e à guarda. Com o
  contador bruto e leve, ainda com RViz/base/guarda ativos, obtivemos:
  LiDAR **9,9998 Hz**, IMU **200,0226 Hz**, odometria/nuvem/path
  **10,256 Hz** (`validation/point_lio/rates_after_motion.json`).
- RViz via X11 e renderização por software abriu em OpenGL 4.5, status
  global **OK**. As nuvens registradas exibiram paredes e objetos estáveis
  antes e depois do avanço; Path e TF estavam ativos. Não foi usado PCD
  persistente. O TF real é `camera_init -> aft_mapped`; a mensagem Odometry
  indica child `body`. Uma janela posterior da guarda travou por um atraso
  isolado de nuvem >0,35 s, enviando zero; ocorreu **após** o pulso e a base
  já estava parada. Isso evidencia que a guarda é conservadora sob carga.
- Base e guarda foram encerradas após o teste. O `limo_base` antigo imprime
  `Aborted` ao receber SIGINT no encerramento, já sem movimento; não houve
  crash do Point-LIO. O container principal de sensor, Point-LIO e RViz pode
  permanecer ativo para inspeção e pode ser parado com
  `docker stop -t 15 limo-point-lio`.

Para repetir o teste com guarda e base remapeada, inicie o container, driver e
Point-LIO como acima. Depois, em terminais distintos:

```bash
docker exec -it limo-point-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; ros2 run limo_base limo_base --ros-args -p port_name:=ttyTHS0 -p pub_odom_tf:=false -r /cmd_vel:=/point_lio_test/cmd_vel -r /odom:=/point_lio_test/wheel_odom -r /limo_status:=/point_lio_test/status -r /imu:=/point_lio_test/base_imu'
docker exec -it limo-point-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; python3 /validation/motion_guard.py'
docker exec -it limo-point-lio bash -lc 'source /workspace/ros2_ws/install/setup.bash; python3 /validation/motion_pulse.py'
```

Só inicie o pulso depois de observar `ready: true` na guarda e uma área livre
à frente. O container monta `validation/point_lio` em `/validation`.
