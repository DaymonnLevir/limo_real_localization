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

## Resultados desta execução

Pendente de concluir compilação e ensaios físicos.
