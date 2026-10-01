# LIMO Pro Real Localization

Ambiente independente em Docker para execução do **AgileX LIMO Pro** com:

- ROS 2 Humble
- controle da base do LIMO Pro
- teleop por teclado
- Livox MID-360
- FAST-LIO2
- RViz2

O sistema foi validado fisicamente no LIMO Pro utilizando plataforma ARM64.

---

## Arquitetura

```text
Docker ROS 2 Humble
│
├── teleop_twist_keyboard
│        ↓
│     /cmd_vel
│        ↓
├── limo_base
│        ↓
│   /dev/ttyTHS0
│        ↓
│     LIMO Pro
│
├── livox_ros_driver2
│        ↓
│   /livox/lidar
│   /livox/imu
│
└── FAST-LIO2
         ↓
     /Odometry
     /path
     /cloud_registered
         ↓
       RViz2
```

---

# Hardware utilizado

- AgileX LIMO Pro
- NVIDIA Jetson integrada ao LIMO
- Livox MID-360
- conexão Ethernet entre Jetson e Livox MID-360

---

# Estrutura do projeto

```text
limo_real_localization/
├── docker/
│   └── Dockerfile
├── Livox-SDK2/
└── ros2_ws/
    └── src/
        ├── fast_lio2/
        ├── livox_ros_driver2/
        ├── limo_base/
        └── limo_msgs/
```

---

# 1. Clonar o repositório

```bash
git clone https://github.com/DaymonnLevir/limo_real_localization.git
```

Entre na pasta:

```bash
cd limo_real_localization
```

A versão completa validada no LIMO pode ser consultada pela tag:

```text
v0.2.0-limo-full-system
```

Para utilizar especificamente essa versão:

```bash
git checkout v0.2.0-limo-full-system
```

---

# 2. Configurar a rede do Livox MID-360

A configuração utilizada durante os testes foi:

```text
LIMO / Jetson:
192.168.1.5/24
192.168.1.50/24

Livox MID-360:
192.168.1.179
```

O Livox está configurado para enviar os dados para:

```text
192.168.1.50
```

Verifique os endereços configurados na interface Ethernet:

```bash
ip -br addr show eth0
```

O esperado é algo semelhante a:

```text
eth0 UP 192.168.1.5/24 192.168.1.50/24
```

Caso o endereço `192.168.1.50` não esteja presente:

```bash
sudo ip addr add 192.168.1.50/24 dev eth0
```

Teste a comunicação com o Livox:

```bash
ping -c 3 192.168.1.179
```

Deve haver resposta do sensor sem perda de pacotes.

---

# 3. Construir a imagem Docker

Na raiz do repositório:

```bash
docker build -t limo-real-fastlio2:humble -f docker/Dockerfile .
```

O build compila:

- Livox-SDK2
- livox_ros_driver2
- limo_msgs
- limo_base
- FAST-LIO2

A imagem criada será:

```text
limo-real-fastlio2:humble
```

---

# 4. Abrir o container

Antes de abrir o container, libere o X11 para que o RViz possa ser exibido:

```bash
xhost +local:docker
```

Execute:

```bash
docker run --rm -it \
  --name limo_independent_test \
  --network host \
  --device=/dev/ttyTHS0:/dev/ttyTHS0 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  limo-real-fastlio2:humble \
  bash
```

O container utiliza:

```text
--network host
```

para acessar diretamente a rede utilizada pelo Livox.

Também é disponibilizada a UART:

```text
/dev/ttyTHS0
```

utilizada para comunicação com a base do LIMO.

O ambiente ROS 2 Humble e o workspace são carregados automaticamente pelo `.bashrc` do container.

---

# 5. Lançar a base do LIMO

No primeiro terminal do container:

```bash
ros2 launch limo_base limo_base.launch.py
```

A configuração validada utiliza:

```text
port_name    = ttyTHS0
odom_frame   = odom
base_frame   = base_link
pub_odom_tf  = True
control_rate = 50
use_mcnamu   = False
```

A base recebe comandos através de:

```text
/cmd_vel
```

e publica, entre outros:

```text
/odom
/limo_status
/imu
```

---

# 6. Lançar o teleop

Abra um novo terminal no LIMO.

Entre no container já em execução:

```bash
docker exec -it limo_independent_test bash
```

Execute:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

As principais teclas são:

```text
        i
   j    k    l
```

O teleop publica mensagens em:

```text
/cmd_vel
```

O `limo_base` recebe essas mensagens e envia os comandos para o chassi através da UART:

```text
/dev/ttyTHS0
```

---

# 7. Lançar o Livox MID-360

Abra outro terminal:

```bash
docker exec -it limo_independent_test bash
```

Execute o driver:

```bash
ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p user_config_path:=/workspace/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
```

Quando a comunicação estiver funcionando deverão aparecer mensagens semelhantes a:

```text
successfully change work mode
successfully enable Livox Lidar imu
livox/imu publish use imu format
livox/lidar publish use PointCloud2 format
```

Verifique os tópicos:

```bash
ros2 topic list | grep livox
```

Esperado:

```text
/livox/imu
/livox/lidar
```

---

# 8. Lançar FAST-LIO2 + RViz

Abra outro terminal:

```bash
docker exec -it limo_independent_test bash
```

Execute:

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=limo_real.yaml \
  use_sim_time:=false \
  rviz:=true
```

O FAST-LIO2 utiliza:

```text
/livox/lidar
/livox/imu
```

e produz, entre outros:

```text
/Odometry
/path
/cloud_registered
/cloud_registered_body
/cloud_effected
```

O RViz2 é iniciado automaticamente pelo launch.

---

# 9. Verificar se o FAST-LIO2 está funcionando

Em outro terminal:

```bash
docker exec -it limo_independent_test bash
```

Liste os tópicos:

```bash
ros2 topic list | grep -E "Odometry|path|cloud|registered"
```

Esperado:

```text
/Odometry
/cloud_effected
/cloud_registered
/cloud_registered_body
/path
```

Verifique a frequência da odometria:

```bash
ros2 topic hz /Odometry
```

Durante os testes com o LIMO real, a publicação ficou aproximadamente em:

```text
9 - 10 Hz
```

---

# 10. Ordem recomendada de inicialização

A sequência validada é:

```text
1. Ligar LIMO Pro
2. Ligar Livox MID-360
3. Configurar 192.168.1.50 na eth0, caso necessário
4. Abrir container Docker
5. Iniciar limo_base
6. Iniciar teleop
7. Iniciar livox_ros_driver2
8. Iniciar FAST-LIO2 + RViz
9. Movimentar o robô pelo teleop
```

---

# Resumo dos terminais

## Terminal 1 - container principal + LIMO base

```bash
xhost +local:docker

docker run --rm -it \
  --name limo_independent_test \
  --network host \
  --device=/dev/ttyTHS0:/dev/ttyTHS0 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  limo-real-fastlio2:humble \
  bash
```

Depois:

```bash
ros2 launch limo_base limo_base.launch.py
```

---

## Terminal 2 - Teleop

```bash
docker exec -it limo_independent_test bash
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Terminal 3 - Livox MID-360

```bash
docker exec -it limo_independent_test bash
```

```bash
ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p user_config_path:=/workspace/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
```

---

## Terminal 4 - FAST-LIO2 + RViz

```bash
docker exec -it limo_independent_test bash
```

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=limo_real.yaml \
  use_sim_time:=false \
  rviz:=true
```

---

# ROS 2 Foxy -> ROS 2 Humble

O pacote `limo_base` disponível originalmente no robô era utilizado em ROS 2 Foxy.

Para tornar o projeto independente dos workspaces já existentes no LIMO, uma cópia do pacote foi integrada ao workspace deste projeto e adaptada para ROS 2 Humble.

Foram realizadas apenas alterações de compatibilidade, incluindo:

- declaração explícita dos tipos utilizados por `declare_parameter`;
- valores padrão dos parâmetros;
- correção de parâmetros booleanos;
- inclusão de `tf2_geometry_msgs`;
- inclusão das dependências necessárias no Docker.

A lógica utilizada para comunicação serial com o LIMO foi preservada.

---

# Sistema validado

O seguinte conjunto foi validado fisicamente:

- [x] Docker ROS 2 Humble
- [x] ARM64 / NVIDIA Jetson
- [x] `/dev/ttyTHS0`
- [x] `limo_base`
- [x] `/cmd_vel`
- [x] teleop por teclado
- [x] movimentação física do LIMO Pro
- [x] Livox MID-360
- [x] `/livox/lidar`
- [x] `/livox/imu`
- [x] FAST-LIO2
- [x] `/Odometry`
- [x] `/path`
- [x] `/cloud_registered`
- [x] RViz2
- [x] LIMO movimentando durante a execução do FAST-LIO2

---

# Independência dos workspaces originais

A execução deste projeto não depende dos outros workspaces ROS presentes no LIMO.

Não é necessário executar:

```text
~/carolina_ros2_ws
~/limo_base_ws
~/limo_ros2_ws
```

A base do LIMO, as mensagens, o driver do Livox e o FAST-LIO2 são compilados e executados dentro do próprio ambiente Docker ROS 2 Humble deste projeto.

---

# Repositório

GitHub:

https://github.com/DaymonnLevir/limo_real_localization

Versão completa validada:

```text
v0.2.0-limo-full-system
```
