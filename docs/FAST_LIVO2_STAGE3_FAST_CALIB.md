# Stage 3A — FAST-Calib original, ROS1 Noetic / ARM64

## Fontes fixadas e ambiente

Fontes clonadas separadamente dentro da imagem, sem alteração do FAST-Calib:

- [hku-mars/FAST-Calib](https://github.com/hku-mars/FAST-Calib): `1018ecfdf9deda51b91a8a11bd11972a0b159008`, `/opt/fast_calib_ws/src/fast_calib`.
- [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2): `21445540f0d100dc86a7e6df312dd70bbdb4afdf`, modo `ROS_EDITION=ROS1`, `package_ROS1.xml`.
- [Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2): `c0796f04c143143899c87a773d9f6b7136453c0b`.

Ubuntu 20.04, ROS Noetic; PCL 1.10, OpenCV 4.2 com ArUco, Eigen, catkin, pcl_ros/pcl_conversions, cv_bridge/image_geometry e rosbag. O manifesto original cita o driver Livox antigo, mas o código contém seus headers e lê também PointCloud2; a compilação não exige instalar aquele driver. Não executar rosdep indiscriminadamente sobre esse manifesto.

```bash
cd /home/agilex/levir_limo_real/FAST_LIVO2_2026-10-04
bash calibration/fast_calib/scripts/build_ros1_calib.sh
bash calibration/fast_calib/scripts/run_ros1_calib_container.sh
```

Build concluído no LIMO ARM64 em 2026-10-05, sem patches no FAST-Calib. Imagem `limo-fast-calib-ros1:noetic`, ID `sha256:2de7c80207f74b8d431fe29dbc71ce92f31954fb2ea91ae3f40868f660e37a41`. Smoke test aprovado: Ubuntu 20.04/Noetic, executáveis single/multi e Livox ROS1, bibliotecas resolvidas, OpenCV 4.2.0/ArUco, v4l-utils 1.18, scripts e master ROS1. Teste com rede isolada, sem dispositivos; nenhum dado físico ou extrínseca foi produzido. Logs da preparação: `/tmp/stage3-build.log` e `/tmp/stage3-smoke.log` no LIMO.

 Container homônimo, rede host, somente `/dev/video0`, bind `calibration/fast_calib` → `/calibration`; UID/GID do usuário. Master exclusivo `http://127.0.0.1:11321`. Não utiliza o ROS do host nem altera `.bashrc`.

## Alvo exato

Especificação cruzada entre `pics/calibration_target.jpg`, `config/qr_params.yaml` e `src/qr_detect.hpp` do SHA fixado:

- Placa plana **1400 × 1000 mm**.
- ArUco **DICT_6X6_250**, IDs vistos de frente: **1 superior esquerdo, 2 superior direito, 3 inferior esquerdo, 4 inferior direito**. Orientação como no desenho oficial.
- Marcadores **200 × 200 mm**, bordas externas a **50 mm** das bordas adjacentes da placa. Centros separados **1100 mm horizontalmente × 700 mm verticalmente**.
- Quatro **furos circulares físicos**, diâmetro **240 mm** (raio 120 mm). Centros em retângulo **500 × 400 mm**, centrado na placa: x=±250 mm, y=±200 mm. Centros inferiores a 300 mm da base; centros laterais a 450 mm da borda correspondente.
- Parâmetros em metros: `marker_size=0.20`, `delta_width_qr_center=0.55`, `delta_height_qr_center=0.35` (**meias distâncias**); `delta_width_circles=0.50`, `delta_height_circles=0.40` (**distâncias completas**); `circle_radius=0.12`. Mínimo 3 marcadores detectados, preferir todos os 4.
- Geometria configurável nesses seis parâmetros. Dicionário/IDs estão fixos no código original. A nota de marcador de 0.16 m refere-se aos dados de exemplo, não ao desenho de 200 mm adotado aqui. Se fabricar outro tamanho, medir e ajustar os parâmetros antes de usar.
- [Desenho oficial](https://github.com/hku-mars/FAST-Calib/blob/1018ecfdf9deda51b91a8a11bd11972a0b159008/pics/calibration_target.jpg). O README fornece [CAD externo](https://drive.google.com/file/d/1hdC8xGCHNP47a-wSLPyjr_tpOeynNFEG/view?usp=sharing); o arquivo externo não foi baixado/verificado. Não há PDF/CAD/gerador versionado no repositório inspecionado. Não imprimir o JPEG presumindo escala física.

## MID-360 e imagem

O host deve possuir `192.168.1.50`; sensor `192.168.1.179`. Na preparação, `eth0` estava em `192.168.1.5/24`. Antes da aquisição, adicionar o IP secundário abaixo (temporário até reiniciar), preservando o endereço existente:

```bash
sudo ip address add 192.168.1.50/24 dev eth0
```

 Configuração independente em `config/MID360_config.json`. Extrínsecos do driver iguais a zero mantêm os pontos no frame LiDAR; não são a extrínseca LiDAR–IMU validada do FAST-LIVO2.

Em terminal dedicado (deixar executando):

```bash
docker exec -it limo-fast-calib-ros1 /stage3_entrypoint.sh bash /calibration/scripts/start_mid360_ros1.sh
```

`xfer_format=0`, `multi_topic=0`: ROS1 `sensor_msgs/PointCloud2`, `/livox/lidar`, 10 Hz. O FAST-Calib oficial aceita x/y/z, e usa a ausência do campo `ring` para selecionar LiDAR sólido. O formato CustomMsg do Driver 2 não é usado.

Captura RGB avulsa, opcional, em outro terminal:

```bash
docker exec -it limo-fast-calib-ros1 /stage3_entrypoint.sh python3 /calibration/scripts/capture_rgb.py /calibration/datasets/preview.png
```

PNG RAW BGR/OpenCV, 640×480, sem retificação/redimensionamento; aquecimento de 30 frames e rejeição de resolução diferente. Usa K original fx=494.55179, fy=493.91134, cx=314.61947, cy=221.95045 e distorção [0.095855,-0.122538,0.003246,0.002270,0]. FAST-Calib fixa k3=0. O YAML validado existente permanece intacto.

## Três cenas estáticas nativas ROS1

Com robô e sensores imóveis, placa totalmente visível por câmera e LiDAR, adquirir uma cena por vez. Reposicionar a placa entre cenas para posições/orientações diferentes, mantendo a geometria rígida câmera–LiDAR. Não movimentar nada durante cada captura. Conferir foco/exposição e visibilidade dos quatro furos.

```bash
docker exec -it limo-fast-calib-ros1 /stage3_entrypoint.sh bash /calibration/scripts/capture_scene.sh scene_01
# Reposicione o alvo e espere estabilizar.
docker exec -it limo-fast-calib-ros1 /stage3_entrypoint.sh bash /calibration/scripts/capture_scene.sh scene_02
# Reposicione o alvo e espere estabilizar.
docker exec -it limo-fast-calib-ros1 /stage3_entrypoint.sh bash /calibration/scripts/capture_scene.sh scene_03
```

Cada diretório `datasets/scene_0N/` recebe `lidar.bag` ROS1 (8 segundos de `/livox/lidar`), `image.png` capturada durante a gravação e marcador `COMPLETE` somente após validação do tipo/tópico/contagem. Sem conversão de bags ROS2. Scripts recusam sobrescrever cenas; uma captura interrompida deve ser inspecionada e movida manualmente antes de tentar novamente.

## Calibração posterior — não executada na preparação

Medir a ROI x/y/z **no frame LiDAR, em metros**, de modo a isolar somente a placa. Em cada cena, preencher os seis valores reais solicitados pelo comando abaixo; não há ROI presumida:

```bash
for scene in scene_01 scene_02 scene_03; do
  read -r -p "$scene: XMIN XMAX YMIN YMAX ZMIN ZMAX (m): " xmin xmax ymin ymax zmin zmax
  docker exec -it limo-fast-calib-ros1 /stage3_entrypoint.sh bash /calibration/scripts/run_fast_calib.sh single "$scene" --roi "$xmin" "$xmax" "$ymin" "$ymax" "$zmin" "$zmax"
done
```

O executável oficial permanece publicando depois de calcular; o wrapper envia SIGINT somente ao seu próprio processo após os quatro arquivos de saída serem gravados (limite de 600 s). Inspecionar `results/scene_0N/`: `single_calib_result.txt`, `circle_center_record.txt`, `qr_detect.png`, `colored_cloud.pcd`, `params.yaml`. Confirmar quatro pares corretos e boa projeção em todas as cenas; arquivos existentes não são sobrescritos pelo wrapper.

Após essa inspeção:

```bash
docker exec -it limo-fast-calib-ros1 /stage3_entrypoint.sh bash /calibration/scripts/run_fast_calib.sh multi
```

Combina os três registros individuais em `results/multi/circle_center_record.txt` e executa o `multi_fast_calib` original, que usa os últimos três blocos (12 pares). Resultado esperado: `results/multi/multi_calib_result.txt`, matriz **T_cam_lidar**, ou seja **p_camera = Rcl × p_lidar + Pcl**, translação em metros. Já é a convenção esperada; não inverter a matriz. A aplicação no FAST-LIVO2 exige validação física posterior; nenhum Rcl/Pcl foi gerado ou aplicado nesta etapa.

## Pendências e limites

A configuração do endereço secundário 192.168.1.50, fabricação/medição do alvo, captura real, ROI por cena e validação da extrínseca ficam para a etapa física. Smoke tests desta preparação não abrem sensores nem calculam calibração. Se um driver ROS2 ocupar portas Livox, pare esse driver manualmente antes de iniciar ROS1; os scripts recusam o conflito, sem encerrar processos. A câmera também deve estar livre. Não alterar a extrínseca LiDAR–IMU validada, habilitar img_en ou iniciar movimento do robô.
