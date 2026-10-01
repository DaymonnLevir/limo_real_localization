# LIMO Pro Real Localization

Ambiente independente em Docker para execução do AgileX LIMO Pro com:

- ROS 2 Humble
- controle da base do LIMO Pro
- teleop por teclado
- Livox MID-360
- FAST-LIO2
- RViz2

O sistema foi validado fisicamente no LIMO Pro utilizando a plataforma ARM64.

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
