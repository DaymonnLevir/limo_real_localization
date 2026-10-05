# Proveniência do CT-LIO no LIMO

- Algoritmo original: https://github.com/chengwei0427/ct-lio
- Port ROS 2 selecionado: https://github.com/inkccc/ct_lio_ros2
- Branch: `main`
- Commit: `bdc683e9fe74de8fc7501086f7ae105d96bc5db1`
- Data de obtenção: 2026-10-05

O GitHub identifica o projeto ROS 2 como fork do repositório original. O README
e o código mantêm CT-ICP contínuo, ESKF e mapa voxel. O port traz mudanças de
engenharia próprias: fila leve, descarte LRU e sincronização de IMU. Não é um
espelho byte a byte do repositório ROS 1.

Alterações locais: pré-processador rejeita nuvem sem tempos por ponto e
identifica o campo Livox `timestamp` FLOAT64 como nanossegundos absolutos;
usa a diferença em nanossegundos antes de converter para segundos relativos.
`test/timestamp_smoke.cpp` cobre o layout real de 26 bytes, intervalo de 0,1 s
e rejeição quando falta o campo. O algoritmo de odometria/otimização permanece
intacto. Perfil, launch e RViz LIMO foram adicionados sem modificar o genérico.
O CMake do port foi ajustado para ligar explicitamente `fmt::fmt`, já presente
na imagem Humble, resolvendo símbolo faltante no link ARM64.
