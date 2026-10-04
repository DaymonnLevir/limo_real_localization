"""
config.py
---------
Parâmetros globais do sistema tmplanner_trees.
"""

import os

# ---------------------------------------------------------------------------
# GP de Exploração (Matérn 3/2 + Filtro de Kalman — Popovic et al., 2017)
# ---------------------------------------------------------------------------

# Comprimento de escala ℓ do kernel Matérn 3/2 [m].
# Controla o raio de influência espacial de cada observação.
# Fit por máxima verossimilhança sobre observations.csv de runs reais
# (forest_clustered_10min, 2026-07) indica ótimo em ℓ ≈ 4–6 m; valores
# grandes (≥13) são fortemente rejeitados pelos dados e causam
# overconfidence em regiões nunca observadas. Manter ≤ 8.
GP_ELL = 8.0

# Desvio padrão do sinal sf (amplitude da covariância) [contagem de árvores].
GP_SIGMA_S = 3.0

# Desvio padrão do ruído de observação σ_n [contagem de árvores].
# Representa sub-contagens por oclusão parcial. Também adicionado à
# diagonal do prior (igual a Popovic et al.).
GP_SIGMA_N = 0.5

# Coeficiente de exploração UCB: β > 0 favorece regiões ainda não visitadas.
# Pode ser sobrescrito por PARÂMETRO via IPP_GP_BETA (usado pelo runner de
# benchmark na variante exploit: proposed_exploit -> 1.5).
GP_BETA = float(os.environ.get('IPP_GP_BETA') or 5.0)
if GP_BETA < 0.0:
    raise ValueError(f'IPP_GP_BETA invalido: {GP_BETA}. Use >= 0.')

# Modelo de observação H usado no update de Kalman do GP e nos fantasy updates:
#   'H_naive'   -> legado: H=e_i, só a célula mais próxima é observada.
#   'footprint' -> H radial esparso, aproximando a área sensoriada pelo LiDAR.
GP_OBSERVATION_MODEL = 'footprint'

# Alcance efetivo do tree-mapper para o H de footprint [m]. O max_range=20 m
# do Octomap é limite de inserção de pontos, não alcance prático de detecção
# de troncos. O valor atual é conservador para não superestimar regiões vistas.
# Nota: como o espaçamento efetivo do grid é ~3.02 m (> R), uma medição exata
# sobre um nó do grid degenera H em one-hot; R>=4.5 corrigiria, mas mudaria a
# comparabilidade com toda a série de benchmarks (todas as runs usam 3.0).
GP_OBSERVATION_FOOTPRINT_RADIUS = 3.0

# Escala radial dos pesos do footprint [m]. Use ~R/2 para favorecer células
# próximas sem zerar a vizinhança observada.
GP_OBSERVATION_FOOTPRINT_SIGMA = 1.75

# ---------------------------------------------------------------------------
# Parâmetros de Planejamento
# ---------------------------------------------------------------------------

# Estratégia ativa: 'greedy_selection' ou 'rrt_informative'.
# Pode ser trocada por PARÂMETRO, sem editar este arquivo nem o código do
# planejador, via a variável de ambiente IPP_PLANEJAMENTO (usada pelo runner de
# benchmark: proposed -> rrt_informative, myopic -> greedy_selection).
# O default mantém o comportamento atual, então o pipeline existente não muda.
PLANEJAMENTO = os.environ.get('IPP_PLANEJAMENTO') or 'rrt_informative'
if PLANEJAMENTO not in ('rrt_informative', 'greedy_selection'):
    raise ValueError(
        f"IPP_PLANEJAMENTO invalido: {PLANEJAMENTO!r}. "
        "Use 'rrt_informative' (proposed) ou 'greedy_selection' (myopic)."
    )

# Velocidade de cruzeiro do robô [m/s]. Só entra no timeout do watchdog.
# LIMO + Nav2 anda ~0.5 m/s (o drone MRS usava 2.0). IPP_REFERENCE_SPEED.
REFERENCE_SPEED = float(os.environ.get('IPP_REFERENCE_SPEED') or 0.5)
if REFERENCE_SPEED <= 0.0:
    raise ValueError(f'IPP_REFERENCE_SPEED invalido: {REFERENCE_SPEED}.')

# Frequência de escaneamento LiDAR [Hz].
# No stub atual, ela entra como custo mínimo de uma medição.
MEASUREMENT_FREQ = 0.5

# Orçamento total de tempo de missão [s].
TIME_BUDGET = 100.0

# Limiar de incerteza total para encerrar a missão.
CONVERGENCE_THRESHOLD = 0.1

# Limiar de incerteza individual por árvore [m].
MAX_TREE_STD_THRESHOLD = 0.08

# Mínimo de setores angulares distintos por árvore antes de aceitar convergência.
MIN_TREE_VIEW_SECTORS = 2

# Coordenada z dos waypoints [m]. Robô terrestre (Nav2): 0.0, para que as
# distâncias 3-D do planner não fiquem infladas. Drone MRS: 2.0.
# Sobrescritível via IPP_FLIGHT_HEIGHT.
FLIGHT_HEIGHT = float(os.environ.get('IPP_FLIGHT_HEIGHT') or 0.0)

# Referencial comum para waypoints, mapa de árvores e posições de qualidade.
# Tem que ser o MESMO frame usado pelo tree_mapper (target_frame) e pelo
# tree_measuring (MAP_FRAME); o feeder descarta /tree_map_full em outro frame.
# Nav2: 'map'. MRS: 'uav1/ground_truth_origin'. IPP_PLANNING_FRAME.
PLANNING_FRAME = os.environ.get('IPP_PLANNING_FRAME') or 'map'

# Resolução da grade de candidatos de waypoints [m]. Sobrescritível por
# PARÂMETRO via IPP_WAYPOINT_GRID_RESOLUTION (ablação de resolução: *_r05 ->
# 0.5 m => M~4225 células; medido offline: prior 0.6 s, update 127 ms/obs,
# replan RRT ~0.6 s — viável).
WAYPOINT_GRID_RESOLUTION = float(
    os.environ.get('IPP_WAYPOINT_GRID_RESOLUTION') or 1.0
)
if WAYPOINT_GRID_RESOLUTION <= 0.0:
    raise ValueError(
        f'IPP_WAYPOINT_GRID_RESOLUTION invalido: {WAYPOINT_GRID_RESOLUTION}.'
    )

# Distância máxima entre waypoints consecutivos [m].
MAX_WAYPOINT_STEP = 5.0

# Parâmetros do RRT informativo.
RRT_N_ITER = 300
RRT_STEP_SIZE = 5.0
# Horizonte do RRT (profundidade de nós fantasiados por caminho). Pode ser
# sobrescrito por PARÂMETRO via IPP_RRT_HORIZON, sem editar este arquivo —
# usado pelo runner de benchmark na ablação de miopia (proposed_h1 -> 1).
# O default preserva o comportamento atual.
RRT_HORIZON = int(os.environ.get('IPP_RRT_HORIZON') or 8)
if RRT_HORIZON < 1:
    raise ValueError(f'IPP_RRT_HORIZON invalido: {RRT_HORIZON}. Use >= 1.')
RRT_GOAL_BIAS = 0.2
RRT_COST_WEIGHT = 0.3

# Critério de avaliação dos caminhos do RRT (ver rrt.py e fantasy_gp.py):
#   'hybrid'    -> combina UCB acumulado e ganho de informação fantasiado.
#                  Mantém preferência por regiões promissoras (média + beta
#                  * incerteza) sem recompensar tanto revisitas.
#   'info_gain' -> ganho de informação com fantasy updates do Kalman:
#                  score = Tr(P0) − Tr(P_caminho). Submodular: revisitar
#                  células já medidas (na missão ou no próprio caminho)
#                  vale ~0, eliminando loops e acampamento.
#   'ucb_sum'   -> legado: soma de UCB estático por nó, sem desconto por
#                  re-observação. Mantido para comparação em benchmarks.
RRT_SCORE_MODE = 'hybrid'

# Pesos do modo híbrido. Os dois termos são normalizados por replan antes da
# soma, então 0.4/0.6 significa "um pouco mais info gain que UCB", não escala
# absoluta. A penalização por custo continua em RRT_COST_WEIGHT.
RRT_HYBRID_UCB_WEIGHT = 0.4
# Peso do termo de info gain (redução A-óptima de traço). Sobrescritível por env
# (IPP_RRT_HYBRID_INFO_GAIN_WEIGHT=0 remove o A-optimal gain, deixando só o UCB).
RRT_HYBRID_INFO_GAIN_WEIGHT = float(os.environ.get('IPP_RRT_HYBRID_INFO_GAIN_WEIGHT') or 0.6)

# Normalização do termo de info gain no score híbrido. Sobrescritível por
# PARÂMETRO via IPP_RRT_INFO_GAIN_NORM (variante exploit):
#   'minmax'   -> legado: renormaliza por replan; o peso relativo do ganho é
#                 constante ao longo da missão.
#   'absolute' -> divide o ganho acumulado por uma referência FIXA derivada do
#                 prior (ver planner.py). Conforme a incerteza global cai, o
#                 termo decai em escala natural e o UCB domina a decisão
#                 progressivamente.
RRT_INFO_GAIN_NORMALIZATION = (
    os.environ.get('IPP_RRT_INFO_GAIN_NORM') or 'minmax'
)
if RRT_INFO_GAIN_NORMALIZATION not in ('minmax', 'absolute'):
    raise ValueError(
        f'IPP_RRT_INFO_GAIN_NORM invalido: {RRT_INFO_GAIN_NORMALIZATION!r}. '
        "Use 'minmax' ou 'absolute'."
    )

# Desconto temporal dentro do horizonte do RRT. Valores <1 reduzem o peso
# de ganhos muito distantes, evitando que uma leaf longa atravesse o mapa e
# ganhe de uma região próxima e ainda incerta.
RRT_FUTURE_DISCOUNT = 0.90

# Termos explícitos de histórico visitado. O histórico usa o mesmo H do GP:
# H_naive penaliza só a célula mais próxima; footprint penaliza a área
# sensoriada pelo LiDAR/tree-mapper.
# Sobrescritíveis por PARÂMETRO (ablação da memória de visitas, ex.:
# proposed_exploit_novisit -> ambos 0).
RRT_VISIT_NOVELTY_WEIGHT = float(
    os.environ.get('IPP_RRT_VISIT_NOVELTY_WEIGHT') or 0.35
)
RRT_VISIT_PENALTY_WEIGHT = float(
    os.environ.get('IPP_RRT_VISIT_PENALTY_WEIGHT') or 0.45
)
if RRT_VISIT_NOVELTY_WEIGHT < 0.0 or RRT_VISIT_PENALTY_WEIGHT < 0.0:
    raise ValueError('Pesos de visita devem ser >= 0.')

# Escala em "massa de visitas" para saturar novidade/penalidade. Como cada
# observação adiciona soma(H)=1 na grade, valores perto de 1 fazem uma célula
# já coberta uma vez perder bastante atratividade; valores maiores são mais
# permissivos.
RRT_VISIT_SATURATION = 1.0

# Decaimento temporal do histórico global de visitas, em número de observações.
# Isso evita transformar regiões antigas em "proibidas" para sempre. O loop
# dentro do caminho fantasiado da RRT continua sendo penalizado sem decay.
# Use <=0 para desabilitar o decay e manter memória permanente.
RRT_VISIT_DECAY_HALFLIFE_OBS = 450.0

# Inicia o cálculo do próximo caminho durante a aproximação ao goal atual.
# Use 0.0 para desabilitar o pré-planejamento.
RRT_PREPLAN_DISTANCE = 1.0

# Máxima mudança normalizada permitida no mapa GP entre calcular um pré-plano
# e consumi-lo após chegar ao goal atual. A métrica é o maior RMS normalizado
# entre média, desvio padrão e UCB na grade de candidatos. Assim, uma árvore
# nova no tree_mapper não invalida sozinha a intenção do RRT; o pré-plano só é
# descartado se o mapa de decisão realmente mudou. Use <=0 para desabilitar
# essa invalidação por mudança de GP.
RRT_PREPLAN_GP_CHANGE_THRESHOLD = 0.20

# Quantos waypoints do caminho escolhido pelo RRT devem ser seguidos antes
# de replanejar. 1 preserva o comportamento legado: envia só o primeiro
# waypoint e replana após chegar. Valores >1 reduzem oscilação entre replans,
# seguindo a intenção de curto prazo da leaf escolhida.
# Sobrescritível por PARÂMETRO via IPP_RRT_COMMITTED_WAYPOINTS (ablação de
# commitment: proposed_c1 -> 1 = avalia com H cheio, executa só o 1º waypoint
# e replaneja — o comportamento legado).
RRT_COMMITTED_WAYPOINTS = int(os.environ.get('IPP_RRT_COMMITTED_WAYPOINTS') or 3)
if RRT_COMMITTED_WAYPOINTS < 1:
    raise ValueError(
        f'IPP_RRT_COMMITTED_WAYPOINTS invalido: {RRT_COMMITTED_WAYPOINTS}.'
    )

# O goal só é considerado concluído quando a odometria está dentro desta
# distância 3D e o Octomap Planner reporta estado idle.
WAYPOINT_ARRIVAL_TOLERANCE = 0.75

# Watchdog do executor de waypoints. O timeout efetivo é dinâmico e só é
# usado quando a odometria não mostra movimento suficiente:
# max(GOAL_TIMEOUT_MIN_SEC,
#     GOAL_TIMEOUT_BUFFER_SEC + fator * distancia / REFERENCE_SPEED),
# limitado por GOAL_TIMEOUT_MAX_SEC. Isso evita matar goals longos enquanto o
# drone ainda se move, mas ainda protege contra goals realmente travados.
GOAL_TIMEOUT_MIN_SEC = 25.0
GOAL_TIMEOUT_MAX_SEC = 45.0
GOAL_TIMEOUT_BUFFER_SEC = 8.0
GOAL_TIMEOUT_SPEED_FACTOR = 3.0

# O watchdog só corta se a odometria não deslocar pelo menos este valor
# durante o timeout dinâmico acima. Se o drone ainda se move, não corta.
GOAL_WATCHDOG_MOTION_EPS = 0.20

# Visualização ao vivo para depuração. Quando habilitada, o planejamento
# fica propositalmente mais lento por redesenhar a árvore durante a expansão.
RRT_DEBUG_PLOT = False
RRT_DEBUG_PLOT_EVERY = 20
RRT_DEBUG_PLOT_PAUSE_SEC = 0.001

# Resolução do mapa contínuo de ganho de informação [m].
INFO_GAIN_MAP_RESOLUTION = 0.5

# Fonte do mapa exibido pelo nó gaussian_feeder.
# Opções:
#   "kalman"     -> média posterior discreta nos waypoints candidatos.
#   "gp_predict" -> média posterior avaliada em uma malha contínua/densa.
GAUSSIAN_FEEDER_MAP_SOURCE = 'kalman'


# Parâmetros de simulação.

# x_min, x_max, y_min, y_max [m] no PLANNING_FRAME. Ajuste para a área real
# de teste via IPP_MAP_BOUNDS="x_min,x_max,y_min,y_max" (ou pelo parâmetro
# ROS map_bounds do gaussian_feeder).
_MAP_BOUNDS_ENV = os.environ.get('IPP_MAP_BOUNDS')
MAP_BOUNDS = (
    tuple(float(v) for v in _MAP_BOUNDS_ENV.split(','))
    if _MAP_BOUNDS_ENV else (-22.753, 22.696, -22.720, 22.588)
)
if len(MAP_BOUNDS) != 4:
    raise ValueError(f'IPP_MAP_BOUNDS invalido: {_MAP_BOUNDS_ENV!r}.')

# ---------------------------------------------------------------------------
# Backend de navegação
# ---------------------------------------------------------------------------

# Quem executa os waypoints: 'nav2' (NavigateToPose) ou 'mrs' (octomap_planner
# do MRS, exige mrs_msgs). Sobrescritível pelo parâmetro ROS
# controller_backend ou por IPP_CONTROLLER_BACKEND.
CONTROLLER_BACKEND = os.environ.get('IPP_CONTROLLER_BACKEND') or 'nav2'

# Quando o Nav2 aborta um goal (sem caminho / recovery esgotado), a posição
# atual é aceita como chegada se estiver a até esta distância do goal —
# equivalente ao "drone travado a ~1 m do tronco" do backend MRS. Mais longe
# que isso, o goal conta como não concluído e o plano commitado é descartado.
NAV2_ABORT_ACCEPT_DISTANCE = 1.5  # [m]

# Fim de missão: o benchmark_planner (dono do time budget) publica aqui
# (std_msgs/String, transient_local) ao encerrar; o gaussian_feeder para de
# planejar, cancela o goal ativo e salva seus dados.
MISSION_DONE_TOPIC = '/ipp/mission_done'
