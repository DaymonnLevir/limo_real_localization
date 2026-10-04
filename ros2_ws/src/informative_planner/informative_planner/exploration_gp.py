"""
exploration_gp.py
-----------------
Processo Gaussiano Matérn 3/2 sobre uma grade discreta de waypoints,
atualizado por Filtro de Kalman sequencial — exatamente como em
Popovic et al. (2017), tmplanner_continuous.

IDEIA GERAL
-----------
O drone voa sobre um campo de árvores e quer saber, de cada posição (x,y)
possível da grade, quantas árvores ficam visíveis. Como ele não pode visitar
tudo de uma vez, usa um Processo Gaussiano (GP) para ESTIMAR esse valor em
lugares ainda não visitados, com base nas observações já feitas.

  GP mapeia:  f(x, y) = N_vis(x, y) = número de árvores visíveis de (x, y)

Esta é uma propriedade ESTÁTICA do espaço (árvores não se movem) — o GP
recebe observações temporalmente consistentes.

O bônus angular (N_new) é calculado analiticamente em planner.py e somado
ao critério de seleção de waypoint. Ele NÃO entra nos dados de treinamento
do GP porque é estado-dependente (muda ao longo da missão).

COMO O GP FUNCIONA (resumo intuitivo)
--------------------------------------
Imagine que você quer estimar a temperatura de uma cidade sem medir em todo
lugar. Se dois pontos são próximos, espera-se que tenham temperaturas
parecidas. O GP formaliza essa ideia:

  - O KERNEL (Matérn 3/2) define o quanto dois pontos são "correlacionados"
    com base na distância entre eles. Pontos perto → alta correlação.
  - A MÉDIA posterior (μ) é a melhor estimativa de N_vis em cada célula.
  - A COVARIÂNCIA posterior (P) mede a incerteza — células não visitadas
    têm alta incerteza; células já observadas têm baixa incerteza.

Kernel Matérn 3/2 isotrópico (Popovic et al.):
  k(x, x') = sf² · (1 + √3·r/ℓ) · exp(−√3·r/ℓ),   r = ‖x − x'‖

  Onde:
    sf = amplitude do sinal (quanto N_vis pode variar)
    ℓ  = comprimento de escala (raio de influência espacial em metros)
    r  = distância euclidiana entre os dois pontos

ATUALIZAÇÃO POR FILTRO DE KALMAN
----------------------------------
Em vez de re-treinar o GP do zero a cada nova observação (o que seria lento),
usa-se o Filtro de Kalman (KF) para atualizar μ e P de forma incremental.
O KF é equivalente ao GP Bayesiano, mas muito mais eficiente.

Equações do KF a cada nova observação z:
  h = H configurado (H_naive=e_i ou footprint radial do sensor)
  S = h^T · P · h + σ_n²    → variância total da observação (incerteza + ruído)
  K = P · h / S              → ganho de Kalman (quanto cada célula deve ser corrigida)
  μ_{t+1} = μ_t + K · (z − h^T μ_t)   → atualiza a média
  P_{t+1} = P_t − K · h^T · P_t        → reduz a incerteza

Para visualização em pontos arbitrários (não necessariamente da grade),
o método predict() usa a fórmula padrão de regressão GP com Matérn 3/2
e as observações acumuladas — matematicamente consistente com o estado KF.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

from informative_planner.observation_model import ObservationModel


class ExplorationGP:
    """
    GP Matérn 3/2 com atualização por Filtro de Kalman (Popovic et al., 2017).

    Parâmetros
    ----------
    candidates_xy : np.ndarray, shape (M, 2)
        Grade fixa de candidatos a waypoint.  Definida uma única vez; o KF
        mantém μ e P sobre esses M pontos ao longo de toda a missão.
    length_scale : float
        Comprimento de escala ℓ [m]. Controla o raio de influência espacial:
        valores grandes → variações suaves; pequenos → variações locais.
    signal_std : float
        Desvio padrão do sinal sf. Define a amplitude máxima das variações
        de N_vis esperadas no campo.
    noise_std : float
        Desvio padrão do ruído de observação σ_n. Representa erros de contagem
        por oclusão parcial de árvores. Também adicionado à diagonal do prior.
    beta : float
        Coeficiente UCB. β > 0 favorece regiões com alta incerteza
        (exploration). β = 0 equivale a só usar a média (pure exploitation).
    observation_model : {'H_naive', 'footprint'}
        Modelo de H usado no update de Kalman.
    observation_footprint_radius : float
        Raio efetivo do footprint usado quando observation_model='footprint'.
    observation_footprint_sigma : float, opcional
        Escala radial dos pesos gaussianos do footprint.
    """

    def __init__(
        self,
        candidates_xy: np.ndarray,
        length_scale: float = 5.0,
        signal_std: float = 3.0,
        noise_std: float = 0.5,
        beta: float = 2.0,
        observation_model: str = ObservationModel.H_NAIVE,
        observation_footprint_radius: float = 6.0,
        observation_footprint_sigma: Optional[float] = None,
    ):
        # Armazena a grade de candidatos como matriz (M, 2)
        self.X_grid = np.asarray(candidates_xy, dtype=float)  # (M, 2)
        self.M = len(self.X_grid)   # número total de células da grade

        # Hiper-parâmetros do kernel e do ruído
        self.ell = length_scale     # comprimento de escala ℓ
        self.sf = signal_std        # amplitude do sinal sf
        self.noise_var = noise_std ** 2   # σ_n² (variância do ruído)
        self.beta = beta            # peso da incerteza no critério UCB
        self.observation_model = ObservationModel(
            self.X_grid,
            mode=observation_model,
            footprint_radius=observation_footprint_radius,
            footprint_sigma=observation_footprint_sigma,
        )

        # ---- Estado inicial do GP (prior) ----

        # Média a priori μ₀ = 0 em todas as células (nada observado ainda)
        self.mean_ = np.zeros(self.M)

        # Covariância a priori P₀ = K_Matérn + σ_n²·I
        # K_Matérn captura a correlação espacial entre células.
        # Somar σ_n²·I garante que P₀ é bem condicionada (inversível)
        # e reflete a incerteza inicial igual a Popovic et al.
        K = self._matern32(self.X_grid, self.X_grid)   # matriz (M, M)
        self.cov_ = K + self.noise_var * np.eye(self.M)

        # Listas para guardar as observações brutas (posição + valor)
        # Usadas apenas em predict() para pontos fora da grade
        self._obs_X: List[np.ndarray] = []   # posições observadas
        self._obs_y: List[float] = []         # valores N_vis observados

    # ------------------------------------------------------------------
    # Interface principal
    # ------------------------------------------------------------------

    def add_observation(self, pos_xy: np.ndarray, n_vis: float) -> None:
        """
        Incorpora uma nova observação e atualiza o posterior via Filtro de Kalman.

        Cada vez que o drone visita um ponto e conta quantas árvores vê,
        chamamos este método. Ele monta H pelo modelo configurado:
          - H_naive: uma célula da grade, igual ao comportamento legado.
          - footprint: pesos radiais nas células dentro do raio efetivo.

        Parâmetros
        ----------
        pos_xy : np.ndarray, shape (2,)
            Posição do drone (x, y) no momento do scan.
        n_vis : float
            Número de árvores visíveis observado: z = N_vis(pos_xy).
        """
        # Usa só as coordenadas x e y (ignora z se vier 3D)
        pos = np.asarray(pos_xy[:2], dtype=float)

        h = self.observation_model.vector_at(pos)

        # Filtro de Kalman com H genérico:
        #   p_h = P·H
        #   S   = Hᵀ·P·H + σ_n²
        p_h = h.cov_times(self.cov_)

        # Variância inovação: S = Hᵀ·P·H + σ_n²
        # "Quanto de incerteza existe nessa observação?"
        S = h.variance_from_cov_times(p_h) + self.noise_var
        if S <= 0.0:
            return

        # Ganho de Kalman: K = P·H / S
        # Define quanto cada célula j deve ser corrigida pela observação.
        # Células correlacionadas com H recebem ganho maior.
        K_gain = p_h / S                               # shape (M,)

        # Inovação: diferença entre o que foi observado e o que o GP previa
        innovation = float(n_vis) - h.mean(self.mean_)

        # Atualiza a média: μ ← μ + K·inovação
        # np.maximum(..., 0.0) impede estimativas negativas (contagem não pode ser < 0)
        self.mean_ = np.maximum(self.mean_ + K_gain * innovation, 0.0)

        # Atualiza a covariância: P ← P − K·p_h^T
        # np.outer(K_gain, p_h) cria a matriz de rank-1 K·Hᵀ·P
        # Subtrair isso reduz a incerteza em toda a grade (sobretudo perto de H)
        self.cov_ = self.cov_ - np.outer(K_gain, p_h)

        # Força simetria numérica em P (evita drift por erros de ponto flutuante)
        self.cov_ = 0.5 * (self.cov_ + self.cov_.T)

        # Salva a observação bruta para uso posterior em predict()
        self._obs_X.append(pos.copy())
        self._obs_y.append(float(n_vis))

    def predict(self, X_test: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calcula o posterior GP em pontos arbitrários (para visualização/debug).

        Usa a fórmula clássica de regressão GP — útil para avaliar o mapa
        em posições que não estão na grade discreta de candidatos.

        O resultado é matematicamente equivalente ao estado KF: ambos
        descrevem a mesma distribuição posterior (Gaussiana).

        Parâmetros
        ----------
        X_test : np.ndarray, shape (N, 2) ou (2,)
            Pontos onde avaliar o posterior.

        Retorna
        -------
        mean : np.ndarray, shape (N,)   — média posterior (estimativa de N_vis)
        std  : np.ndarray, shape (N,)   — desvio padrão (incerteza da estimativa)
        """
        X_test = np.atleast_2d(X_test)   # garante shape (N, 2) mesmo se vier (2,)
        N = len(X_test)

        # Sem observações ainda → retorna prior: média 0, desvio = amplitude sf
        if not self._obs_X:
            return np.zeros(N), np.full(N, self.sf)

        X_obs = np.array(self._obs_X)   # posições observadas, shape (n, 2)
        y_obs = np.array(self._obs_y)   # valores observados, shape (n,)
        n = len(X_obs)

        # Matriz de kernel entre observações: K(X_obs, X_obs) + σ_n²·I
        # σ_n²·I é adicionado para tornar o sistema linear bem-condicionado
        K_obs = self._matern32(X_obs, X_obs) + self.noise_var * np.eye(n)

        # Kernel cruzado: correlação entre pontos de teste e observações
        K_star = self._matern32(X_test, X_obs)   # shape (N, n)

        # alpha = K_obs⁻¹ · y_obs  (vetor de "pesos" das observações)
        # Resolve o sistema linear em vez de inverter K_obs diretamente (mais estável)
        alpha = np.linalg.solve(K_obs, y_obs)

        # Média posterior: μ* = K_star · alpha
        # np.maximum(..., 0.0) impede estimativas negativas
        mean = np.maximum(K_star @ alpha, 0.0)

        # Variância posterior: σ²* = k(x*,x*) − K_star · K_obs⁻¹ · K_star^T
        # V = K_obs⁻¹ · K_star^T  (resolve outro sistema linear, mais eficiente)
        V = np.linalg.solve(K_obs, K_star.T)     # shape (n, N)

        # k_ss = diagonal do kernel prior nos pontos de teste = sf² (kernel próprio)
        k_ss = np.full(N, self.sf ** 2)

        # Variância = prior − redução por observações; garante ≥ 0 por segurança
        var = np.maximum(k_ss - np.sum(K_star * V.T, axis=1), 0.0)

        return mean, np.sqrt(var)   # retorna desvio padrão, não variância

    def ucb_grid(self) -> np.ndarray:
        """
        Calcula o critério UCB (Upper Confidence Bound) para toda a grade.

        UCB = μ + β · σ

        Esse critério equilibra dois objetivos:
          - Exploitation (μ alto): visitar onde se espera ver muitas árvores.
          - Exploration  (σ alto): visitar onde há mais incerteza.

        O parâmetro β controla esse trade-off: β grande → mais exploração.

        Retorna shape (M,). O planner escolhe a célula com maior UCB.
        """
        # Diagonal de cov_ = variância de cada célula; np.maximum evita raiz de negativo
        stds = np.sqrt(np.maximum(np.diag(self.cov_), 0.0))
        return self.mean_ + self.beta * stds

    def predict_trace_reduction(self, candidate_idx: int) -> float:
        """
        Estima quanto a incerteza total do mapa diminuiria se o drone
        visitasse a célula j (sem precisar fazer a medição de fato).

        Fórmula analítica (redução de trace de P):
          Δtrace = ‖P·H‖² / (Hᵀ·P·H + σ_n²)

        Quanto maior o valor, mais "valioso" é visitar essa célula do ponto
        de vista de redução de incerteza. Pode ser usado no lugar do UCB.
        """
        pos = self.X_grid[int(candidate_idx), :2]
        h = self.observation_model.vector_at(pos)
        p_h = h.cov_times(self.cov_)
        S = h.variance_from_cov_times(p_h) + self.noise_var
        if S <= 0.0:
            return 0.0
        return float(np.dot(p_h, p_h) / S)  # ‖P·H‖² / S

    def compute_trace(self) -> float:
        """
        Retorna a incerteza total do mapa = trace(P) = soma das variâncias.

        Útil para monitorar a convergência da missão: conforme o drone
        explora, trace(P) vai diminuindo. Quando estabilizar, o mapa
        está suficientemente conhecido.
        """
        return float(np.trace(self.cov_))

    # ------------------------------------------------------------------
    # Kernel
    # ------------------------------------------------------------------

    def _matern32(self, X1: np.ndarray, X2: np.ndarray) -> np.ndarray:
        """
        Calcula a matriz de covariância Matérn 3/2 isotrópica entre dois
        conjuntos de pontos X1 e X2.

        Fórmula (Popovic et al., 2017):
          k(x, x') = sf² · (1 + √3·r/ℓ) · exp(−√3·r/ℓ),   r = ‖x − x'‖

        A Matérn 3/2 é uma escolha popular para campos físicos porque:
          - Funções amostradas são diferenciáveis uma vez (suaves mas não
            infinitamente lisas como o kernel RBF/Gaussiano).
          - Decai mais rápido que RBF para distâncias grandes → menos
            "correlação global" artificial.

        Parâmetros
        ----------
        X1 : np.ndarray, shape (n1, 2)
        X2 : np.ndarray, shape (n2, 2)

        Retorna
        -------
        K : np.ndarray, shape (n1, n2)   — matriz de covariância
        """
        # diff[i, j, :] = X1[i] - X2[j]  →  shape (n1, n2, 2)
        diff = X1[:, None, :] - X2[None, :, :]

        # r[i, j] = ‖X1[i] - X2[j]‖  →  distância euclidiana, shape (n1, n2)
        r = np.linalg.norm(diff, axis=-1)

        # z = √3 · r / ℓ  (distância normalizada)
        z = np.sqrt(3.0) * r / self.ell

        # Aplica a fórmula Matérn 3/2: sf² · (1 + z) · exp(-z)
        return self.sf ** 2 * (1.0 + z) * np.exp(-z)

    # ------------------------------------------------------------------

    @property
    def n_observations(self) -> int:
        """Número de observações registradas até o momento."""
        return len(self._obs_X)
