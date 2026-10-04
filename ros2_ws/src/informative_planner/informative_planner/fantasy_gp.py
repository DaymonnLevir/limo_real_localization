"""
fantasy_gp.py
-------------
Atualizações "fantasiadas" (simuladas) do Filtro de Kalman sobre a grade
do ExplorationGP, para avaliar caminhos candidatos por GANHO DE INFORMAÇÃO
em vez de soma de UCB.

POR QUE ISSO É POSSÍVEL
-----------------------
A equação de atualização da covariância do Kalman

    P⁺ = P − P·h·hᵀ·P / (hᵀ·P·h + σ_n²)

não contém o valor medido z — apenas ONDE se mede (h). Logo, é possível
simular ("fantasiar") as medições de um caminho candidato e calcular
exatamente quanta incerteza o mapa perderia, sem saber o que seria
observado. Este é o fundamento do IPP de Popovic et al. (2017/2020).

GANHO DE INFORMAÇÃO E SUBMODULARIDADE
-------------------------------------
O ganho de um caminho τ = (x₁, …, x_k) é a redução do trace (A-ótimo):

    I(τ) = Tr(P₀) − Tr(P_k)

Cada visita reduz P na célula observada (e vizinhas correlacionadas);
uma SEGUNDA visita à mesma célula encontra P[i,i] já colapsado para
~σ_n² e ganha quase nada. Revisitas e loops perdem valor automaticamente
— sem penalidade ad hoc. Essa propriedade (retornos decrescentes) é a
submodularidade do ganho de informação Gaussiano (Krause & Guestrin).

IMPLEMENTAÇÃO EFICIENTE (rank-1, sem copiar P)
-----------------------------------------------
Os updates são rank-1 sequenciais. Definindo

    u_j = P_{j−1}·h_j / √S_j,     S_j = hⱼᵀ·P_{j−1}·h_j + σ_n²

vale  P_k = P₀ − Σ_j u_j·uⱼᵀ.  Cada estado fantasiado guarda apenas o
seu vetor u (M floats) e aponta para o pai — perfeito para a árvore do
RRT, onde estados irmãos compartilham a ancestralidade. Para criar um
filho que observa com h′:

    p = P₀·h′ − Σ_{j ∈ ancestrais} (u_jᵀ·h′)·u_j
    S = h′ᵀ·p + σ_n²
    u = p / √S

E o bônus: o ganho incremental do nó é exatamente ‖u‖², pois

    ΔTr_j = ‖P_{j−1}·h_j‖² / S_j = ‖u_j‖²

Custo por nó: O(M·depth). É a generalização em caminho do
ExplorationGP.predict_trace_reduction(), que cobre só o primeiro passo.

MODELO DE OBSERVAÇÃO (h)
------------------------
O mesmo ObservationModel do ExplorationGP monta h. Em H_naive, h = e_i e
o comportamento é idêntico ao legado. Em footprint, h tem pesos esparsos
nas células dentro do raio efetivo do sensor.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from informative_planner.exploration_gp import ExplorationGP
from informative_planner.observation_model import ObservationVector


class FantasyState:
    """
    Posterior fantasiado após uma sequência de observações simuladas.

    Imutável e encadeável: cada estado guarda somente o vetor rank-1 `u`
    da sua própria observação e uma referência ao estado pai. A cadeia
    de ancestrais reconstrói P_k = P₀ − Σ u_j·uⱼᵀ implicitamente, sem
    nunca materializar P_k.

    Atributos
    ---------
    gain : float
        Redução de trace produzida por ESTA observação (= ‖u‖²).
    gain_accum : float
        Redução de trace acumulada desde a raiz: Tr(P₀) − Tr(P_k).
    depth : int
        Número de observações fantasiadas desde a raiz.
    """

    __slots__ = ('_owner', '_parent', '_u', 'gain', 'gain_accum', 'depth')

    def __init__(
        self,
        owner: 'FantasyGP',
        parent: Optional['FantasyState'],
        u: Optional[np.ndarray],
        gain: float,
    ):
        self._owner = owner
        self._parent = parent
        self._u = u
        self.gain = float(gain)
        self.gain_accum = (
            parent.gain_accum + float(gain) if parent is not None else 0.0
        )
        self.depth = parent.depth + 1 if parent is not None else 0

    def observe_vector(self, h: ObservationVector) -> 'FantasyState':
        """
        Retorna o estado após fantasiar uma medição com vetor H genérico.

        Aplica o downdate rank-1 do Kalman usando P₀·H corrigido pelas
        observações fantasiadas dos ancestrais:

            p = P₀·H − Σ_j (u_jᵀ·H)·u_j
            S = Hᵀ·p + σ_n²
            u = p / √S          (ganho desta observação: ‖u‖²)
        """
        owner = self._owner

        p = owner.P0[:, h.indices] @ h.weights
        state = self
        while state is not None and state._u is not None:
            p -= float(state._u[h.indices] @ h.weights) * state._u
            state = state._parent

        S = h.variance_from_cov_times(p) + owner.noise_var
        if S <= 0.0:
            return FantasyState(owner, self, None, 0.0)

        u = p / np.sqrt(S)
        return FantasyState(owner, self, u, float(u @ u))

    def observe(self, cell_index: int) -> 'FantasyState':
        """
        Retorna o estado após fantasiar uma medição na célula `cell_index`.

        Aplica o downdate rank-1 do Kalman usando a coluna do prior P₀
        corrigida pelas observações fantasiadas dos ancestrais:

            p = P₀[:, i] − Σ_j u_j[i]·u_j
            S = p[i] + σ_n²
            u = p / √S          (ganho desta observação: ‖u‖²)
        """
        h = self._owner.observation_model.vector_for_cell_index(cell_index)
        return self.observe_vector(h)

    def observe_at(self, pos_xy: np.ndarray) -> 'FantasyState':
        """Fantasia uma medição em pos_xy com o ObservationModel configurado."""
        return self.observe_vector(self._owner.observation_model.vector_at(pos_xy))


class FantasyGP:
    """
    Fábrica de estados fantasiados sobre o posterior ATUAL do ExplorationGP.

    Construa uma instância por chamada de planejamento (ela congela uma
    cópia de P no instante do replan) e use `root()` como estado inicial
    da árvore RRT. Estados irmãos compartilham ancestrais — a memória
    total é O(nós × M).

    Exemplo
    -------
    >>> fantasy = FantasyGP(exploration_gp)
    >>> s0 = fantasy.root()
    >>> s1 = s0.observe_at(np.array([3.0, -2.0]))
    >>> s2 = s1.observe_at(np.array([8.0, -2.0]))
    >>> s2.gain_accum      # Tr(P₀) − Tr(P₂): ganho de informação do caminho
    """

    def __init__(self, exploration_gp: ExplorationGP):
        # Cópia defensiva: o KF real pode atualizar cov_ entre replans; o
        # planejamento deve raciocinar sobre um snapshot consistente.
        self.P0 = np.array(exploration_gp.cov_, dtype=float, copy=True)
        self.noise_var = float(exploration_gp.noise_var)
        self.X_grid = exploration_gp.X_grid
        self.observation_model = exploration_gp.observation_model
        self.trace0 = float(np.trace(self.P0))

    def root(self) -> FantasyState:
        """Estado inicial: nenhum update fantasiado (P = P₀)."""
        return FantasyState(self, None, None, 0.0)

    def cell_index_for(self, pos_xy: np.ndarray) -> int:
        """Célula da grade mais próxima (mesmo snapping do add_observation)."""
        return self.observation_model.cell_index_for(pos_xy)
