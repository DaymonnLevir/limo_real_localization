"""
tree_map.py
-----------
Mapa de estado das árvores da floresta.

Mantém a lista de árvores, trata visibilidade e oclusão, e registra
medições RANSAC nos objetos Tree. Não há Processo Gaussiano nem Filtro
de Kalman para o DAP — cada árvore acumula suas próprias medições RANSAC
e a incerteza é derivada da cobertura angular da circunferência do tronco.
"""

from typing import List, Tuple

import numpy as np

import config
from tree import Tree


class TreeMap:
    """
    Contêiner de estado das árvores com lógica de visibilidade e oclusão.
    """

    def __init__(self, trees: List[Tree]):
        self.trees = trees

    def update(self, measurements: np.ndarray,
               visible_indices: List[int],
               drone_xy: np.ndarray) -> None:
        """
        Registra medições RANSAC nas árvores visíveis.

        Cada árvore acumula suas leituras; a estimativa de DAP e a
        incerteza angular são recalculadas automaticamente em Tree.
        """
        for i, k in enumerate(visible_indices):
            tree = self.trees[k]
            tree.update_dap(float(measurements[i]))
            tree.num_observations += 1
            tree.add_observation_angle(drone_xy)

    def get_visible_trees(self, drone_xy: np.ndarray,
                          max_range: float = None) -> List[int]:
        """Determina quais árvores são visíveis de uma posição do drone."""
        if max_range is None:
            max_range = config.LIDAR_MAX_RANGE

        visible = []
        for k, tree in enumerate(self.trees):
            tree_xy = np.array([tree.x, tree.y])
            dist = np.linalg.norm(tree_xy - drone_xy)
            if dist > max_range:
                continue
            if not self._is_occluded(drone_xy, tree_xy, k):
                visible.append(k)
        return visible

    def compute_total_uncertainty(self) -> float:
        """Incerteza total = soma dos desvios padrão de DAP de todas as árvores."""
        return sum(t.dap_std for t in self.trees)

    def most_uncertain_trees(self, top_k: int = 5) -> List[Tuple[int, float]]:
        """Retorna as árvores com maior desvio padrão de DAP."""
        stds = [(t.tree_id, t.dap_std) for t in self.trees]
        return sorted(stds, key=lambda x: -x[1])[:top_k]

    def _is_occluded(self, drone_xy: np.ndarray,
                     target_xy: np.ndarray,
                     target_idx: int) -> bool:
        """Verifica se outro tronco bloqueia o raio até a árvore alvo."""
        ray = target_xy - drone_xy
        ray_len = np.linalg.norm(ray)
        if ray_len < 1e-6:
            return False

        ray_unit = ray / ray_len

        for k, other in enumerate(self.trees):
            if k == target_idx:
                continue

            other_xy = np.array([other.x, other.y])
            t = np.dot(other_xy - drone_xy, ray_unit)
            if t <= 0.0 or t >= ray_len:
                continue

            closest_on_ray = drone_xy + t * ray_unit
            dist_to_ray = np.linalg.norm(other_xy - closest_on_ray)

            if dist_to_ray < other.trunk_radius():
                return True

        return False

    def __repr__(self) -> str:
        observed = sum(1 for t in self.trees if t.num_observations > 0)
        return (
            f"TreeMap(N={len(self.trees)} árvores, "
            f"observadas={observed}, "
            f"Σσ={self.compute_total_uncertainty():.4f} m)"
        )
