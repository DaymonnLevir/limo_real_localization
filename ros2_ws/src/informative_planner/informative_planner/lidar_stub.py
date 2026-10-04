"""
lidar_stub.py
-------------
STUB: Simula o sensor LiDAR e o processamento RANSAC para extração de DAP.

Em um sistema real, este módulo seria substituído por:
  1. Driver ROS do LiDAR (ex: Velodyne, Ouster, Livox)
  2. Pipeline de processamento de nuvem de pontos (PCL / Open3D)
  3. Segmentação de troncos (clustering + filtro de altura)
  4. Ajuste de círculo via RANSAC

O STUB retorna:
  - DAP medido = DAP real + ruído Gaussiano (distância-dependente)

A incerteza acumulada de cada árvore é calculada em Tree.dap_std com
base na cobertura angular da circunferência, não no ruído de cada scan.
"""

import numpy as np
from typing import List

from tree import Tree
import config


class LiDARStub:
    """
    STUB do sensor LiDAR + pipeline RANSAC de extração de DAP.

    Parâmetros
    ----------
    seed : int, opcional
        Semente para reproducibilidade da simulação.
    """

    def __init__(self, seed: int = 42):
        self._rng = np.random.default_rng(seed)

    def scan(self, drone_xy: np.ndarray,
             trees: List[Tree],
             visible_indices: List[int]) -> np.ndarray:
        """
        STUB: Simula um scan LiDAR e extração de DAP por RANSAC.

        Para cada árvore visível, retorna uma medição de DAP com ruído
        Gaussiano proporcional à distância (mais longe = mais ruído, pois
        há menos pontos LiDAR incidindo no tronco).

        Parâmetros
        ----------
        drone_xy : np.ndarray, shape (2,)
            Posição (x, y) do drone [m].
        trees : List[Tree]
            Lista completa de árvores.
        visible_indices : List[int]
            Índices das árvores atualmente visíveis pelo LiDAR.

        Retorna
        -------
        measurements : np.ndarray, shape (m,)
            DAP medido para cada árvore visível [m].
        """
        measurements = []
        for k in visible_indices:
            tree = trees[k]
            var = self.scan_variance(drone_xy, tree)
            noise = self._rng.normal(0.0, np.sqrt(var))
            dap_measured = max(tree.true_dap + noise, 0.01)
            measurements.append(dap_measured)
        return np.array(measurements)

    @staticmethod
    def scan_variance(drone_xy: np.ndarray, tree: Tree) -> float:
        """
        Variância de ruído de um único scan RANSAC.

        Quanto mais longe a árvore, menos pontos LiDAR incidem no tronco
        e mais impreciso é o fit do círculo.
        """
        tree_xy = np.array([tree.x, tree.y])
        dist = np.linalg.norm(tree_xy - drone_xy)
        return config.LIDAR_BASE_VARIANCE + config.LIDAR_DIST_VARIANCE_COEFF * dist
