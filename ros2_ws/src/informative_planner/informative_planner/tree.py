"""
tree.py
-------
Define a classe Tree: unidade fundamental do sistema.

Cada árvore tem:
  - Um ID único
  - Posição 2D no mundo (x, y) [m]
  - DAP verdadeiro (apenas para simulação/stub)
  - Lista de medições RANSAC acumuladas
  - Estimativa atual do DAP (média das medições RANSAC)
  - Incerteza do DAP derivada da cobertura angular da circunferência
"""

from dataclasses import dataclass, field


@dataclass
class Tree:
    """
    Representa uma árvore individual na floresta.

    Atributos
    ---------
    tree_id : int
        Identificador único da árvore.
    x : float
        Coordenada X no referencial do mundo [m].
    y : float
        Coordenada Y no referencial do mundo [m].
    true_dap : float
        DAP real da árvore [m]. Usado APENAS pelo stub do LiDAR para
        simular medições ruidosas. Não deve ser acessado pelo planejador.
    dap_measurements : list
        Leituras de DAP retornadas pelo RANSAC a cada scan. A estimativa
        final é a média dessas leituras.
    dap_estimate : float
        Média das medições RANSAC acumuladas [m]. Zero enquanto não observada.
    num_observations : int
        Contador de quantas vezes esta árvore foi diretamente observada.
    observed_angles : list
        Ângulos de observação já usados para medir esta árvore. Cada ângulo
        representa de que lado o drone viu o tronco.
    """

    tree_id: int
    x: float
    y: float
    true_dap: float = 0.25          # DAP real (apenas stub) [m]
    dap_measurements: list = field(default_factory=list)  # leituras RANSAC [m]
    dap_estimate: float = 0.0       # Média das medições RANSAC [m]
    num_observations: int = 0
    observed_angles: list = field(default_factory=list)

    def update_dap(self, measurement: float) -> None:
        """Acumula uma medição RANSAC e atualiza a estimativa de DAP."""
        import numpy as np
        self.dap_measurements.append(float(measurement))
        self.dap_estimate = float(np.mean(self.dap_measurements))

    @property
    def dap_std(self) -> float:
        """
        Incerteza do DAP baseada na cobertura angular da circunferência.

        A lógica: quanto mais lados do tronco o RANSAC já viu, menor a
        incerteza. Cobertura total (todos os setores) → std mínimo.
        Árvore nunca observada → std máximo (prior).
        """
        from config import RANSAC_MIN_STD, RANSAC_MAX_STD
        if not self.dap_measurements:
            return RANSAC_MAX_STD
        coverage = self.angular_coverage_ratio()
        return RANSAC_MAX_STD * (1.0 - coverage) + RANSAC_MIN_STD * coverage

    def position(self):
        """Retorna a posição como array [x, y]."""
        import numpy as np
        return np.array([self.x, self.y])

    def trunk_radius(self) -> float:
        """
        Raio estimado do tronco [m] para uso no modelo de oclusão.
        Usa a estimativa de DAP se disponível, ou um valor padrão.
        """
        from config import DEFAULT_TRUNK_RADIUS
        if self.dap_estimate > 0.05:
            return self.dap_estimate / 2.0
        return DEFAULT_TRUNK_RADIUS

    def observation_angle_from(self, observer_xy) -> float:
        """
        Ângulo do observador ao redor do tronco [rad].

        Dois scans em lados opostos da árvore produzem ângulos bem diferentes,
        o que representa maior cobertura geométrica para o RANSAC.
        """
        import numpy as np
        observer_xy = np.asarray(observer_xy, dtype=float)
        return float(np.arctan2(observer_xy[1] - self.y,
                                observer_xy[0] - self.x))

    def angle_sector(self, angle: float, sector_degrees: float = None) -> int:
        """Converte um ângulo contínuo em um setor angular discreto."""
        import numpy as np
        import config

        if sector_degrees is None:
            sector_degrees = config.RANSAC_VIEW_SECTOR_DEGREES
        sector_width = np.deg2rad(sector_degrees)
        wrapped = (angle + np.pi) % (2.0 * np.pi)
        return int(np.floor(wrapped / sector_width))

    def angle_sector_count(self, extra_angle: float = None) -> int:
        """
        Quantos setores angulares distintos já foram usados para esta árvore.

        `extra_angle` permite simular uma nova observação sem alterar o estado.
        """
        angles = list(self.observed_angles)
        if extra_angle is not None:
            angles.append(extra_angle)
        return len({self.angle_sector(angle) for angle in angles})

    def has_seen_similar_angle(self, angle: float) -> bool:
        """Retorna True se o novo ângulo cai em um setor já observado."""
        new_sector = self.angle_sector(angle)
        return any(self.angle_sector(old) == new_sector
                   for old in self.observed_angles)

    def angular_coverage_ratio(self, extra_angle: float = None) -> float:
        """
        Cobertura angular aproximada usada para simular a qualidade do RANSAC.

        0 significa nenhum lado visto; 1 significa setores suficientes para um
        ajuste de tronco bem condicionado.
        """
        import config
        count = self.angle_sector_count(extra_angle=extra_angle)
        return min(count / config.RANSAC_REQUIRED_VIEW_SECTORS, 1.0)

    def add_observation_angle(self, observer_xy) -> None:
        """Registra de qual lado esta árvore foi observada."""
        self.observed_angles.append(self.observation_angle_from(observer_xy))

    def __repr__(self) -> str:
        return (
            f"Tree(id={self.tree_id}, "
            f"pos=({self.x:.1f}, {self.y:.1f}), "
            f"dap_est={self.dap_estimate:.3f}m ± {self.dap_std:.3f}m, "
            f"obs={self.num_observations}, "
            f"sectors={self.angle_sector_count()})"
        )
