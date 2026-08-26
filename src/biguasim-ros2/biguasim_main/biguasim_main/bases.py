"""Sorteio determinístico das bases de pouso móveis da Fase 1.

As regras (CBR 2026, Fase 1 / Obs. 2 e 3) colocam 6 bases móveis em qualquer
lugar da arena, a pelo menos 0,5 m das paredes e a uma altura de 0 a 1,5 m.
Aqui o sorteio é por seed: a mesma seed dá sempre a mesma arena, o que permite
repetir uma corrida de teste tantas vezes quanto necessário.

Geometria, em coordenadas de mundo do CompetionMap (centro da arena = 0,0,0),
lida da Figura 3 das regras. O canto onde ficam a base de decolagem e a casinha
é o canto (+x, +y): a base de decolagem é 2 x 1,5 m e cai em x[2,4] y[2.5,4],
cujo centro (3, 3.25) é onde o drone nasce (config.yaml: location [3, 3.4, ...]).
"""

import random

ARENA_HALF = 4.0        # arena 8 x 8 m
WALL_MARGIN = 0.5       # regra: bases a >= 0,5 m das paredes

# (x_min, x_max, y_min, y_max) das regiões proibidas.
KEEP_OUT = [
    (-4.0, 2.0, 2.0, 4.0),   # casinha (ambiente confinado da Fase 4), 6 x 2 m
    (2.0, 4.0, 2.5, 4.0),    # base de decolagem, 2 x 1,5 m
]

BASE_HALF = 0.5         # base de pouso 1 x 1 m


def sample_bases(count, seed, z_min=0.0, z_max=1.5, min_spacing=1.5):
    """Sorteia `count` posições [x, y, z] válidas para as bases móveis."""
    rng = random.Random(seed)
    limit = ARENA_HALF - WALL_MARGIN
    chosen = []

    for _ in range(count):
        for _attempt in range(1000):
            x = rng.uniform(-limit, limit)
            y = rng.uniform(-limit, limit)
            if _blocked(x, y) or _too_close(x, y, chosen, min_spacing):
                continue
            chosen.append([x, y, rng.uniform(z_min, z_max)])
            break
        else:
            raise RuntimeError(
                f"não foi possível posicionar {count} bases com "
                f"min_spacing={min_spacing} m na área livre da arena"
            )

    return chosen


def _blocked(x, y):
    """A base inteira (1 x 1 m) tem que ficar fora das regiões proibidas."""
    return any(
        x_min - BASE_HALF < x < x_max + BASE_HALF
        and y_min - BASE_HALF < y < y_max + BASE_HALF
        for x_min, x_max, y_min, y_max in KEEP_OUT
    )


def _too_close(x, y, chosen, min_spacing):
    return any(
        (x - cx) ** 2 + (y - cy) ** 2 < min_spacing ** 2 for cx, cy, _ in chosen
    )
