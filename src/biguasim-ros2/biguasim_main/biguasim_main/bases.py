"""Sorteio determinístico das bases de pouso móveis da Fase 1.

As regras (CBR 2026, Fase 1 / Obs. 2 e 3) colocam 6 bases móveis em qualquer
lugar da arena, inclusive em cima da casinha da Fase 4, a pelo menos 0,5 m das
paredes e a uma altura de 0 a 1,5 m. Aqui o sorteio é por seed: a mesma seed dá
sempre a mesma arena, o que permite repetir uma corrida de teste tantas vezes
quanto necessário.

Geometria, em coordenadas de mundo do CompetionMap (centro da arena = 0,0,0),
lida da Figura 3 das regras. O canto onde ficam a base de decolagem e a casinha
é o canto (+x, +y): a base de decolagem é 2 x 1,5 m e cai em x[2,4] y[2.5,4],
cujo centro (3, 3.25) é onde o drone nasce (config.yaml: location [3, 3.4, ...]).
"""

import random

ARENA_HALF = 4.0        # arena 8 x 8 m
WALL_MARGIN = 0.5       # regra: bases a >= 0,5 m das paredes
BASE_HALF = 0.5         # base de pouso 1 x 1 m

# A casinha e a base de decolagem vêm do config.yaml, como
# [x_min, x_max, y_min, y_max] em metros. Estes são só os defaults.
DEFAULT_HOUSE = (-4.0, 2.0, 2.0, 4.0)
DEFAULT_HOUSE_HEIGHT = 1.5
DEFAULT_TAKEOFF = (2.0, 4.0, 2.0, 4.0)


def sample_bases(count, seed, z_min=0.0, z_max=1.5, min_spacing=1.5,
                 house=DEFAULT_HOUSE, house_height=DEFAULT_HOUSE_HEIGHT,
                 takeoff=DEFAULT_TAKEOFF):
    """Sorteia `count` posições [x, y, z] válidas para as bases móveis.

    A casinha não é um buraco no sorteio: um ponto que cai sobre ela vale, desde
    que a base caiba inteira no telhado — e aí z é a altura da casinha, não um
    valor sorteado.
    """
    rng = random.Random(seed)

    # O sorteio é ganancioso ponto a ponto, então um arranjo pode se fechar
    # antes da última base caber. Nesse caso vale mais recomeçar o sorteio do
    # que insistir no arranjo travado — o rng segue de onde parou, então o
    # resultado continua determinístico para a seed.
    for _layout in range(100):
        chosen = _one_layout(rng, count, z_min, z_max, min_spacing,
                             house, house_height, takeoff)
        if chosen is not None:
            return chosen

    raise RuntimeError(
        f"não foi possível posicionar {count} bases com "
        f"min_spacing={min_spacing} m na área livre da arena"
    )


def _one_layout(rng, count, z_min, z_max, min_spacing, house, house_height,
                takeoff):
    """Uma tentativa de arranjo completo. None se alguma base não coube."""
    limit = ARENA_HALF - WALL_MARGIN
    chosen = []

    for _ in range(count):
        for _attempt in range(200):
            x = rng.uniform(-limit, limit)
            y = rng.uniform(-limit, limit)
            z = rng.uniform(z_min, z_max)

            if _overlaps(x, y, takeoff):
                continue
            if _overlaps(x, y, house):
                # Encostou na casinha: só vale em cima dela, inteira no telhado.
                if not _inside(x, y, house, inset=BASE_HALF):
                    continue
                z = house_height
            if _too_close(x, y, chosen, min_spacing):
                continue

            chosen.append([x, y, z])
            break
        else:
            return None

    return chosen


def _overlaps(x, y, region):
    """A base de 1 x 1 m encosta na região?"""
    x_min, x_max, y_min, y_max = region
    return (x_min - BASE_HALF < x < x_max + BASE_HALF
            and y_min - BASE_HALF < y < y_max + BASE_HALF)


def _inside(x, y, region, inset):
    x_min, x_max, y_min, y_max = region
    return (x_min + inset <= x <= x_max - inset
            and y_min + inset <= y <= y_max - inset)


def _too_close(x, y, chosen, min_spacing):
    return any(
        (x - cx) ** 2 + (y - cy) ** 2 < min_spacing ** 2 for cx, cy, _ in chosen
    )
