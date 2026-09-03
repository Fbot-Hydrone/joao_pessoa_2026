#!/usr/bin/env python3
"""
Testes do sorteio das bases móveis da Fase 1.

O sorteio é a única coisa neste repo que decide ONDE as bases estão, e todo
número medido contra "a verdade" — o recall do detector, o erro do mapa, se um
pouso foi numa base ou no chão — é medido contra a saída desta função. Um
defeito aqui não aparece como erro: aparece como uma missão que parece pior do
que é, ou melhor.

O caso que motivou o arquivo: MEDIDO 2026-09-03, seed 100. A base sorteada
sobre a casinha nascia DENTRO dela, 10 cm abaixo do telhado. A regra estava
certa — só spawna em cima, e inteira no telhado — e o número é que estava
errado, `house_height` a 1,5 contra um telhado a 1,6.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from biguasim_main.bases import (  # noqa: E402
    ARENA_HALF, BASE_HALF, DEFAULT_HOUSE, DEFAULT_HOUSE_HEIGHT,
    DEFAULT_TAKEOFF, WALL_MARGIN, _inside, _overlaps, sample_bases)


SEEDS = [0, 3, 10, 42, 100, 777]


# ── a regra da casinha ──────────────────────────────────────────────────────

def test_a_base_sobre_a_casinha_fica_no_TELHADO():
    """Nem sorteada, nem dentro: exatamente a altura do telhado.

    Este é o teste que o defeito de 2026-09-03 teria pego. Uma base sobre a
    casinha a qualquer z != house_height está atravessando a estrutura.
    """
    for seed in SEEDS:
        for x, y, z in sample_bases(6, seed):
            if _overlaps(x, y, DEFAULT_HOUSE):
                assert z == pytest.approx(DEFAULT_HOUSE_HEIGHT), (
                    f"seed {seed}: base em ({x:.2f}, {y:.2f}) sobre a casinha "
                    f"com z={z:.2f}, e o telhado está em "
                    f"{DEFAULT_HOUSE_HEIGHT}")


def test_uma_base_sobre_a_casinha_cabe_inteira_no_telhado():
    """Encostar não basta — metade de uma base pendurada na beirada cairia."""
    for seed in SEEDS:
        for x, y, _ in sample_bases(6, seed):
            if _overlaps(x, y, DEFAULT_HOUSE):
                assert _inside(x, y, DEFAULT_HOUSE, BASE_HALF), (
                    f"seed {seed}: base em ({x:.2f}, {y:.2f}) toca a casinha "
                    "sem caber inteira nela")


def test_o_telhado_e_mais_alto_que_qualquer_base_sorteada():
    """Se z_max alcançasse o telhado, 'em cima da casinha' deixaria de ser uma
    altura distinta e o caso especial não teria como ser conferido."""
    assert DEFAULT_HOUSE_HEIGHT > 1.5


# ── as regras do regulamento ────────────────────────────────────────────────

def test_nada_nasce_na_base_de_decolagem():
    """O drone começa ali. Uma base sob ele arruína a corrida antes de armar."""
    for seed in SEEDS:
        for x, y, _ in sample_bases(6, seed):
            assert not _overlaps(x, y, DEFAULT_TAKEOFF), (
                f"seed {seed}: base em ({x:.2f}, {y:.2f}) na base de decolagem")


def test_todas_ficam_dentro_da_arena_e_longe_das_paredes():
    limit = ARENA_HALF - WALL_MARGIN
    for seed in SEEDS:
        for x, y, _ in sample_bases(6, seed):
            assert -limit <= x <= limit and -limit <= y <= limit, (
                f"seed {seed}: base em ({x:.2f}, {y:.2f}) a menos de "
                f"{WALL_MARGIN} m da parede")


def test_a_altura_sorteada_respeita_a_faixa_do_regulamento():
    """0 a 1,5 m — exceto sobre a casinha, que é uma superfície, não um sorteio."""
    for seed in SEEDS:
        for x, y, z in sample_bases(6, seed):
            if _overlaps(x, y, DEFAULT_HOUSE):
                continue
            assert 0.0 <= z <= 1.5, f"seed {seed}: z={z:.2f} fora de [0, 1.5]"


def test_as_bases_nao_se_encostam():
    for seed in SEEDS:
        pts = sample_bases(6, seed)
        for i, (x1, y1, _) in enumerate(pts):
            for x2, y2, _ in pts[i + 1:]:
                d = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
                assert d >= 1.5 - 1e-9, f"seed {seed}: duas bases a {d:.2f} m"


# ── o contrato de que todo número medido depende ────────────────────────────

def test_sempre_o_numero_de_bases_pedido():
    for seed in SEEDS:
        for count in (1, 3, 6):
            assert len(sample_bases(count, seed)) == count


def test_a_mesma_seed_da_sempre_a_mesma_arena():
    """Sem isto, `--seed N` deixa de reproduzir uma corrida e toda medição
    contra a verdade vira anedota."""
    for seed in SEEDS:
        assert sample_bases(6, seed) == sample_bases(6, seed)


def test_seeds_diferentes_dao_arenas_diferentes():
    layouts = {tuple(map(tuple, sample_bases(6, s))) for s in SEEDS}
    assert len(layouts) == len(SEEDS)
