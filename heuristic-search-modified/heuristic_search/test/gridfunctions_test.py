from heuristic_search.gridfunctions import generate_probability
from heuristic_search.gridfunctions import add_htt_cells

import pytest

def test_probability_p_1():
    ''' Should always return true if p = 1 '''
    assert generate_probability(1)


def test_probability_p_0():
    ''' Should always return false if p = 0 '''
    assert not generate_probability(0)


def test_probability_p_below0():
    ''' Should raise error if p < 0 '''
    with pytest.raises(ValueError):
        generate_probability(-0.5)


def test_probability_p_greaterthan1():
    ''' Should raise error if p > 1 '''
    with pytest.raises(ValueError):
        generate_probability(1.1)


def test_add_htt_cells():
    ''' Should generate n hard-to-traverse cells '''
    n = 5
    htt_cells = add_htt_cells(n, 0.5)
    assert len(htt_cells) == n
