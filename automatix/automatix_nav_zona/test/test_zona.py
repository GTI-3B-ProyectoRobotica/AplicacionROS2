import pytest
import sys
sys.path.insert(1, '../automatix_nav_zona')
from automatix_nav_zona.zona import Zona as s


@pytest.mark.parametrize("xi, xs, yi, ys, esperado", [
    (1,1,1,1, (1,1)),
    (1,2,1,2, (1.5,1.5)),
    (-2,1,-2,1, (-0.5,-0.5)),
    (1,1,1,2, (1,1.5)),
    (1,-1,1,-1, (0,0)),
])
def test_colision(xi,xs,yi,ys, esperado):
    assert s.calcular_punto_medio(s, xi,xs,yi,ys) == esperado