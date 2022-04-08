import pytest
import sys
sys.path.insert(1, '../automatix_guardar_zona_service')
from automatix_guardar_zona_service.automatix_guardar_zona_server import Service as s


@pytest.mark.parametrize("zonas, esperado", [
    (["zonas:0.0.0.0"], ""),
    (["zonas-0,0,0,0"], ""),
    (["zonas:0,0,0.0"], ""),
    (["zonas:0,0,0,0"], "zonas:0,0,0,0;"),
    (["zonas:0,0,0,0;"], ""),
    ([""], "")
])
def test_colision(zonas, esperado):
    assert s.validar_zonas(s, zonas) == esperado