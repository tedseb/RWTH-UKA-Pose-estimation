import sys
sys.path.append('/home/trainerai/trainerai-core/')
from src.station_manager.src import SMResponse

def test_sm_response():
    assert SMResponse(503, 1, {"station": 1, "exercise": 105, "set_id": 1}) == SMResponse(503, 1, {"station": 1, "exercise": 105, "set_id": 1})
    assert SMResponse(503, 1, {"station": 1, "exercise": 105}) == SMResponse(503, 1, {"station": 1, "exercise": 105, "error": 1})
    assert SMResponse(503, 1, {"station": 1, "exercise": 105, "error": 1}) == SMResponse(503, 1, {"station": 1, "exercise": 105})
    assert SMResponse(503, 1, {"station": 1, "exercise": 105}) != SMResponse(503, 1, {"station": 5, "exercise": 105})
    assert SMResponse(503, 1, {"station": 1, "exercise": 105}) != SMResponse(505, 1, {"station": 1, "exercise": 105})
    assert SMResponse(503, 1, {"station": 1, "exercise": 105}) != SMResponse(505, 1, {"station": 1})
    assert SMResponse(503, 1, {"station": 1}) != SMResponse(505, 1, {"station": 1, "exercise": 105})
