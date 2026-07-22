from parking_robot_system.task_dispatcher import decide


def test_no_data():
    assert decide(None, data_ready=False) == (False, "관제 데이터 없음(재시도)")


def test_nonexistent():
    assert decide(None, data_ready=True) == (False, "존재하지 않는 구역")


def test_occupied():
    info = {"exists": True, "occupied": True}
    assert decide(info, data_ready=True) == (False, "해당 구역에 차량이 있어 주차 불가")


def test_empty_accepted():
    info = {"exists": True, "occupied": False}
    accepted, msg = decide(info, data_ready=True)
    assert accepted is True
