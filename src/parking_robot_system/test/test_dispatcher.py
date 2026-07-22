from parking_robot_system.task_dispatcher import decide, decide_exit


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


# --- 출차(EXIT) 판정: 입차와 점유 조건이 반대 ---
def test_exit_no_data():
    assert decide_exit(None, data_ready=False) == (False, "관제 데이터 없음(재시도)")


def test_exit_nonexistent():
    assert decide_exit(None, data_ready=True) == (False, "존재하지 않는 구역")


def test_exit_empty_rejected():
    info = {"exists": True, "occupied": False}
    assert decide_exit(info, data_ready=True) == (False, "해당 구역에 차량이 없어 출차 불가")


def test_exit_occupied_accepted():
    info = {"exists": True, "occupied": True}
    accepted, msg = decide_exit(info, data_ready=True)
    assert accepted is True
