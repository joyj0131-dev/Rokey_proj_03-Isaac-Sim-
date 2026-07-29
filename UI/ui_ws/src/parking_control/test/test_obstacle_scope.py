from parking_control.core.obstacle_scope import (
    obstacle_affects_team,
    obstacle_scope,
    robot_team_role,
)


def test_zone_prefix_selects_only_its_robot_team():
    assert obstacle_affects_team("entry", "통로 막힘: ZIN03", -6.875)
    assert not obstacle_affects_team("exit", "통로 막힘: ZIN03", -6.875)
    assert obstacle_affects_team("exit", "통로 막힘: ZOUT02", 7.075)
    assert not obstacle_affects_team("entry", "통로 막힘: ZOUT02", 7.075)


def test_location_is_fallback_when_description_has_no_zone():
    assert obstacle_scope("장애물 감지", -6.875) == "entry"
    assert obstacle_scope("장애물 감지", 6.875) == "exit"


def test_unknown_or_cross_team_alert_fails_safe_to_all_teams():
    for role in ("entry", "exit"):
        assert obstacle_affects_team(role, "장애물 감지", 0.0)
        assert obstacle_affects_team(
            role, "통로 막힘: ZIN03, ZOUT01", 0.0
        )


def test_robot_id_role_mapping():
    assert robot_team_role("entry_lead") == "entry"
    assert robot_team_role("entry_follow") == "entry"
    assert robot_team_role("exit_lead") == "exit"
    assert robot_team_role("exit_follow") == "exit"
    assert robot_team_role("robot_1") is None
