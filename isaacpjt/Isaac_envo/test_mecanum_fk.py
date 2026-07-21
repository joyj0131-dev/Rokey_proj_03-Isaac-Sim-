#!/usr/bin/env python3
"""IK→FK 왕복이 항등이어야 한다 (순수 파이썬, Isaac 불필요)."""
from mecanum_drive import wheel_velocities_from_cmd_vel, cmd_vel_from_wheel_velocities


def test_fk_roundtrip():
    for cmd in ((0.4, 0.0, 0.0), (0.0, 0.3, 0.0), (0.0, 0.0, 0.5), (0.2, -0.1, 0.3)):
        omegas = wheel_velocities_from_cmd_vel(*cmd)
        back = cmd_vel_from_wheel_velocities(omegas)
        for a, b in zip(cmd, back):
            assert abs(a - b) < 1e-9, f"{cmd} -> {back}"


if __name__ == "__main__":
    test_fk_roundtrip()
    print("FK_ROUNDTRIP=PASS")
