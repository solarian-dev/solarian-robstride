"""zero_sta: the window a motor's position falls in after power-up."""

from __future__ import annotations

from test_motion_control import clock, make_bus  # noqa: F401  (clock is a fixture)


def test_motors_on_the_factory_window_are_switched_and_saved(clock):
    bus, motors = make_bus({1: 0.0, 2: 0.0, 3: 0.0})
    motors.zero_stas[2] = 1

    assert bus.centre_power_up_positions(["j1", "j2", "j3"]) == ["j1", "j3"]
    assert motors.zero_stas == {1: 1, 2: 1, 3: 1}
    assert motors.saves == {1: 1, 2: 0, 3: 1}


def test_a_second_pass_changes_nothing(clock):
    bus, motors = make_bus({1: 0.0})
    bus.centre_power_up_positions(["j1"])

    assert bus.centre_power_up_positions(["j1"]) == []
    assert motors.saves == {1: 1}
