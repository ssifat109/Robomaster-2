import math
import numpy as np
import pytest
from rm_flatness_nav.core import (solve_flat_polynomial, evaluate, flatness_inverse,
                                  horizon, wheel_rates, motor_voltages, Supervisor, Observation)


@pytest.mark.parametrize('distance', [0.21, 0.30, 0.8, 1.5, 1.65, 1.8, 2.0])
def test_boundaries_and_speed(distance):
    T = horizon(distance)
    a = solve_flat_polynomial([0, 0], [.06, 0], [distance-.2, 0], [0, 0], T)
    p0, v0, _ = evaluate(a, 0)
    pf, vf, _ = evaluate(a, T)
    np.testing.assert_allclose(p0, [0, 0], atol=1e-12)
    np.testing.assert_allclose(v0, [.06, 0], atol=1e-12)
    np.testing.assert_allclose(pf, [distance-.2, 0], atol=1e-12)
    np.testing.assert_allclose(vf, [0, 0], atol=1e-12)
    speeds = [flatness_inverse(*evaluate(a, t)[1:])[0] for t in np.linspace(0, T, 1001)]
    assert max(speeds) <= .16+1e-12


def test_circular_inversion_and_zero_endpoint():
    assert flatness_inverse([0, 2], [-4, 0]) == (2, 2)
    assert flatness_inverse([0, 0], [1, 2]) == (0, 0)


@pytest.mark.parametrize('T', [0, -1, float('nan'), float('inf')])
def test_bad_horizon(T):
    with pytest.raises(ValueError):
        solve_flat_polynomial([0, 0], [0, 0], [1, 0], [0, 0], T)


def test_wheels_and_voltage_transient():
    np.testing.assert_allclose(wheel_rates(.1, 0), [2]*4)
    np.testing.assert_allclose(wheel_rates(0, .3), [1.2, -1.2, -1.2, 1.2])
    np.testing.assert_allclose(motor_voltages([2]*4, [2]*4, 1), [.76]*4)
    np.testing.assert_allclose(motor_voltages([0]*4, [2]*4, 1), [-.12]*4)
    np.testing.assert_allclose(motor_voltages([0]*4, [0]*4, 1), [0]*4)


def test_search_align_chase_loss():
    s = Supervisor()
    assert s.step(0, None) == (0, .3)
    assert s.step(1, Observation(1, 1.6, -.4)) == (0, -.3)
    assert s.state == 'ALIGNING'
    assert s.step(1.1, Observation(1.1, 1.6, 0)) == (0, 0)
    assert s.step(1.2, Observation(1.2, 1.6, 0))[0] > 0
    assert s.state == 'CHASING'
    assert s.step(2, Observation(1.2, 1.6, 0)) == (0, -.3)
    assert s.state == 'SEARCHING'


@pytest.mark.parametrize('tof,distance', [(1, .2), (.2, 1), (.19, 1)])
def test_priority_guard_and_latch(tof, distance):
    s = Supervisor()
    assert s.step(0, Observation(0, distance, 1), tof) == (0, 0)
    assert s.state == 'ARRIVED'
    assert s.step(1, None, 2) == (0, 0)


def test_stale_and_invalid_sensors():
    s = Supervisor()
    assert s.step(1, Observation(0, .1, 0)) == (0, .3)
    assert s.step(1, None, healthy=False) == (0, 0)
    assert s.state == 'SENSOR_HOLD'
    assert s.step(2, Observation(2, float('nan'), 0)) == (0, .3)


def test_missed_endpoint_is_not_success():
    s = Supervisor()
    s.step(0, Observation(0, 1.65, 0))
    assert s.step(20, Observation(20, .3, 0)) == (0, 0)
    assert s.state == 'STOPPED_SHORT'


def test_close_reacquisition_cannot_follow_reversing_cubic():
    s = Supervisor()
    assert s.step(0, Observation(0, .23, 0)) == (0, 0)
    assert s.state == 'STOPPED_SHORT'
    assert s.reason == 'short_cubic_would_reverse'
