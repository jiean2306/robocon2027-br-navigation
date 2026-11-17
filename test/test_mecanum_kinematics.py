from br_nav_sim.mecanum_kinematics import inverse_kinematics, forward_kinematics


def test_round_trip():
    params = (0.20, -0.10, 0.30, 0.075, 0.20, 0.20)
    wheels = inverse_kinematics(*params)
    result = forward_kinematics(*wheels, 0.075, 0.20, 0.20)
    assert all(abs(a-b) < 1e-9 for a, b in zip(result, params[:3]))


def test_forward_only_equal_wheel_speeds():
    wheels = inverse_kinematics(0.3, 0.0, 0.0, 0.075, 0.2, 0.2)
    assert max(wheels) - min(wheels) < 1e-9
