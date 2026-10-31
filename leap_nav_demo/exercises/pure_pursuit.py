"""Pure Pursuit geometry exercise. No ROS, simulation or motion output."""
import math


def angular_velocity(lookahead_xy: tuple[float, float], forward_speed_mps: float) -> float:
    """Return angular velocity in rad/s for a given forward speed in m/s.

    lookahead_xy is (x, y) in METRES, already expressed in the ROBOT frame:
    x is forward; y is left. Example: (0.4, 0.2) means 40cm ahead, 20cm left.
    This exercise accepts x >= 0 and forward speed >= 0.

    Geometry: curvature = 2*y / (x*x + y*y); angular velocity = speed * curvature.
    Example: (0.4, 0.2), speed 0.1 -> curvature 2.0/m -> angular velocity +0.2 rad/s.
    Positive angular velocity means a left turn.

    Only the geometric term is implemented here. Full Nav2 RPP also selects the
    lookahead point, transforms frames, regulates speed, checks collision and
    handles heading rotation. This file never sends a robot command.
    """
    forward_x_m, left_y_m = lookahead_xy
    if not all(math.isfinite(value) for value in (forward_x_m, left_y_m, forward_speed_mps)):
        raise ValueError('Coordinates and speed must be finite')
    if forward_x_m < 0 or forward_speed_mps < 0:
        raise ValueError('This forward-only exercise requires x >= 0 and speed >= 0')
    distance_squared_m2 = forward_x_m**2 + left_y_m**2
    # Humble RPP suppresses the curvature term when distance squared <= 0.001 m^2.
    # Returning zero angular velocity here is not a robot stopping policy.
    if distance_squared_m2 <= 0.001:
        return 0.0
    # TODO 1: assign curvature_per_m using left_y_m and distance_squared_m2.
    curvature_per_m=2*left_y_m/distance_squared_m2
    # TODO 2: assign angular_velocity_rps using forward_speed_mps and curvature_per_m.
    angular_velocity_rps=forward_speed_mps*curvature_per_m
    return angular_velocity_rps


def check_examples():
    cases = [
        ('straight ahead', (0.5, 0.0), 0.1, 0.0),
        ('left turn', (0.4, 0.2), 0.1, 0.2),
        ('right turn', (0.4, -0.2), 0.1, -0.2),
        ('farther point at same bearing', (0.8, 0.4), 0.1, 0.1),
        ('twice the speed at same point', (0.4, 0.2), 0.2, 0.4),
        ('zero speed', (0.4, 0.2), 0.0, 0.0),
        ('near-zero point', (0.0, 0.0), 0.1, 0.0),
    ]
    for name, point, speed, expected in cases:
        result = angular_velocity(point, speed)
        assert math.isclose(result, expected, abs_tol=1e-9), f'{name}: expected {expected}, got {result}'
        print(f'PASS: {name}; angular velocity = {result:.3f} rad/s')


def main():
    try:
        check_examples()
    except (NotImplementedError, AssertionError, TypeError, ValueError) as error:
        print(f'EXERCISE: {error}')
        return 2
    print('Geometry only; no ROS node, full RPP controller or robot motion has run.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
