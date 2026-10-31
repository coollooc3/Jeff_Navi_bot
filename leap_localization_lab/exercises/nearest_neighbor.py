"""Second exercise: choose candidate correspondences; do not optimize poses yet."""
from copy import deepcopy
import json
import math
from pathlib import Path

from point_transform import transform_points


def nearest_point(point, reference_points):
    """Return (nearest_xy, squared_distance); an empty reference gives (None, inf).

    All coordinates must already be expressed in the SAME frame, in metres.
    nearest_xy is an (x, y) tuple. Keep the first candidate in an exact tie.
    Do not modify the input points.
    """
    px, py = point
    best_point = None
    best_distance_squared = math.inf
    for qx, qy in reference_points:
        distance_squared=(px-qx)**2+(py-qy)**2
        if distance_squared<best_distance_squared:
            best_point=(qx,qy)
            best_distance_squared=distance_squared
        # TODO 1: compute the squared distance from (px, py) to (qx, qy).
        # TODO 2: if it is smaller, update BOTH best_point and best_distance_squared.
        # Replace the following raise with your implementation.

    return best_point, best_distance_squared


def check_exercise():
    cases = [
        ('unordered candidates', [1.1, 1.], [[2., 1.], [1., 1.]], (1., 1.), .01),
        ('negative coordinates', [-2., -3.], [[-1., -1.], [-2., -3.]], (-2., -3.), 0.),
        ('first candidate wins tie', [0., 0.], [[-1., 0.], [1., 0.]], (-1., 0.), 1.),
        ('empty reference', [0., 0.], [], None, math.inf),
    ]
    for name, point, reference, wanted_point, wanted_squared in cases:
        original = deepcopy((point, reference))
        matched, squared = nearest_point(point, reference)
        assert (point, reference) == original, f'{name}: input changed'
        assert matched == wanted_point, f'{name}: expected point {wanted_point}, got {matched}'
        assert math.isclose(squared, wanted_squared, abs_tol=1e-9), (
            f'{name}: expected squared distance {wanted_squared}, got {squared}')
        print(f'PASS: {name}')


def main():
    try:
        check_exercise()
    except (NotImplementedError, AssertionError, TypeError, ValueError) as error:
        print(f'EXERCISE: {error}')
        return 2
    data = json.loads((Path(__file__).resolve().parent/'scan_pair.json').read_text(encoding='utf-8'))
    reference = data['target_earlier']['points_m']
    source = data['source_later']['points_m']
    odom_pose = data['source_pose_in_target_xy_yaw']
    # A declared artificial change to the initial guess, not another robot run.
    perturbed_pose = [odom_pose[0]+.10, odom_pose[1], odom_pose[2]+math.radians(5.)]
    for label, pose in (('odom initial guess', odom_pose), ('initial guess +10cm X / +5deg yaw', perturbed_pose)):
        transformed = transform_points(source, *pose)
        pairs = [nearest_point(point, reference) for point in transformed]
        rms = math.sqrt(sum(squared for _, squared in pairs)/len(pairs))
        print(f'{label}: {len(pairs)} candidate pairs, nearest-point RMS = {rms*100:.3f} cm')
    print('These are candidate distances, NOT pose errors. No outlier rejection or pose correction has run.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
