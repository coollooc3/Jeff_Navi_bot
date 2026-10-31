"""Third exercise: translation-only alignment with a fixed initial yaw."""
import json
import math
from pathlib import Path
from statistics import median

from nearest_neighbor import nearest_point
from point_transform import transform_points


def estimate_translation(pairs):
    """Return the mean (target-source) difference as (dx, dy), in metres.

    pairs is [((source_x, source_y), (target_x, target_y)), ...].
    Both points are already in the EARLIER scan frame.
    This is the least-squares translation for these FIXED correspondences.
    Empty input must raise ValueError. Do not change the inputs.
    """
    if not pairs:
        raise ValueError('No candidate correspondences')
    # TODO: sum target-source in X and Y, then divide each sum by len(pairs).
    # Replace this raise with your implementation, retaining the empty-input guard.
    raise NotImplementedError('Complete estimate_translation: return dx, dy.')


def check_exercise():
    cases = [
        ('remove X shift', [((1.1, 1.), (1., 1.)), ((2.1, 2.), (2., 2.))], (-.1, 0.)),
        ('both axes', [((1., 2.), (3., 1.)), ((4., 5.), (6., 4.))], (2., -1.)),
        ('average, not last pair', [((0., 0.), (1., 2.)), ((1., 1.), (4., 5.))], (2., 3.)),
    ]
    for name, pairs, wanted in cases:
        before = repr(pairs)
        received = estimate_translation(pairs)
        assert repr(pairs) == before, f'{name}: input changed'
        assert len(received) == 2
        assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(received, wanted)), (
            f'{name}: expected {wanted}, got {received}')
        print(f'PASS: {name}')
    try:
        estimate_translation([])
    except ValueError:
        print('PASS: empty pairs rejected')
    else:
        raise AssertionError('Empty pairs must raise ValueError')


def match_points(source, reference, pose):
    points = transform_points(source, *pose)
    candidates = [(point, *nearest_point(point, reference)) for point in points]
    # This is a correspondence gate for THIS offline exercise, not a robot safety distance.
    pairs = [(point, target) for point, target, squared in candidates if squared <= .20**2]
    return pairs, {
        'all_candidate_rms_cm': 100*math.sqrt(sum(d2 for _, _, d2 in candidates)/len(candidates)),
        'median_candidate_cm': 100*median(math.sqrt(d2) for _, _, d2 in candidates),
        'retained_pairs': len(pairs), 'total_pairs': len(candidates),
    }


def main():
    try:
        check_exercise()
    except (NotImplementedError, AssertionError, TypeError, ValueError) as error:
        print(f'EXERCISE: {error}')
        return 2
    folder = Path(__file__).resolve().parent
    data = json.loads((folder/'scan_pair.json').read_text(encoding='utf-8'))
    reference = data['target_earlier']['points_m']
    source = data['source_later']['points_m']
    odom_pose = data['source_pose_in_target_xy_yaw']
    pose = [odom_pose[0]+.10, odom_pose[1], odom_pose[2]]
    history = []
    for iteration in range(20):
        pairs, stats = match_points(source, reference, pose)
        dx, dy = estimate_translation(pairs)
        if not all(math.isfinite(v) for v in (dx, dy)):
            raise ValueError('Correction must be finite')
        print(f'iteration {iteration+1:02}: pairs={len(pairs)}, dx={dx*100:+.3f}cm, dy={dy*100:+.3f}cm')
        history.append({'pose_before': pose.copy(), 'correction_m': [dx, dy], **stats})
        # Corrections are expressed in the earlier frame, so add directly to its X/Y.
        pose[0] += dx
        pose[1] += dy
        if math.hypot(dx, dy) < 1e-5:
            break
    _, final_stats = match_points(source, reference, pose)
    result = {'session': data['session'], 'simulation_only': True, 'yaw_fixed': True,
              'correspondence_gate_m': .20, 'odom_initial_pose': odom_pose,
              'artificial_start_pose': [odom_pose[0]+.10, odom_pose[1], odom_pose[2]],
              'refined_pose': pose, 'iterations': history, 'final_statistics': final_stats,
              'limit': 'Translation only; candidate RMS is not pose error; odom is not ground truth.'}
    output = folder/'translation_result.json'
    output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(f'X difference from original odom guess: {(pose[0]-odom_pose[0])*100:+.3f}cm (started at +10cm)')
    print(f'Final candidate RMS (all points): {final_stats["all_candidate_rms_cm"]:.3f}cm')
    print(f'SAVED: {output}')
    print('Translation only. Yaw was held fixed; original odom is NOT ground truth.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
