"""First scan-alignment exercise. Run this file directly; no ROS nodes needed."""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path


def transform_points(points, tx, ty, yaw):
    """Express source-frame points in the target frame.

    points: [[x, y], ...], measured in metres in the later scan's frame.
    tx, ty: location of that source frame's origin in the earlier scan's frame.
    yaw: source frame's counterclockwise heading in the target frame, in radians.

    Fill ONLY the two None expressions below. Input points must stay unchanged.
    c and s are the cosine and sine of that heading.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    transformed = []
    for x, y in points:
        x_target = c*x-s*y+tx  # TODO: source point's x coordinate in the target frame
        y_target = s*x+c*y+ty # TODO: source point's y coordinate in the target frame
        if x_target is None or y_target is None:
            raise NotImplementedError('Fill x_target and y_target in transform_points first.')
        transformed.append([x_target, y_target])
    return transformed


def check_exercise():
    """Small cases with known answers; real scan overlap is a separate observation."""
    cases = [
        ('translation', [[1., 2.]], (2., -1., 0.), [[3., 1.]]),
        ('quarter turn', [[1., 0.], [0., 1.]], (2., 1., math.pi/2), [[2., 2.], [1., 1.]]),
        ('clockwise turn', [[2., 0.]], (0., 0., -math.pi/2), [[0., -2.]]),
        ('empty scan', [], (1., 2., .3), []),
    ]
    for name, points, pose, expected in cases:
        original = deepcopy(points)
        actual = transform_points(points, *pose)
        assert points == original, f'{name}: input was changed'
        assert len(actual) == len(expected), f'{name}: wrong point count'
        for received, wanted in zip(actual, expected):
            assert len(received) == 2, f'{name}: each point needs x and y'
            assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(received, wanted)), (
                f'{name}: expected {expected}, got {actual}')
        print(f'PASS: {name}')


def write_plot(path, target, source, transformed=None):
    """Write an SVG using only the Python standard library."""
    all_points = target+source+(transformed or [])
    xmin = min(p[0] for p in all_points)-.15
    xmax = max(p[0] for p in all_points)+.15
    ymin = min(p[1] for p in all_points)-.15
    ymax = max(p[1] for p in all_points)+.15
    scale = min(410/(xmax-xmin), 410/(ymax-ymin))
    panels = [(target, source, '#d35400', 'Before: each scan uses its own frame')]
    if transformed is not None:
        panels.append((target, transformed, '#16805d', 'After: both expressed in the earlier frame'))
    width = 500*len(panels)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} 530">',
             '<rect width="100%" height="100%" fill="white"/>',
             '<g font-family="sans-serif" font-size="14">']
    for i, (fixed, moving, color, title) in enumerate(panels):
        left = 45+500*i
        parts.append(f'<text x="{left}" y="28">{title}</text>')
        for points, ink in ((fixed, '#2469a0'), (moving, color)):
            for x, y in points:
                px = left+(x-xmin)*scale
                py = 465-(y-ymin)*scale
                parts.append(f'<circle cx="{px:.3f}" cy="{py:.3f}" r="1.8" fill="{ink}" opacity=".7"/>')
        parts.extend([
            f'<text x="{left}" y="490" fill="#2469a0">Blue: earlier scan</text>',
            f'<text x="{left}" y="511" fill="{color}">Orange/green: later scan; coordinates in metres</text>',
        ])
    parts.append('</g></svg>')
    path.write_text('\n'.join(parts), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview', action='store_true', help='See raw scans before filling the exercise')
    args = parser.parse_args()
    folder = Path(__file__).resolve().parent
    data = json.loads((folder/'scan_pair.json').read_text(encoding='utf-8'))
    target = data['target_earlier']['points_m']
    source = data['source_later']['points_m']
    pose = data['source_pose_in_target_xy_yaw']
    print(f'Recorded simulation: {data["session"]}; points {len(target)}/{len(source)}')
    print(f'Later scan origin in earlier frame: x={pose[0]:.4f}m, y={pose[1]:.4f}m, yaw={math.degrees(pose[2]):.2f}deg')
    transformed = None
    if not args.preview:
        try:
            check_exercise()
        except (NotImplementedError, AssertionError, TypeError, ValueError) as error:
            print(f'EXERCISE: {error}')
            return 2
        transformed = transform_points(source, *pose)
    output = folder/('scan_preview.svg' if args.preview else 'scan_alignment.svg')
    write_plot(output, target, source, transformed)
    print(f'PLOT: {output}')
    print('This only changes coordinates using odom. No matching, ICP or SLAM correction has run.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
