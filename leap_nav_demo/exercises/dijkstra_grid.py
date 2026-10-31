"""Weighted four-neighbour grid lesson. Offline only; does not replace NavFn."""
import argparse
from copy import deepcopy
import heapq
import json
import math
from pathlib import Path

Cell = tuple[int, int]  # (column, row), both zero-based; grid[row][column].


def relax_neighbor(current: Cell, neighbor: Cell, step_cost: float,
                   cost_so_far: dict, came_from: dict, frontier: list) -> bool:
    """Update ONE candidate route in place; return True only if it improves.

    Complete example of the six inputs:
        current = (0, 0)
        neighbor = (1, 0)
        step_cost = 3
        cost_so_far = {(0, 0): 4, (1, 0): 10}
        came_from = {}
        frontier = []

    New candidate cost is 4 + 3 = 7, cheaper than 10. Required updates:
        cost_so_far[(1, 0)] = 7
        came_from[(1, 0)] = (0, 0)
        heapq.heappush(frontier, (7, (1, 0)))
        return True

    Rules:
    - candidate = recorded cost of current + step_cost;
    - get an unseen neighbor's old cost with dict.get(neighbor, math.inf);
    - ONLY a strictly smaller candidate updates all three containers;
    - equal/worse candidates return False and change nothing;
    - frontier stores (total_cost, cell), not (step_cost, cell).
    heapq.heappush adds an item; heappop removes the smallest-cost item.
    """
    # TODO: implement the rules above. Replace this raise with your code.
    candidate = step_cost+cost_so_far[current]
    old_cost=cost_so_far.get(neighbor, math.inf)
    if candidate<old_cost:
        cost_so_far[neighbor]=candidate
        came_from[neighbor]=current
        heapq.heappush(frontier,(candidate,neighbor))
        return True
    else:
        return False



def dijkstra(grid: list[list], start: Cell, goal: Cell):
    """Return (path, total_cost, expansion_order); no path gives ([], inf, ...).

    Numbers are positive entry costs; None is blocked. Start costs zero.
    Four directions only; cells are array indices, NOT ROS map coordinates.
    """
    if not grid or not grid[0] or any(len(row) != len(grid[0]) for row in grid):
        raise ValueError('Use a nonempty rectangular grid')
    if any(value is not None and (not math.isfinite(value) or value <= 0)
           for row in grid for value in row):
        raise ValueError('Entry costs must be positive and finite, or None for blocked')
    height, width = len(grid), len(grid[0])
    for x, y in (start, goal):
        if not (0 <= x < width and 0 <= y < height):
            raise ValueError('Start and goal must be inside the grid')
        if grid[y][x] is None:
            return [], math.inf, []
    frontier = [(0, start)]
    cost_so_far, came_from, expanded = {start: 0}, {}, []
    while frontier:
        total_cost, current = heapq.heappop(frontier)
        if total_cost > cost_so_far[current]:
            continue  # An earlier, more expensive queue item became stale.
        expanded.append((current, total_cost))
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            return list(reversed(path)), total_cost, expanded
        x, y = current
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            nx, ny = x+dx, y+dy
            if 0 <= nx < width and 0 <= ny < height and grid[ny][nx] is not None:
                relax_neighbor(current, (nx, ny), grid[ny][nx], cost_so_far, came_from, frontier)
    return [], math.inf, expanded


def remaining_cost_estimate(cell: Cell, goal: Cell, minimum_step_cost: float) -> float:
    """Manhattan distance times the cheapest step: a lower bound for this grid."""
    return (abs(cell[0]-goal[0]) + abs(cell[1]-goal[1])) * minimum_step_cost


def astar(grid: list[list], start: Cell, goal: Cell):
    """Reference A* for comparison with the unchanged Dijkstra teaching loop.

    Queue items explicitly store (f, g, cell). Only g enters cost_so_far.
    h uses the cheapest entry cost, so fractional positive costs also work.
    """
    if not grid or not grid[0] or any(len(row) != len(grid[0]) for row in grid):
        raise ValueError('Use a nonempty rectangular grid')
    if any(value is not None and (not math.isfinite(value) or value <= 0)
           for row in grid for value in row):
        raise ValueError('Entry costs must be positive and finite, or None for blocked')
    height, width = len(grid), len(grid[0])
    for x, y in (start, goal):
        if not (0 <= x < width and 0 <= y < height):
            raise ValueError('Start and goal must be inside the grid')
        if grid[y][x] is None:
            return [], math.inf, []
    minimum_step_cost = min(value for row in grid for value in row if value is not None)
    frontier = [(remaining_cost_estimate(start, goal, minimum_step_cost), 0, start)]
    cost_so_far, came_from, expanded = {start: 0}, {}, []
    while frontier:
        priority_f, current_g, current = heapq.heappop(frontier)
        if current_g > cost_so_far[current]:
            continue
        expanded.append((current, current_g))
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            return list(reversed(path)), current_g, expanded
        x, y = current
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            nx, ny = x+dx, y+dy
            if not (0 <= nx < width and 0 <= ny < height) or grid[ny][nx] is None:
                continue
            neighbor = (nx, ny)
            candidate_g = current_g + grid[ny][nx]
            if candidate_g < cost_so_far.get(neighbor, math.inf):
                cost_so_far[neighbor] = candidate_g
                came_from[neighbor] = current
                priority_f = candidate_g + remaining_cost_estimate(neighbor, goal, minimum_step_cost)
                heapq.heappush(frontier, (priority_f, candidate_g, neighbor))
    return [], math.inf, expanded


def check_astar_examples():
    for name, grid, start, goal in (
        ('weighted detour', GRID, START, GOAL),
        ('unreachable', [[1, None, 1]], (0,0), (2,0)),
        ('same cell', [[1]], (0,0), (0,0)),
        ('fractional costs', [[.2, .8, .2], [.2, .2, .2]], (0,0), (2,0)),
    ):
        expected_cost = dijkstra(grid, start, goal)[1]
        path, actual_cost, _ = astar(grid, start, goal)
        assert math.isclose(actual_cost, expected_cost), name
        if path:
            assert path[0] == start and path[-1] == goal, name
            assert all(abs(a[0]-b[0])+abs(a[1]-b[1]) == 1 for a,b in zip(path,path[1:])), name
            assert all(grid[y][x] is not None for x,y in path), name
            assert math.isclose(sum(grid[y][x] for x,y in path[1:]), actual_cost), name
        print(f'PASS: A* {name}')


def write_comparison_svg(grid, start, goal, results, output: Path):
    """Write a portable diagram using only the standard library."""
    cell, margin, top = 22, 40, 62
    rows, cols = len(grid), len(grid[0])
    panel_width = cols*cell + margin
    width, height = 2*panel_width + margin, top + rows*cell + 76
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="white"/>',
           '<g font-family="sans-serif" font-size="14">']
    for index, result in enumerate(results):
        left = margin + index*panel_width
        visited = {tuple(item[0]) for item in result['expanded']}
        svg.append(f'<text x="{left}" y="25" font-size="19">{result["algorithm"]}</text>')
        svg.append(f'<text x="{left}" y="46">Expanded: {len(visited)} | Cost: {result["cost"]} | Moves: {len(result["path"])-1}</text>')
        for y, row in enumerate(grid):
            for x, value in enumerate(row):
                fill = '#343a40' if value is None else '#d3e8f3' if (x,y) in visited else '#ffffff'
                svg.append(f'<rect x="{left+x*cell}" y="{top+y*cell}" width="{cell}" height="{cell}" fill="{fill}" stroke="#d1d7dc"/>')
        points = ' '.join(f'{left+(x+.5)*cell},{top+(y+.5)*cell}' for x,y in result['path'])
        svg.append(f'<polyline points="{points}" fill="none" stroke="#d55e00" stroke-width="3"/>')
        for label, point, color in (('S', start, '#008c69'), ('G', goal, '#a34f8d')):
            x, y = point
            svg.append(f'<rect x="{left+x*cell+1}" y="{top+y*cell+1}" width="{cell-2}" height="{cell-2}" fill="{color}"/>')
            svg.append(f'<text x="{left+(x+.5)*cell}" y="{top+(y+.72)*cell}" text-anchor="middle" fill="white">{label}</text>')
    svg.extend([f'<text x="40" y="{height-43}">Blue: expanded | Orange: path | Dark: obstacle | White: not expanded</text>',
                f'<text x="40" y="{height-20}">Array grid; four directions; entry cost 1. This is a teaching example, not a Nav2 benchmark.</text>', '</g></svg>'])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(svg), encoding='utf-8')


def compare_searches(output: Path):
    grid = [[1 for _ in range(20)] for _ in range(13)]
    for y in range(10):
        grid[y][10] = None
    start, goal = (1,6), (18,6)
    results = []
    for name, search in (('Dijkstra', dijkstra), ('A*', astar)):
        path, cost, expanded = search(grid, start, goal)
        assert path and cost == 25, name
        results.append({'algorithm': name, 'path': path, 'cost': cost, 'expanded': expanded})
        print(f'{name}: cost={cost}, moves={len(path)-1}, expanded={len(expanded)}')
    if output.suffix.lower() != '.svg':
        raise ValueError('--output must end in .svg')
    write_comparison_svg(grid, start, goal, results, output)
    output.with_suffix('.json').write_text(json.dumps({
        'grid': grid, 'start': start, 'goal': goal, 'results': results,
        'limits': 'One teaching grid, not a Nav2 benchmark; tie breaking and heuristic affect expansions.'}, indent=2), encoding='utf-8')
    print('PLOT:', output)
    print('A* orders by f=g+h; cost_so_far still records only g. No ROS or motion output.')


def check_examples():
    current, neighbor = (0, 0), (1, 0)
    for name, old_cost, improved in (
        ('unseen neighbor', None, True), ('cheaper route', 10, True),
        ('equal route keeps old parent', 7, False), ('more expensive route', 5, False),
    ):
        costs = {current: 4}
        parents, queue = {}, []
        if old_cost is not None:
            costs[neighbor] = old_cost
            parents[neighbor] = (2, 0)
            queue.append((old_cost, neighbor))
        before = deepcopy((costs, parents, queue))
        assert relax_neighbor(current, neighbor, 3, costs, parents, queue) is improved, name
        if improved:
            assert costs == {current: 4, neighbor: 7}, name
            assert parents == {neighbor: current}, name
            assert heapq.heappop(queue) == (7, neighbor), name
        else:
            assert (costs, parents, queue) == before, name
        print(f'PASS: {name}')
    path, cost, _ = dijkstra(GRID, START, GOAL)
    assert path[0] == START and path[-1] == GOAL and len(path)-1 == 6 and cost == 6
    assert all(abs(a[0]-b[0])+abs(a[1]-b[1]) == 1 for a,b in zip(path,path[1:]))
    assert sum(GRID[y][x] for x,y in path[1:]) == cost
    print('PASS: cheaper detour wins over the shorter expensive route')
    assert dijkstra([[1, None, 1]], (0,0), (2,0))[:2] == ([], math.inf)
    print('PASS: unreachable goal')
    assert dijkstra([[1]], (0,0), (0,0))[:2] == ([(0,0)], 0)
    print('PASS: start equals goal')


GRID = [[1, 1, 1, 1, 1],
        [1, 8, 8, 8, 1],
        [None, None, None, None, None]]
START, GOAL = (0, 1), (4, 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lesson', action='store_true', help='Show the priority queue example before filling the exercise')
    parser.add_argument('--compare', action='store_true', help='Run Dijkstra/A* on the same grid and save a diagram')
    parser.add_argument('--output', type=Path, default=Path.home()/'leap_nav_data/reports/astar_comparison.svg')
    args = parser.parse_args()
    if args.lesson:
        queue = []
        for item in ((8, (1,1)), (1, (0,0)), (3, (2,0))):
            heapq.heappush(queue, item)
        while queue:
            total_cost, cell = heapq.heappop(queue)
            print(f'NEXT: total_cost={total_cost}, cell={cell}')
        print('The queue selects cost 1, then 3, then 8. It does not use insertion order.')
        return 0
    try:
        check_examples()
        if args.compare:
            check_astar_examples()
            compare_searches(args.output.expanduser())
            return 0
        path, cost, expanded = dijkstra(GRID, START, GOAL)
    except (NotImplementedError, AssertionError, KeyError, TypeError, ValueError) as error:
        print(f'EXERCISE: {error}')
        return 2
    print('GRID: numbers are entry costs; # is blocked; entering START costs zero.')
    for y, row in enumerate(GRID):
        print(' '.join('S' if (x,y)==START else 'G' if (x,y)==GOAL else
                       '#' if value is None else str(value) for x,value in enumerate(row)))
    print('EXPANDED:', expanded)
    print('PATH:', path)
    print(f'COST: {cost}; MOVES: {len(path)-1}; direct middle route: cost 25, moves 4.')
    print('Educational Dijkstra only; no ROS node, NavFn replacement or motion output.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
