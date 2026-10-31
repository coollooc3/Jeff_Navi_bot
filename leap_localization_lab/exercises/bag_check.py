"""Read bag files offline; optionally inspect patrol status. No ROS node is created."""
import argparse
from copy import deepcopy
import json
from pathlib import Path


PATROL_TERMINAL_STATES = ('COMPLETED', 'CANCELED', 'FAILED')


def new_topic_stats() -> dict:
    """Create the dictionary passed to update_topic_stats for ONE topic."""
    return {'count': 0, 'first_record_ns': None,
            'last_record_ns': None, 'max_gap_ns': None}


def update_topic_stats(stats: dict, record_time_ns: int) -> None:
    """Update ONE topic's dictionary in place when ONE message is read.

    stats starts as:
        {'count': 0, 'first_record_ns': None,
         'last_record_ns': None, 'max_gap_ns': None}
    record_time_ns is an integer, e.g. 1_100_000_000 means 1.1 seconds.

    Required behavior:
    - count every message;
    - set first_record_ns only for the first message;
    - update last_record_ns for every message;
    - max_gap_ns is None until there are at least TWO messages;
    - compare the new time with the PREVIOUS last_record_ns;
    - equal timestamps are allowed (their gap is zero).
    Do not return a new dictionary; update the given stats and return None.
    """
    if not isinstance(record_time_ns, int):
        raise TypeError('Recording timestamp must be integer nanoseconds')
    if stats['last_record_ns'] is not None and record_time_ns < stats['last_record_ns']:
        raise ValueError('Recording timestamps moved backwards within one topic')
    # TODO: implement the count, first/last timestamp and maximum-gap updates.
    # Replace this raise. Keep the validation above.
    previous=stats['last_record_ns']

    if previous is None:
        stats['first_record_ns']=record_time_ns

    else:
        gap_ns=record_time_ns-previous

        if stats['max_gap_ns']is None or gap_ns>stats['max_gap_ns']:
            stats['max_gap_ns']=gap_ns
    stats['count']+=1
    stats['last_record_ns']=record_time_ns


def find_missing_topics(topic_stats: dict, required_names: list[str]) -> list[str]:
    """Return required topics that are missing OR have count == 0.

    Example inputs:
        topic_stats = {'/scan': {'count': 3}, '/odom': {'count': 0}}
        required_names = ['/scan', '/odom', '/map']
    Expected result: ['/odom', '/map'].

    Keep required_names order; do not change either input.
    Other fields in each topic's statistics can be ignored for this check.
    """
    missing = []
    for name in required_names:
        # TODO: append name when it is absent, or when its count is zero.
        # Replace this raise with your implementation.
        if name not in topic_stats or name in topic_stats and topic_stats[name]['count']==0:
            missing.append(name)

    return missing


def parse_patrol_status(status_text: str) -> dict:
    """Read the JSON text stored in ONE std_msgs/msg/String.data.

    Example input (a str, NOT a dictionary):
        '{"state":"CANCELED","current_waypoint":"A","completed":0,
          "total":3,"reason":"operator_cancel","goal_unresolved":false}'

    Return the fields below plus is_terminal (a Python bool).
    COMPLETED, CANCELED and FAILED are terminal states.
    IDLE, RUNNING and CANCELING are NOT terminal states.
    A terminal state does not by itself prove that the robot stopped;
    keep goal_unresolved exactly as recorded, even when it is True.
    """
    # TODO 1: turn status_text into a dictionary named status, using json.loads.
    status = json.loads(status_text)
    # TODO 2: assign is_terminal by checking status['state'] against the three terminal states.
    is_terminal=status['state'] in PATROL_TERMINAL_STATES
    return {
        'state': status['state'],
        'current_waypoint': status['current_waypoint'],
        'completed': status['completed'], 'total': status['total'],
        'reason': status['reason'], 'goal_unresolved': status['goal_unresolved'],
        'is_terminal': is_terminal,
    }


def patrol_completion_count_mismatch(status: dict) -> bool:
    """Flag COMPLETED with unequal counts; cancellation/failure may be incomplete."""
    return status['state'] == 'COMPLETED' and status['completed'] != status['total']


def check_patrol_expectation(status: dict | None, user_desired_state: str) -> dict:
    """Check the LAST recorded status without changing it or proving physical stop."""
    if user_desired_state not in PATROL_TERMINAL_STATES:
        raise ValueError('Expected patrol state must be COMPLETED, CANCELED or FAILED')
    if status is not None and not isinstance(status, dict):
        raise ValueError('Patrol status must be a dictionary or None')
    actual_state = None if status is None else status.get('state')
    reason = ''
    if status is None:
        reason = 'patrol_status_missing'
    elif actual_state not in PATROL_TERMINAL_STATES:
        reason = 'patrol_not_terminal'
    elif type(status.get('goal_unresolved')) is not bool:
        reason = 'goal_resolution_unknown'
    elif status['goal_unresolved']:
        reason = 'goal_unresolved'
    elif actual_state != user_desired_state:
        reason = 'state_mismatch'
    elif actual_state == 'COMPLETED':
        counts = (status.get('completed'), status.get('total'))
        if any(type(count) is not int or count < 0 for count in counts):
            reason = 'invalid_completion_counts'
        elif patrol_completion_count_mismatch(status):
            reason = 'completed_count_mismatch'
    return {'expected_state': user_desired_state, 'actual_state': actual_state,
            'passed': not bool(reason), 'reason': reason}


def check_patrol_examples():
    for state in ('IDLE', 'RUNNING', 'CANCELING', 'COMPLETED', 'CANCELED', 'FAILED'):
        payload = {'state': state, 'current_waypoint': 'A', 'completed': 0,
                   'total': 3, 'reason': 'example', 'goal_unresolved': False}
        result = parse_patrol_status(json.dumps(payload))
        expected = {**payload, 'is_terminal': state in ('COMPLETED', 'CANCELED', 'FAILED')}
        assert result == expected, f'{state}: expected {expected}, got {result}'
        assert type(result['is_terminal']) is bool, 'is_terminal must be a bool'
        print(f'PASS: {state}')
    payload['state'] = 'FAILED'
    payload['goal_unresolved'] = True
    assert parse_patrol_status(json.dumps(payload))['goal_unresolved'] is True
    print('PASS: unresolved goal is preserved, not treated as stopped')
    # A later RUNNING message must not inherit a previous terminal result.
    parse_patrol_status(json.dumps(payload))
    payload['state'] = 'RUNNING'
    result = parse_patrol_status(json.dumps(payload))
    assert result['state'] == 'RUNNING' and result['is_terminal'] is False
    print('PASS: later RUNNING is not a previous terminal result')
    cases = [
        ('COMPLETED 3/3 is consistent', 'COMPLETED', 3, 3, False),
        ('COMPLETED 2/3 is inconsistent', 'COMPLETED', 2, 3, True),
        ('COMPLETED 4/3 is inconsistent', 'COMPLETED', 4, 3, True),
        ('CANCELED 0/3 is allowed', 'CANCELED', 0, 3, False),
        ('FAILED 1/3 is allowed', 'FAILED', 1, 3, False),
        ('RUNNING 0/3 is allowed', 'RUNNING', 0, 3, False),
    ]
    for name, state, completed, total, expected in cases:
        status = {'state': state, 'completed': completed, 'total': total}
        before = status.copy()
        assert patrol_completion_count_mismatch(status) is expected, name
        assert status == before, f'{name}: input changed'
        print(f'PASS: {name}')


def check_required_topic_examples():
    cases = [
        ('missing and empty', {'/scan': {'count': 3}, '/odom': {'count': 0}},
         ['/scan', '/odom', '/map'], ['/odom', '/map']),
        ('all required topics have messages', {'/scan': {'count': 3}, '/odom': {'count': 2}},
         ['/odom', '/scan'], []),
        ('no required topics', {}, [], []),
    ]
    for name, topics, required, expected in cases:
        before = deepcopy((topics, required))
        received = find_missing_topics(topics, required)
        assert (topics, required) == before, f'{name}: input changed'
        assert received == expected, f'{name}: expected {expected}, got {received}'
        print(f'PASS: {name}')


def check_exercise():
    cases = [
        ('empty topic', [], (0, None, None, None)),
        ('single message', [1_000_000_000], (1, 1_000_000_000, 1_000_000_000, None)),
        ('largest gap is in the middle', [1_000_000_000, 1_100_000_000, 1_500_000_000, 1_600_000_000],
         (4, 1_000_000_000, 1_600_000_000, 400_000_000)),
        ('equal timestamps are allowed', [1_000_000_000, 1_000_000_000],
         (2, 1_000_000_000, 1_000_000_000, 0)),
    ]
    keys = ('count', 'first_record_ns', 'last_record_ns', 'max_gap_ns')
    for name, times, expected in cases:
        stats = new_topic_stats()
        for timestamp in times:
            assert update_topic_stats(stats, timestamp) is None, 'Update the input dictionary in place'
        actual = tuple(stats[key] for key in keys)
        assert actual == expected, f'{name}: expected {expected}, got {actual}'
        print(f'PASS: {name}')
    stats = new_topic_stats()
    update_topic_stats(stats, 1_000_000_000)
    before = deepcopy(stats)
    try:
        update_topic_stats(stats, 900_000_000)
    except ValueError:
        assert stats == before, 'Rejected samples must not change the statistics'
        print('PASS: backwards timestamp rejected')
    else:
        raise AssertionError('Backwards timestamps must raise ValueError')


def summarize_bag(bag: Path, storage_id: str, include_patrol: bool = False) -> dict:
    # Import here so --check also works without a ROS installation.
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    patrol = {'topic': '/patrol/status', 'messages': 0,
              'last_record_ns': None, 'last_status': None}
    if include_patrol:
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import String
        if '/patrol/status' in types and types['/patrol/status'] != 'std_msgs/msg/String':
            raise ValueError('/patrol/status must have type std_msgs/msg/String')
    by_topic = {name: new_topic_stats() for name in types}
    while reader.has_next():
        topic_name, serialized_data, record_time_ns = reader.read_next()
        update_topic_stats(by_topic[topic_name], record_time_ns)
        if include_patrol and topic_name == '/patrol/status':
            message = deserialize_message(serialized_data, String)
            try:
                last_status = parse_patrol_status(message.data)
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f'Invalid /patrol/status at record_ns={record_time_ns}: {error}') from error
            patrol['messages'] += 1
            patrol['last_record_ns'] = record_time_ns
            patrol['last_status'] = last_status
    topics = {}
    for name, stats in sorted(by_topic.items()):
        gap = stats['max_gap_ns']
        topics[name] = {'type': types[name], **stats,
                        'max_record_gap_s': None if gap is None else gap/1_000_000_000}
    report = {
        'bag': str(bag), 'storage_id': storage_id,
        'total_messages': sum(stats['count'] for stats in by_topic.values()),
        'time_basis': 'Bag recording timestamps, not message header stamps or analysis wall time.',
        'limits': [('Only /patrol/status payloads are decoded; no source freshness check.'
                    if include_patrol else 'No payload decoding or source freshness check.'),
                   'Recording gaps alone do not prove packet loss or network latency.',
                   'Equal recording timestamps do not prove duplicated message payloads.',
                   'The timestamp clock depends on how the bag was recorded; this tool does not identify it.'],
        'topics': topics,
    }
    if include_patrol:
        last = patrol['last_status']
        patrol['completion_count_check'] = None
        if last is not None:
            mismatch = patrol_completion_count_mismatch(last)
            patrol['completion_count_check'] = {
                'passed': not mismatch,
                'reason': 'completed_count_mismatch' if mismatch else '',
            }
        patrol['limits'] = ('Last recorded status only; no stop verification or cancel-latency measurement. '
                            'Completion check only compares counts when state is COMPLETED. '
                            'Exit code reports requested data/expectation checks, not physical stop; '
                            'an expected FAILED result may pass.')
        report['patrol'] = patrol
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?', type=Path, help='Bag directory or storage file')
    parser.add_argument('--storage-id', choices=('mcap', 'sqlite3'), default='mcap')
    parser.add_argument('--output', type=Path, help='Optional JSON report outside the input bag')
    parser.add_argument('--check', action='store_true', help='Run the small timestamp examples only')
    parser.add_argument('--check-required', action='store_true', help='Check the new required-topic exercise only')
    parser.add_argument('--check-patrol', action='store_true', help='Check JSON parsing, terminal states and completion counts without ROS')
    parser.add_argument('--patrol', action='store_true', help='Decode /patrol/status and report its LAST recorded state')
    parser.add_argument('--expect-state', dest='user_desired_state', choices=PATROL_TERMINAL_STATES,
                        help='Require this terminal patrol result; implies --patrol, no physical stop check')
    parser.add_argument('--require', nargs='+', metavar='TOPIC', help='Required topics, each needing at least one message')
    args = parser.parse_args()
    if args.bag is None and not (args.check or args.check_required or args.check_patrol):
        parser.error('Provide a bag path or use --check / --check-required / --check-patrol')
    if args.check_patrol:
        try:
            check_patrol_examples()
        except (NotImplementedError, AssertionError, KeyError, TypeError, ValueError) as error:
            print(f'EXERCISE: {error}')
            return 2
        return 0
    if args.check_required:
        try:
            check_required_topic_examples()
        except (NotImplementedError, AssertionError, TypeError, ValueError) as error:
            print(f'EXERCISE: {error}')
            return 2
        return 0
    try:
        check_exercise()
    except (NotImplementedError, AssertionError, TypeError, ValueError) as error:
        print(f'EXERCISE: {error}')
        return 2
    if args.check:
        return 0
    bag = args.bag.expanduser().resolve()
    if not bag.exists():
        parser.error(f'Bag does not exist: {bag}')
    output = args.output.expanduser().resolve() if args.output else None
    input_directory = bag if bag.is_dir() else bag.parent
    if output is not None and (output == input_directory or input_directory in output.parents):
        parser.error('Write the report outside the input bag directory to preserve recorded data')
    include_patrol = args.patrol or args.user_desired_state is not None
    try:
        report = summarize_bag(bag, args.storage_id, include_patrol=include_patrol)
    except NotImplementedError as error:
        print(f'EXERCISE: {error}')
        return 2
    except ImportError as error:
        print(f'ERROR: {error}')
        print('Source the Humble environment to make rosbag2_py available.')
        return 1
    except (RuntimeError, ValueError) as error:
        print(f'ERROR: {error}')
        return 1
    missing = []
    if args.require:
        try:
            missing = find_missing_topics(report['topics'], args.require)
        except (NotImplementedError, TypeError, ValueError) as error:
            print(f'EXERCISE: {error}')
            return 2
        report['required_topic_check'] = {
            'required': args.require, 'missing_or_empty': missing, 'passed': not missing}
    print(f'{"TOPIC":<25} {"COUNT":>8} {"FIRST_s":>10} {"LAST_s":>10} {"MAX_GAP_s":>11}')
    for name, stats in report['topics'].items():
        def seconds(value):
            return '-' if value is None else f'{value/1_000_000_000:.6f}'
        print(f'{name:<25} {stats["count"]:>8} {seconds(stats["first_record_ns"]):>10} '
              f'{seconds(stats["last_record_ns"]):>10} {seconds(stats["max_gap_ns"]):>11}')
    print(f'TOTAL: {report["total_messages"]} messages')
    print(report['time_basis'])
    if args.require:
        print('REQUIRED TOPICS: ' + ('missing or empty: '+', '.join(missing) if missing else 'passed'))
    patrol_check_failed = False
    if include_patrol:
        last = report['patrol']['last_status']
        print('PATROL: ' + ('no status messages; result unknown' if last is None
                             else json.dumps(last, ensure_ascii=False)))
        check = report['patrol']['completion_count_check']
        if check is None:
            print('PATROL COMPLETION COUNT: not checked (no status messages)')
        else:
            print('PATROL COMPLETION COUNT: ' + ('passed' if check['passed'] else 'failed: '+check['reason']))
        patrol_check_failed = check is None or not check['passed']
        if args.user_desired_state is not None:
            expectation = check_patrol_expectation(last, args.user_desired_state)
            report['patrol']['expectation_check'] = expectation
            print('PATROL EXPECTATION: ' + json.dumps(expectation, ensure_ascii=False))
            patrol_check_failed = patrol_check_failed or not expectation['passed']
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
        print(f'SAVED: {output}')
    return 1 if missing or patrol_check_failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
