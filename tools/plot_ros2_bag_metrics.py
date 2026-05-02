#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


DEFAULT_POSE_TOPICS = (
    '/aqua_slam/orb_odom',          # SLAM pose + velocity (camera frame)
    '/aqua_slam/orb_path',          # SLAM trajectory (camera frame)
    '/aqua_slam/orb_odom_body',     # SLAM pose + velocity (body FLU frame)
    '/aqua_slam/orb_path_body',     # SLAM trajectory (body FLU frame)
    '/aqua_slam/dvl_imu_pose',      # DVL+IMU dead-reckoning pose
    '/aqua_slam/dvl_imu_path',      # DVL+IMU dead-reckoning trajectory
    '/apriltag_slam/GT',            # ground truth, for comparison
)
LOOP_TOPIC_HINTS = ('loop', 'map_merge')


@dataclass
class Series3:
    t: list[float] = field(default_factory=list)
    x: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)
    z: list[float] = field(default_factory=list)

    def append(self, t: float, v: Any) -> None:
        self.t.append(t)
        self.x.append(float(v.x))
        self.y.append(float(v.y))
        self.z.append(float(v.z))

    def __len__(self) -> int:
        return len(self.t)


@dataclass
class PoseSeries:
    t: list[float] = field(default_factory=list)
    px: list[float] = field(default_factory=list)
    py: list[float] = field(default_factory=list)
    pz: list[float] = field(default_factory=list)
    roll: list[float] = field(default_factory=list)
    pitch: list[float] = field(default_factory=list)
    yaw: list[float] = field(default_factory=list)
    vx: list[float] = field(default_factory=list)
    vy: list[float] = field(default_factory=list)
    vz: list[float] = field(default_factory=list)

    def append(self, t: float, position: Any, orientation: Any, velocity: Any = None) -> None:
        r, p, y = quat_to_rpy(orientation)
        self.t.append(t)
        self.px.append(float(position.x))
        self.py.append(float(position.y))
        self.pz.append(float(position.z))
        self.roll.append(math.degrees(r))
        self.pitch.append(math.degrees(p))
        self.yaw.append(math.degrees(y))
        if velocity is not None:
            self.vx.append(float(velocity.x))
            self.vy.append(float(velocity.y))
            self.vz.append(float(velocity.z))

    def __len__(self) -> int:
        return len(self.t)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Plot IMU, DVL, pose, velocity, and loop hints from a ROS2 bag.',
    )
    parser.add_argument(
        'bag',
        type=Path,
        help='ROS2 bag directory (e.g. results/slam_20260501_120000).',
    )
    parser.add_argument('--output-dir', type=Path, help='Directory for PNG/CSV outputs.')
    parser.add_argument('--imu-topic', default='/imu/data')
    parser.add_argument('--dvl-topic', default='/bluerov2/dvl')
    parser.add_argument(
        '--pose-topic',
        action='append',
        help='Pose/Odometry/Path topic to plot. Can be repeated. Defaults to AQUA-SLAM outputs, GT, and depth.',
    )
    parser.add_argument(
        '--log-file',
        type=Path,
        help='Optional AQUA-SLAM console log. Loop-closure keyword hits are written to loop_events.txt.',
    )
    parser.add_argument('--show', action='store_true', help='Show matplotlib windows after saving PNGs.')
    return parser.parse_args()


def require_plot_deps() -> Any:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            'Missing plotting dependencies. Install them in the environment you use to run this script:\n'
            '  python3 -m pip install rosbags matplotlib'
        ) from exc
    return plt


def require_bag_dep() -> Any:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise SystemExit(
            'Missing rosbags. Install it with:\n'
            '  python3 -m pip install rosbags'
        ) from exc
    return AnyReader


def resolve_bag_path(path: Path) -> list[Path]:
    """Return a list of paths to pass to AnyReader.

    Accepts a bag directory (with or without metadata.yaml) or a direct
    .mcap / .bag file.  When metadata.yaml is absent, falls back to
    passing the individual MCAP files so the bag can still be read.
    """
    path = path.expanduser().resolve()
    if path.name == 'metadata.yaml':
        path = path.parent
    if path.is_dir():
        if (path / 'metadata.yaml').exists():
            return [path]
        mcap_files = sorted(path.glob('*.mcap'))
        if mcap_files:
            return mcap_files
        raise SystemExit(f'No metadata.yaml or .mcap files found in: {path}')
    return [path]


def output_dir_for(bag: Path, requested: Path | None) -> Path:
    if requested is not None:
        out = requested.expanduser().resolve()
    else:
        out = bag.parent / f'{bag.name}_plots'
    out.mkdir(parents=True, exist_ok=True)
    return out


def stamp_to_sec(stamp: Any) -> float:
    sec = getattr(stamp, 'sec', 0)
    nsec = getattr(stamp, 'nanosec', getattr(stamp, 'nsec', 0))
    return float(sec) + float(nsec) * 1e-9


def msg_time(msg: Any, bag_timestamp_ns: int) -> float:
    header = getattr(msg, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is not None:
        t = stamp_to_sec(stamp)
        if t > 0.0:
            return t
    return bag_timestamp_ns * 1e-9


def quat_to_rpy(q: Any) -> tuple[float, float, float]:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def normalize_times(series: dict[str, Any], t0: float | None) -> float | None:
    # Each series is normalized to its own first timestamp (t=0 at first sample).
    # Topics in the result bag can use different time references (e.g. GT uses
    # relative bag time starting near 0, while SLAM outputs use ROS system time
    # ~1.25e8 s), so a single global t0 would leave most series un-normalized.
    any_data = False
    for value in series.values():
        times = getattr(value, 't', None)
        if times:
            any_data = True
            series_t0 = times[0]
            for i in range(len(times)):
                times[i] = times[i] - series_t0
    return 0.0 if any_data else None


def extract_pose(msg: Any) -> tuple[Any, Any, Any | None] | None:
    velocity = None
    if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
        pose = msg.pose.pose
        if hasattr(msg, 'twist') and hasattr(msg.twist, 'twist'):
            velocity = msg.twist.twist.linear
    elif hasattr(msg, 'pose'):
        pose = msg.pose
    else:
        return None
    return pose.position, pose.orientation, velocity


def append_path(topic_series: PoseSeries, msg: Any, fallback_time: float) -> None:
    poses = getattr(msg, 'poses', [])
    for pose_stamped in poses:
        t = msg_time(pose_stamped, int(fallback_time * 1e9))
        pose = pose_stamped.pose
        topic_series.append(t, pose.position, pose.orientation)


def parse_loop_log(path: Path) -> list[tuple[int, str]]:
    if path is None:
        return []
    keywords = re.compile(r'(loop|CorrectLoop|LoopClosing|mbLoopDetected)', re.IGNORECASE)
    events = []
    with path.expanduser().open('r', errors='replace') as handle:
        for line_no, line in enumerate(handle, start=1):
            if keywords.search(line):
                events.append((line_no, line.strip()[:160]))
    return events


def choose_connections(reader: Any, topics: set[str]) -> dict[str, list[Any]]:
    selected: dict[str, list[Any]] = {topic: [] for topic in topics}
    for conn in reader.connections:
        if conn.topic in selected:
            selected[conn.topic].append(conn)
    return selected


def print_topics(reader: Any) -> None:
    print('Available topics:')
    for conn in sorted(reader.connections, key=lambda c: c.topic):
        print(f'  {conn.topic:45s} {conn.msgtype}')


def read_bag(args: argparse.Namespace) -> tuple[dict[str, Any], list[tuple[float, str]], list[str]]:
    AnyReader = require_bag_dep()
    warnings: list[str] = []

    pose_topics = tuple(args.pose_topic) if args.pose_topic else DEFAULT_POSE_TOPICS
    wanted_topics = {args.imu_topic, args.dvl_topic, *pose_topics}

    imu_acc = Series3()
    imu_gyro = Series3()
    dvl_vel = Series3()
    poses = {topic: PoseSeries() for topic in pose_topics}
    loop_events: list[tuple[float, str]] = []

    bag_dir = args.bag.expanduser().resolve()
    if not bag_dir.exists():
        raise SystemExit(f'Bag path does not exist: {bag_dir}')
    bag_paths = resolve_bag_path(args.bag)

    try:
        with AnyReader(bag_paths) as reader:
            print_topics(reader)
            selected = choose_connections(reader, wanted_topics)
            active_connections = [conn for conns in selected.values() for conn in conns]
            loop_connections = [
                conn for conn in reader.connections
                if any(hint in conn.topic.lower() for hint in LOOP_TOPIC_HINTS)
            ]
            active_connections.extend(loop_connections)
            if not active_connections:
                raise SystemExit('No selected topics were found in this bag.')

            # nav_msgs/Path accumulates all poses in each message; keep only the last one.
            latest_path_msgs: dict[str, tuple[Any, float]] = {}

            for conn, timestamp, rawdata in reader.messages(connections=active_connections):
                msg = reader.deserialize(rawdata, conn.msgtype)
                t = msg_time(msg, timestamp)

                if conn.topic == args.imu_topic:
                    imu_acc.append(t, msg.linear_acceleration)
                    imu_gyro.append(t, msg.angular_velocity)
                elif conn.topic == args.dvl_topic:
                    if hasattr(msg, 'twist') and hasattr(msg.twist, 'twist'):
                        dvl_vel.append(t, msg.twist.twist.linear)
                    elif hasattr(msg, 'velocity'):
                        dvl_vel.append(t, msg.velocity)
                elif conn.topic in poses:
                    if 'Path' in conn.msgtype:
                        latest_path_msgs[conn.topic] = (msg, t)
                    else:
                        result = extract_pose(msg)
                        if result is not None:
                            poses[conn.topic].append(t, result[0], result[1], result[2])
                elif any(hint in conn.topic.lower() for hint in LOOP_TOPIC_HINTS):
                    loop_events.append((t, conn.topic))

            for topic, (msg, t) in latest_path_msgs.items():
                append_path(poses[topic], msg, t)

    except SystemExit:
        raise
    except Exception as exc:
        raise SystemExit(
            f'Failed to read bag: {exc}\n'
            'The recording may have been interrupted before the MCAP file was finalized.\n'
            'Use a bag that was stopped cleanly with Ctrl+C.'
        ) from exc

    series: dict[str, Any] = {
        'imu_acc': imu_acc,
        'imu_gyro': imu_gyro,
        'dvl_vel': dvl_vel,
        **{f'pose:{topic}': data for topic, data in poses.items()},
    }
    t0 = normalize_times(series, None)
    if t0 is not None:
        loop_events = [(t - t0, label) for t, label in loop_events]

    imu_gyro.x = [math.degrees(v) for v in imu_gyro.x]
    imu_gyro.y = [math.degrees(v) for v in imu_gyro.y]
    imu_gyro.z = [math.degrees(v) for v in imu_gyro.z]

    if not len(dvl_vel):
        warnings.append(f'No DVL samples found on {args.dvl_topic}. Try --dvl-topic /dvl/data for the unadapted bag.')
    if not len(imu_acc):
        warnings.append(f'No IMU samples found on {args.imu_topic}.')
    return series, loop_events, warnings


def add_loop_lines(ax: Any, loop_events: list[tuple[float, str]]) -> None:
    for t, _ in loop_events:
        ax.axvline(t, color='tab:red', alpha=0.25, linewidth=1.0)


def plot_xyz(plt: Any, out: Path, title: str, ylabel: str, data: Series3, loop_events: list[tuple[float, str]]) -> None:
    if not len(data):
        return
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(data.t, data.x, label='x')
    ax.plot(data.t, data.y, label='y')
    ax.plot(data.t, data.z, label='z')
    add_loop_lines(ax, loop_events)
    ax.set_title(title)
    ax.set_xlabel('time [s]')
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out, dpi=160)


def plot_pose(plt: Any, out: Path, topic: str, data: PoseSeries, loop_events: list[tuple[float, str]]) -> None:
    if not len(data):
        return
    has_vel = len(data.vx) > 0
    n_time = 3 if has_vel else 2
    fig, axes = plt.subplots(n_time + 1, 1, figsize=(11, 3 * (n_time + 1)))

    row = 0
    axes[row].plot(data.t, data.px, label='x')
    axes[row].plot(data.t, data.py, label='y')
    axes[row].plot(data.t, data.pz, label='z')
    axes[row].set_ylabel('position [m]')
    axes[row].legend()
    row += 1

    axes[row].plot(data.t, data.roll, label='roll')
    axes[row].plot(data.t, data.pitch, label='pitch')
    axes[row].plot(data.t, data.yaw, label='yaw')
    axes[row].set_ylabel('attitude [deg]')
    axes[row].legend()
    row += 1

    if has_vel:
        axes[row].plot(data.t, data.vx, label='vx')
        axes[row].plot(data.t, data.vy, label='vy')
        axes[row].plot(data.t, data.vz, label='vz')
        axes[row].set_ylabel('velocity [m/s]')
        axes[row].legend()
        row += 1

    axes[row].plot(data.px, data.py)
    axes[row].set_title('XY trajectory (top-down)')
    axes[row].set_xlabel('x [m]')
    axes[row].set_ylabel('y [m]')
    axes[row].axis('equal')

    for ax in axes[:row]:
        add_loop_lines(ax, loop_events)
        ax.set_xlabel('time [s]')
        ax.grid(True, alpha=0.3)
    axes[row].grid(True, alpha=0.3)
    fig.suptitle(f'Pose: {topic}')
    fig.tight_layout()
    fig.savefig(out, dpi=160)


def write_csv(out: Path, name: str, data: Series3) -> None:
    if not len(data):
        return
    with (out / f'{name}.csv').open('w') as handle:
        handle.write('t,x,y,z\n')
        for row in zip(data.t, data.x, data.y, data.z):
            handle.write(f'{row[0]:.9f},{row[1]:.9f},{row[2]:.9f},{row[3]:.9f}\n')


def write_pose_csv(out: Path, topic: str, data: PoseSeries) -> None:
    if not len(data):
        return
    has_vel = len(data.vx) > 0
    safe = re.sub(r'[^A-Za-z0-9_.-]+', '_', topic).strip('_')
    with (out / f'pose_{safe}.csv').open('w') as handle:
        header = 't,x,y,z,roll_deg,pitch_deg,yaw_deg'
        if has_vel:
            header += ',vx,vy,vz'
        handle.write(header + '\n')
        cols = (data.t, data.px, data.py, data.pz, data.roll, data.pitch, data.yaw)
        if has_vel:
            cols = (*cols, data.vx, data.vy, data.vz)
        for row in zip(*cols):
            handle.write(','.join(f'{v:.9f}' for v in row) + '\n')


def main() -> None:
    args = parse_args()
    plt = require_plot_deps()
    bag = args.bag.expanduser().resolve()
    out = output_dir_for(bag, args.output_dir)

    series, loop_events, warnings = read_bag(args)
    log_events = parse_loop_log(args.log_file) if args.log_file else []

    plot_xyz(plt, out / 'imu_acceleration.png', 'IMU linear acceleration', 'm/s^2', series['imu_acc'], loop_events)
    plot_xyz(plt, out / 'imu_angular_velocity.png', 'IMU angular velocity', 'deg/s', series['imu_gyro'], loop_events)
    plot_xyz(plt, out / 'dvl_velocity.png', 'DVL velocity', 'm/s', series['dvl_vel'], loop_events)

    for key, data in series.items():
        if not key.startswith('pose:'):
            continue
        topic = key.split(':', 1)[1]
        safe = re.sub(r'[^A-Za-z0-9_.-]+', '_', topic).strip('_')
        plot_pose(plt, out / f'pose_{safe}.png', topic, data, loop_events)

    write_csv(out, 'imu_acceleration', series['imu_acc'])
    write_csv(out, 'imu_angular_velocity', series['imu_gyro'])
    write_csv(out, 'dvl_velocity', series['dvl_vel'])
    for key, data in series.items():
        if key.startswith('pose:'):
            write_pose_csv(out, key.split(':', 1)[1], data)

    with (out / 'loop_events.txt').open('w') as handle:
        handle.write('Bag topic loop hints:\n')
        for t, label in loop_events:
            handle.write(f'{t:.9f} {label}\n')
        handle.write('\nLog loop keyword hits:\n')
        for line_no, text in log_events:
            handle.write(f'line {line_no}: {text}\n')

    for warning in warnings:
        print(f'warning: {warning}', file=sys.stderr)
    print(f'Wrote plots and CSV files to: {out}')
    if loop_events:
        print(f'Found {len(loop_events)} loop-related bag topic messages.')
    if log_events:
        print(f'Found {len(log_events)} loop-related log lines.')

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
