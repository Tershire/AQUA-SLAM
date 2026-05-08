#!/usr/bin/env python3

# AQUA-SLAM$ python3 tools/compare_odom_and_path.py results/slam_YYYYMMDD_HHMMSS/

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class PoseSeries:
    t: list[float] = field(default_factory=list)
    x: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)
    z: list[float] = field(default_factory=list)
    roll: list[float] = field(default_factory=list)
    pitch: list[float] = field(default_factory=list)
    yaw: list[float] = field(default_factory=list)

    def append(self, t: float, position: Any, orientation: Any) -> None:
        roll, pitch, yaw = quat_to_rpy(orientation)
        self.t.append(t)
        self.x.append(float(position.x))
        self.y.append(float(position.y))
        self.z.append(float(position.z))
        self.roll.append(math.degrees(roll))
        self.pitch.append(math.degrees(pitch))
        self.yaw.append(math.degrees(yaw))

    def normalize(self, t0: float) -> None:
        self.t = [t - t0 for t in self.t]

    def __len__(self) -> int:
        return len(self.t)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Compare AQUA-SLAM ORB odometry and path topics from a ROS2 bag.',
    )
    parser.add_argument(
        'bag',
        type=Path,
        help='ROS2 bag directory, metadata.yaml, .mcap, or .bag file.',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        help='Directory for PNG outputs. Defaults to <bag>_orb_compare.',
    )
    parser.add_argument('--odom-topic', default='/aqua_slam/orb_odom')
    parser.add_argument('--path-topic', default='/aqua_slam/orb_path')
    parser.add_argument('--odom-body-topic', default='/aqua_slam/orb_odom_body')
    parser.add_argument('--path-body-topic', default='/aqua_slam/orb_path_body')
    parser.add_argument(
        '--apriltag-topic',
        default='/apriltag_slam/GT',
        help='AprilTag SLAM GT topic to overlay as reference (dark gray). Set to empty string to disable.',
    )
    parser.add_argument('--show', action='store_true', help='Show matplotlib windows after saving PNGs.')
    return parser.parse_args()


def require_plot_deps() -> Any:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            'Missing matplotlib. Install dependencies with:\n'
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
    bag = bag.expanduser().resolve()
    if requested is not None:
        out = requested.expanduser().resolve()
    else:
        base = bag.parent if bag.name == 'metadata.yaml' else bag
        out = base.parent / f'{base.name}_orb_compare'
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


def extract_pose(msg: Any) -> tuple[Any, Any] | None:
    if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
        pose = msg.pose.pose
        return pose.position, pose.orientation
    if hasattr(msg, 'pose'):
        return msg.pose.position, msg.pose.orientation
    return None


def append_path(series: PoseSeries, msg: Any, fallback_time: float) -> None:
    for pose_stamped in getattr(msg, 'poses', []):
        t = msg_time(pose_stamped, int(fallback_time * 1e9))
        series.append(t, pose_stamped.pose.position, pose_stamped.pose.orientation)


def choose_connections(reader: Any, topics: set[str]) -> list[Any]:
    return [conn for conn in reader.connections if conn.topic in topics]


def print_topics(reader: Any, topics: set[str]) -> None:
    print('Available selected topics:')
    for conn in sorted(reader.connections, key=lambda c: c.topic):
        if conn.topic in topics:
            print(f'  {conn.topic:35s} {conn.msgtype}')


def normalize_pair(odom: PoseSeries, path: PoseSeries) -> float | None:
    first_times = []
    if odom.t:
        first_times.append(odom.t[0])
    if path.t:
        first_times.append(path.t[0])
    if not first_times:
        return None
    t0 = min(first_times)
    odom.normalize(t0)
    path.normalize(t0)
    return t0


def read_bag(
    args: argparse.Namespace,
) -> tuple[dict[str, tuple[PoseSeries, PoseSeries, str]], PoseSeries | None]:
    AnyReader = require_bag_dep()
    configured_pairs = (
        ('orb', args.odom_topic, args.path_topic, 'ORB camera frame'),
        ('orb_body', args.odom_body_topic, args.path_body_topic, 'ORB body frame'),
    )
    wanted_topics = {topic for _, odom, path, _ in configured_pairs for topic in (odom, path)}
    at_topic: str = getattr(args, 'apriltag_topic', '/apriltag_slam/GT')
    if at_topic:
        wanted_topics.add(at_topic)

    odom_series = {odom: PoseSeries() for _, odom, _, _ in configured_pairs}
    path_series = {path: PoseSeries() for _, _, path, _ in configured_pairs}
    apriltag_series = PoseSeries()
    latest_path_msgs: dict[str, tuple[Any, float]] = {}
    apriltag_path_msg: tuple[Any, float] | None = None

    bag_path = args.bag.expanduser().resolve()
    if not bag_path.exists():
        raise SystemExit(f'Bag path does not exist: {bag_path}')

    try:
        with AnyReader(resolve_bag_path(args.bag)) as reader:
            print_topics(reader, wanted_topics)
            connections = choose_connections(reader, wanted_topics)
            if not connections:
                raise SystemExit('No ORB odom/path topics were found in this bag.')

            for conn, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, conn.msgtype)
                t = msg_time(msg, timestamp)
                if conn.topic in odom_series:
                    pose = extract_pose(msg)
                    if pose is not None:
                        odom_series[conn.topic].append(t, pose[0], pose[1])
                elif conn.topic in path_series:
                    latest_path_msgs[conn.topic] = (msg, t)
                elif at_topic and conn.topic == at_topic:
                    if 'Path' in conn.msgtype:
                        apriltag_path_msg = (msg, t)
                    else:
                        pose = extract_pose(msg)
                        if pose is not None:
                            apriltag_series.append(t, pose[0], pose[1])

            for topic, (msg, t) in latest_path_msgs.items():
                append_path(path_series[topic], msg, t)

            if apriltag_path_msg is not None:
                append_path(apriltag_series, apriltag_path_msg[0], apriltag_path_msg[1])

    except SystemExit:
        raise
    except Exception as exc:
        raise SystemExit(
            f'Failed to read bag: {exc}\n'
            'Use a bag that was stopped cleanly, or pass the bag directory containing metadata.yaml.'
        ) from exc

    # Normalize ORB pairs and collect absolute t0 values for apriltag time alignment.
    result: dict[str, tuple[PoseSeries, PoseSeries, str]] = {}
    pair_t0s: list[float] = []
    for key, odom_topic, path_topic, title in configured_pairs:
        odom = odom_series[odom_topic]
        path = path_series[path_topic]
        t0 = normalize_pair(odom, path)
        if t0 is not None:
            pair_t0s.append(t0)
        result[key] = (odom, path, title)

    # Normalize apriltag timestamps.  If from the same session (within 1 hour of the ORB
    # data), use the same absolute t0 so the time axes align.  Otherwise normalize
    # independently (both start at t = 0) since wall-clock synchronisation is meaningless.
    apriltag: PoseSeries | None = None
    if at_topic and len(apriltag_series):
        main_t0 = min(pair_t0s) if pair_t0s else apriltag_series.t[0]
        if abs(apriltag_series.t[0] - main_t0) <= 3600.0:
            apriltag_series.normalize(main_t0)
        else:
            apriltag_series.normalize(apriltag_series.t[0])
        apriltag = apriltag_series
    elif at_topic:
        print(f'warning: no AprilTag GT samples found on {at_topic}')

    return result, apriltag


def safe_name(name: str) -> str:
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', name).strip('_')


_APRILTAG_COLOR = '#444444'

# Fixed rotation from AprilTag GT frame → AQUA-SLAM world frame.
# Derived empirically (Pearson correlations on synchronized bag data):
#   GT.x → +AQUA.y  (r=+0.998, primary forward axis, both decrease over trajectory)
#   GT.z → -AQUA.x  (r=-0.654, lateral axis, ranges 0.12 m vs 0.16 m)
#   GT.y → -AQUA.z  (r=-0.424, vertical axis, ranges 0.04 m vs 0.04 m)
# Matrix: R = [[0, 0,-1],[1, 0, 0],[0,-1, 0]]  = Rz(+90°) * Rx(-90°)
# Quaternion equivalent (x,y,z,w): (-0.5, -0.5, 0.5, 0.5)
_Q_GT_TO_AQUA = (-0.5, -0.5, 0.5, 0.5)  # (x, y, z, w)
# R_cam_body = [[0,-1,0],[0,0,-1],[1,0,0]]: right-multiply to convert cam→body orientation
_Q_CAM_BODY = (0.5, -0.5, 0.5, 0.5)  # (x, y, z, w)


def _quat_mul(q1: tuple, q2: tuple) -> tuple:
    """Quaternion product q1 ⊗ q2, both (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_conj(q: tuple) -> tuple:
    """Quaternion conjugate = inverse for unit quaternions, (x,y,z,w) → (-x,-y,-z,w)."""
    return (-q[0], -q[1], -q[2], q[3])


def _rpy_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple:
    """ZYX Euler angles (degrees) → quaternion (x, y, z, w)."""
    r = math.radians(roll_deg) / 2.0
    p = math.radians(pitch_deg) / 2.0
    y = math.radians(yaw_deg) / 2.0
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def align_apriltag(src: PoseSeries, ref: PoseSeries, body_frame: bool = False) -> PoseSeries:
    """Transform AprilTag GT poses into the AQUA-SLAM world frame and align to ref's start.

    Position: apply R = Rz(+90°)*Rx(−90°), then translate so GT[0] == ref[0].
    Orientation: apply frame rotation, then left-multiply by q_delta = q_ref[0] ⊗ q_gt[0]⁻¹
      so that GT attitude starts at the same value as ref (analogous to position alignment).
    """
    if not len(src) or not len(ref):
        return src
    sx0, sy0, sz0 = src.x[0], src.y[0], src.z[0]
    rx0, ry0, rz0 = ref.x[0], ref.y[0], ref.z[0]

    # Frame-rotate all GT quaternions first, then compute orientation alignment delta.
    quats: list[tuple] = []
    for i in range(len(src.t)):
        q_gt = _rpy_to_quat(src.roll[i], src.pitch[i], src.yaw[i])
        q_cam = _quat_mul(_Q_GT_TO_AQUA, q_gt)
        quats.append(_quat_mul(q_cam, _Q_CAM_BODY) if body_frame else q_cam)

    q_ref_first = _rpy_to_quat(ref.roll[0], ref.pitch[0], ref.yaw[0])
    q_delta = _quat_mul(q_ref_first, _quat_conj(quats[0]))

    out = PoseSeries()
    for i in range(len(src.t)):
        dx = src.x[i] - sx0
        dy = src.y[i] - sy0
        dz = src.z[i] - sz0
        out.t.append(src.t[i])
        out.x.append(rx0 + (-dz))   # -GT.z
        out.y.append(ry0 + dx)      # +GT.x
        out.z.append(rz0 + (-dy))   # -GT.y
        qx, qy, qz, qw = _quat_mul(q_delta, quats[i])
        roll_r, pitch_r, yaw_r = quat_to_rpy(type('Q', (), {'x': qx, 'y': qy, 'z': qz, 'w': qw})())
        out.roll.append(math.degrees(roll_r))
        out.pitch.append(math.degrees(pitch_r))
        out.yaw.append(math.degrees(yaw_r))
    return out


def style_axis(ax: Any) -> None:
    ax.grid(True, alpha=0.28)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)


def _plot_frame(
    fig: Any,
    gs: Any,
    r0: int,
    title: str,
    odom: PoseSeries,
    path: PoseSeries,
    at: 'PoseSeries | None',
) -> None:
    """Render one frame block (3 rows) into the given GridSpec starting at row r0."""
    ax_xy = fig.add_subplot(gs[r0:r0 + 3, 0])
    ax_px, ax_py, ax_pz = [fig.add_subplot(gs[r0 + r, 1]) for r in range(3)]
    ax_rr, ax_pp, ax_yw = [fig.add_subplot(gs[r0 + r, 2]) for r in range(3)]

    # XY trajectory
    if len(odom): ax_xy.plot(odom.x, odom.y, color='tab:blue', alpha=0.7, linewidth=1.6, label='odom')
    if len(path): ax_xy.plot(path.x, path.y, color='tab:red',  alpha=0.7, linewidth=1.6, label='path')
    if at:        ax_xy.plot(at.x,   at.y,   color=_APRILTAG_COLOR, alpha=0.7, linewidth=1.4, label='GT', zorder=0)
    ax_xy.set_xlabel('x [m]'); ax_xy.set_ylabel('y [m]')
    ax_xy.axis('equal'); ax_xy.set_title(f'{title}: XY')
    ax_xy.legend(fontsize='small'); style_axis(ax_xy)

    # Position over time — one subplot per axis
    for ax, o_v, p_v, g_v, ylabel in [
        (ax_px, odom.x, path.x, at.x if at else None, 'x [m]'),
        (ax_py, odom.y, path.y, at.y if at else None, 'y [m]'),
        (ax_pz, odom.z, path.z, at.z if at else None, 'z [m]'),
    ]:
        if len(odom): ax.plot(odom.t, o_v, color='tab:blue', alpha=0.8, linewidth=1.5, label='odom')
        if len(path): ax.plot(path.t, p_v, color='tab:red',  alpha=0.8, linewidth=1.5, label='path')
        if at:        ax.plot(at.t,   g_v, color=_APRILTAG_COLOR, alpha=0.8, linewidth=1.2, label='GT')
        ax.set_ylabel(ylabel); ax.set_xlim(left=0); style_axis(ax)
    ax_px.set_title(f'{title}: position'); ax_px.legend(fontsize='small')
    ax_pz.set_xlabel('time [s]')

    # Attitude over time — one subplot per axis
    for ax, o_v, p_v, g_v, ylabel in [
        (ax_rr, odom.roll,  path.roll,  at.roll  if at else None, 'roll [°]'),
        (ax_pp, odom.pitch, path.pitch, at.pitch if at else None, 'pitch [°]'),
        (ax_yw, odom.yaw,   path.yaw,   at.yaw   if at else None, 'yaw [°]'),
    ]:
        if len(odom): ax.plot(odom.t, o_v, color='tab:blue', alpha=0.8, linewidth=1.5, label='odom')
        if len(path): ax.plot(path.t, p_v, color='tab:red',  alpha=0.8, linewidth=1.5, label='path')
        if at:        ax.plot(at.t,   g_v, color=_APRILTAG_COLOR, alpha=0.8, linewidth=1.2, label='GT')
        ax.set_ylabel(ylabel); ax.set_xlim(left=0); style_axis(ax)
    ax_rr.set_title(f'{title}: attitude'); ax_rr.legend(fontsize='small')
    ax_yw.set_xlabel('time [s]')


def plot_pair(
    plt: Any,
    out: Path,
    title: str,
    odom: PoseSeries,
    path: PoseSeries,
    apriltag: PoseSeries | None = None,
    body_frame: bool = False,
) -> None:
    at: PoseSeries | None = None
    if apriltag is not None and len(apriltag):
        at = align_apriltag(apriltag, path if len(path) else odom, body_frame=body_frame)

    fig = plt.figure(figsize=(18, 10))
    gs = fig.add_gridspec(3, 3, hspace=0.42, wspace=0.30)
    _plot_frame(fig, gs, 0, title, odom, path, at)
    fig.savefig(out, dpi=180, bbox_inches='tight')
    print(f'Wrote {out}')


def plot_combined(
    plt: Any,
    out: Path,
    data: dict[str, tuple[PoseSeries, PoseSeries, str]],
    apriltag: PoseSeries | None = None,
) -> None:
    fig = plt.figure(figsize=(18, 18))
    gs = fig.add_gridspec(6, 3, hspace=0.42, wspace=0.30)

    for frame_idx, key in enumerate(('orb', 'orb_body')):
        odom, path, title = data[key]
        at: PoseSeries | None = None
        if apriltag is not None and len(apriltag):
            at = align_apriltag(apriltag, path if len(path) else odom, body_frame=(key == 'orb_body'))
        _plot_frame(fig, gs, frame_idx * 3, title, odom, path, at)

    fig.savefig(out, dpi=180, bbox_inches='tight')
    print(f'Wrote {out}')


def warn_missing(data: dict[str, tuple[PoseSeries, PoseSeries, str]]) -> None:
    for key, (odom, path, title) in data.items():
        if not len(odom):
            print(f'warning: no odom samples found for {title} ({key})')
        if not len(path):
            print(f'warning: no path samples found for {title} ({key})')


def main() -> None:
    args = parse_args()
    plt = require_plot_deps()
    out_dir = output_dir_for(args.bag, args.output_dir)
    data, apriltag = read_bag(args)

    warn_missing(data)
    for key, (odom, path, title) in data.items():
        plot_pair(plt, out_dir / f'{safe_name(key)}_odom_path_compare.png', title, odom, path, apriltag,
                  body_frame=key.endswith('_body'))
    plot_combined(plt, out_dir / 'orb_odom_path_compare_all.png', data, apriltag)

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
