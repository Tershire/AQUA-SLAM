#!/usr/bin/env python3

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Convert a ROS1 bag to rosbag2 and adapt DVL topics for AQUA-SLAM.',
    )
    parser.add_argument('src', type=Path, help='Source ROS1 bag file')
    parser.add_argument(
        '--tmp-ros2',
        type=Path,
        help='Temporary rosbag2 path. Defaults next to the source bag.',
    )
    parser.add_argument(
        '--dst',
        type=Path,
        help='Final rosbag2 path. Defaults to <src_stem>_aqua_ros2 next to the source bag.',
    )
    return parser.parse_args()


def convert_ros1_to_ros2(src: Path, tmp_ros2: Path) -> None:
    if tmp_ros2.exists():
        shutil.rmtree(tmp_ros2)

    subprocess.run(
        [
            'rosbags-convert',
            '--src',
            str(src),
            '--dst',
            str(tmp_ros2),
        ],
        check=True,
    )


def adapt_dvl_topic(src_ros2: Path, dst_ros2: Path) -> None:
    if dst_ros2.exists():
        shutil.rmtree(dst_ros2)

    jazzy_store = get_typestore(Stores.ROS2_JAZZY)
    odom_type = 'nav_msgs/msg/Odometry'
    dvl_src_topic = '/dvl/data'
    dvl_dst_topic = '/bluerov2/DVL'

    with AnyReader([src_ros2]) as reader, Writer(dst_ros2, version=9) as writer:
        conn_map = {}
        dvl_conn = None

        for conn in reader.connections:
            if conn.topic == dvl_src_topic:
                dvl_conn = conn
                continue

            conn_map[conn.id] = writer.add_connection(
                conn.topic,
                conn.msgtype,
                typestore=reader.typestore,
                serialization_format=conn.ext.serialization_format,
                offered_qos_profiles=conn.ext.offered_qos_profiles,
            )

        odom_conn = writer.add_connection(
            dvl_dst_topic,
            odom_type,
            typestore=jazzy_store,
            serialization_format='cdr',
        )

        for conn, timestamp, rawdata in reader.messages():
            if dvl_conn is not None and conn.id == dvl_conn.id:
                dvl_msg = reader.deserialize(rawdata, conn.msgtype)
                if not dvl_msg.velocity_valid:
                    continue

                odom_msg = jazzy_store.types[odom_type](
                    header=dvl_msg.header,
                    child_frame_id='',
                    pose=jazzy_store.types['geometry_msgs/msg/PoseWithCovariance'](
                        pose=jazzy_store.types['geometry_msgs/msg/Pose'](
                            position=jazzy_store.types['geometry_msgs/msg/Point'](x=0.0, y=0.0, z=0.0),
                            orientation=jazzy_store.types['geometry_msgs/msg/Quaternion'](
                                x=0.0, y=0.0, z=0.0, w=1.0,
                            ),
                        ),
                        covariance=np.zeros(36, dtype=np.float64),
                    ),
                    twist=jazzy_store.types['geometry_msgs/msg/TwistWithCovariance'](
                        twist=jazzy_store.types['geometry_msgs/msg/Twist'](
                            linear=dvl_msg.velocity,
                            angular=jazzy_store.types['geometry_msgs/msg/Vector3'](x=0.0, y=0.0, z=0.0),
                        ),
                        covariance=np.zeros(36, dtype=np.float64),
                    ),
                )
                writer.write(
                    odom_conn,
                    timestamp,
                    jazzy_store.serialize_cdr(odom_msg, odom_type),
                )
                continue

            writer.write(conn_map[conn.id], timestamp, rawdata)


def main() -> None:
    args = parse_args()
    src = args.src.resolve()
    tmp_ros2 = (args.tmp_ros2 or src.with_name(f'{src.stem}_ros2')).resolve()
    dst_ros2 = (args.dst or src.with_name(f'{src.stem}_aqua_ros2')).resolve()

    convert_ros1_to_ros2(src, tmp_ros2)
    adapt_dvl_topic(tmp_ros2, dst_ros2)

    print(f'Prepared AQUA-SLAM rosbag2 at: {dst_ros2}')
    print(f'Temporary intermediate rosbag2 at: {tmp_ros2}')


if __name__ == '__main__':
    main()
