# AQUA-SLAM

Tightly-coupled underwater Acoustic-Visual-Inertial SLAM integrating stereo cameras, IMU, and DVL.

> Shida Xu, Kaicheng Zhang, and Sen Wang. "AQUA-SLAM: Tightly-Coupled Underwater Acoustic-Visual-Inertial SLAM with Sensor Calibration." IEEE Transactions on Robotics, 2025. [[IEEE](https://ieeexplore.ieee.org/abstract/document/10938346)] [[PDF](https://arxiv.org/pdf/2503.11420)]

---

## Branches

| Branch | Description |
|--------|-------------|
| `main` | ROS1 Noetic — original release |
| `simulation/stonefish` | ROS2 Jazzy — Stonefish simulation + ROS2 port |

---

## Guides

| Document | Description |
|----------|-------------|
| [documents/installation_guide.md](documents/installation_guide.md) | NVIDIA driver, nvidia-container-toolkit, Docker setup |
| [documents/simulation_guide.md](documents/simulation_guide.md) | Running AQUA-SLAM with the Stonefish simulator |
| [documents/example_run_guide.md](documents/example_run_guide.md) | Running AQUA-SLAM on a recorded bag (ROS2) |
| [documents/coordinate_frames.md](documents/coordinate_frames.md) | Coordinate frame conventions |
| [documents/topic_structure.md](documents/topic_structure.md) | ROS topic layout |
