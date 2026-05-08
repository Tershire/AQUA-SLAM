# Installation Guide

---

## 1. NVIDIA driver (host)

Stonefish requires GPU rendering. The NVIDIA driver must be installed on the host machine.

```bash
# Check current driver
nvidia-smi
```

If not installed, install via Ubuntu's additional drivers or directly:

```bash
sudo apt update
sudo apt install nvidia-driver-580   # match your GPU; check https://www.nvidia.com/drivers
sudo reboot
```

Verify after reboot:

```bash
nvidia-smi
```

---

## 2. nvidia-container-toolkit (host)

Enables GPU passthrough into Docker containers. Install on the **host**, not inside any container.

```bash
# Add NVIDIA container toolkit repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit

# Restart Docker to apply
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Verify GPU is accessible in Docker:

```bash
docker run --rm --gpus all nvidia/cuda:12.0-base-ubuntu22.04 nvidia-smi
```

---

## 3. Docker + Docker Compose v2 (host)

```bash
# Install Docker Engine
sudo apt install -y docker.io

# Add user to docker group (avoids sudo for every docker command)
sudo usermod -aG docker $USER
newgrp docker

# Verify Docker Compose v2 is available
docker compose version
```

---

## 4. Build the simulator image

The Dockerfile installs ROS2 Jazzy, builds Stonefish 1.6 and stonefish_ros2 from source.
Build takes **10–20 minutes**.

```bash
git clone https://github.com/Tershire/aqua-slam-testbed.git
cd aqua-slam-testbed/docker
docker compose build
```

Verify the image was created:

```bash
docker images | grep aqua
```

Expected output:

```
aqua/stonefish-sim   jazzy-dev   ...
```

---

## 5. Allow X11 display (per session)

Required for Stonefish's OpenGL window to appear on the host desktop.

```bash
xhost +local:docker
```

Add to `~/.bashrc` to run automatically:

```bash
echo 'xhost +local:docker > /dev/null' >> ~/.bashrc
```

---

## 6. Verify GPU inside the simulator container

```bash
cd aqua-slam-testbed/docker
docker compose up -d
docker exec -it aqua_slam_testbed bash

# Inside container — should show the GPU
nvidia-smi     # may not be installed; GPU access is verified by Stonefish rendering instead
```

If `nvidia-smi` is not in the container, verify GPU rendering works by launching the simulator and checking that the Stonefish window opens with hardware-accelerated graphics (not a black or very slow window).

Camera topic rate of **5–20 Hz** confirms GPU rendering is active.
A rate below **1 Hz** indicates the container is falling back to CPU rendering — check that `xhost +local:docker` was run and the container was started after that.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Camera rate < 1 Hz | GPU not accessible or X11 permission missing | Run `xhost +local:docker`, then `docker compose restart` |
| `Failed to load driver: nvidia-drm` in RViz log | Normal on some setups | Ignore — RViz still runs via OpenGL |
| `package 'aqua_slam' not found` | Workspace not sourced | `source ~/ros2_ws/install/setup.bash` |
| Stonefish window does not open | `DISPLAY` not set | Ensure `echo $DISPLAY` returns `:1` or similar on host before starting container |
