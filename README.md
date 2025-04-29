# ICUAS 2025 Competition - Solution (Team Kaizen)

## Docker Installation

This project uses Docker to create a consistent environment for simulation and development.

- Ensure you have **Ubuntu** installed (recommended: Ubuntu 24.04 LTS).
- Install Docker Engine by following [this guide](https://docs.docker.com/engine/install/ubuntu/).
- (Optional but recommended) Set up Docker for non-root usage by following [these steps](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).

If you have an NVIDIA GPU, install `nvidia-container-toolkit` as per [these instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

Enable graphical applications inside Docker:

```bash
xhost +local:docker
echo "xhost +local:docker > /dev/null" >> ~/.profile
source ~/.profile
```

---

## Clone the Competition Repository

```bash
git clone https://github.com/larics/icuas25_competition.git
cd icuas25_competition
```

Set the environment variable (needed for Docker build):

```bash
export DOCKER_BUILDKIT=1
```

---

## Build Docker Image

From the root of the `icuas25_competition` project, build the Docker image:

```bash
docker build --ssh default -t crazysim_icuas_img .
```

---

## Create and Run the Docker Container

**First-time setup (creates container and launches it):**

```bash
./first_run.sh
```

This creates a container named `crazysim_icuas_cont` and starts it.  
The ROS 2 workspace is located at `/root/CrazySim/ros2_ws` inside the container.

**Subsequent usage (after container is created):**

```bash
docker start -i crazysim_icuas_cont
```

Other useful Docker commands:

```bash
docker exec -it crazysim_icuas_cont bash     # Open a new bash session inside container
docker stop crazysim_icuas_cont               # Stop the container
docker rm crazysim_icuas_cont                 # Delete the container (if needed)
```

---

## Running the Simulation

Once inside the container:

1. Navigate to the startup folder:

```bash
cd /root/CrazySim/ros2_ws/src/icuas25_competition/startup
```
or use the alias:

```bash
cd_icuas25_competition
```

2. Start the simulation:

```bash
./start.sh
```

> If needed, make `start.sh` executable:
> ```bash
> chmod +x start.sh
> ```

---

## Running Different Worlds

To launch the **City 1 world**:

- Edit the `_setup.sh` file:
  - Set `ENV_NAME=city_1_world`
- Then start again:

```bash
./start.sh
```

**Note:** This does not spawn ArUco markers by default. Add them manually if needed.

---

## Important Topics and Controls

- `cf_x/odom` — UAV odometry
- `cf_x/image` — UAV camera feed
- `cf_x/battery_status` — UAV battery status
- UAV control via:
  - `cf_x/cmd_vel` — horizontal velocity control
  - `cf_x/cmd_hover` — hover control
  - `cf_x/start_trajectory`, `cf_x/upload_trajectory` — trajectory services
  - `cf_x/go_to` — direct position commands

---

## Notes

- If running multiple simulations on the same network, ensure **unique ROS_DOMAIN_ID** in `.bashrc` inside the container to avoid simulation conflicts.
- Tools included inside the container:
  - **Tmuxinator** — advanced terminal management
  - **Ranger** — CLI file browser
  - **Htop** — system monitor
  - **VS Code Dev Containers** — continue using VS Code inside Docker.

---
