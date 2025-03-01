# Autodrive F1tenth Development Environment

This repository provides a fully configured development environment for the [Autodrive F1tenth Sim Racing competition](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing/). Using vscode, Docker, and devcontainers, it simplifies the setup process, allowing you to focus on developing and testing your autonomous racing stack.

This environment was battle-tested by the VAUL team during the [2nd F1tenth Sim Racing League](https://autodrive-ecosystem.github.io/competitions/f1tenth-sim-racing-cdc-2024/), ensuring reliability and ease of use.

## Prerequisites

- vscode
- Docker
- Docker Compose
- Foxglove

## Quick Rundown

This repository uses two Docker containers: **devkit** and **simulator**. Those containers are orchestrated with docker-compose.

### Simulator container

The simulator container runs the car's physics using the Unity-based [Autodrive Simulator](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE/tree/AutoDRIVE-Simulator). It is built on top of the official [Autodrive F1tenth Sim Image](https://hub.docker.com/r/autodriveecosystem/autodrive_f1tenth_sim). We run it in headless mode to save compute power.

### Devkit container

The devkit container is where development takes place. It is designed to be opened as a devcontainer in vscode. This container will receive the WebSocket messages from the simulator (port 4567) and convert them to ROS2 messages that your autonomous racing stack can use. It also has a Foxglove bridge on port (8765) to allow for visualization on the host machine.

On devcontainer startup, it installs every vscode extensions, build your code and start two [GNU Screen](https://www.gnu.org/software/screen/) sessions running:

- The Foxglove bridge for visualization.
- The Autodrive bridge for message conversion.

Your code runs inside the devkit container with the `src` directory from this repository mounted as a Docker volume. Like the simulator, this container is based on the official [Autodrive F1tenth Devkit Image](https://hub.docker.com/r/autodriveecosystem/autodrive_f1tenth_api).

If your host computer is properly configured, your GPU should also be forwarded to the devkit container, which you can test with this command: `docker exec -it devkit nvidia-smi`.

## Installation

First, we need to clone the repository and make sure that the images are building.

1. Cloning the repo

```bash
git clone git@github.com:vaul-ulaval/autodrive_sim_stack.git --recurse-submodules
cd autodrive_sim_stack
```

2. Building and launching the containers

```bash
docker compose up --build -d
```

3. Verify that both containers are active (devkit and simulator)

```bash
docker ps
```

4. Open Foxglove on `http://localhost:8765` and see the lidar scan

## Development

Once all the installation steps are completed, we can move on to the vscode and devcontainer integration:

1. Open the repo in vscode
2. Install the devcontainer vscode extension
3. Restart vscode
4. Once reopened, do Ctrl+Shift+P then `Dev Containers: Reopen in Dev Container`
5. You should now have a vscode backend running in the container so you should see the ros2 workspace.
6. Test a colcon build with Ctrl+Shift+b
7. If everything builds, you are now ready to develop! You can also test a keyboard_teleop for fun

```bash
ros2 run autodrive_f1tenth teleop_keyboard
```

## Submitting Competition Docker Image

For the competitions, teams are required to build and push a docker image running their code. Here is how we do it.

1. Add a GNU screen session that launches your ROS nodes in [devkit-startup.bash](./devkit-startup.bash).
2. Test that your autonomous racing stack actually starts

```bash
docker compose up --build -d
```

3. If it properly starts up, build the final docker image

```bash
docker build -f devkit.Dockerfile --target final -t <your-image-name> .
```

3. Push your image to Dockerhub

```bash
docker login
docker push <your-image-name>
```

4. Make sure you sure you share docker image with the competition organizers.

5. Great job, you have now submitted your code for the competition!

## Contributions

If you have any ideas on how to improve the development environment, feel free to open a Pull Request or a Github Issue. We hope we can make this development environment even better with your help!
