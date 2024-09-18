## Run locally

* Copy [assignment_package](DNRS_assignment_2/assignment_package) to ros 2 workspace
* In workspace 
```bash
colcon build
```
* Source install
```bash
source install/local_setup.bash
```
* Launch package
```bash
ros2 launch assignment_package launch.py
```

## Run by docker compose

* Enter to folder
```bash
cd DNRS_assignment_2
```
* Build docker images
```bash
docker compose build
```
* Start docker services
```
docker compose up -d
```
* Enter to container
```bash
docker compose run --rm ros-humble
```
* Launch package
```bash
ros2 launch assignment_package launch.py
```

## Run in dev container

* Start dev container and open shell
* Go to `ros2_ws`
```bash
cd ~/DNRS_assignment_2/ros2_ws
```
* Build the workspace
```bash
colcon build
```
* Source install
```bash
local_setup
```
* Launch package
```bash
ros2 launch assignment_package launch.py
```
