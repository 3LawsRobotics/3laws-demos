# Warehouse demo

<br />
<p align="center">
  <a href="https://github.com/3LawsRobotics/3laws-demos">
    <img src="media/demo.png" alt="Logo" width="640" height="435">
  </a>

  <h3 align="center">Demo of Supervisor for teleoperated AMRs.</h3>
</p>

## Requirements

This demo has been tested in `Ubuntu 22.04`, with `ROS2 Humble` and `Gazebo Fortress`. Make sure to install the following dependencies:

1. ROS2 Humble (the `ros-humble-desktop` package): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
2. Gazebo Sim (`Fortress` version): https://gazebosim.org/docs/fortress/install_ubuntu/
3. xterm: `sudo apt install xterm`

## Notes
* For some reason, the gazebo physics make the robot wheels skid at the bigging of the simulation. Once you drive a couple meters away from the initial location, the robot should behave as expected.

* In this configuration of the Supervisor, obstacles are detected via the laserscan. This means the supervisor has no information about direction the workers are walking towards. Given such perception modality, the supervisor will not let you drive into the workers, and stop or backup if a worker is getting close to it, but it won't prevent adversarial workers from colliding with it. Ensuring avoidance of adversarial agents is a capability of **3Laws Supervisor Pro**. Contact info@3laws.io to inquire about a demo of those capabilities.

## Getting started

1. In order to run that demo, you must first install 3Laws Supervisor on your machine. For that follow point 1. in the getting started instructions of [3Laws doc](https://docs.3laws.io/en/latest/sources/getting_started.html#install-supervisor).

2. Then, clone the current repository into a ros workspace:
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone git@github.com:3LawsRobotics/3laws-demos.git
```

3. Update the `lll_warehouse_demo/config/amr_supervisor_config_teleop.yaml` config file with your license token (you can get trial license at https://3laws.io/#trial_download):
```
24.   license_token: "" <<  Put token between the quots
```

4. Have rosdep install all the packages dependencies for you:
```
cd ~/ros_ws
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO"
```

5. Build the workspace:
```
cd ~/ros_ws
colcon build --symlink-install
```
> :warning: The `--symlink-install` is important as you'll see in the next steps

6. You should now be able to run the demo:
```bash
source ~/ros_ws/install/setup.bash
ros2 launch lll_warehouse_demo demo.launch.py
```
To drive the robot around, use the "W", "A", "Q", "E" keys as explained in the `xterm` window of the `lll_teleop_twist_keyboard` node.

## Visualization
You can start a pre-configured rviz window for what's happening with:

```bash
source ~/ros_ws/install/setup.bash
ros2 launch lll_warehouse_demo rviz.launch.py
```

## Using the control panel
You can also use the 3Laws control panel to configure and monitor the Supervisor.

1. For that, first create a symlink for `amr_supervisor_config_teleop.yaml` into `~/.3laws/config/supervisor.yaml`:
```bash
ln -sf "$HOME/ros_ws/src/3laws-demos/lll_warehouse_demo/config/amr_supervisor_config_teleop.yaml" "$HOME/.3laws/config/supervisor.yaml"
```

2. Then, start the control panel backend:
```bash
3laws control-panel run
```

3. You can now navigate to the control panel interface in your favorite browser at http://localhost:8000/, and follow the [control panel documentation](https://docs.3laws.io/en/latest/sources/user_guide/control_panel.html).
