# U-ARM xArm6 Teleoperation

A standalone Python teleoperation system that uses a Feetech-powered U-ARM as
the leader for an xArm6 follower. The current backend displays and controls a
visible xArm6 with a Robotiq gripper in ManiSkill. A physical xArm6 backend is
planned.

The runtime servo path is read-only: it does not change IDs, calibration,
EEPROM, torque state, or goal positions.

## Architecture

```text
Feetech U-ARM -> calibrated joint angles -> xArm6 mapping -> backend
                                                        |- ManiSkill
                                                        `- physical xArm6 (planned)
```

The leader is expected to have Feetech STS servos at IDs 1-7, ordered from the
base to the trigger. The CAD/rest pose must already be persistently calibrated
to raw position 2047.

## New computer setup

The recommended host is Ubuntu 22.04 or 24.04 with a Vulkan-capable GPU and its
vendor driver installed. ManiSkill and its rendering stack should run in a
dedicated Python 3.11 environment.

### 1. Clone the repository

```bash
git clone https://github.com/de-ja/uarm-xarm6-teleop.git
cd uarm-xarm6-teleop
```

### 2. Create the Python environment

```bash
conda create -n uarm-teleop --override-channels -c conda-forge python=3.11
conda activate uarm-teleop
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

For development and tests, also install the development tools:

```bash
python -m pip install -e ".[dev]"
python -m unittest discover -s tests -v
```

### 3. Configure serial-port access

Add the current user to Ubuntu's serial-device group:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in for the group change to take effect. After connecting the
Bus Servo Adapter, find its device name:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

The default is `/dev/ttyACM0`. Use `--device` or edit the TOML configuration if
the adapter receives a different name.

### 4. Verify Vulkan

Install the Ubuntu Vulkan diagnostic utility if it is not already available:

```bash
sudo apt install vulkan-tools
vulkaninfo --summary
```

Resolve Vulkan errors before starting the graphical simulation. The read-only
hardware check does not require Vulkan.

### 5. Download the xArm6 assets

ManiSkill installs robot assets separately from its Python package. Download
the xArm6 and Robotiq files once per computer:

```bash
python -m mani_skill.utils.download_asset xarm6 -y
python -m mani_skill.utils.download_asset robotiq_2f -y
```

### 6. Verify the U-ARM and simulator

With the U-ARM connected and supported in its calibrated CAD pose:

```bash
uarm-monitor --once
uarm-sim --check-only
uarm-sim
```

The final command should open a window containing the visible xArm6 follower.

## Check the leader

Connect the Bus Servo Adapter, leave motor torque disabled, and read one sample:

```bash
uarm-monitor --once
```

For the interactive terminal monitor:

```bash
uarm-monitor
```

Both commands default to `/dev/ttyACM0` at 1,000,000 baud. Override the device
when necessary:

```bash
uarm-monitor --device /dev/ttyUSB0
```

## Run the visible follower

Start in the calibrated CAD pose, then run:

```bash
uarm-sim
```

This opens an `xarm6_robotiq` follower in ManiSkill's `Empty-v1` scene. Press
Ctrl-C to stop.

To verify the hardware path without initializing Vulkan or opening a window:

```bash
uarm-sim --check-only
```

## Tune the follower reference pose

Open a simulation that does not step physics or connect to the U-ARM:

```bash
uarm-tune-reference
```

The xArm6 is selected automatically. In the **Articulation** window, drag the
first six sliders (`joint1` through `joint6`) and ignore the gripper sliders
and `+` buttons.
Close the viewer when the pose matches the physical U-ARM. The command prints a
`reference_degrees = [...]` line to copy into the `[xarm6]` section of the TOML
configuration.

## Configuration

Runtime configuration is in [`configs/uarm_xarm6.toml`](configs/uarm_xarm6.toml),
and every command loads it automatically. To use a different configuration,
pass it explicitly:

```bash
uarm-sim --config configs/uarm_xarm6.toml
```

If a leader joint has the opposite sign, change its corresponding entry in
`leader.directions` from `1` to `-1`. The `[xarm6]` configuration separately
defines the follower's initial `reference_degrees` and `joint_directions`.
The calibrated U-ARM CAD pose maps to the configured xArm6 reference pose.
Joint-specific follower signs match the observed Feetech U-ARM orientation;
subsequent U-ARM angles are applied as relative joint displacements.

## Safety boundary

ManiSkill controls only a simulated robot. A physical xArm6 will require a
separate backend using the UFACTORY xArm SDK, together with joint limits,
communication timeouts, rate limits, and an operator deadman control.
