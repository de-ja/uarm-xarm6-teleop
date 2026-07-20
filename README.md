# U-ARM xArm6 Teleoperation

A standalone Python teleoperation system that uses a Feetech-powered U-ARM as
the leader for an xArm6 follower. It supports a visible xArm6 with a Robotiq
gripper in ManiSkill and an experimental, guarded physical backend for an
xArm6 with the standard xArm Gripper.

The runtime servo path is read-only: it does not change IDs, calibration,
EEPROM, torque state, or goal positions.

## Architecture

```text
Feetech U-ARM -> calibrated joint angles -> xArm6 mapping -> backend
                                                        |- ManiSkill
                                                        `- physical xArm6
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

Install UFACTORY's maintained Python SDK only on a computer that will connect
to the physical follower:

```bash
python -m pip install -e ".[physical]"
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

The `[physical_xarm]` section contains the physical control rate, conservative
joint speed and acceleration, startup tolerances, target-jump threshold,
watchdog timeout, joint bounds, and standard xArm Gripper positions. Keep
`robot_ip` blank in committed configuration; pass it at runtime or put it in a
private override TOML.

## Physical xArm6 workflow

The physical backend reconstructs the useful part of the original U-ARM
[`servo2xarm.py`](https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm/blob/main/src/uarm/scripts/Follower_Arm/xarm/servo2xarm.py): it streams joint targets at
20 Hz in xArm mode 6 and commands the standard xArm Gripper. It deliberately
does not copy the original script's hard-coded IP, automatic error handling,
or automatic startup motion.

First validate the complete mapping without connecting to the xArm. This reads
the U-ARM and prints physical targets, but never imports or opens the xArm SDK:

```bash
uarm-real --once
uarm-real
```

After the xArm IP is known and the computer is on the same network, inspect the
follower. Inspection reads status, errors, joints, and gripper position; it does
not enable motion:

```bash
uarm-real --robot-ip 192.168.1.XXX --inspect
```

Before enabling motion:

1. Support the U-ARM in its calibrated CAD pose with leader torque disabled.
2. In xArm Studio, place the physical xArm6 within 5 degrees per joint of the
   target printed by `uarm-real --once`. The program will not move it into place.
3. Resolve every controller error and warning in xArm Studio.
4. Clear the workspace, keep the emergency stop in hand, and use a second person
   as a spotter for the first test.
5. Start at low configured speed and be ready to press the hardware emergency
   stop.

Then start physical teleoperation:

```bash
uarm-real --robot-ip 192.168.1.XXX --enable-motion
```

The program prints both poses and requires the exact robot IP to be typed before
it enables mode 6. Ctrl-C, an SDK error or warning, a disconnected follower, an
out-of-range target, a target jump over the configured threshold, or a command
gap longer than 250 ms stops streaming and requests xArm state 4. Restart the
program after any safety stop; it never clears controller faults automatically.

## Safety boundary

ManiSkill controls only a simulated robot. The physical backend adds software
guards, but they do not replace the xArm controller's limits, a cleared physical
workspace, close supervision, or the hardware emergency stop. State 4 stops
motion without deliberately releasing the arm's brakes or disabling motor
power. Test without a payload and at low speed before increasing either.
