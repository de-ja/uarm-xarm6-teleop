# U-ARM xArm6 Teleoperation

A standalone Python teleoperation system that uses a Feetech-powered U-ARM as
the leader for an xArm6 follower. It supports a visible xArm6 with a Robotiq
gripper in ManiSkill and an experimental, guarded physical backend for an
xArm6 with the xArm Gripper G2.

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
to raw position 2047. Servo 7 uses its separately measured
`leader.gripper_zero_position` as the open-trigger reference and
`leader.gripper_pressed_position` as the fully closed reference.

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

Install the browser operator console on a computer that will run the web
backend. Include `physical` as well when the console will command the xArm:

```bash
python -m pip install -e ".[web]"
python -m pip install -e ".[web,physical]"  # physical follower host
```

### 3. Configure serial-port access

Add the current user to Ubuntu's serial-device group:

```bash
sudo usermod -aG dialout "$USER"
```

On a follower host with local cameras, also grant persistent video-device
access so the web service does not depend on a desktop-session ACL:

```bash
sudo usermod -aG video "$USER"
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

## Follower and operator PC deployment

The operator PC runs only a modern browser. The follower PC owns every hardware
connection and runs the Python backend:

```text
Operator browser
  `- TCP 8000 (HTTP, WebSocket telemetry, MJPEG video)
       `- follower PC
            |- USB serial -> Feetech U-ARM
            |- USB -> V4L2 cameras
            `- Ethernet/xArm SDK -> xArm6 controller
```

The browser sends supervisory operations such as inspect, start, and stop. The
20 Hz leader sampling, mapping, safety checks, and xArm commands remain local to
the follower PC.

### 1. Install the follower runtime

On the follower PC, clone the repository and install only the physical and web
extras. ManiSkill and Vulkan are not required for this deployment:

```bash
git clone https://github.com/de-ja/uarm-xarm6-teleop.git
cd uarm-xarm6-teleop
conda create -n uarm-teleop --override-channels -c conda-forge python=3.11
conda activate uarm-teleop
python -m pip install --upgrade pip
python -m pip install -e ".[physical,web]"
sudo usermod -aG dialout,video "$USER"
```

Log out and back in after changing groups. Confirm that the hardware is
visible:

```bash
id -nG
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/v4l/by-id/* /dev/video*
```

For tests and frontend development, install the optional tools separately:

```bash
python -m pip install -e ".[physical,web,dev]"
conda install --override-channels -c conda-forge nodejs=22
cd frontend
npm ci
npm test
npm run check
npm run build
cd ..
pytest -q
```

The production frontend is already committed under
`src/uarm_xarm6_teleop/web/dist`; Node.js is not required just to run it.

### 2. Configure the follower

Keep robot-specific values out of the committed configuration. Create a local
override containing the actual serial device and xArm controller address:

```bash
cat > configs/local.toml <<'EOF'
[serial]
device = "/dev/ttyACM0"

[physical_xarm]
robot_ip = "192.168.1.XXX"
EOF
```

The override is merged with the conservative limits in
`configs/uarm_xarm6.toml`. Cameras are deliberately absent from the file: the
backend discovers them at runtime and prefers stable `/dev/v4l/by-id` links.

### 3. Configure the robot LAN

Use a trusted wired network or an isolated Gigabit switch. One example subnet
is:

```text
Follower PC:      192.168.1.100/24
Operator PC:      192.168.1.101/24
xArm controller:  192.168.1.XXX/24
```

Do not forward the console port from a router; the application does not provide
authentication or TLS. When UFW is enabled, restrict the console to the
operator PC:

```bash
sudo ufw allow from 192.168.1.101 to any port 8000 proto tcp
```

From the operator PC, verify the follower after starting `uarm-web`:

```bash
ping 192.168.1.100
curl http://192.168.1.100:8000/api/health
curl http://192.168.1.100:8000/api/cameras
```

#### Wi-Fi-only operator connection

On a shared or public Wi-Fi network, never bind the unauthenticated console to
`0.0.0.0` or the Wi-Fi address. Keep it on the follower's loopback interface
and carry HTTP, WebSocket telemetry, and camera streams through an encrypted
SSH tunnel instead.

On the follower PC, confirm that SSH is active, start the console locally, and
find the current Wi-Fi address:

```bash
systemctl is-active ssh
uarm-web --config configs/local.toml --host 127.0.0.1 --port 8000 --no-browser
ip -4 -brief address
```

Use SSH key authentication. On the operator PC, open the tunnel with the
follower's username and current Wi-Fi address:

```bash
ssh -N \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 \
  -L 127.0.0.1:8000:127.0.0.1:8000 \
  FOLLOWER_USER@FOLLOWER_WIFI_IP
```

Keep that terminal open and browse to `http://127.0.0.1:8000` on the operator
PC. Do not also start a local `uarm-web` process on the operator. If SSH cannot
reach the follower, the Wi-Fi likely uses client isolation; use an
institution-approved private hotspot, router, or VPN rather than exposing port
8000 directly.

### 4. Verify and launch

On the follower PC, perform the read-only checks first:

```bash
conda activate uarm-teleop
uarm-monitor --config configs/local.toml --once
uarm-real --config configs/local.toml --once
uarm-real --config configs/local.toml --robot-ip 192.168.1.XXX --inspect
```

For a trusted, isolated robot LAN, bind the packaged console only to the
follower's robot-network address:

```bash
uarm-web --config configs/local.toml --host 192.168.1.100 --port 8000 --no-browser
```

For shared Wi-Fi, use the loopback-bound SSH tunnel procedure above instead.
On the operator PC, open the URL for the selected connection method. Select one
or more discovered cameras, connect the leader, inspect the xArm, and run a dry
run. Start physical motion only after the displayed target matches the physical
xArm, all controller warnings are clear, the workspace is clear, and the
hardware emergency stop is in hand.

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
watchdog timeout, joint bounds, and xArm Gripper G2 limits. Keep
`robot_ip` blank in committed configuration; pass it at runtime or put it in a
private override TOML.

## Web operator console

The browser console wraps the guarded hardware backends in an explicit
supervisory state machine. It supports leader connection, read-only robot
inspection, dry-run mapping, guarded physical start, live joint and gripper
telemetry, dynamically discovered camera streams, session events, and software
stop.

```bash
uarm-web
```

The server binds to `127.0.0.1:8000` and opens the console in the default
browser. To load a private configuration or suppress the browser launch:

```bash
uarm-web --config configs/local.toml --no-browser
```

The normal workflow is:

1. Connect the leader. This is read-only; the leader pose and mapped target are
   sampled continuously, and torque-enabled leader IDs are shown as a warning.
2. Enter the xArm controller IP and inspect it. Inspection cannot enable
   motion.
3. Start a dry run and confirm the displayed targets first.
4. For physical motion, complete the safety dialog and type the exact inspected
   robot IP. The backend repeats all leader, alignment, controller, joint-limit,
   target-jump, gripper, and watchdog checks before entering mode 6.
5. Use **Stop motion** or the hardware emergency stop. The UI control requests
   xArm state 4 but is not an emergency stop.

### Camera streams

The operator console discovers V4L2 cameras at runtime and never stores a
`/dev/videoN` assignment in configuration. When udev provides stable
`/dev/v4l/by-id` links, those links are preferred. Multi-interface devices such
as depth cameras are grouped by their physical bus, so their preferred RGB
source appears once instead of exposing metadata and depth nodes as separate
cameras.

Use the camera checkboxes in the console to start one or more feeds. Capture is
opened only while a feed has browser subscribers, and multiple viewers of the
same source share one capture worker. Browser video uses 1280x720 at 15 FPS as
a low-latency MJPEG stream on the same HTTP connection as the console. JPEG
quality is automatic: each operator reports measured delivery latency once per
second, and the follower reduces quality quickly above the 75 ms target while
restoring it gradually on a healthy link. The active `AUTO Q...` value is shown
beside each feed.

A video failure marks that feed offline, but it does not act as an emergency
stop. Loss of the final telemetry WebSocket still requests the existing
software stop; the hardware emergency stop remains the authoritative safety
control.

### Latency measurements

The Runtime panel reports **Leader to xArm** latency during physical
teleoperation. It starts at the timestamp of a completed leader sample and ends
when mapping, safety validation, xArm SDK checks, and asynchronous command
submission have completed. It measures the local command path, not the xArm's
mechanical settling time or tracking error.

Each active camera reports capture-to-browser delivery latency beside its live
state. Every MJPEG part carries the follower's frame-capture timestamp. The
operator browser calibrates its clock offset against `/api/time`, then compares
that timestamp with the time the exact JPEG arrives. This includes follower
JPEG encoding and network delivery, but not the camera sensor's internal
exposure/readout delay or the display panel's scan-out time. The browser
recalibrates every 30 seconds to limit clock drift. The same measurement drives
automatic JPEG quality between Q35 and Q85; the stream always skips to the
newest captured frame rather than deliberately queueing stale frames.

The last browser telemetry connection closing requests a software stop. A page
reload or temporary browser/network failure may therefore stop a run, and a
stopped physical session never resumes automatically.

### Frontend development

The production frontend is compiled into the Python package. During frontend
development, run the API and Vite servers separately:

```bash
uarm-web --no-browser
cd frontend
npm ci
npm run dev
```

Open `http://127.0.0.1:5173`. Vite proxies `/api` and `/ws` to the FastAPI
server on port 8000. Validate a frontend change with:

```bash
npm test
npm run check
npm run build
```

The backend architecture and reviewed upstream references are documented in
[`docs/references.md`](docs/references.md).

## Physical xArm6 workflow

The physical backend reconstructs the useful part of the original U-ARM
[`servo2xarm.py`](https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm/blob/main/src/uarm/scripts/Follower_Arm/xarm/servo2xarm.py): it streams joint targets at
20 Hz in xArm mode 6 and commands the xArm Gripper G2. It deliberately
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

### Proportional trigger mapping and G2 force limit

Servo 7 currently uses proportional gripper motion. Its measured raw open
position (`2457`) maps to a fully open G2, and its fully pressed position
(`2757`) maps to a fully closed G2. The 300-count (`26.37 deg`) trigger stroke
is clamped at both endpoints. Toggle mode and its hysteresis settings remain in
the code and configuration but are inactive while `gripper_mode` is
`"proportional"`.

The G2 uses opening width in millimetres: `84` is open and `0` is closed. Every
motion command includes `gripper_force = 20`, on the SDK's dimensionless
`1-100` scale, and a speed of `50 mm/s`. This is a conservative starting limit,
not a value in newtons. The G2 controller enforces the force setting directly.
Additionally, a reported grasp state freezes the measured jaw position and
blocks further closing until an opening toggle; any gripper error aborts the
teleoperation run. Do not increase the force until low-risk grasp tests show it
is necessary.

## Safety boundary

ManiSkill controls only a simulated robot. The physical backend adds software
guards, but they do not replace the xArm controller's limits, a cleared physical
workspace, close supervision, or the hardware emergency stop. State 4 stops
motion without deliberately releasing the arm's brakes or disabling motor
power. Test without a payload and at low speed and G2 force before increasing
either.
