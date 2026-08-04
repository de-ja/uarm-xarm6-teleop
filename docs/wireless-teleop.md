# Wireless two-computer teleoperation

This guide runs the U-ARM leader on a laptop and the xArm6 follower on a
desktop. The computers communicate through a private laptop hotspot or private
router. The network does not need internet access.

## Architecture

```text
Laptop                                             Desktop follower
USB -> U-ARM -> uarm-leader (TCP 8765) ----------> uarm-station -> xArm SDK -> xArm6
Browser -----------------------------------------> web console (TCP 8000)
                                                    `- USB -> V4L2 cameras
```

The browser is only the operator interface. It never reads the U-ARM. The
desktop backend authenticates to the laptop's `uarm-leader` service
and requests one fresh seven-servo sample for each controller update. Mapping,
safety validation, physical commands, and the xArm watchdog remain on the
desktop.

The laptop and desktop therefore use two independent connections:

- Desktop backend connects to laptop TCP port 8765 for leader samples.
- Laptop browser connects to desktop TCP port 8000 for the UI, telemetry, and
  video.

Clicking **Connect leader** from the laptop browser lets the desktop derive the
laptop's source address automatically. The browser does not carry joint data;
it only pairs the existing direct backend connection. No address is entered in
the application.

## Requirements

Laptop:

- Ubuntu and the project repository.
- The `uarm-teleop` Conda environment.
- U-ARM Bus Servo Adapter connected by USB.
- Membership in the `dialout` group.
- Wi-Fi connected to the private router, or hosting the private hotspot.

Desktop:

- Ubuntu and the same project revision as the laptop.
- The `uarm-teleop` Conda environment.
- Wi-Fi connected to the laptop hotspot/private router.
- A separate connection to the xArm controller, normally Ethernet.
- Optional USB cameras and membership in the `video` group.

Use different IP subnets for Wi-Fi and the xArm controller network. For
example:

```text
Private Wi-Fi:
  Laptop:          10.42.0.1/24
  Desktop:         10.42.0.20/24

xArm Ethernet:
  Desktop:         192.168.1.100/24
  xArm controller: 192.168.1.XXX/24
```

## 1. Install the software

Pull the same project revision on both computers before installing the
commands.

On the laptop:

```bash
cd uarm-xarm6-teleop
git pull --ff-only
conda activate uarm-teleop
python -m pip install -e ".[remote]"
sudo usermod -aG dialout "$USER"
```

On the desktop:

```bash
cd uarm-xarm6-teleop
git pull --ff-only
conda activate uarm-teleop
python -m pip install -e ".[physical,web,remote]"
sudo usermod -aG video "$USER"
```

Log out and back in if either group membership was just changed.

To run visible simulation on the desktop as well, install the simulation extra
and its assets there:

```bash
python -m pip install -e ".[physical,web,remote,sim]"
python -m mani_skill.utils.download_asset xarm6 -y
python -m mani_skill.utils.download_asset robotiq_2f -y
```

## 2. Establish the private network

Connect the desktop to the laptop hotspot, or connect both computers to a
private router. Internet access is not required. Do not use client-isolated
public Wi-Fi such as a school guest network.

The desktop station prints its usable console addresses automatically. These
commands are still useful for network diagnostics:

```bash
ip -4 -brief address
```

If needed, verify connectivity in both directions, replacing the placeholders:

```bash
# Run on the desktop
ping -c 3 LAPTOP_PRIVATE_IP

# Run on the laptop
ping -c 3 DESKTOP_PRIVATE_IP
```

Do not continue until both addresses are reachable.

## 3. Create and transfer the shared token

The token is a password for the leader-sample connection. **Never commit it,
upload it to Git, paste it into an issue, or store it inside the repository.**
The recommended path is outside the repository.

Create it once on the laptop:

```bash
install -d -m 700 ~/.config/uarm
umask 077
python -c 'import secrets; print(secrets.token_urlsafe(32))' \
  > ~/.config/uarm/leader.token
chmod 600 ~/.config/uarm/leader.token
```

Copy the same file to the desktop with an approved secure method, such as an
encrypted USB drive or `scp`. On the desktop, place and protect it with:

```bash
install -d -m 700 ~/.config/uarm
chmod 600 ~/.config/uarm/leader.token
```

Both commands refuse token files accessible by group or other users. If the
token is ever disclosed, generate a new one and replace it on both computers.
Deleting it from the latest Git commit is not sufficient if it entered Git
history.

## 4. Configure each computer

Keep machine-specific configuration outside the repository as well.

On the laptop, create `~/.config/uarm/laptop.toml` with the actual U-ARM serial
device:

```toml
[serial]
device = "/dev/ttyACM0"
```

Find the adapter if necessary:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

On the desktop, create `~/.config/uarm/desktop.toml` with the actual xArm
controller address:

```toml
[physical_xarm]
robot_ip = "192.168.1.XXX"
```

The desktop's configured serial device is not opened in browser-paired mode.
Both computers retain the default servo ID order from the base
configuration, and the connection is rejected if those IDs differ.

## 5. Verify and start the laptop leader

Before starting the network service, perform one read-only local check:

```bash
conda activate uarm-teleop
uarm-monitor --config ~/.config/uarm/laptop.toml --once
```

Then start the leader service. It binds to all laptop interfaces and uses the
standard token path by default:

```bash
uarm-leader --config ~/.config/uarm/laptop.toml
```

If the U-ARM is already `/dev/ttyACM0`, plain `uarm-leader` is sufficient.

Leave this terminal running. Do not run `uarm-monitor` or another leader
process at the same time because only one process can own the serial port.

## 6. Start the desktop follower backend

With the xArm connected to the desktop, run and press Enter:

```bash
conda activate uarm-teleop
uarm-station
```

The station automatically loads `~/.config/uarm/desktop.toml` when present,
binds the console to the desktop's network interfaces, and prints the LAN URLs
that can be opened from the laptop. It does not ask for the laptop IP.

The default remote-leader timeout is 200 ms. Do not increase it beyond the
physical xArm watchdog timeout without reviewing the fail-safe behavior.

## 7. Connect from the laptop browser

Open one of the URLs printed by `uarm-station` on the laptop, for example:

```text
http://DESKTOP_PRIVATE_IP:8000
```

Use this first-run sequence:

1. Click **Connect leader** and confirm that all seven displayed leader joints
   update continuously.
2. Move one leader joint at a time and verify the mapped target direction.
3. Start **Dry run** and watch the loop rate and last-sample age.
4. If installed, start **Simulation** and verify the visible follower.
5. Inspect the physical xArm read-only.
6. Start physical mode only after pose, limits, controller state, workspace,
   and emergency-stop readiness have been checked.

The **Connect leader** request supplies the laptop source address to the
desktop, which then opens the authenticated WebSocket connection back to
`uarm-leader`. The browser is not in the leader-sample path. Closing the final
telemetry page still requests the existing software stop during physical
operation.

## Firewall rules

When UFW is active, restrict each service to the other computer. Run on the
laptop:

```bash
sudo ufw allow from DESKTOP_PRIVATE_IP to any port 8765 proto tcp
```

Run on the desktop:

```bash
sudo ufw allow from LAPTOP_PRIVATE_IP to any port 8000 proto tcp
```

Do not forward either port through an internet-facing router.

## Failure behavior

- The desktop requests one sample at a time, so delayed samples do not collect
  in a playback queue.
- An invalid token, incompatible protocol, wrong servo ID order, stale sequence,
  malformed sample, timeout, or Wi-Fi disconnect faults the controller.
- During physical operation, the desktop backend requests a safe stop and the
  xArm watchdog remains the final command-gap safeguard.
- A stopped or faulted physical session never resumes automatically. Reconnect
  and repeat the checks before restarting.
- The token authenticates the connection but plain `ws://` does not provide
  transport encryption. Use it only over the private WPA-protected hotspot or
  router network.

## Troubleshooting

`Connection refused` or `Could not connect`:

- Confirm `uarm-leader` is still running.
- Confirm **Connect leader** was clicked in the laptop browser, not a browser
  running on the desktop.
- From the desktop, run `nc -vz LAPTOP_PRIVATE_IP 8765`.
- Check the laptop firewall rule.

`Leader authentication failed`:

- Confirm both machines have byte-for-byte copies of the same token file.
- Generate and deploy a new token if its origin is uncertain.

`Token file ... accessible by group or other users`:

```bash
chmod 600 ~/.config/uarm/leader.token
```

`Follower servo ID order does not match the leader`:

- Pull the same project revision on both machines.
- Compare `serial.ids` in both effective configurations.

`Could not open /dev/tty...` or the serial device is busy:

- Check the device path and `dialout` membership.
- Stop every other `uarm-monitor`, `uarm-leader`, or local `uarm-web` process
  using that adapter.

The webpage does not open:

- Confirm `uarm-station` is running on the desktop.
- From the laptop, run
  `curl http://DESKTOP_PRIVATE_IP:8000/api/health`.
- Check the desktop firewall rule.

The xArm is unreachable while Wi-Fi works:

- Verify the desktop's Ethernet address and the xArm address are on the same
  subnet.
- Confirm the Wi-Fi subnet and xArm Ethernet subnet are different.
- Check `ip route` on the desktop for the route to the xArm controller.

## Shutdown

1. Stop physical motion from the UI and verify the xArm is no longer moving.
2. Disconnect the controller in the UI.
3. Stop `uarm-station` on the desktop with Ctrl-C.
4. Stop `uarm-leader` on the laptop with Ctrl-C.
