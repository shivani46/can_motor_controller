# can_motor_controller

A ROS 2 (Jazzy) Rust node, built on [`rclrs`](https://github.com/ros2-rust/ros2_rust), that bridges `/cmd_vel` to a simulated CAN-connected diff-drive motor controller and publishes `/odom` from simulated encoder feedback.

It's a testbed for the kind of bridge you'd write for a real CAN-based motor controller board (e.g. an ODrive, a custom STM32 board, etc.) — everything downstream of the CAN bus is swappable for real hardware without touching the ROS side.

## What it does

- Subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`), converts body-frame velocity into left/right wheel speeds using standard diff-drive kinematics, and streams them out as CAN frames at a fixed rate.
- Reads encoder-tick CAN frames coming back from the motor controller, integrates them into a 2D pose, and publishes `/odom` (`nav_msgs/msg/Odometry`).
- Ships with `sim/robot_can_sim.py`, a `python-can` script that plays the role of the motor controller: it consumes the command frames, simulates motor acceleration + a command-timeout safety stop, and emits encoder frames back.
- Runs entirely over a virtual CAN interface (`vcan0`) via SocketCAN, so no hardware is required.

## Architecture

```
 ROS 2 graph                     can_motor_controller (Rust)                SocketCAN (vcan0)          robot_can_sim.py
┌────────────┐   Twist          ┌─────────────────────────────┐   0x100 frame  ┌───────────┐   0x100 frame   ┌──────────────────┐
│ /cmd_vel   ├─────────────────►│ Worker<TargetSpeed>          ├───────────────►│           ├────────────────►│ decode cmd       │
│ publisher  │                  │  - subscription callback      │  (20 Hz timer) │  vcan0    │                 │ ramp motor speed │
└────────────┘                  │  - repeating CAN-send timer   │                │           │                 │ toward target    │
                                 └─────────────────────────────┘                │           │                 └────────┬─────────┘
┌────────────┐   Odometry       ┌─────────────────────────────┐   0x200 frame  │           │   0x200 frame            │
│ /odom      │◄─────────────────┤ background thread             │◄───────────────┤           │◄─────────────────────────┘
│ subscriber │                  │  - blocking CAN read           │  (50 Hz)       └───────────┘   cumulative encoder ticks
└────────────┘                  │  - tick-delta -> pose integrate│
                                 │  - publish Odometry            │
                                 └─────────────────────────────┘
```

Two independent data paths, deliberately decoupled from ROS message timing:

- **/cmd_vel → CAN**: the subscription callback only updates a small shared `TargetSpeed { left_mm_s, right_mm_s }` state (via an `rclrs::Worker`). A separate repeating timer (`cmd_publish_rate_hz`, default 20 Hz) is what actually sends the CAN frame. This means CAN commands keep flowing at a steady rate even if `/cmd_vel` is published sporadically — which matters because the simulator (and any real motor controller) has a command-timeout safety stop that will zero the motors out if frames stop arriving.
- **CAN → /odom**: a dedicated background thread blocks on `CanSocket::read_frame()` so it isn't tied to the ROS executor's spin loop, integrates wheel-tick deltas into pose using the standard midpoint/exact-arc diff-drive model, and publishes directly.

## CAN protocol

Wire format shared between `src/protocol.rs` (Rust) and `sim/robot_can_sim.py` (Python) — keep both in sync if you change it:

| CAN ID | Direction | Bytes | Contents |
|--------|-----------|-------|----------|
| `0x100` | host → robot | 8 | `i16 left_mm_s, i16 right_mm_s` (little-endian), 4 reserved bytes. Target linear speed of each wheel, mm/s. |
| `0x200` | robot → host | 8 | `i32 left_ticks, i32 right_ticks` (little-endian). Cumulative encoder ticks per wheel. |

The controller sends `0x100` continuously at `cmd_publish_rate_hz`; the simulator streams `0x200` continuously at its own feedback rate (default 50 Hz), whether or not the robot is moving.

## Parameters

| Name | Default | Meaning |
|------|---------|---------|
| `can_interface` | `vcan0` | SocketCAN interface name |
| `wheel_radius_m` | `0.033` | Wheel radius, meters |
| `wheel_separation_m` | `0.16` | Distance between the two wheels, meters |
| `ticks_per_rev` | `360` | Encoder ticks per wheel revolution |
| `max_wheel_speed_mm_s` | `500` | Clamp applied to commanded wheel speed |
| `cmd_publish_rate_hz` | `20.0` | Rate at which `0x100` command frames are sent |
| `odom_frame_id` | `odom` | `/odom` header frame |
| `base_frame_id` | `base_link` | `/odom` child frame |

## Running it

Bring up the virtual CAN interface (needed once per boot — WSL2/most systems don't persist it across reboots):

```bash
~/ros2_rust_ws/src/can_motor_controller/scripts/setup_vcan.sh up
```

Then launch the simulator and the node together:

```bash
source ~/ros2_rust_ws/install/setup.bash
ros2 launch can_motor_controller can_motor_controller.launch.xml
# or on a different vcan interface:
ros2 launch can_motor_controller can_motor_controller.launch.xml can_interface:=vcan1
```

Drive it and watch odometry update:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
ros2 topic echo /odom
```

To run the two pieces by hand instead of via launch (useful for debugging):

```bash
python3 ~/ros2_rust_ws/src/can_motor_controller/sim/robot_can_sim.py --channel vcan0
ros2 run can_motor_controller can_motor_controller
```

Building and testing:

```bash
cd ~/ros2_rust_ws
colcon build --packages-select can_motor_controller
cd src/can_motor_controller && cargo test   # protocol + diff-drive kinematics unit tests
```

## An rclrs gotcha: `Time::to_ros_msg()`

`rclrs::Time::to_ros_msg()` looks like the obvious way to stamp a message header:

```rust
header.stamp = clock.now().to_ros_msg()?;
```

This fails to compile once you're also using a "real" workspace message crate (here, `std_msgs::msg::Header`), with an error like:

```
expected `builtin_interfaces::msg_idiomatic::Time`, found `ros_env::builtin_interfaces::msg::Time`
```

The cause: `rclrs` depends on a crate called `ros-env` (visible in its own `Cargo.toml`) that bundles its *own* copies of common message types, used internally for `rclrs`'s doctests/examples. `Time::to_ros_msg()` returns `ros_env::builtin_interfaces::msg::Time`, which is a different, incompatible type from the real `builtin_interfaces::msg::Time` that `std_msgs::msg::Header::stamp` actually expects — even though they're structurally identical and even named the same. Cargo has no reason to unify them since they come from genuinely different crates in the dependency graph.

The fix is to skip the convenience method and build the timestamp by hand from the raw nanosecond count instead (see `src/main.rs`):

```rust
let now_ns = clock.now().nsec;
let stamp = builtin_interfaces::msg::Time {
    sec: now_ns.div_euclid(1_000_000_000) as i32,
    nanosec: now_ns.rem_euclid(1_000_000_000) as u32,
};
```

This requires depending directly on the `builtin_interfaces` crate (which resolves through this workspace's `.cargo/config.toml` patch to the real generated bindings, the same ones `std_msgs`/`nav_msgs`/`geometry_msgs` use) rather than relying on whatever `rclrs` re-exports.

## License

Apache License 2.0 (see `package.xml`).
