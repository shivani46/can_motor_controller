mod diff_drive;
mod protocol;

use std::{sync::Arc, time::Duration, time::Instant};

use anyhow::{Error, Result};
use diff_drive::{yaw_to_quaternion, Pose2D, WheelGeometry};
use rclrs::*;
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, Id, Socket, StandardId};

/// Shared state between the /cmd_vel subscription and the CAN-send timer.
/// Forwarding through a fixed-rate timer (rather than sending straight from
/// the subscription callback) keeps CAN frames flowing at a steady rate even
/// if /cmd_vel is published sporadically, matching the motor controller's
/// command-timeout watchdog on the other end of the bus.
struct TargetSpeed {
    left_mm_s: i16,
    right_mm_s: i16,
}

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("can_motor_controller")?;

    let can_interface = node
        .declare_parameter("can_interface")
        .default(Arc::<str>::from("vcan0"))
        .mandatory()?;
    let wheel_radius_m = node
        .declare_parameter("wheel_radius_m")
        .default(0.033)
        .mandatory()?;
    let wheel_separation_m = node
        .declare_parameter("wheel_separation_m")
        .default(0.16)
        .mandatory()?;
    let ticks_per_rev = node
        .declare_parameter("ticks_per_rev")
        .default(360_i64)
        .mandatory()?;
    let max_wheel_speed_mm_s = node
        .declare_parameter("max_wheel_speed_mm_s")
        .default(500_i64)
        .mandatory()?;
    let cmd_publish_rate_hz = node
        .declare_parameter("cmd_publish_rate_hz")
        .default(20.0)
        .mandatory()?;
    let odom_frame_id = node
        .declare_parameter("odom_frame_id")
        .default(Arc::<str>::from("odom"))
        .mandatory()?;
    let base_frame_id = node
        .declare_parameter("base_frame_id")
        .default(Arc::<str>::from("base_link"))
        .mandatory()?;

    let geometry = WheelGeometry {
        wheel_radius_m: wheel_radius_m.get(),
        wheel_separation_m: wheel_separation_m.get(),
        ticks_per_rev: ticks_per_rev.get(),
    };
    let max_wheel_speed_mm_s = max_wheel_speed_mm_s.get() as f64;

    let can_tx = CanSocket::open(&can_interface.get())?;
    let can_rx = CanSocket::open(&can_interface.get())?;
    println!(
        "[can_motor_controller] bridging /cmd_vel <-> {} <-> /odom",
        can_interface.get()
    );

    // --- /cmd_vel -> CAN ---
    let worker = node.create_worker(TargetSpeed { left_mm_s: 0, right_mm_s: 0 });

    let _cmd_vel_sub = worker.create_subscription::<geometry_msgs::msg::Twist, _>(
        "cmd_vel",
        move |target: &mut TargetSpeed, msg: geometry_msgs::msg::Twist| {
            let (left_mm_s, right_mm_s) =
                geometry.cmd_vel_to_wheel_mm_s(msg.linear.x, msg.angular.z);
            target.left_mm_s = left_mm_s.clamp(-max_wheel_speed_mm_s, max_wheel_speed_mm_s) as i16;
            target.right_mm_s = right_mm_s.clamp(-max_wheel_speed_mm_s, max_wheel_speed_mm_s) as i16;
        },
    )?;

    let cmd_can_id = StandardId::new(protocol::CMD_VEL_CAN_ID).expect("valid standard CAN ID");
    let _cmd_timer = worker.create_timer_repeating(
        Duration::from_secs_f64(1.0 / cmd_publish_rate_hz.get()),
        move |target: &mut TargetSpeed| {
            let data = protocol::encode_cmd_frame(target.left_mm_s, target.right_mm_s);
            let frame = CanFrame::new(cmd_can_id, &data).expect("8 bytes fits a classic CAN frame");
            if let Err(err) = can_tx.write_frame(&frame) {
                eprintln!("[can_motor_controller] failed to send CAN cmd frame: {err}");
            }
        },
    )?;

    // --- CAN -> /odom ---
    let odom_pub = node.create_publisher::<nav_msgs::msg::Odometry>("odom")?;
    let clock = node.get_clock();
    let wheel_separation_m = geometry.wheel_separation_m;
    let mm_per_tick = geometry.mm_per_tick();

    std::thread::spawn(move || {
        let mut pose = Pose2D::default();
        let mut last_ticks: Option<(i32, i32)> = None;
        let mut last_update = Instant::now();

        loop {
            let frame = match can_rx.read_frame() {
                Ok(frame) => frame,
                Err(err) => {
                    eprintln!("[can_motor_controller] CAN read error: {err}");
                    std::thread::sleep(Duration::from_millis(100));
                    continue;
                }
            };

            let Id::Standard(id) = frame.id() else { continue };
            if id.as_raw() != protocol::ENCODER_CAN_ID {
                continue;
            }
            let Some((left_ticks, right_ticks)) = protocol::decode_encoder_frame(frame.data())
            else {
                continue;
            };

            let now = Instant::now();
            let dt_s = (now - last_update).as_secs_f64();
            last_update = now;

            let Some((prev_left, prev_right)) = last_ticks.replace((left_ticks, right_ticks))
            else {
                continue;
            };
            let delta_left_mm = left_ticks.wrapping_sub(prev_left) as f64 * mm_per_tick;
            let delta_right_mm = right_ticks.wrapping_sub(prev_right) as f64 * mm_per_tick;

            let update = pose.integrate(
                delta_left_mm / 1000.0,
                delta_right_mm / 1000.0,
                wheel_separation_m,
                dt_s,
            );
            pose = update.pose;

            // `Time::to_ros_msg()` returns rclrs's `ros-env` testing crate's Time type,
            // not the real `builtin_interfaces` crate used by std_msgs::msg::Header, so
            // the timestamp is built by hand from the raw nanosecond count instead.
            let now_ns = clock.now().nsec;
            let stamp = builtin_interfaces::msg::Time {
                sec: now_ns.div_euclid(1_000_000_000) as i32,
                nanosec: now_ns.rem_euclid(1_000_000_000) as u32,
            };

            let odom = nav_msgs::msg::Odometry {
                header: std_msgs::msg::Header {
                    stamp,
                    frame_id: odom_frame_id.get().to_string(),
                },
                child_frame_id: base_frame_id.get().to_string(),
                pose: geometry_msgs::msg::PoseWithCovariance {
                    pose: geometry_msgs::msg::Pose {
                        position: geometry_msgs::msg::Point {
                            x: pose.x,
                            y: pose.y,
                            z: 0.0,
                        },
                        orientation: yaw_to_quaternion(pose.theta),
                    },
                    ..Default::default()
                },
                twist: geometry_msgs::msg::TwistWithCovariance {
                    twist: geometry_msgs::msg::Twist {
                        linear: geometry_msgs::msg::Vector3 {
                            x: update.linear_x_m_s,
                            y: 0.0,
                            z: 0.0,
                        },
                        angular: geometry_msgs::msg::Vector3 {
                            x: 0.0,
                            y: 0.0,
                            z: update.angular_z_rad_s,
                        },
                    },
                    ..Default::default()
                },
            };

            if let Err(err) = odom_pub.publish(&odom) {
                eprintln!("[can_motor_controller] failed to publish odom: {err}");
            }
        }
    });

    println!("[can_motor_controller] spinning...");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
