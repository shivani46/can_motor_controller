#!/usr/bin/env python3
"""Diff-drive robot simulator that sits on the other end of vcan0.

Listens for wheel-speed command frames and streams back cumulative
encoder-tick frames, standing in for the real motor controller board
that can_motor_controller (the Rust ROS2 node) will talk to over CAN.

Wire protocol (must stay in sync with the Rust node's CAN frame codec):

  CMD_VEL_CAN_ID  = 0x100  (host -> robot, 8 bytes)
      i16 left_mm_s   (little-endian, target left wheel linear speed, mm/s)
      i16 right_mm_s  (little-endian, target right wheel linear speed, mm/s)
      4 reserved bytes (zero)

  ENCODER_CAN_ID  = 0x200  (robot -> host, 8 bytes)
      i32 left_ticks   (little-endian, cumulative left encoder ticks)
      i32 right_ticks  (little-endian, cumulative right encoder ticks)
"""
import argparse
import math
import struct
import threading
import time

import can

CMD_VEL_CAN_ID = 0x100
ENCODER_CAN_ID = 0x200
CMD_STRUCT = struct.Struct("<hh4x")   # left_mm_s, right_mm_s, padding
ENCODER_STRUCT = struct.Struct("<ii")  # left_ticks, right_ticks


class DiffDriveRobot:
    """Simulated motor + encoder dynamics for one diff-drive robot."""

    def __init__(self, wheel_radius_m, ticks_per_rev, max_wheel_speed_mm_s, accel_mm_s2):
        self.ticks_per_mm = ticks_per_rev / (2 * math.pi * wheel_radius_m * 1000.0)
        self.max_wheel_speed_mm_s = max_wheel_speed_mm_s
        self.accel_mm_s2 = accel_mm_s2

        self.target_left_mm_s = 0.0
        self.target_right_mm_s = 0.0
        self.actual_left_mm_s = 0.0
        self.actual_right_mm_s = 0.0
        self.left_ticks = 0.0
        self.right_ticks = 0.0
        self.lock = threading.Lock()

    def set_target(self, left_mm_s, right_mm_s):
        clamp = self.max_wheel_speed_mm_s
        with self.lock:
            self.target_left_mm_s = max(-clamp, min(clamp, left_mm_s))
            self.target_right_mm_s = max(-clamp, min(clamp, right_mm_s))

    def stop(self):
        with self.lock:
            self.target_left_mm_s = 0.0
            self.target_right_mm_s = 0.0

    def step(self, dt):
        """Advance motor dynamics and encoder accumulators by dt seconds."""
        max_delta = self.accel_mm_s2 * dt
        with self.lock:
            self.actual_left_mm_s += _clamp(
                self.target_left_mm_s - self.actual_left_mm_s, -max_delta, max_delta
            )
            self.actual_right_mm_s += _clamp(
                self.target_right_mm_s - self.actual_right_mm_s, -max_delta, max_delta
            )
            self.left_ticks += self.actual_left_mm_s * dt * self.ticks_per_mm
            self.right_ticks += self.actual_right_mm_s * dt * self.ticks_per_mm
            return self.actual_left_mm_s, self.actual_right_mm_s, self.left_ticks, self.right_ticks


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def cmd_listener(bus, robot, last_cmd_time, stop_event):
    while not stop_event.is_set():
        msg = bus.recv(timeout=0.2)
        if msg is None or msg.arbitration_id != CMD_VEL_CAN_ID or msg.dlc < 4:
            continue
        left_mm_s, right_mm_s = CMD_STRUCT.unpack(msg.data.ljust(8, b"\x00"))
        robot.set_target(left_mm_s, right_mm_s)
        last_cmd_time[0] = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--channel", default="vcan0")
    parser.add_argument("--rate", type=float, default=50.0, help="encoder feedback rate, Hz")
    parser.add_argument("--wheel-radius", type=float, default=0.033, help="meters")
    parser.add_argument("--ticks-per-rev", type=int, default=360)
    parser.add_argument("--max-wheel-speed", type=float, default=500.0, help="mm/s")
    parser.add_argument("--accel", type=float, default=800.0, help="mm/s^2, simulated motor inertia")
    parser.add_argument("--cmd-timeout", type=float, default=0.5, help="seconds before safety stop")
    parser.add_argument("--quiet", action="store_true", help="suppress periodic status prints")
    args = parser.parse_args()

    robot = DiffDriveRobot(args.wheel_radius, args.ticks_per_rev, args.max_wheel_speed, args.accel)
    bus = can.interface.Bus(channel=args.channel, interface="socketcan")

    last_cmd_time = [time.monotonic()]
    stop_event = threading.Event()
    listener_thread = threading.Thread(
        target=cmd_listener, args=(bus, robot, last_cmd_time, stop_event), daemon=True
    )
    listener_thread.start()

    print(f"[robot_can_sim] listening on {args.channel}, feedback @ {args.rate} Hz "
          f"(cmd=0x{CMD_VEL_CAN_ID:03X}, encoder=0x{ENCODER_CAN_ID:03X})")

    dt = 1.0 / args.rate
    last_print = 0.0
    try:
        while True:
            loop_start = time.monotonic()
            if loop_start - last_cmd_time[0] > args.cmd_timeout:
                robot.stop()

            actual_left, actual_right, left_ticks, right_ticks = robot.step(dt)

            frame = can.Message(
                arbitration_id=ENCODER_CAN_ID,
                is_extended_id=False,
                data=ENCODER_STRUCT.pack(int(round(left_ticks)), int(round(right_ticks))),
            )
            bus.send(frame)

            if not args.quiet and loop_start - last_print > 0.5:
                print(f"L: target={robot.target_left_mm_s:7.1f} actual={actual_left:7.1f} mm/s "
                      f"ticks={int(left_ticks):8d} | "
                      f"R: target={robot.target_right_mm_s:7.1f} actual={actual_right:7.1f} mm/s "
                      f"ticks={int(right_ticks):8d}")
                last_print = loop_start

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, dt - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        listener_thread.join(timeout=1.0)
        bus.shutdown()


if __name__ == "__main__":
    main()
