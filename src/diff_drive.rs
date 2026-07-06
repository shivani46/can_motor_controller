//! Diff-drive kinematics: cmd_vel <-> per-wheel speeds, and wheel-tick
//! integration into a 2D pose for odometry.

#[derive(Clone, Copy)]
pub struct WheelGeometry {
    pub wheel_radius_m: f64,
    pub wheel_separation_m: f64,
    pub ticks_per_rev: i64,
}

impl WheelGeometry {
    pub fn mm_per_tick(&self) -> f64 {
        (2.0 * std::f64::consts::PI * self.wheel_radius_m * 1000.0) / self.ticks_per_rev as f64
    }

    /// Converts a body-frame velocity command into (left, right) wheel speeds in mm/s.
    pub fn cmd_vel_to_wheel_mm_s(&self, linear_x_m_s: f64, angular_z_rad_s: f64) -> (f64, f64) {
        let half_separation_mm = self.wheel_separation_m * 1000.0 / 2.0;
        let linear_mm_s = linear_x_m_s * 1000.0;
        let angular_mm_s = angular_z_rad_s * half_separation_mm;
        (linear_mm_s - angular_mm_s, linear_mm_s + angular_mm_s)
    }
}

#[derive(Default, Clone, Copy)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

/// Result of integrating one encoder update: the new pose plus the body-frame
/// velocity estimate over that update (for the twist half of `Odometry`).
pub struct OdomUpdate {
    pub pose: Pose2D,
    pub linear_x_m_s: f64,
    pub angular_z_rad_s: f64,
}

impl Pose2D {
    /// Integrates wheel travel (in meters) over `dt` seconds using the
    /// midpoint (exact-arc) diff-drive model.
    pub fn integrate(&self, left_dist_m: f64, right_dist_m: f64, wheel_separation_m: f64, dt_s: f64) -> OdomUpdate {
        let d_center = (left_dist_m + right_dist_m) / 2.0;
        let d_theta = (right_dist_m - left_dist_m) / wheel_separation_m;
        let mid_theta = self.theta + d_theta / 2.0;

        let pose = Pose2D {
            x: self.x + d_center * mid_theta.cos(),
            y: self.y + d_center * mid_theta.sin(),
            theta: self.theta + d_theta,
        };

        let (linear_x_m_s, angular_z_rad_s) = if dt_s > 0.0 {
            (d_center / dt_s, d_theta / dt_s)
        } else {
            (0.0, 0.0)
        };

        OdomUpdate { pose, linear_x_m_s, angular_z_rad_s }
    }
}

pub fn yaw_to_quaternion(yaw: f64) -> geometry_msgs::msg::Quaternion {
    geometry_msgs::msg::Quaternion {
        x: 0.0,
        y: 0.0,
        z: (yaw / 2.0).sin(),
        w: (yaw / 2.0).cos(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_line_travel_moves_along_x() {
        let pose = Pose2D::default();
        let update = pose.integrate(0.1, 0.1, 0.16, 1.0);
        assert!((update.pose.x - 0.1).abs() < 1e-9);
        assert!((update.pose.y).abs() < 1e-9);
        assert!((update.pose.theta).abs() < 1e-9);
        assert!((update.linear_x_m_s - 0.1).abs() < 1e-9);
    }

    #[test]
    fn in_place_rotation_only_changes_theta() {
        let pose = Pose2D::default();
        let update = pose.integrate(-0.05, 0.05, 0.16, 1.0);
        assert!((update.pose.x).abs() < 1e-9);
        assert!((update.pose.y).abs() < 1e-9);
        assert!(update.pose.theta > 0.0);
    }

    #[test]
    fn wheel_mm_s_matches_expected_diff_drive_equations() {
        let geometry = WheelGeometry {
            wheel_radius_m: 0.033,
            wheel_separation_m: 0.16,
            ticks_per_rev: 360,
        };
        let (left, right) = geometry.cmd_vel_to_wheel_mm_s(0.2, 0.0);
        assert!((left - 200.0).abs() < 1e-9);
        assert!((right - 200.0).abs() < 1e-9);
    }
}
