//! CAN wire format shared with `sim/robot_can_sim.py`. Keep both sides in sync:
//!
//!   0x100 cmd frame (host -> robot), 8 bytes:
//!     i16 left_mm_s, i16 right_mm_s (little-endian), 4 reserved bytes
//!
//!   0x200 encoder frame (robot -> host), 8 bytes:
//!     i32 left_ticks, i32 right_ticks (little-endian, cumulative)

pub const CMD_VEL_CAN_ID: u16 = 0x100;
pub const ENCODER_CAN_ID: u16 = 0x200;

pub fn encode_cmd_frame(left_mm_s: i16, right_mm_s: i16) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&left_mm_s.to_le_bytes());
    data[2..4].copy_from_slice(&right_mm_s.to_le_bytes());
    data
}

pub fn decode_encoder_frame(data: &[u8]) -> Option<(i32, i32)> {
    let left = i32::from_le_bytes(data.get(0..4)?.try_into().ok()?);
    let right = i32::from_le_bytes(data.get(4..8)?.try_into().ok()?);
    Some((left, right))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_encoder_frame() {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&174i32.to_le_bytes());
        data[4..8].copy_from_slice(&(-42i32).to_le_bytes());
        assert_eq!(decode_encoder_frame(&data), Some((174, -42)));
    }

    #[test]
    fn encodes_cmd_frame_little_endian() {
        assert_eq!(encode_cmd_frame(200, -300), [200, 0, 212, 254, 0, 0, 0, 0]);
    }
}
