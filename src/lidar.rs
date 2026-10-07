//! Driver for the DFRobot 8×8 matrix laser ranging sensor shipped with the Maqueen Plus V3.
//!
//! Framed request/response protocol over I2C (default address 0x30). A request is
//! `[0x55, len_hi, len_lo, cmd, payload…]` with `len = payload.len() + 1`. The
//! response is read byte-wise in separate I2C reads: status (0x53 ok, 0x63 failed,
//! 0xFF not ready yet), command echo, little-endian length, payload.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

use crate::{Error, Result};

const HEADER: u8 = 0x55;
const CMD_SET_MODE: u8 = 1;
const CMD_FIXED_POINT: u8 = 3;
const CMD_AVOID_OBSTACLE: u8 = 6;
const CMD_CONFIG_AVOID: u8 = 7;
const STATUS_SUCCESS: u8 = 0x53;
const STATUS_FAILED: u8 = 0x63;
const STATUS_NOT_READY: u8 = 0xFF;
/// Largest response payload we accept (obstacle data is 8 bytes).
const MAX_PAYLOAD: usize = 32;

/// Selectable I2C address of the sensor (DIP switches).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum LidarAddress {
    /// 0x30 (factory default).
    Addr30 = 0x30,
    /// 0x31.
    Addr31 = 0x31,
    /// 0x32.
    Addr32 = 0x32,
    /// 0x33.
    Addr33 = 0x33,
}

/// Operating mode chosen at [`MatrixLidar::init`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LidarMode {
    /// Full 8×8 matrix; use [`MatrixLidar::point`].
    Matrix8x8,
    /// 4×4 matrix with on-sensor obstacle analysis; use [`MatrixLidar::obstacle_data`].
    ObstacleAvoidance4x4,
}

/// Direction the sensor suggests to pass an obstacle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Suggestion {
    /// Pass on the left.
    Left,
    /// Pass on the right.
    Right,
    /// Path ahead is free.
    Front,
    /// Value not documented by DFRobot.
    Unknown(u8),
}

impl Suggestion {
    fn from_raw(v: u8) -> Self {
        match v {
            1 => Suggestion::Left,
            2 => Suggestion::Right,
            3 => Suggestion::Front,
            other => Suggestion::Unknown(other),
        }
    }
}

/// Result of one obstacle-avoidance computation on the sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ObstacleData {
    /// Suggested direction.
    pub suggestion: Suggestion,
    /// Obstacle closer than 100 mm.
    pub emergency: bool,
    /// Nearest distance on the left, mm.
    pub left_mm: u16,
    /// Nearest distance ahead, mm.
    pub front_mm: u16,
    /// Nearest distance on the right, mm.
    pub right_mm: u16,
}

/// Driver for the matrix lidar.
pub struct MatrixLidar<I2C, D> {
    i2c: I2C,
    delay: D,
    addr: u8,
    timeout_ms: u32,
}

impl<I2C: I2c, D: DelayNs> MatrixLidar<I2C, D> {
    /// Create the driver. The sensor needs about 3 s after power-on before it answers.
    pub fn new(i2c: I2C, delay: D, addr: LidarAddress) -> Self {
        Self {
            i2c,
            delay,
            addr: addr as u8,
            timeout_ms: 8000,
        }
    }

    /// How long to wait for a response before returning [`Error::Timeout`]. Default 8000 ms (as in MakeCode).
    pub fn set_timeout_ms(&mut self, ms: u32) {
        self.timeout_ms = ms;
    }

    /// Give back the bus and delay handles.
    pub fn release(self) -> (I2C, D) {
        (self.i2c, self.delay)
    }

    async fn send(&mut self, cmd: u8, payload: &[u8]) -> Result<(), I2C::Error> {
        let mut frame = [0u8; 4 + 8];
        let len = payload.len() as u16 + 1;
        frame[0] = HEADER;
        frame[1] = (len >> 8) as u8;
        frame[2] = len as u8;
        frame[3] = cmd;
        frame[4..4 + payload.len()].copy_from_slice(payload);
        self.i2c
            .write(self.addr, &frame[..4 + payload.len()])
            .await
            .map_err(Error::I2c)?;
        self.delay.delay_ms(10).await;
        Ok(())
    }

    async fn read_byte(&mut self) -> Result<u8, I2C::Error> {
        let mut b = [0u8; 1];
        self.i2c.read(self.addr, &mut b).await.map_err(Error::I2c)?;
        Ok(b[0])
    }

    /// Poll for a response. `expected_cmd == None` accepts any command echo (set-mode
    /// answers with the config-avoid command id). Returns the payload length.
    async fn recv(
        &mut self,
        expected_cmd: Option<u8>,
        out: &mut [u8; MAX_PAYLOAD],
    ) -> Result<usize, I2C::Error> {
        for _ in 0..self.timeout_ms {
            let status = self.read_byte().await?;
            match status {
                STATUS_NOT_READY => {
                    self.delay.delay_ms(1).await;
                    continue;
                }
                STATUS_FAILED => return Err(Error::LidarFailed(STATUS_FAILED)),
                STATUS_SUCCESS => {
                    let cmd = self.read_byte().await?;
                    if let Some(want) = expected_cmd
                        && cmd != want
                    {
                        return Err(Error::LidarFailed(cmd));
                    }
                    let mut len = [0u8; 2];
                    self.i2c
                        .read(self.addr, &mut len)
                        .await
                        .map_err(Error::I2c)?;
                    let len = u16::from_le_bytes(len) as usize;
                    if len == 0 {
                        return Ok(0);
                    }
                    if len > MAX_PAYLOAD {
                        return Err(Error::InvalidData);
                    }
                    self.i2c
                        .read(self.addr, &mut out[..len])
                        .await
                        .map_err(Error::I2c)?;
                    return Ok(len);
                }
                _ => {
                    // Unknown status byte: keep polling, like the MakeCode extension.
                    self.delay.delay_ms(1).await;
                }
            }
        }
        Err(Error::Timeout)
    }

    /// Select the operating mode. Must be called once before any measurement.
    pub async fn init(&mut self, mode: LidarMode) -> Result<(), I2C::Error> {
        let (obstacle, matrix) = match mode {
            LidarMode::Matrix8x8 => (0, 8),
            LidarMode::ObstacleAvoidance4x4 => (1, 4),
        };
        self.send(CMD_SET_MODE, &[obstacle, 0, 0, matrix]).await?;
        let mut out = [0u8; MAX_PAYLOAD];
        self.recv(None, &mut out).await?;
        Ok(())
    }

    /// Distance in mm measured by the beam at column `x`, row `y` (0..=7).
    pub async fn point(&mut self, x: u8, y: u8) -> Result<u16, I2C::Error> {
        if x > 7 || y > 7 {
            return Err(Error::InvalidArgument);
        }
        self.send(CMD_FIXED_POINT, &[x, y]).await?;
        let mut out = [0u8; MAX_PAYLOAD];
        let len = self.recv(Some(CMD_FIXED_POINT), &mut out).await?;
        if len < 2 {
            return Err(Error::InvalidData);
        }
        Ok(u16::from_le_bytes([out[0], out[1]]))
    }

    /// Distance (100..=500 mm) at which the obstacle-avoidance mode starts suggesting a direction.
    /// DFRobot advises at most 200 mm on the Maqueen, otherwise the beams see the floor.
    pub async fn set_obstacle_distance(&mut self, mm: u16) -> Result<(), I2C::Error> {
        if !(100..=500).contains(&mm) {
            return Err(Error::InvalidArgument);
        }
        let [hi, lo] = mm.to_be_bytes();
        self.send(CMD_CONFIG_AVOID, &[hi, lo]).await?;
        let mut out = [0u8; MAX_PAYLOAD];
        self.recv(Some(CMD_CONFIG_AVOID), &mut out).await?;
        Ok(())
    }

    /// Fetch one obstacle-avoidance computation. Requires [`LidarMode::ObstacleAvoidance4x4`].
    pub async fn obstacle_data(&mut self) -> Result<ObstacleData, I2C::Error> {
        self.send(CMD_AVOID_OBSTACLE, &[]).await?;
        let mut out = [0u8; MAX_PAYLOAD];
        let len = self.recv(Some(CMD_AVOID_OBSTACLE), &mut out).await?;
        if len < 8 {
            return Err(Error::InvalidData);
        }
        Ok(ObstacleData {
            suggestion: Suggestion::from_raw(out[0]),
            emergency: out[1] != 0,
            left_mm: u16::from_le_bytes([out[2], out[3]]),
            front_mm: u16::from_le_bytes([out[4], out[5]]),
            right_mm: u16::from_le_bytes([out[6], out[7]]),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embassy_futures::block_on;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};

    fn lidar(expectations: &[T]) -> MatrixLidar<Mock, NoopDelay> {
        MatrixLidar::new(Mock::new(expectations), NoopDelay, LidarAddress::Addr30)
    }

    fn finish(l: MatrixLidar<Mock, NoopDelay>) {
        let (mut i2c, _) = l.release();
        i2c.done();
    }

    #[test]
    fn init_sends_set_mode_and_accepts_any_command_echo() {
        let mut l = lidar(&[
            T::write(0x30, vec![0x55, 0, 5, 1, 1, 0, 0, 4]),
            T::read(0x30, vec![0x53]),
            T::read(0x30, vec![7]),
            T::read(0x30, vec![0, 0]),
        ]);
        block_on(l.init(LidarMode::ObstacleAvoidance4x4)).unwrap();
        finish(l);
    }

    #[test]
    fn init_matrix_mode_payload() {
        let mut l = lidar(&[
            T::write(0x30, vec![0x55, 0, 5, 1, 0, 0, 0, 8]),
            T::read(0x30, vec![0x53]),
            T::read(0x30, vec![1]),
            T::read(0x30, vec![0, 0]),
        ]);
        block_on(l.init(LidarMode::Matrix8x8)).unwrap();
        finish(l);
    }

    #[test]
    fn point_polls_until_ready_and_returns_mm() {
        let mut l = lidar(&[
            T::write(0x30, vec![0x55, 0, 3, 3, 3, 4]),
            T::read(0x30, vec![0xFF]),
            T::read(0x30, vec![0xFF]),
            T::read(0x30, vec![0x53]),
            T::read(0x30, vec![3]),
            T::read(0x30, vec![2, 0]),
            T::read(0x30, vec![0x2C, 0x01]),
        ]);
        assert_eq!(block_on(l.point(3, 4)).unwrap(), 300);
        finish(l);
    }

    #[test]
    fn point_rejects_index_out_of_range() {
        let mut l = lidar(&[]);
        assert_eq!(block_on(l.point(8, 0)), Err(Error::InvalidArgument));
        finish(l);
    }

    #[test]
    fn failed_status_is_reported() {
        let mut l = lidar(&[
            T::write(0x30, vec![0x55, 0, 3, 3, 0, 0]),
            T::read(0x30, vec![0x63]),
        ]);
        assert_eq!(block_on(l.point(0, 0)), Err(Error::LidarFailed(0x63)));
        finish(l);
    }

    #[test]
    fn wrong_command_echo_is_reported() {
        let mut l = lidar(&[
            T::write(0x30, vec![0x55, 0, 3, 3, 0, 0]),
            T::read(0x30, vec![0x53]),
            T::read(0x30, vec![6]),
        ]);
        assert_eq!(block_on(l.point(0, 0)), Err(Error::LidarFailed(6)));
        finish(l);
    }

    #[test]
    fn not_ready_forever_times_out() {
        let mut l = lidar(&[
            T::write(0x30, vec![0x55, 0, 3, 3, 0, 0]),
            T::read(0x30, vec![0xFF]),
            T::read(0x30, vec![0xFF]),
            T::read(0x30, vec![0xFF]),
        ]);
        l.set_timeout_ms(3); // one poll per millisecond
        assert_eq!(block_on(l.point(0, 0)), Err(Error::Timeout));
        finish(l);
    }

    #[test]
    fn set_obstacle_distance_sends_big_endian_mm() {
        let mut l = lidar(&[
            T::write(0x30, vec![0x55, 0, 3, 7, 0x00, 0xC8]),
            T::read(0x30, vec![0x53]),
            T::read(0x30, vec![7]),
            T::read(0x30, vec![0, 0]),
        ]);
        block_on(l.set_obstacle_distance(200)).unwrap();
        assert_eq!(
            block_on(l.set_obstacle_distance(50)),
            Err(Error::InvalidArgument)
        );
        finish(l);
    }

    #[test]
    fn obstacle_data_parses_eight_bytes() {
        let mut l = lidar(&[
            T::write(0x30, vec![0x55, 0, 1, 6]),
            T::read(0x30, vec![0x53]),
            T::read(0x30, vec![6]),
            T::read(0x30, vec![8, 0]),
            T::read(0x30, vec![2, 1, 0x10, 0x00, 0x20, 0x00, 0x30, 0x00]),
        ]);
        let d = block_on(l.obstacle_data()).unwrap();
        assert_eq!(
            d,
            ObstacleData {
                suggestion: Suggestion::Right,
                emergency: true,
                left_mm: 16,
                front_mm: 32,
                right_mm: 48
            }
        );
        finish(l);
    }
}
