//! Driver for the Maqueen Plus V3 robot coprocessor (I2C address 0x10).

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::{Error as I2cError, ErrorKind, I2c};

use crate::registers::*;
use crate::{Error, Result};

/// Driver for the robot coprocessor.
///
/// Every method performs one or two short I2C transactions. Reads follow the
/// MakeCode extension exactly: a one-byte write of the register index, a STOP,
/// then a read — never a repeated-start `write_read`.
pub struct MaqueenPlusV3<I2C, D> {
    i2c: I2C,
    delay: D,
    init_attempts: u32,
    pid_timeout_ms: u32,
}

impl<I2C: I2c, D: DelayNs> MaqueenPlusV3<I2C, D> {
    /// Create the driver. Call [`init`](Self::init) before anything else.
    pub fn new(i2c: I2C, delay: D) -> Self {
        Self {
            i2c,
            delay,
            init_attempts: 20,
            pid_timeout_ms: 30_000,
        }
    }

    /// How many times [`init`](Self::init) polls the version register (500 ms apart). Default 20.
    pub fn set_init_attempts(&mut self, attempts: u32) {
        self.init_attempts = attempts;
    }

    /// Give back the bus and delay handles.
    pub fn release(self) -> (I2C, D) {
        (self.i2c, self.delay)
    }

    async fn write(&mut self, bytes: &[u8]) -> Result<(), I2C::Error> {
        self.i2c.write(ADDRESS, bytes).await.map_err(Error::I2c)
    }

    async fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c.write(ADDRESS, &[reg]).await.map_err(Error::I2c)?;
        self.i2c.read(ADDRESS, buf).await.map_err(Error::I2c)
    }

    async fn read_u8(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut b = [0u8; 1];
        self.read_reg(reg, &mut b).await?;
        Ok(b[0])
    }

    /// Reset the coprocessor and wait until it reports a firmware version.
    ///
    /// Mirrors the "initialize via I2C until success" block: write reset, wait
    /// 100 ms, then poll the version length every 500 ms. Returns
    /// [`Error::NotReady`] after the configured number of attempts.
    ///
    /// The coprocessor does not ACK its address while it reboots after the reset
    /// (about 200 ms on firmware V1.1.0), so an address NACK during polling counts as
    /// "not ready yet" rather than a bus error — exactly as MakeCode's `i2cReadNumber`,
    /// which returns 0 on a NACK. Other I2C errors are still propagated.
    pub async fn init(&mut self) -> Result<(), I2C::Error> {
        self.write(&[RESET, 1]).await?;
        self.delay.delay_ms(100).await;
        for attempt in 0..self.init_attempts {
            match self.read_u8(VERSION_LEN).await {
                Ok(0) => {}
                Ok(_) => return Ok(()),
                Err(Error::I2c(e)) if matches!(e.kind(), ErrorKind::NoAcknowledge(_)) => {}
                Err(e) => return Err(e),
            }
            if attempt + 1 < self.init_attempts {
                self.delay.delay_ms(500).await;
            }
        }
        Err(Error::NotReady)
    }

    /// Read the firmware version string into `buf` and return it as `&str`.
    pub async fn version<'a>(&mut self, buf: &'a mut [u8]) -> Result<&'a str, I2C::Error> {
        let len = self.read_u8(VERSION_LEN).await? as usize;
        if len == 0 {
            return Err(Error::NotReady);
        }
        let len = len.min(buf.len());
        self.read_reg(VERSION_DATA, &mut buf[..len]).await?;
        core::str::from_utf8(&buf[..len]).map_err(|_| Error::InvalidData)
    }

    /// Run a motor at `speed` (0..255; below ~30 the wheel does not move).
    pub async fn set_motor(
        &mut self,
        motor: Motor,
        dir: Direction,
        speed: u8,
    ) -> Result<(), I2C::Error> {
        let d = dir.motor_byte();
        match motor {
            Motor::Left => self.write(&[LEFT_MOTOR, d, speed]).await,
            Motor::Right => self.write(&[RIGHT_MOTOR, d, speed]).await,
            Motor::Both => self.write(&[LEFT_MOTOR, d, speed, d, speed]).await,
        }
    }

    /// Stop a motor.
    pub async fn stop(&mut self, motor: Motor) -> Result<(), I2C::Error> {
        match motor {
            Motor::Left => self.write(&[LEFT_MOTOR, 0, 0]).await,
            Motor::Right => self.write(&[RIGHT_MOTOR, 0, 0]).await,
            Motor::Both => self.write(&[LEFT_MOTOR, 0, 0, 0, 0]).await,
        }
    }

    /// Set a headlight colour (`CarLightColor::Off` switches it off).
    pub async fn set_headlight(
        &mut self,
        side: Side,
        colour: CarLightColor,
    ) -> Result<(), I2C::Error> {
        let c = colour as u8;
        match side {
            Side::Left => self.write(&[LEFT_LED, c]).await,
            Side::Right => self.write(&[RIGHT_LED, c]).await,
            Side::Both => self.write(&[LEFT_LED, c, c]).await,
        }
    }

    /// Digital state of the five line sensors.
    pub async fn line_state(&mut self) -> Result<LineState, I2C::Error> {
        Ok(LineState::from_raw(self.read_u8(LINE_STATE).await?))
    }

    /// Raw 16-bit ADC reading of one line sensor (white ≈ 3800, black ≈ 2700 in practice).
    pub async fn line_adc(&mut self, sensor: LineSensor) -> Result<u16, I2C::Error> {
        let mut b = [0u8; 2];
        self.read_reg(sensor.adc_register(), &mut b).await?;
        Ok(u16::from_le_bytes(b))
    }

    /// ADC readings of all five line sensors (five separate reads, left to right).
    pub async fn line_adc_all(&mut self) -> Result<LineAdc, I2C::Error> {
        Ok(LineAdc {
            l2: self.line_adc(LineSensor::L2).await?,
            l1: self.line_adc(LineSensor::L1).await?,
            m: self.line_adc(LineSensor::M).await?,
            r1: self.line_adc(LineSensor::R1).await?,
            r2: self.line_adc(LineSensor::R2).await?,
        })
    }

    /// Speed level of the built-in line following.
    pub async fn set_patrol_speed(&mut self, speed: PatrolSpeed) -> Result<(), I2C::Error> {
        self.write(&[PATROL_SPEED, speed as u8]).await
    }

    /// Switch the built-in line following on or off. It runs inside the coprocessor.
    pub async fn set_patrolling(&mut self, on: bool) -> Result<(), I2C::Error> {
        let v = if on { CTRL_PATROL_ON } else { CTRL_PATROL_OFF };
        self.write(&[CONTROL, v]).await
    }

    /// Behaviour at a crossroad while line following.
    pub async fn set_crossroad_mode(&mut self, mode: CrossroadMode) -> Result<(), I2C::Error> {
        self.write(&[CROSSROAD_MODE, mode as u8]).await
    }

    /// Behaviour at a T-junction while line following.
    pub async fn set_t_junction_mode(&mut self, mode: TJunctionMode) -> Result<(), I2C::Error> {
        self.write(&[T_JUNCTION_MODE, mode as u8]).await
    }

    /// Behaviour at a left-or-straight junction while line following.
    pub async fn set_left_or_straight_mode(
        &mut self,
        mode: LeftOrStraightMode,
    ) -> Result<(), I2C::Error> {
        self.write(&[LEFT_OR_STRAIGHT_MODE, mode as u8]).await
    }

    /// Behaviour at a right-or-straight junction while line following.
    pub async fn set_right_or_straight_mode(
        &mut self,
        mode: RightOrStraightMode,
    ) -> Result<(), I2C::Error> {
        self.write(&[RIGHT_OR_STRAIGHT_MODE, mode as u8]).await
    }

    /// Intersection the line sensors currently see.
    ///
    /// Works with built-in line following on or off. The values 0..=4 were
    /// verified against the MakeCode block on hardware.
    pub async fn intersection(&mut self) -> Result<Intersection, I2C::Error> {
        Ok(Intersection::from_raw(self.read_u8(INTERSECTION).await?))
    }

    /// Upper bound for [`Wait::UntilDone`] polling in milliseconds. Default 30 000.
    pub fn set_pid_timeout_ms(&mut self, ms: u32) {
        self.pid_timeout_ms = ms;
    }

    async fn wait_pid_done(&mut self) -> Result<(), I2C::Error> {
        let polls = (self.pid_timeout_ms / 10).max(1);
        for _ in 0..polls {
            self.delay.delay_ms(10).await;
            if self.read_u8(PID_BUSY).await? != 1 {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    /// Drive a distance in cm under PID control (distance ±2 cm per DFRobot).
    pub async fn pid_distance(
        &mut self,
        dir: Direction,
        cm: u16,
        wait: Wait,
    ) -> Result<(), I2C::Error> {
        let [hi, lo] = cm.to_be_bytes();
        self.write(&[PID_DIST_DIR, dir.pid_byte()]).await?;
        self.write(&[PID_DIST_SPEED, PID_SPEED]).await?;
        self.write(&[PID_DIST_HI, hi]).await?;
        self.write(&[PID_DIST_LO, lo]).await?;
        self.write(&[CONTROL, CTRL_PID_START]).await?;
        match wait {
            Wait::UntilDone => self.wait_pid_done().await,
            Wait::Return => Ok(()),
        }
    }

    /// Turn by `degrees` (-180..=180) under PID control (angle ±5° per DFRobot).
    /// Positive values match the MakeCode block's positive direction.
    pub async fn pid_angle(&mut self, degrees: i16, wait: Wait) -> Result<(), I2C::Error> {
        if !(-180..=180).contains(&degrees) {
            return Err(Error::InvalidArgument);
        }
        let dir = if degrees >= 0 { 1 } else { 2 };
        self.write(&[PID_ANGLE_DIR, dir]).await?;
        self.write(&[PID_ANGLE_SPEED, PID_SPEED]).await?;
        self.write(&[PID_ANGLE, degrees.unsigned_abs() as u8])
            .await?;
        self.write(&[CONTROL, CTRL_PID_START]).await?;
        match wait {
            Wait::UntilDone => self.wait_pid_done().await,
            Wait::Return => Ok(()),
        }
    }

    /// Whether a PID move is still running.
    pub async fn pid_busy(&mut self) -> Result<bool, I2C::Error> {
        Ok(self.read_u8(PID_BUSY).await? == 1)
    }

    /// Abort the running PID move.
    pub async fn pid_stop(&mut self) -> Result<(), I2C::Error> {
        self.write(&[CONTROL, CTRL_PID_STOP]).await
    }

    /// Real-time wheel speeds from the encoders.
    pub async fn wheel_speed(&mut self) -> Result<WheelSpeed, I2C::Error> {
        let mut b = [0u8; 2];
        self.read_reg(WHEEL_SPEED, &mut b).await?;
        Ok(WheelSpeed {
            left: b[0],
            right: b[1],
        })
    }

    /// Ambient light on the left and right sensors.
    pub async fn light_sensors(&mut self) -> Result<LightLevels, I2C::Error> {
        let mut b = [0u8; 4];
        self.read_reg(LIGHT_SENSORS, &mut b).await?;
        Ok(LightLevels {
            left: u16::from_be_bytes([b[0], b[1]]),
            right: u16::from_be_bytes([b[2], b[3]]),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embassy_futures::block_on;
    use embedded_hal_async::i2c::{ErrorKind, NoAcknowledgeSource};
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};

    const NACK: ErrorKind = ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address);

    fn robot(expectations: &[T]) -> MaqueenPlusV3<Mock, NoopDelay> {
        MaqueenPlusV3::new(Mock::new(expectations), NoopDelay)
    }

    fn finish(robot: MaqueenPlusV3<Mock, NoopDelay>) {
        let (mut i2c, _) = robot.release();
        i2c.done();
    }

    #[test]
    fn init_resets_then_polls_version_until_nonzero() {
        let mut r = robot(&[
            T::write(0x10, vec![0x49, 1]),
            T::write(0x10, vec![0x32]),
            T::read(0x10, vec![0]),
            T::write(0x10, vec![0x32]),
            T::read(0x10, vec![5]),
        ]);
        block_on(r.init()).unwrap();
        finish(r);
    }

    #[test]
    fn init_keeps_polling_while_coprocessor_nacks_after_reset() {
        // The real coprocessor (firmware V1.1.0) does not ACK for ~200 ms after RESET,
        // so the first poll 100 ms later gets an address NACK. That is "not ready yet".
        let mut r = robot(&[
            T::write(0x10, vec![0x49, 1]),
            T::write(0x10, vec![0x32]).with_error(NACK),
            T::write(0x10, vec![0x32]),
            T::read(0x10, vec![6]),
        ]);
        block_on(r.init()).unwrap();
        finish(r);
    }

    #[test]
    fn init_reports_not_ready_when_coprocessor_never_acks() {
        let mut r = robot(&[
            T::write(0x10, vec![0x49, 1]),
            T::write(0x10, vec![0x32]).with_error(NACK),
            T::write(0x10, vec![0x32]).with_error(NACK),
        ]);
        r.set_init_attempts(2);
        assert_eq!(block_on(r.init()), Err(Error::NotReady));
        finish(r);
    }

    #[test]
    fn init_propagates_non_nack_bus_errors() {
        let mut r = robot(&[
            T::write(0x10, vec![0x49, 1]),
            T::write(0x10, vec![0x32]).with_error(ErrorKind::Bus),
        ]);
        assert_eq!(block_on(r.init()), Err(Error::I2c(ErrorKind::Bus)));
        finish(r);
    }

    #[test]
    fn init_gives_up_after_configured_attempts() {
        let mut r = robot(&[
            T::write(0x10, vec![0x49, 1]),
            T::write(0x10, vec![0x32]),
            T::read(0x10, vec![0]),
            T::write(0x10, vec![0x32]),
            T::read(0x10, vec![0]),
        ]);
        r.set_init_attempts(2);
        assert_eq!(block_on(r.init()), Err(Error::NotReady));
        finish(r);
    }

    #[test]
    fn version_reads_length_then_string() {
        let mut r = robot(&[
            T::write(0x10, vec![0x32]),
            T::read(0x10, vec![4]),
            T::write(0x10, vec![0x33]),
            T::read(0x10, b"V3.1".to_vec()),
        ]);
        let mut buf = [0u8; 32];
        assert_eq!(block_on(r.version(&mut buf)).unwrap(), "V3.1");
        finish(r);
    }

    #[test]
    fn set_motor_writes_direction_and_speed() {
        let mut r = robot(&[
            T::write(0x10, vec![0x00, 0, 120]),
            T::write(0x10, vec![0x02, 1, 80]),
            T::write(0x10, vec![0x00, 0, 200, 0, 200]),
        ]);
        block_on(r.set_motor(Motor::Left, Direction::Forward, 120)).unwrap();
        block_on(r.set_motor(Motor::Right, Direction::Backward, 80)).unwrap();
        block_on(r.set_motor(Motor::Both, Direction::Forward, 200)).unwrap();
        finish(r);
    }

    #[test]
    fn stop_writes_zeros() {
        let mut r = robot(&[
            T::write(0x10, vec![0x02, 0, 0]),
            T::write(0x10, vec![0x00, 0, 0, 0, 0]),
        ]);
        block_on(r.stop(Motor::Right)).unwrap();
        block_on(r.stop(Motor::Both)).unwrap();
        finish(r);
    }

    #[test]
    fn headlights_write_colour_codes() {
        let mut r = robot(&[
            T::write(0x10, vec![0x0B, 1]),
            T::write(0x10, vec![0x0C, 4]),
            T::write(0x10, vec![0x0B, 7, 7]),
        ]);
        block_on(r.set_headlight(Side::Left, CarLightColor::Red)).unwrap();
        block_on(r.set_headlight(Side::Right, CarLightColor::Blue)).unwrap();
        block_on(r.set_headlight(Side::Both, CarLightColor::White)).unwrap();
        finish(r);
    }

    #[test]
    fn intersection_decodes_register_61() {
        let mut r = robot(&[
            T::write(0x10, vec![61]),
            T::read(0x10, vec![0]),
            T::write(0x10, vec![61]),
            T::read(0x10, vec![2]),
            T::write(0x10, vec![61]),
            T::read(0x10, vec![9]),
        ]);
        assert_eq!(block_on(r.intersection()).unwrap(), Intersection::None);
        assert_eq!(block_on(r.intersection()).unwrap(), Intersection::TJunction);
        assert_eq!(
            block_on(r.intersection()).unwrap(),
            Intersection::Unknown(9)
        );
        finish(r);
    }

    #[test]
    fn line_state_reads_register_0x1d() {
        let mut r = robot(&[T::write(0x10, vec![0x1D]), T::read(0x10, vec![0x0C])]);
        let s = block_on(r.line_state()).unwrap();
        assert!(s.l1() && s.m());
        assert!(!s.l2() && !s.r1() && !s.r2());
        finish(r);
    }

    #[test]
    fn line_adc_reads_little_endian_u16() {
        let mut r = robot(&[T::write(0x10, vec![0x22]), T::read(0x10, vec![0x8C, 0x0E])]);
        assert_eq!(block_on(r.line_adc(LineSensor::M)).unwrap(), 0x0E8C); // 3724
        finish(r);
    }

    #[test]
    fn line_adc_all_reads_five_registers() {
        let mut r = robot(&[
            T::write(0x10, vec![0x26]),
            T::read(0x10, vec![1, 0]),
            T::write(0x10, vec![0x24]),
            T::read(0x10, vec![2, 0]),
            T::write(0x10, vec![0x22]),
            T::read(0x10, vec![3, 0]),
            T::write(0x10, vec![0x20]),
            T::read(0x10, vec![4, 0]),
            T::write(0x10, vec![0x1E]),
            T::read(0x10, vec![5, 0]),
        ]);
        let a = block_on(r.line_adc_all()).unwrap();
        assert_eq!(
            a,
            LineAdc {
                l2: 1,
                l1: 2,
                m: 3,
                r1: 4,
                r2: 5
            }
        );
        finish(r);
    }

    #[test]
    fn patrol_and_junction_modes_write_expected_bytes() {
        let mut r = robot(&[
            T::write(0x10, vec![63, 3]),
            T::write(0x10, vec![60, 0x05]),
            T::write(0x10, vec![60, 0x08]),
            T::write(0x10, vec![69, 3]),
            T::write(0x10, vec![70, 1]),
            T::write(0x10, vec![71, 4]),
            T::write(0x10, vec![72, 2]),
            T::write(0x10, vec![61]),
            T::read(0x10, vec![2]),
        ]);
        block_on(r.set_patrol_speed(PatrolSpeed::S3)).unwrap();
        block_on(r.set_patrolling(true)).unwrap();
        block_on(r.set_patrolling(false)).unwrap();
        block_on(r.set_crossroad_mode(CrossroadMode::Straight)).unwrap();
        block_on(r.set_t_junction_mode(TJunctionMode::Left)).unwrap();
        block_on(r.set_left_or_straight_mode(LeftOrStraightMode::Stop)).unwrap();
        block_on(r.set_right_or_straight_mode(RightOrStraightMode::Right)).unwrap();
        assert_eq!(block_on(r.intersection()).unwrap(), Intersection::TJunction);
        finish(r);
    }

    #[test]
    fn pid_distance_writes_sequence_and_waits_for_busy_flag() {
        let mut r = robot(&[
            T::write(0x10, vec![64, 1]),
            T::write(0x10, vec![85, 2]),
            T::write(0x10, vec![65, 0x01]),
            T::write(0x10, vec![66, 0x2C]),
            T::write(0x10, vec![60, 0x06]),
            T::write(0x10, vec![87]),
            T::read(0x10, vec![1]),
            T::write(0x10, vec![87]),
            T::read(0x10, vec![0]),
        ]);
        block_on(r.pid_distance(Direction::Forward, 300, Wait::UntilDone)).unwrap();
        finish(r);
    }

    #[test]
    fn pid_distance_return_does_not_poll() {
        let mut r = robot(&[
            T::write(0x10, vec![64, 2]),
            T::write(0x10, vec![85, 2]),
            T::write(0x10, vec![65, 0]),
            T::write(0x10, vec![66, 20]),
            T::write(0x10, vec![60, 0x06]),
        ]);
        block_on(r.pid_distance(Direction::Backward, 20, Wait::Return)).unwrap();
        finish(r);
    }

    #[test]
    fn pid_angle_negative_uses_direction_2_and_magnitude() {
        let mut r = robot(&[
            T::write(0x10, vec![67, 2]),
            T::write(0x10, vec![86, 2]),
            T::write(0x10, vec![68, 90]),
            T::write(0x10, vec![60, 0x06]),
        ]);
        block_on(r.pid_angle(-90, Wait::Return)).unwrap();
        finish(r);
    }

    #[test]
    fn pid_angle_rejects_out_of_range() {
        let mut r = robot(&[]);
        assert_eq!(
            block_on(r.pid_angle(181, Wait::Return)),
            Err(Error::InvalidArgument)
        );
        finish(r);
    }

    #[test]
    fn pid_wait_times_out() {
        let mut r = robot(&[
            T::write(0x10, vec![67, 1]),
            T::write(0x10, vec![86, 2]),
            T::write(0x10, vec![68, 45]),
            T::write(0x10, vec![60, 0x06]),
            T::write(0x10, vec![87]),
            T::read(0x10, vec![1]),
            T::write(0x10, vec![87]),
            T::read(0x10, vec![1]),
        ]);
        r.set_pid_timeout_ms(20); // two polls of 10 ms
        assert_eq!(
            block_on(r.pid_angle(45, Wait::UntilDone)),
            Err(Error::Timeout)
        );
        finish(r);
    }

    #[test]
    fn pid_wait_polls_at_least_once_even_with_short_timeout() {
        let mut r = robot(&[
            T::write(0x10, vec![67, 1]),
            T::write(0x10, vec![86, 2]),
            T::write(0x10, vec![68, 45]),
            T::write(0x10, vec![60, 0x06]),
            T::write(0x10, vec![87]),
            T::read(0x10, vec![1]),
        ]);
        r.set_pid_timeout_ms(5); // less than one 10 ms poll interval: still polls once
        assert_eq!(
            block_on(r.pid_angle(45, Wait::UntilDone)),
            Err(Error::Timeout)
        );
        finish(r);
    }

    #[test]
    fn pid_busy_and_stop() {
        let mut r = robot(&[
            T::write(0x10, vec![87]),
            T::read(0x10, vec![1]),
            T::write(0x10, vec![60, 0x10]),
        ]);
        assert!(block_on(r.pid_busy()).unwrap());
        block_on(r.pid_stop()).unwrap();
        finish(r);
    }

    #[test]
    fn wheel_speed_and_light_sensors() {
        let mut r = robot(&[
            T::write(0x10, vec![76]),
            T::read(0x10, vec![50, 25]),
            T::write(0x10, vec![78]),
            T::read(0x10, vec![0x01, 0x02, 0x03, 0x04]),
        ]);
        assert_eq!(
            block_on(r.wheel_speed()).unwrap(),
            WheelSpeed {
                left: 50,
                right: 25
            }
        );
        assert_eq!(
            block_on(r.light_sensors()).unwrap(),
            LightLevels {
                left: 0x0102,
                right: 0x0304
            }
        );
        finish(r);
    }
}
