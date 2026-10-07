//! Register addresses and public types of the Maqueen Plus V3 coprocessor (I2C 0x10).

/// I2C address of the robot coprocessor.
pub const ADDRESS: u8 = 0x10;

/// Register: left motor `[direction, speed]`.
pub const LEFT_MOTOR: u8 = 0x00;
/// Register: right motor `[direction, speed]`.
pub const RIGHT_MOTOR: u8 = 0x02;
/// Register: left headlight colour (0..7).
pub const LEFT_LED: u8 = 0x0B;
/// Register: right headlight colour (0..7).
pub const RIGHT_LED: u8 = 0x0C;
/// Register: digital line sensor state, one bit per sensor.
pub const LINE_STATE: u8 = 0x1D;
/// Register: version string length (0 while the coprocessor boots).
pub const VERSION_LEN: u8 = 0x32;
/// Register: version string bytes.
pub const VERSION_DATA: u8 = 0x33;
/// Register: system reset (write 1).
pub const RESET: u8 = 0x49;
/// Register: line-patrol / PID control byte.
pub const CONTROL: u8 = 60;
/// Register: intersection currently seen, see [`Intersection`].
pub const INTERSECTION: u8 = 61;
/// Register: line-following speed level 1..5.
pub const PATROL_SPEED: u8 = 63;
/// Register: PID distance direction (1 forward, 2 backward).
pub const PID_DIST_DIR: u8 = 64;
/// Register: PID distance high byte (cm).
pub const PID_DIST_HI: u8 = 65;
/// Register: PID distance low byte (cm).
pub const PID_DIST_LO: u8 = 66;
/// Register: PID angle direction (1 positive, 2 negative).
pub const PID_ANGLE_DIR: u8 = 67;
/// Register: PID angle magnitude 0..180.
pub const PID_ANGLE: u8 = 68;
/// Register: behaviour at a crossroad.
pub const CROSSROAD_MODE: u8 = 69;
/// Register: behaviour at a T-junction.
pub const T_JUNCTION_MODE: u8 = 70;
/// Register: behaviour at a left-or-straight junction.
pub const LEFT_OR_STRAIGHT_MODE: u8 = 71;
/// Register: behaviour at a right-or-straight junction.
pub const RIGHT_OR_STRAIGHT_MODE: u8 = 72;
/// Register: real-time wheel speed `[left, right]`, cm/s × 5.
pub const WHEEL_SPEED: u8 = 76;
/// Register: light sensors, two big-endian u16 (left, right).
pub const LIGHT_SENSORS: u8 = 78;
/// Register: PID distance speed (the extension always writes 2).
pub const PID_DIST_SPEED: u8 = 85;
/// Register: PID angle speed (the extension always writes 2).
pub const PID_ANGLE_SPEED: u8 = 86;
/// Register: PID busy flag (1 while a PID move runs).
pub const PID_BUSY: u8 = 87;

/// `CONTROL` value: enable built-in line following.
pub const CTRL_PATROL_ON: u8 = 0x05;
/// `CONTROL` value: disable built-in line following.
pub const CTRL_PATROL_OFF: u8 = 0x08;
/// `CONTROL` value: start the configured PID move.
pub const CTRL_PID_START: u8 = 0x06;
/// `CONTROL` value: abort a PID move.
pub const CTRL_PID_STOP: u8 = 0x10;
/// Speed byte the extension hard-codes for PID moves.
pub const PID_SPEED: u8 = 2;

/// Which motor a command addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Motor {
    /// Left wheel.
    Left,
    /// Right wheel.
    Right,
    /// Both wheels in one I2C write.
    Both,
}

/// Direction of travel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    /// Forward.
    Forward,
    /// Backward.
    Backward,
}

impl Direction {
    /// Byte used by the motor registers (0 forward, 1 backward).
    pub const fn motor_byte(self) -> u8 {
        match self {
            Direction::Forward => 0,
            Direction::Backward => 1,
        }
    }

    /// Byte used by the PID distance register (1 forward/CW, 2 backward/CCW).
    pub const fn pid_byte(self) -> u8 {
        match self {
            Direction::Forward => 1,
            Direction::Backward => 2,
        }
    }
}

/// Which headlight a command addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Side {
    /// Left headlight.
    Left,
    /// Right headlight.
    Right,
    /// Both headlights.
    Both,
}

/// Colours the RGB headlights can show.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum CarLightColor {
    /// Off.
    Off = 0,
    /// Red.
    Red = 1,
    /// Green.
    Green = 2,
    /// Yellow.
    Yellow = 3,
    /// Blue.
    Blue = 4,
    /// Purple.
    Purple = 5,
    /// Cyan.
    Cyan = 6,
    /// White.
    White = 7,
}

/// The five line-tracking sensors, left to right.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LineSensor {
    /// Outer left.
    L2,
    /// Inner left.
    L1,
    /// Middle.
    M,
    /// Inner right.
    R1,
    /// Outer right.
    R2,
}

impl LineSensor {
    /// Bit of this sensor in the `LINE_STATE` register.
    pub const fn mask(self) -> u8 {
        match self {
            LineSensor::L2 => 0x10,
            LineSensor::L1 => 0x08,
            LineSensor::M => 0x04,
            LineSensor::R1 => 0x02,
            LineSensor::R2 => 0x01,
        }
    }

    /// Register holding this sensor's 16-bit ADC value (little-endian).
    pub const fn adc_register(self) -> u8 {
        match self {
            LineSensor::R2 => 0x1E,
            LineSensor::R1 => 0x20,
            LineSensor::M => 0x22,
            LineSensor::L1 => 0x24,
            LineSensor::L2 => 0x26,
        }
    }
}

/// Digital state of all five line sensors. A set bit means "line detected".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LineState(u8);

impl LineState {
    /// Wrap the raw `LINE_STATE` byte.
    pub const fn from_raw(raw: u8) -> Self {
        LineState(raw)
    }
    /// The raw register byte.
    pub const fn raw(self) -> u8 {
        self.0
    }
    /// Whether the given sensor sees the line.
    pub const fn is_on(self, sensor: LineSensor) -> bool {
        self.0 & sensor.mask() != 0
    }
    /// Outer left sensor.
    pub const fn l2(self) -> bool {
        self.is_on(LineSensor::L2)
    }
    /// Inner left sensor.
    pub const fn l1(self) -> bool {
        self.is_on(LineSensor::L1)
    }
    /// Middle sensor.
    pub const fn m(self) -> bool {
        self.is_on(LineSensor::M)
    }
    /// Inner right sensor.
    pub const fn r1(self) -> bool {
        self.is_on(LineSensor::R1)
    }
    /// Outer right sensor.
    pub const fn r2(self) -> bool {
        self.is_on(LineSensor::R2)
    }
}

/// Analog (ADC) readings of all five line sensors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LineAdc {
    /// Outer left.
    pub l2: u16,
    /// Inner left.
    pub l1: u16,
    /// Middle.
    pub m: u16,
    /// Inner right.
    pub r1: u16,
    /// Outer right.
    pub r2: u16,
}

/// Speed level of the built-in line following.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PatrolSpeed {
    /// Slowest.
    S1 = 1,
    /// Level 2.
    S2 = 2,
    /// Level 3.
    S3 = 3,
    /// Level 4.
    S4 = 4,
    /// Fastest.
    S5 = 5,
}

/// What the line follower does at a crossroad.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum CrossroadMode {
    /// Turn left.
    Left = 1,
    /// Turn right.
    Right = 2,
    /// Go straight.
    Straight = 3,
    /// Stop.
    Stop = 4,
}

/// What the line follower does at a T-junction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TJunctionMode {
    /// Turn left.
    Left = 1,
    /// Turn right.
    Right = 2,
    /// Stop.
    Stop = 4,
}

/// What the line follower does where it can turn left or go straight.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum LeftOrStraightMode {
    /// Turn left.
    Left = 1,
    /// Go straight.
    Straight = 3,
    /// Stop.
    Stop = 4,
}

/// What the line follower does where it can turn right or go straight.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum RightOrStraightMode {
    /// Turn right.
    Right = 2,
    /// Go straight.
    Straight = 3,
    /// Stop.
    Stop = 4,
}

/// Intersection the line sensors currently see, as classified by the coprocessor.
///
/// Reported whether or not the built-in line following is switched on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Intersection {
    /// No intersection.
    None,
    /// Crossroad: left, right and straight.
    Crossroad,
    /// T-junction: left and right.
    TJunction,
    /// Left or straight.
    LeftOrStraight,
    /// Right or straight.
    RightOrStraight,
    /// A value the firmware is not known to report.
    Unknown(u8),
}

impl Intersection {
    /// Decode the raw `INTERSECTION` byte.
    pub const fn from_raw(raw: u8) -> Self {
        match raw {
            0 => Intersection::None,
            1 => Intersection::Crossroad,
            2 => Intersection::TJunction,
            3 => Intersection::LeftOrStraight,
            4 => Intersection::RightOrStraight,
            other => Intersection::Unknown(other),
        }
    }
}

/// Whether a PID move should block until the coprocessor reports completion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Wait {
    /// Poll the busy flag every 10 ms and return when the move is done ("no interruption" block).
    UntilDone,
    /// Return immediately after starting the move ("allow interruption" block).
    Return,
}

/// Real-time wheel speeds as reported by the encoders.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WheelSpeed {
    /// Raw left value; cm/s = raw / 5.
    pub left: u8,
    /// Raw right value; cm/s = raw / 5.
    pub right: u8,
}

impl WheelSpeed {
    /// Left wheel speed in cm/s.
    pub fn left_cm_per_s(&self) -> f32 {
        self.left as f32 / 5.0
    }
    /// Right wheel speed in cm/s.
    pub fn right_cm_per_s(&self) -> f32 {
        self.right as f32 / 5.0
    }
}

/// Ambient light readings of the two light sensors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LightLevels {
    /// Left sensor.
    pub left: u16,
    /// Right sensor.
    pub right: u16,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_state_bits_match_makecode_masks() {
        // MakeCode: L2=0x10, L1=0x08, M=0x04, R1=0x02, R2=0x01
        let s = LineState::from_raw(0x10 | 0x01);
        assert!(s.l2() && s.r2());
        assert!(!s.l1() && !s.m() && !s.r1());
        assert!(s.is_on(LineSensor::L2));
        assert!(!s.is_on(LineSensor::M));
        assert_eq!(LineState::from_raw(0x04).raw(), 0x04);
    }

    #[test]
    fn line_sensor_adc_registers_match_makecode() {
        assert_eq!(LineSensor::R2.adc_register(), 0x1E);
        assert_eq!(LineSensor::R1.adc_register(), 0x20);
        assert_eq!(LineSensor::M.adc_register(), 0x22);
        assert_eq!(LineSensor::L1.adc_register(), 0x24);
        assert_eq!(LineSensor::L2.adc_register(), 0x26);
    }

    #[test]
    fn direction_bytes_differ_between_motor_and_pid_registers() {
        assert_eq!(Direction::Forward.motor_byte(), 0);
        assert_eq!(Direction::Backward.motor_byte(), 1);
        assert_eq!(Direction::Forward.pid_byte(), 1);
        assert_eq!(Direction::Backward.pid_byte(), 2);
    }

    #[test]
    fn enum_discriminants_match_makecode() {
        assert_eq!(CarLightColor::White as u8, 7);
        assert_eq!(CarLightColor::Off as u8, 0);
        assert_eq!(PatrolSpeed::S5 as u8, 5);
        assert_eq!(CrossroadMode::Stop as u8, 4);
        assert_eq!(TJunctionMode::Right as u8, 2);
        assert_eq!(LeftOrStraightMode::Straight as u8, 3);
        assert_eq!(RightOrStraightMode::Right as u8, 2);
    }

    #[test]
    fn wheel_speed_converts_raw_to_cm_per_s() {
        let w = WheelSpeed {
            left: 50,
            right: 25,
        };
        assert_eq!(w.left_cm_per_s(), 10.0);
        assert_eq!(w.right_cm_per_s(), 5.0);
    }
}
