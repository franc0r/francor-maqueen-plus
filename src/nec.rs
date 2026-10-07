//! Pure NEC infrared decoder. Feed it edge timings; it returns decoded frames.

/// Logic level of the IR receiver output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PinLevel {
    /// Line low (carrier present; NEC "mark").
    Low,
    /// Line high (idle; NEC "space").
    High,
}

/// One decoded NEC frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NecFrame {
    /// 16-bit address (low byte first as transmitted). Standard remotes send `addr | !addr << 8`.
    pub address: u16,
    /// Command byte. This is what the MakeCode "read IR key value" block returns.
    pub command: u8,
    /// `true` for a repeat frame (key held down); `address`/`command` repeat the last frame.
    pub repeat: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    LeadSpace,
    RepeatMark,
    BitMark,
    BitSpace,
    FinalMark,
}

/// NEC decoder state machine. Call [`feed`](Self::feed) once per edge.
#[derive(Debug)]
pub struct Decoder {
    state: State,
    bits: u32,
    count: u8,
    last: Option<NecFrame>,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

const fn within(value: u32, lo: u32, hi: u32) -> bool {
    value >= lo && value <= hi
}

impl Decoder {
    /// New decoder in the idle state.
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            bits: 0,
            count: 0,
            last: None,
        }
    }

    /// Drop any partial frame (call after a long silence or a timeout).
    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.bits = 0;
        self.count = 0;
    }

    /// Report that the line was at `level` for `duration_us` microseconds and has just changed.
    /// Returns a frame when one completes.
    pub fn feed(&mut self, level: PinLevel, duration_us: u32) -> Option<NecFrame> {
        use PinLevel::*;
        let mark_ok = within(duration_us, 300, 900);
        self.state = match (self.state, level) {
            (State::Idle, Low) if within(duration_us, 7000, 11000) => State::LeadSpace,
            (State::LeadSpace, High) if within(duration_us, 3500, 5500) => {
                self.bits = 0;
                self.count = 0;
                State::BitMark
            }
            (State::LeadSpace, High) if within(duration_us, 1800, 2800) => State::RepeatMark,
            (State::RepeatMark, Low) if mark_ok => {
                self.state = State::Idle;
                return self.last.map(|f| NecFrame { repeat: true, ..f });
            }
            (State::BitMark, Low) if mark_ok => State::BitSpace,
            (State::BitSpace, High)
                if within(duration_us, 300, 900) || within(duration_us, 1200, 2200) =>
            {
                if duration_us >= 1200 {
                    self.bits |= 1 << self.count;
                }
                self.count += 1;
                if self.count == 32 {
                    State::FinalMark
                } else {
                    State::BitMark
                }
            }
            (State::FinalMark, Low) if mark_ok => {
                self.state = State::Idle;
                let [addr_lo, addr_hi, cmd, cmd_inv] = self.bits.to_le_bytes();
                if cmd ^ cmd_inv != 0xFF {
                    return None;
                }
                let frame = NecFrame {
                    address: u16::from_le_bytes([addr_lo, addr_hi]),
                    command: cmd,
                    repeat: false,
                };
                self.last = Some(frame);
                return Some(frame);
            }
            _ => State::Idle,
        };
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Feed a full NEC frame for `bits` (LSB first) and return what the decoder emits.
    fn send_frame(dec: &mut Decoder, bits: u32) -> Option<NecFrame> {
        assert_eq!(dec.feed(PinLevel::Low, 9000), None);
        assert_eq!(dec.feed(PinLevel::High, 4500), None);
        let mut out = None;
        for i in 0..32 {
            assert_eq!(dec.feed(PinLevel::Low, 562), None);
            let space = if bits >> i & 1 == 1 { 1687 } else { 562 };
            assert_eq!(dec.feed(PinLevel::High, space), None);
        }
        // final mark
        if let Some(f) = dec.feed(PinLevel::Low, 562) {
            out = Some(f);
        }
        out
    }

    fn frame_bits(addr_lo: u8, addr_hi: u8, cmd: u8, cmd_inv: u8) -> u32 {
        addr_lo as u32 | (addr_hi as u32) << 8 | (cmd as u32) << 16 | (cmd_inv as u32) << 24
    }

    #[test]
    fn decodes_standard_frame() {
        let mut dec = Decoder::new();
        let f = send_frame(&mut dec, frame_bits(0x00, 0xFF, 0x45, !0x45)).unwrap();
        assert_eq!(
            f,
            NecFrame {
                address: 0xFF00,
                command: 0x45,
                repeat: false
            }
        );
    }

    #[test]
    fn rejects_frame_with_bad_command_inverse() {
        let mut dec = Decoder::new();
        assert_eq!(
            send_frame(&mut dec, frame_bits(0x00, 0xFF, 0x45, 0x45)),
            None
        );
    }

    #[test]
    fn repeat_frame_repeats_last_command() {
        let mut dec = Decoder::new();
        send_frame(&mut dec, frame_bits(0x00, 0xFF, 0x16, !0x16)).unwrap();
        assert_eq!(dec.feed(PinLevel::Low, 9000), None);
        assert_eq!(dec.feed(PinLevel::High, 2250), None);
        let f = dec.feed(PinLevel::Low, 562).unwrap();
        assert_eq!(
            f,
            NecFrame {
                address: 0xFF00,
                command: 0x16,
                repeat: true
            }
        );
    }

    #[test]
    fn repeat_without_previous_frame_is_ignored() {
        let mut dec = Decoder::new();
        dec.feed(PinLevel::Low, 9000);
        dec.feed(PinLevel::High, 2250);
        assert_eq!(dec.feed(PinLevel::Low, 562), None);
    }

    #[test]
    fn noise_resets_to_idle_and_next_frame_still_decodes() {
        let mut dec = Decoder::new();
        dec.feed(PinLevel::Low, 9000);
        dec.feed(PinLevel::High, 4500);
        dec.feed(PinLevel::Low, 562);
        assert_eq!(dec.feed(PinLevel::High, 3000), None); // invalid bit space
        let f = send_frame(&mut dec, frame_bits(0x00, 0xFF, 0x0C, !0x0C)).unwrap();
        assert_eq!(f.command, 0x0C);
    }

    #[test]
    fn tolerates_30us_timer_granularity() {
        // embassy-time on the nRF52 ticks at 32768 Hz (~30.5 µs); make sure ±60 µs still decodes.
        let mut dec = Decoder::new();
        assert_eq!(dec.feed(PinLevel::Low, 8940), None);
        assert_eq!(dec.feed(PinLevel::High, 4560), None);
        for i in 0..32 {
            assert_eq!(dec.feed(PinLevel::Low, 500), None);
            let bits = frame_bits(0x00, 0xFF, 0x18, !0x18);
            let space = if bits >> i & 1 == 1 { 1750 } else { 620 };
            assert_eq!(dec.feed(PinLevel::High, space), None);
        }
        assert_eq!(dec.feed(PinLevel::Low, 620).unwrap().command, 0x18);
    }
}
