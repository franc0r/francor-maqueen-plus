//! Colour type and PWM bit-stream encoder for the four WS2812 LEDs on pin P15.

/// PWM duty word for a WS2812 "0" bit (0.4375 µs high of 1.25 µs, i.e. 7/20; bit 15 inverts polarity).
pub const T0H: u16 = 0x8000 | 7;
/// PWM duty word for a WS2812 "1" bit (0.8125 µs high of 1.25 µs, i.e. 13/20).
pub const T1H: u16 = 0x8000 | 13;
/// PWM duty word that keeps the line low (reset/latch).
pub const RESET: u16 = 0x8000;
/// PWM words per LED (8 bits × G, R, B).
pub const WORDS_PER_LED: usize = 24;

/// Number of PWM words needed for `leds` LEDs plus the trailing reset word.
pub const fn buffer_len(leds: usize) -> usize {
    leds * WORDS_PER_LED + 1
}

/// An RGB colour, 8 bits per channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Rgb {
    /// Red.
    pub r: u8,
    /// Green.
    pub g: u8,
    /// Blue.
    pub b: u8,
}

impl Rgb {
    /// Off.
    pub const BLACK: Rgb = Rgb::new(0, 0, 0);
    /// Red.
    pub const RED: Rgb = Rgb::new(255, 0, 0);
    /// Green.
    pub const GREEN: Rgb = Rgb::new(0, 255, 0);
    /// Blue.
    pub const BLUE: Rgb = Rgb::new(0, 0, 255);
    /// Yellow.
    pub const YELLOW: Rgb = Rgb::new(255, 255, 0);
    /// White.
    pub const WHITE: Rgb = Rgb::new(255, 255, 255);

    /// Build from channels.
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Rgb { r, g, b }
    }

    /// Build from a `0xRRGGBB` value (the MakeCode colour format).
    pub const fn from_u32(rgb: u32) -> Self {
        Rgb::new((rgb >> 16) as u8, (rgb >> 8) as u8, rgb as u8)
    }

    /// Scale all channels by `brightness / 255`.
    pub const fn scale(self, brightness: u8) -> Self {
        let s = brightness as u32;
        Rgb::new(
            (self.r as u32 * s / 255) as u8,
            (self.g as u32 * s / 255) as u8,
            (self.b as u32 * s / 255) as u8,
        )
    }

    /// Colour from hue (0..360), saturation (0..100) and lightness (0..100).
    pub fn hsl(h: u16, s: u8, l: u8) -> Self {
        let h = (h % 360) as i32;
        let s = s.min(100) as i32;
        let l = l.min(100) as i32;
        let c = (100 - (2 * l - 100).abs()) * s; // chroma × 10000, 0..10000
        let hp = h % 120;
        let x = c * (60 - (hp - 60).abs()) / 60;
        let (r1, g1, b1) = match h / 60 {
            0 => (c, x, 0),
            1 => (x, c, 0),
            2 => (0, c, x),
            3 => (0, x, c),
            4 => (x, 0, c),
            _ => (c, 0, x),
        };
        let m = l * 100 - c / 2;
        let to8 = |v: i32| ((v + m) * 255 / 10000).clamp(0, 255) as u8;
        Rgb::new(to8(r1), to8(g1), to8(b1))
    }
}

/// Encode `leds` into PWM duty words in `out` (GRB order, MSB first), followed by one
/// reset word. Returns the number of words written, or `None` if `out` is too small.
pub fn encode(leds: &[Rgb], brightness: u8, out: &mut [u16]) -> Option<usize> {
    let needed = buffer_len(leds.len());
    if out.len() < needed {
        return None;
    }
    let mut i = 0;
    for led in leds {
        let led = led.scale(brightness);
        for byte in [led.g, led.r, led.b] {
            for bit in (0..8).rev() {
                out[i] = if byte >> bit & 1 == 1 { T1H } else { T0H };
                i += 1;
            }
        }
    }
    out[i] = RESET;
    Some(needed)
}

/// `N` fully saturated colours with hues spread evenly from `start_hue` to `end_hue`
/// (degrees, clockwise), like the MakeCode rainbow block.
pub fn rainbow<const N: usize>(start_hue: u16, end_hue: u16) -> [Rgb; N] {
    let start = (start_hue % 360) as u32;
    let span = ((end_hue % 360) as u32 + 360 - start) % 360;
    let mut out = [Rgb::BLACK; N];
    for (i, slot) in out.iter_mut().enumerate() {
        let hue = if N > 1 {
            start + span * i as u32 / (N as u32 - 1)
        } else {
            start
        };
        *slot = Rgb::hsl(hue as u16, 100, 50);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn buffer_len_is_24_words_per_led_plus_reset() {
        assert_eq!(buffer_len(4), 97);
    }

    #[test]
    fn encode_writes_grb_msb_first_and_reset() {
        let mut out = [0u16; 25];
        let n = encode(&[Rgb::new(0x80, 0x01, 0xFF)], 255, &mut out).unwrap();
        assert_eq!(n, 25);
        // G = 0x01 -> 0000 0001
        assert_eq!(&out[0..8], &[T0H, T0H, T0H, T0H, T0H, T0H, T0H, T1H]);
        // R = 0x80 -> 1000 0000
        assert_eq!(&out[8..16], &[T1H, T0H, T0H, T0H, T0H, T0H, T0H, T0H]);
        // B = 0xFF
        assert_eq!(&out[16..24], &[T1H; 8]);
        assert_eq!(out[24], RESET);
    }

    #[test]
    fn encode_four_leds_places_reset_at_word_96() {
        let leds = [
            Rgb::new(0x12, 0x34, 0x56),
            Rgb::RED,
            Rgb::GREEN,
            Rgb::new(0xAB, 0xCD, 0xEF),
        ];
        let mut out = [0u16; 97];
        let n = encode(&leds, 255, &mut out).unwrap();
        assert_eq!(n, 97);

        let bit = |byte: u8, i: usize| if byte >> (7 - i) & 1 == 1 { T1H } else { T0H };

        // LED 0 at word 0: GRB order, MSB first.
        for i in 0..8 {
            assert_eq!(out[i], bit(0x34, i), "LED0 G bit {i}");
            assert_eq!(out[8 + i], bit(0x12, i), "LED0 R bit {i}");
            assert_eq!(out[16 + i], bit(0x56, i), "LED0 B bit {i}");
        }
        // LED 3 starts at word 3 * 24 = 72.
        for i in 0..8 {
            assert_eq!(out[72 + i], bit(0xCD, i), "LED3 G bit {i}");
            assert_eq!(out[80 + i], bit(0xAB, i), "LED3 R bit {i}");
            assert_eq!(out[88 + i], bit(0xEF, i), "LED3 B bit {i}");
        }
        assert_eq!(out[96], RESET);
    }

    #[test]
    fn encode_applies_brightness() {
        let mut out = [0u16; 25];
        encode(&[Rgb::new(255, 255, 255)], 0, &mut out).unwrap();
        assert_eq!(&out[0..24], &[T0H; 24]);
    }

    #[test]
    fn encode_rejects_short_buffer() {
        let mut out = [0u16; 24];
        assert_eq!(encode(&[Rgb::RED], 255, &mut out), None);
    }

    #[test]
    fn from_u32_and_scale() {
        assert_eq!(Rgb::from_u32(0xFFA500), Rgb::new(0xFF, 0xA5, 0x00));
        assert_eq!(Rgb::new(200, 100, 0).scale(128), Rgb::new(100, 50, 0));
    }

    #[test]
    fn hsl_primary_colours() {
        assert_eq!(Rgb::hsl(0, 100, 50), Rgb::new(255, 0, 0));
        assert_eq!(Rgb::hsl(120, 100, 50), Rgb::new(0, 255, 0));
        assert_eq!(Rgb::hsl(240, 100, 50), Rgb::new(0, 0, 255));
        assert_eq!(Rgb::hsl(60, 100, 50), Rgb::new(255, 255, 0));
        assert_eq!(Rgb::hsl(0, 0, 0), Rgb::BLACK);
    }

    #[test]
    fn rainbow_spreads_hue_across_leds() {
        let r: [Rgb; 4] = rainbow(0, 270);
        assert_eq!(r[0], Rgb::hsl(0, 100, 50));
        assert_eq!(r[1], Rgb::hsl(90, 100, 50));
        assert_eq!(r[2], Rgb::hsl(180, 100, 50));
        assert_eq!(r[3], Rgb::hsl(270, 100, 50));
    }
}
