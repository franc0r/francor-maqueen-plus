# francor-maqueen-plus

Async, `no_std`, hardware-agnostic driver for the DFRobot Maqueen Plus V3 robot,
built on `embedded-hal-async`:

- `MaqueenPlusV3`: robot coprocessor at I2C 0x10 (motors, headlights, line sensors,
  built-in line following, PID moves, telemetry)
- `MatrixLidar`: 8×8 matrix laser ranging sensor (I2C 0x30–0x33)
- `nec::Decoder`: pure NEC infrared decoder fed with edge timings
- `ws2812`: colour type and bit-stream encoder for the four RGB LEDs

Enable the `defmt` feature for `defmt::Format` impls.

For the BBC micro:bit V2 (embassy-nrf) see
[francor-maqueen-plus-microbit](https://github.com/franc0r/francor-maqueen-plus-microbit).

## Development

```bash
cargo test
cargo build --target thumbv7em-none-eabihf --features defmt
```
