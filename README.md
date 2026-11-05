Motor test rig
===

Minimal ESP32-C5 firmware for testing CAN communication with new motors.

- While the **ack (green) button** is held, it sends motor mode 33 (speed
  mode, torque 200) and speed +1000 to both wheel motors every 20 ms, and the
  lift motor speed mode 33 with speed +1000 (0x300), after a one-time lift
  torque limit of 200 (0x301).
- On release it sends mode 0 (and speed 0) to all three motors.
- Motor state frames (0x141-0x144) and lift status/position (0x310, 0x311) are
  printed to serial, at most one line per ID every 500 ms, together with
  button presses and CAN/I2C errors.

STO is not checked and the safety relay is not reset, so the safety circuit
must already be in a state that lets the motors run.

Pins: CAN TX GPIO24, RX GPIO25, STB GPIO23; I2C SDA GPIO2, SCL GPIO3; IO
expander reset GPIO0. The button is read from the MCP23017 at 0x21, port A
bit 1 (ack).

```bash
cargo run --release   # builds, flashes with espflash and opens the monitor
```

To change the commands (speed, torque, ramps) edit the constants at the top of
`src/main.rs`.
