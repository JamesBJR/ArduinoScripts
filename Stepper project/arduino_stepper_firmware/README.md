# arduino_stepper_basic_improved

A minimal PlatformIO project for Arduino Uno driving a stepper via a Big Easy Driver (or similar step/dir driver).

## Hardware (typical Big Easy Driver)
- STEP -> D2
- DIR  -> D3
- EN   -> D4 (LOW = enabled)
- VMOT: external motor supply (use correct voltage for your motor)
- GND: common ground between motor PSU and Arduino
- Optional SPEED_POT on A0 for live speed control

**Recommended power & protection**
- Power the motor driver from a dedicated motor PSU.
- Power the Uno via USB or its own 7–12 V barrel jack (or 5 V regulated to 5V pin).
- Tie grounds together.
- Place a bulk electrolytic capacitor (e.g., 47–100 µF, ≥ 25 V rating appropriate to your VMOT) across VMOT and GND near the driver.
- Set current limit on the driver to match your motor (e.g., start ~0.7–0.8 A/phase for a small NEMA 11 and increase if needed).

## Build & Upload (PlatformIO)
```
pio run -e uno
pio run -e uno -t upload
pio device monitor -b 115200
```

## Serial Commands
- `G <steps>`  : move relative steps (e.g., `G 3200`)
- `R`          : +1 revolution
- `L`          : -1 revolution
- `V <steps/s>`: set max speed
- `A <steps/s^2>`: set acceleration
- `E`          : enable (EN LOW)
- `D`          : disable (EN HIGH)
- `Z`          : soft stop

## Notes
- If you use microstepping, update `STEPS_PER_REV` accordingly (200 * microstep).
- The example continuously updates max speed from the A0 pot; if you don't want that, remove the block in `loop()`.
