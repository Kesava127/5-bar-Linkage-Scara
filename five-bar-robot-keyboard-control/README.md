# Five-Bar Robot --- Keyboard-Controlled End Effector

Arduino-based control for a five-bar parallel linkage. The Arduino
calculates inverse kinematics and moves two stepper motors to update the
end-effector target using keyboard commands sent through the Arduino IDE
Serial Monitor.

> **Project status:** Prototype / experimental. This project does not
> include homing sensors, encoder feedback, collision detection, or
> automatic calibration. Use low movement increments and keep the
> mechanism clear during testing.

## Features

-   Inverse kinematics for a five-bar linkage.
-   Serial Monitor keyboard control using `W`, `A`, `S`, and `D`.
-   Configurable Cartesian target increment.
-   Signed step commands for two stepper motors.
-   Simultaneous step pulses using a simple Bresenham-style stepping
    loop.
-   No Python script or CmdMessenger library required.

## Hardware assumptions

-   Arduino Uno or another compatible 5 V AVR board with the same pin
    mapping.
-   CNC Shield V3-style stepper interface.
-   Two stepper motors and compatible stepper drivers.
-   Driver microstepping configured for **1/16**.
-   Motor step angle: **1.8°**.
-   Link lengths entered in millimetres.

Check the pinout and electrical compatibility of your exact board and
driver before connecting power.

## Link dimensions

  Parameter      Value Description
  ----------- -------- -----------------------------
  `l1`          100 mm Left proximal link
  `l2`          100 mm Left distal link
  `l3`          100 mm Right distal link
  `l4`          100 mm Right proximal link
  `l5`           50 mm Distance between motor axes

The inverse-kinematics code assumes the left motor pivot is at `(0, 0)`
and the right motor pivot is at `(l5, 0)`.

## Pin mapping

  Arduino pin   Signal    Purpose
  ------------- --------- --------------------------------------
  D2            `X_STP`   Left step pulse
  D5            `X_DIR`   Left direction
  D3            `Y_STP`   Right step pulse
  D6            `Y_DIR`   Right direction
  D8            `EN`      Driver enable, active LOW
  D4            `Z_STP`   Unused Z step output, configured LOW
  D7            `Z_DIR`   Unused Z direction output

This pin mapping is intended for the CNC Shield V3 convention. Confirm
the mapping against your shield revision.

## Repository layout

``` text
five-bar-robot-keyboard-control/
├── five_bar_control.ino
├── README.md
├── LICENSE
├── .gitignore
└── docs/
    └── wiring-and-operation.md
```

## Setup

1.  Install the Arduino IDE.
2.  Open `five_bar_control.ino`.
3.  Select the correct board and processor under **Tools → Board**.
4.  Select the correct port under **Tools → Port**.
5.  Upload the sketch.
6.  Open **Tools → Serial Monitor**.
7.  Set the baud rate to **115200**.
8.  Set line ending to **No line ending**.
9.  Use the keys below to send one command at a time.

No external Arduino library is required.

## Keyboard controls

The Serial Monitor does not reliably pass arrow-key input as simple
characters, so this project uses WASD.

  -----------------------------------------------------------------------
  Key                                 Action
  ----------------------------------- -----------------------------------
  `W`                                 Increase target Y

  `S`                                 Decrease target Y

  `A`                                 Decrease target X

  `D`                                 Increase target X

  `+`                                 Increase increment by 1 mm, up to
                                      10 mm

  `-`                                 Decrease increment by 1 mm, down to
                                      1 mm

  `X`                                 Disable movement commands

  `E`                                 Re-enable movement commands

  `R`                                 Reset software target/angle state
                                      to the configured initial pose
  -----------------------------------------------------------------------

The initial increment is **1 mm**. Each key sent to the Serial Monitor
is processed as a single movement command.

### Changing the movement increment

Edit this line in the sketch to change the initial movement increment:

``` cpp
float increment = 1.0;
```

### Changing the assumed initial position

Edit:

``` cpp
const float HOME_X = 50.0;
const float HOME_Y = 100.0;
```

These values are software assumptions only; they do **not** physically
home the mechanism.

## Important calibration and safety notes

1.  **Align the mechanism before moving.** At startup, the code
    calculates joint angles for `(X, Y) = (50 mm, 100 mm)` and treats
    those angles as the current position. It does not move the motors to
    that pose or measure their actual positions. Physically position the
    mechanism to match the assumed pose before sending movement
    commands.
2.  **Microstepping must match.** `STEPS_PER_DEGREE = 8.888888889`
    assumes a 1.8° stepper configured for 1/16 microstepping. Change
    this value if your hardware differs.
3.  **Check motor direction at low speed.** Set `INVERT_LEFT` or
    `INVERT_RIGHT` to `true` if the corresponding motor moves in the
    wrong direction. Test one small movement at a time.
4.  **No feedback is used.** The software tracks commanded steps only.
    Missed steps, stalls, or manually moving the linkage will cause the
    software position estimate to drift.
5.  **Workspace checks are mathematical, not physical.** The sketch
    checks whether the selected IK equations return a valid solution,
    but it does not enforce mechanical joint limits, avoid collisions,
    or guarantee the linkage cannot hit its frame.
6.  **Movement is not a true Cartesian trajectory planner.** Both motors
    step together for each target update, but interpolating joint motion
    does not guarantee a perfectly straight end-effector path between
    targets.
7.  **Emergency stop:** `X` disables further movement commands but does
    not cut motor power or instantly interrupt a move already executing.
    Keep a physical power switch accessible.

## Troubleshooting

### The sketch does not compile

-   Confirm the selected board is an Arduino AVR-compatible board if
    using the documented pin setup.
-   This sketch uses only built-in headers (`math.h` and `stdlib.h`); no
    CmdMessenger library is needed.
-   Avoid using `B1` as a variable name on AVR Arduino boards because it
    conflicts with a predefined macro.

### Motors move in the wrong direction

Change:

``` cpp
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = false;
```

Set the relevant flag to `true`, upload again, and retest carefully.

### The end effector moves unpredictably

-   Stop the mechanism and verify the physical initial pose.
-   Verify link lengths and pivot spacing.
-   Verify the driver microstepping setting and motor step angle.
-   Check for skipped steps, binding, or loose couplings.
-   Start with a 1 mm increment and a clear workspace.

### The target is outside the reachable workspace

The code rejects target coordinates for which the selected
inverse-kinematics branch has no real solution. Try a nearby target and
verify the dimensions and coordinate origin.

## Limitations / next steps

-   Add homing switches or another repeatable calibration method.
-   Add mechanical joint limits and a software-defined safe workspace.
-   Improve trajectory planning so the end effector follows a controlled
    Cartesian path.
-   Add acceleration/deceleration to reduce missed steps.
-   Add feedback using encoders if accurate position confirmation is
    required.
-   Consider an independent hardware emergency stop.

## License

Released under the MIT License. See [`LICENSE`](LICENSE).
