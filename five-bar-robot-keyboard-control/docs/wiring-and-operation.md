# Wiring and Operation Notes

## Wiring overview

Use the CNC Shield V3 or equivalent driver board according to its own
documentation.

  Function           Sketch pin
  ------------------ ----------------
  Left motor STEP    D2
  Left motor DIR     D5
  Right motor STEP   D3
  Right motor DIR    D6
  Driver enable      D8, active LOW

Motor power must come from a suitable motor supply connected to the
driver board. Do not power stepper motors directly from Arduino GPIO
pins. Confirm the driver current limit and motor supply voltage before
testing.

## First movement checklist

-   [ ] Mechanism is free to move and clear of obstructions.
-   [ ] Motor drivers are configured for 1/16 microstepping.
-   [ ] Link dimensions and pivot spacing are measured.
-   [ ] Mechanism is physically aligned to the assumed initial pose
    `(50 mm, 100 mm)`.
-   [ ] Serial Monitor is set to 115200 baud and No line ending.
-   [ ] The first command is a single small movement.
-   [ ] Motor direction and linkage motion are verified before continued
    use.

## Important software behavior

`R` resets the software's assumed coordinates and joint-angle state. It
does not home the motors or physically return the mechanism to the
initial position. Use it only after manually realigning the mechanism to
the assumed pose.
