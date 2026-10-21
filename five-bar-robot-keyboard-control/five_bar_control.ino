#include <math.h>
#include <stdlib.h>

// ---------------- STEPPER DRIVER PINS (CNC shield V3) ----------------
#define EN     8
#define X_DIR  5
#define Y_DIR  6
#define Z_DIR  7
#define X_STP  2
#define Y_STP  3
#define Z_STP  4

// Flip these if a joint turns the wrong way (mirrored motor mounting)
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = false;

// ---------------- LINK DIMENSIONS (mm) ----------------
const float l1 = 100.0;   // left proximal
const float l2 = 100.0;   // left distal
const float l3 = 100.0;   // right distal
const float l4 = 100.0;   // right proximal
const float l5 = 50.0;    // distance between motor axes

// 1.8-degree stepper, 1/16 microstepping
const float STEPS_PER_DEGREE = 8.888888889;

// ---------------- MOVEMENT SETTINGS ----------------
const unsigned int pulseDelay = 500;   // us, high and low each
float increment = 1.0;

const float HOME_X = 50.0;
const float HOME_Y = 100.0;

float x = HOME_X;
float y = HOME_Y;

// Commanded joint angles (degrees, kept in 0..360)
float theta1Current = 0.0;
float theta2Current = 0.0;

bool initialized = false;
bool movementEnabled = true;

// ---------------- SYNCHRONISED STEPPER CONTROL ----------------
// Both motors are stepped together (Bresenham) so they start and
// finish at the same time and the end effector moves in a straight line.
void stepMotors(long s1, long s2) {
  if (s1 == 0 && s2 == 0) return;

  digitalWrite(X_DIR, ((s1 > 0) != INVERT_LEFT)  ? HIGH : LOW);
  digitalWrite(Y_DIR, ((s2 > 0) != INVERT_RIGHT) ? HIGH : LOW);
  delayMicroseconds(10);

  long a1 = labs(s1);
  long a2 = labs(s2);
  long n  = (a1 > a2) ? a1 : a2;
  long e1 = 0, e2 = 0;

  for (long i = 0; i < n; i++) {
    e1 += a1;
    e2 += a2;
    bool p1 = false, p2 = false;
    if (e1 >= n) { e1 -= n; p1 = true; }
    if (e2 >= n) { e2 -= n; p2 = true; }

    if (p1) digitalWrite(X_STP, HIGH);
    if (p2) digitalWrite(Y_STP, HIGH);
    delayMicroseconds(pulseDelay);
    if (p1) digitalWrite(X_STP, LOW);
    if (p2) digitalWrite(Y_STP, LOW);
    delayMicroseconds(pulseDelay);
  }
}

// ---------------- INVERSE KINEMATICS ----------------
// Left motor at (0,0), right motor at (l5,0).
bool inverseKinematics(float px, float py,
                       float &theta1, float &theta2) {
  float aLeft = px;
  float bLeft = py;
  float cLeft = (l1*l1 - l2*l2 + px*px + py*py) / (2.0*l1);
  float dLeft = aLeft*aLeft + bLeft*bLeft - cLeft*cLeft;

  float aRight = px - l5;
  float bRight = py;
  float cRight = (l4*l4 + l5*l5 - l3*l3
                  - 2.0*px*l5 + px*px + py*py) / (2.0*l4);
  float dRight = aRight*aRight + bRight*bRight - cRight*cRight;

  if (dLeft < -0.0001 || dRight < -0.0001) return false;

  if (dLeft  < 0.0) dLeft  = 0.0;
  if (dRight < 0.0) dRight = 0.0;

  theta1 = 2.0 * atan2(-bLeft - sqrt(dLeft),   -aLeft  - cLeft);
  theta2 = 2.0 * atan2(-bRight + sqrt(dRight), -aRight - cRight);

  theta1 *= 180.0 / PI;
  theta2 *= 180.0 / PI;

  while (theta1 < 0)    theta1 += 360.0;
  while (theta1 >= 360) theta1 -= 360.0;
  while (theta2 < 0)    theta2 += 360.0;
  while (theta2 >= 360) theta2 -= 360.0;

  return true;
}

// ---------------- MOVE END EFFECTOR ----------------
void moveTo(float newX, float newY) {
  float theta1Target, theta2Target;

  if (!inverseKinematics(newX, newY, theta1Target, theta2Target)) {
    Serial.println("Target outside reachable workspace!");
    return;
  }

  float delta1 = theta1Target - theta1Current;
  float delta2 = theta2Target - theta2Current;

  // Shortest angular movement
  while (delta1 > 180.0)  delta1 -= 360.0;
  while (delta1 < -180.0) delta1 += 360.0;
  while (delta2 > 180.0)  delta2 -= 360.0;
  while (delta2 < -180.0) delta2 += 360.0;

  long steps1 = lround(delta1 * STEPS_PER_DEGREE);
  long steps2 = lround(delta2 * STEPS_PER_DEGREE);

  stepMotors(steps1, steps2);

  // Update commanded angles from the steps actually issued
  theta1Current += steps1 / STEPS_PER_DEGREE;
  theta2Current += steps2 / STEPS_PER_DEGREE;
  while (theta1Current < 0)    theta1Current += 360.0;
  while (theta1Current >= 360) theta1Current -= 360.0;
  while (theta2Current < 0)    theta2Current += 360.0;
  while (theta2Current >= 360) theta2Current -= 360.0;

  x = newX;
  y = newY;

  Serial.print("X: ");
  Serial.print(x, 2);
  Serial.print(" mm | Y: ");
  Serial.print(y, 2);
  Serial.print(" mm | Theta1: ");
  Serial.print(theta1Current, 2);
  Serial.print(" deg | Theta2: ");
  Serial.print(theta2Current, 2);
  Serial.println(" deg");
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  pinMode(EN, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(X_STP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(Y_STP, OUTPUT);
  pinMode(Z_DIR, OUTPUT);
  pinMode(Z_STP, OUTPUT);

  digitalWrite(X_STP, LOW);
  digitalWrite(Y_STP, LOW);
  digitalWrite(Z_STP, LOW);
  digitalWrite(EN, LOW); // Enable drivers, active LOW

  Serial.println();
  Serial.println("Five-Bar Robot End-Effector Control");
  Serial.println("-----------------------------------");
  Serial.println("W = Up (+Y)");
  Serial.println("S = Down (-Y)");
  Serial.println("A = Left (-X)");
  Serial.println("D = Right (+X)");
  Serial.println("+ = Increase increment");
  Serial.println("- = Decrease increment");
  Serial.println("X = Disable movement");
  Serial.println("E = Re-enable movement");
  Serial.println("R = Reset software coordinates");
  Serial.println();

  if (inverseKinematics(x, y, theta1Current, theta2Current)) {
    initialized = true;
    Serial.println("Initial IK calculated.");
    Serial.println("Initial target: X=50 mm, Y=100 mm");
    Serial.println("IMPORTANT: Physically align the robot");
    Serial.println("to this pose before commanding movement.");
  } else {
    Serial.println("Initial position unreachable!");
  }
}

// ---------------- MAIN LOOP ----------------
void loop() {
  if (!Serial.available()) return;

  char key = Serial.read();

  if (key == '\r' || key == '\n' || key == ' ') return;

  if (key == 'X' || key == 'x') {
    movementEnabled = false;
    Serial.println("Movement commands disabled. Press E to re-enable.");
    return;
  }

  if (key == 'E' || key == 'e') {
    movementEnabled = true;
    Serial.println("Movement commands enabled.");
    return;
  }

  if (key == '+') {
    increment = min(increment + 1.0f, 10.0f);
    Serial.print("Increment: ");
    Serial.print(increment);
    Serial.println(" mm");
    return;
  }

  if (key == '-') {
    increment = max(increment - 1.0f, 1.0f);
    Serial.print("Increment: ");
    Serial.print(increment);
    Serial.println(" mm");
    return;
  }

  if (key == 'R' || key == 'r') {
    Serial.println("Resetting software coordinates only.");
    Serial.println("Physically realign the robot before moving.");
    x = HOME_X;
    y = HOME_Y;
    initialized = inverseKinematics(x, y, theta1Current, theta2Current);
    if (initialized) {
      movementEnabled = true;
      Serial.println("Software state reset.");
    }
    return;
  }

  if (!movementEnabled) {
    Serial.println("Movement disabled. Press E to re-enable.");
    return;
  }

  if (!initialized) {
    Serial.println("Initial position not initialized.");
    return;
  }

  float newX = x;
  float newY = y;

  switch (key) {
    case 'W': case 'w': newY += increment; break;
    case 'S': case 's': newY -= increment; break;
    case 'A': case 'a': newX -= increment; break;
    case 'D': case 'd': newX += increment; break;
    default: return;
  }

  moveTo(newX, newY);
}
