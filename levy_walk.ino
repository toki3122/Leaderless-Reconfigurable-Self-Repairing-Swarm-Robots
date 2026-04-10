#include <ESP32Servo.h>
// ═══════════════════════════════════════════════════════
// SERVO SETUP
// SG90 360° continuous rotation servo
// speed control: 90 = stop, <90 = one way, >90 = other way
// ═══════════════════════════════════════════════════════
Servo leftServo;
Servo rightServo;
const int LEFT_SERVO_PIN = 18;
const int RIGHT_SERVO_PIN = 19;
const int SHARP_TURN_MS = 600; //90° pivot
const int SLOW_TURN_MS  = 800; //existing slow sweep
// ── servo speed constants ─────────────────────────────
const int STOP = 90; // both servos stopped
const int FULL_FWD = 50; // full forward speed
const int FULL_REV = 120; // full reverse speed
const int TURN_SPEED = 35; // how much to offset for turning
// ═══════════════════════════════════════════════════════
// LÉVY WALK PARAMETERS
// ═══════════════════════════════════════════════════════
const float LEVY_ALPHA = 1.0; // 0.0-2.0 range
                                   // lower = more long moves
                                   // higher = more short moves
const int MIN_MOVE_MS = 400; // shortest possible move (ms)
const int MAX_MOVE_MS = 6000; // longest possible move (ms)
const int MIN_TURN_MS = 200; // shortest turn duration
const int MAX_TURN_MS = 1200; // longest turn duration
const int DIRECTION_CHECK_MS = 100; // check every 100ms
// ═══════════════════════════════════════════════════════
// MOVEMENT STATE
// ═══════════════════════════════════════════════════════
enum MoveState {
  MOVE_FORWARD,
  TURN_LEFT,
  TURN_RIGHT,
  MOVE_BACKWARD
};
MoveState currentMove = MOVE_FORWARD;
unsigned long moveStart = 0;
unsigned long moveDuration = 0;
// ═══════════════════════════════════════════════════════
// LÉVY STEP SIZE GENERATOR
// uses power law distribution — occasionally returns
// ═══════════════════════════════════════════════════════
float levySample() {
  // power law sampling using inverse transform method
  // generates numbers with heavy tail distribution
  // most values small, occasional very large value
  float u = (float)random(1, 10000) / 10000.0; // uniform 0.0 to 1.0
  return pow(u, -1.0 / LEVY_ALPHA); // power law transform
}
unsigned long levyStepDuration() {
  float raw = levySample();
  // raw value is typically 1.0 to ~50.0
  float normalized = (raw - 1.0) / 49.0; // 0.0 to 1.0
  normalized = constrain(normalized, 0.0, 1.0);
  unsigned long duration = MIN_MOVE_MS +
    (unsigned long)(normalized * (MAX_MOVE_MS - MIN_MOVE_MS));

  return duration;
}
void driveForward() {
  leftServo.write(FULL_FWD);
  rightServo.write(FULL_REV); // mirrored — servos face opposite directions
}
void driveBackward() {
  leftServo.write(FULL_REV);
  rightServo.write(FULL_FWD);
}
void turnLeft() {
  leftServo.write(STOP + TURN_SPEED);
  rightServo.write(STOP - TURN_SPEED);
}

void turnRight() {
  leftServo.write(STOP - TURN_SPEED);
  rightServo.write(STOP + TURN_SPEED);
}

void stopMotors() {
  leftServo.write(STOP);
  rightServo.write(STOP);
}
void applyMovement() {
  switch (currentMove) {
    case MOVE_FORWARD: driveForward(); break;
    case TURN_LEFT: turnLeft(); break;
    case TURN_RIGHT: turnRight(); break;
    case MOVE_BACKWARD: driveBackward(); break;
  }
}
// ═══════════════════════════════════════════════════════
// NEXT MOVE PICKER
// weighted random — forward moves happen most often
// long forward = Lévy step duration
// turns = short fixed duration
// backward = rare, short — just to escape corners
// ═══════════════════════════════════════════════════════
void pickNextMove() {
  int r = random(0, 100);
  if (r < 55) {
    // 55% forward — Lévy duration
    currentMove = MOVE_FORWARD;
    moveDuration = levyStepDuration();
  } else if (r < 65) {
    // 10% sharp left snap
    currentMove = TURN_LEFT;
    moveDuration = SHARP_TURN_MS;
  } else if (r < 75) {
    // 10% sharp right snap
    currentMove = TURN_RIGHT;
    moveDuration = SHARP_TURN_MS;
  } else if (r < 85) {
    // 10% slow left sweep
    currentMove = TURN_LEFT;
    moveDuration = SLOW_TURN_MS;
  } else if (r < 95) {
    // 10% slow right sweep
    currentMove = TURN_RIGHT;
    moveDuration = SLOW_TURN_MS;
  } else {
    // 5% reverse
    currentMove = MOVE_BACKWARD;
    moveDuration = random(MIN_MOVE_MS, MIN_MOVE_MS * 2);
  }
  moveStart = millis();
  applyMovement();
  Serial.print("New move: ");
  switch (currentMove) {
    case MOVE_FORWARD:  Serial.print("FORWARD");  break;
    case TURN_LEFT:     Serial.print("LEFT");     break;
    case TURN_RIGHT:    Serial.print("RIGHT");    break;
    case MOVE_BACKWARD: Serial.print("BACKWARD"); break;
  }
  Serial.print(" duration: ");
  Serial.print(moveDuration);
  Serial.println("ms");
}
// ═══════════════════════════════════════════════════════
// EXTERNAL INTERRUPT POINTS
// FSM calls these to take over from wander
// ═══════════════════════════════════════════════════════
bool wanderActive = true; // set false to pause wander
void pauseWander() {
  wanderActive = false;
  stopMotors();
  Serial.println("Wander paused — FSM taking control");
}

void resumeWander() {
  wanderActive = true;
  pickNextMove(); // start fresh move when resuming
  Serial.println("Wander resumed");
}
void setup() {
  Serial.begin(115200);
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  stopMotors();
  delay(500);
  // seed random number generator with floating ADC noise
  // gives different random sequence every power cycle
  randomSeed(analogRead(0));
  pickNextMove();
  Serial.println("Lévy walk ready");
}
unsigned long lastCheck = 0;
void loop() {
  // ── only run wander if FSM has not taken over ──────
  if (!wanderActive) return;
  unsigned long now = millis();
  // ── check if current move has finished ────────────
  if (now - moveStart >= moveDuration) {
    pickNextMove(); // pick and start next move immediately
  }
  // ──sensor checks go here ────────────────────
  //check SOS every 100ms without blocking
  if (now - lastCheck > DIRECTION_CHECK_MS) {
    lastCheck = now;
    // plug in your checkSOS() from earlier code here:
    // if (checkSOS(ax, ay, az)) {
    // pauseWander();
    // myData.sosFlag = true;
    // // FSM will handle the rest
    // }
    // plug in your broadcastState() here:
    // broadcastState();
  }
}
