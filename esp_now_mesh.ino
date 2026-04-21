#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ═══════════════════════════════════════════════════════════════════════════
// [ESPNOW] — SHARED DATA STRUCT
// ═══════════════════════════════════════════════════════════════════════════

typedef struct BotData {
  uint8_t  botID;
  float    posX;
  float    posY;
  float    velocity;
  int      battery;
  bool     sosFlag;
  bool     mergeStatus;
  float    pitch;
  float    roll;
  uint8_t  mergePartnerID;
} BotData;

// ═══════════════════════════════════════════════════════════════════════════
// [ESPNOW] — SWARM STATE STORAGE
// ═══════════════════════════════════════════════════════════════════════════

BotData myData;
BotData swarm[3];
bool    peerSeen[3] = {false, false, false};
int     lastRSSI[3] = {-100, -100, -100};

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint8_t THIS_BOT_ID = 0; //change for every bot *****very important*****

float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;
float AngleRoll  = 0;
float AnglePitch = 0;
float AngleRollAcc, AnglePitchAcc;
float GyroCalRoll = 0, GyroCalPitch = 0, GyroCalYaw = 0;
float TempC;
unsigned long prevTime = 0;

bool unstable = false;
bool roughTerrain = false;
bool stuck = false;

bool warmedUp             = false;
unsigned long warmupStart = 0;

float prevAccX = 0, prevAccY = 0, prevAccZ = 0;

float posX = 0, posY = 0;
float velX = 0, velY = 0;
float heading = 0;

const float VEL_DECAY = 0.98;
const float ACCEL_DEADZONE = 0.1;
const float ACCEL_BIAS_TC  = 0.995;
float accelBiasX = 0, accelBiasY = 0;

String sosReason  = "";
unsigned long stuckTimer = 0;
unsigned long flatTimer  = 0;   
const float   TILT_RAISE = 20.0;
const float   TILT_CLEAR = 15.0;
const float   JERK_THRESH = 0.25;
const float   MOTION_THRESH = 0.095;
const unsigned long STUCK_TIME_MS = 5000;
const unsigned long SOS_CLEAR_TIME_MS = 3000;

Servo leftServo;
Servo rightServo;

const int LEFT_SERVO_PIN  = 18;
const int RIGHT_SERVO_PIN = 19;

const int STOP       = 90;
const int FULL_FWD   = 50;
const int FULL_REV   = 120;
const int TURN_SPEED = 35;

const int SHARP_TURN_MS = 600;
const int SLOW_TURN_MS  = 800;

const float LEVY_ALPHA         = 1.0;
const int   MIN_MOVE_MS        = 400;
const int   MAX_MOVE_MS        = 6000;
const int   DIRECTION_CHECK_MS = 100;

enum MoveState { MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, MOVE_BACKWARD };
MoveState     currentMove  = MOVE_FORWARD;
unsigned long moveStart    = 0;
unsigned long moveDuration = 0;
bool          wanderActive = true;

unsigned long lastBroadcast = 0;
unsigned long lastCheck     = 0;

void imu_read() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  int16_t TempRaw = Wire.read() << 8 | Wire.read();
  int16_t GyroX   = Wire.read() << 8 | Wire.read();
  int16_t GyroY   = Wire.read() << 8 | Wire.read();
  int16_t GyroZ   = Wire.read() << 8 | Wire.read();

  AccX  = AccXLSB / 4096.0;
  AccY  = AccYLSB / 4096.0;
  AccZ  = AccZLSB / 4096.0;
  TempC = (TempRaw / 340.0) + 36.53;

  RateRoll  = (GyroX / 65.5) - GyroCalRoll;
  RatePitch = (GyroY / 65.5) - GyroCalPitch;
  RateYaw   = (GyroZ / 65.5) - GyroCalYaw;
}

void updateHeading(float dt) {
  heading += RateYaw * dt;
  if (heading > 360) heading -= 360;
  if (heading < 0)   heading += 360;
}

// ═══════════════════════════════════════════════════════════════════════════
// DEAD RECKONING POSITION
// ═══════════════════════════════════════════════════════════════════════════

void updatePosition(float dt) {
  if (dt <= 0 || dt > 0.5) return;

  float pitchRad = AnglePitch * 0.01745;
  float rollRad  = AngleRoll  * 0.01745;

  float linAccX = AccX - sin(pitchRad);
  float linAccY = AccY + sin(rollRad);

  accelBiasX = ACCEL_BIAS_TC * accelBiasX + (1 - ACCEL_BIAS_TC) * linAccX;
  accelBiasY = ACCEL_BIAS_TC * accelBiasY + (1 - ACCEL_BIAS_TC) * linAccY;
  linAccX -= accelBiasX;
  linAccY -= accelBiasY;

  if (abs(linAccX) < ACCEL_DEADZONE) linAccX = 0;
  if (abs(linAccY) < ACCEL_DEADZONE) linAccY = 0;

  float aX = linAccX * 9.81;
  float aY = linAccY * 9.81;

  float headRad = heading * 0.01745;
  float worldAX = aX * cos(headRad) - aY * sin(headRad);
  float worldAY = aX * sin(headRad) + aY * cos(headRad);

  float decay = (linAccX == 0 && linAccY == 0) ? VEL_DECAY : 1.0;
  velX = (velX + worldAX * dt) * decay;
  velY = (velY + worldAY * dt) * decay;

  posX += velX * dt;
  posY += velY * dt;
}
// stuck check skipped when wanderActive==false
void updateSOS() {
  unsigned long now = millis();

  // ── SOS is active
  if (myData.sosFlag) {
    bool isCalm = (!unstable && !roughTerrain);

    if (isCalm) {
      if (flatTimer == 0) flatTimer = now;
      if ((now - flatTimer) > SOS_CLEAR_TIME_MS) {
        myData.sosFlag = false;
        sosReason      = "";
        flatTimer      = 0;
        stuckTimer     = 0;
        Serial.println("SOS cleared");
      }
    } else {
      flatTimer = 0;  //restarted calm timer
    }
    return;
  }

  if (unstable) {
    myData.sosFlag = true;
    sosReason      = "TILT";
    stuckTimer     = 0;
    flatTimer      = 0;
    return;
  }

  if (roughTerrain && stuck) {
    myData.sosFlag = true;
    sosReason      = "VIBRATION+STUCK";
    stuckTimer     = 0;
    flatTimer      = 0;
    return;
  }
  // only count stuck when wander is active
  if (stuck && wanderActive) {
    if (stuckTimer == 0) stuckTimer = now;
    if ((now - stuckTimer) > STUCK_TIME_MS) {
      myData.sosFlag = true;
      sosReason      = "STUCK_5SEC";
      stuckTimer     = 0;
      flatTimer      = 0;
      return;
    }
  } else {
    stuckTimer = 0;
  }
}

void resetPosition(float knownX, float knownY) {
  posX = knownX;
  posY = knownY;
  velX = 0;
  velY = 0;
  Serial.println("Position reset");
}

void onDataReceiveFull(const esp_now_recv_info_t *info,
                       const uint8_t *data, int len) {
  BotData incoming;
  memcpy(&incoming, data, sizeof(BotData));

  if (incoming.botID < 3) {
    swarm[incoming.botID]    = incoming;
    peerSeen[incoming.botID] = true;
    lastRSSI[incoming.botID] = info->rx_ctrl->rssi;
  }

  if (incoming.sosFlag && !myData.mergeStatus) {
    Serial.print("SOS from bot ");
    Serial.print(incoming.botID);
    Serial.print("  RSSI: ");
    Serial.println(info->rx_ctrl->rssi);
  }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}


// ═══════════════════════════════════════════════════════════════════════════
// finding nearest bot
// ═══════════════════════════════════════════════════════════════════════════

int findNearestSOSBot() {
  int bestRSSI  = -100;
  int nearestID = -1;

  for (int i = 0; i < 3; i++) {
    if (i == myData.botID) continue;
    if (swarm[i].sosFlag && peerSeen[i] && lastRSSI[i] > bestRSSI) {
      bestRSSI  = lastRSSI[i];
      nearestID = i;
    }
  }
  return nearestID;
}
// ═══════════════════════════════════════════════════════════════════════════
//own state broadcasting
// ═══════════════════════════════════════════════════════════════════════════
void broadcastState() {
  esp_err_t result = esp_now_send(
    broadcastAddress,
    (uint8_t *)&myData,
    sizeof(BotData)
  );
  if (result != ESP_OK) {
    Serial.println("Broadcast failed");
  }
}
// ═══════════════════════════════════════════════════════════════════════════
// LÉVY STEP SIZE GENERATOR
// ═══════════════════════════════════════════════════════════════════════════
float levySample() {
  float u = (float)random(1, 10000) / 10000.0;
  return pow(u, -1.0 / LEVY_ALPHA);
}

unsigned long levyStepDuration() {
  float raw        = levySample();
  float normalized = (raw - 1.0) / 49.0;
  normalized       = constrain(normalized, 0.0, 1.0);
  return MIN_MOVE_MS + (unsigned long)(normalized * (MAX_MOVE_MS - MIN_MOVE_MS));
}

void driveForward()  { leftServo.write(FULL_FWD);          rightServo.write(FULL_REV); }
void driveBackward() { leftServo.write(FULL_REV);          rightServo.write(FULL_FWD); }
void turnLeft()      { leftServo.write(STOP + TURN_SPEED); rightServo.write(STOP - TURN_SPEED); }
void turnRight()     { leftServo.write(STOP - TURN_SPEED); rightServo.write(STOP + TURN_SPEED); }
void stopMotors()    { leftServo.write(STOP);              rightServo.write(STOP); }

void applyMovement() {
  switch (currentMove) {
    case MOVE_FORWARD:  driveForward();  break;
    case TURN_LEFT:     turnLeft();      break;
    case TURN_RIGHT:    turnRight();     break;
    case MOVE_BACKWARD: driveBackward(); break;
  }
}
// ═══════════════════════════════════════════════════════════════════════════
// NEXT MOVE PICKER
// ═══════════════════════════════════════════════════════════════════════════
void pickNextMove() {
  int r = random(0, 100);
  if      (r < 55) { currentMove = MOVE_FORWARD;  moveDuration = levyStepDuration(); }
  else if (r < 65) { currentMove = TURN_LEFT;      moveDuration = SHARP_TURN_MS; }
  else if (r < 75) { currentMove = TURN_RIGHT;     moveDuration = SHARP_TURN_MS; }
  else if (r < 85) { currentMove = TURN_LEFT;      moveDuration = SLOW_TURN_MS; }
  else if (r < 95) { currentMove = TURN_RIGHT;     moveDuration = SLOW_TURN_MS; }
  else             { currentMove = MOVE_BACKWARD;  moveDuration = random(MIN_MOVE_MS, MIN_MOVE_MS * 2); }

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

// ═══════════════════════════════════════════════════════════════════════════
// WANDER PAUSE / RESUME
// ═══════════════════════════════════════════════════════════════════════════
void pauseWander() {
  wanderActive = false;
  stopMotors();
  Serial.println("Wander paused — FSM taking control");
}

void resumeWander() {
  wanderActive = true;
  pickNextMove();
  Serial.println("Wander resumed");
}
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(200);
  for (int i = 0; i < 4000; i++) {
    imu_read();
    GyroCalRoll  += RateRoll;
    GyroCalPitch += RatePitch;
    GyroCalYaw   += RateYaw;
    delay(1);
  }
  GyroCalRoll  /= 4000;
  GyroCalPitch /= 4000;
  GyroCalYaw   /= 4000;
  //warm up sensor 
  for (int i = 0; i < 10; i++) { imu_read(); delay(20); }
  prevAccX    = AccX;
  prevAccY    = AccY;
  prevAccZ    = AccZ;
  warmupStart = millis();
  prevTime    = millis();
  Serial.println("IMU ready — calibration done");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("Bot ");
  Serial.print(THIS_BOT_ID);
  Serial.print(" MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed — restarting");
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataReceiveFull);
  esp_now_register_send_cb(onDataSent);

  //add broadcast peer 
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
  }

  //initialise own data struct
  myData.botID          = THIS_BOT_ID;
  myData.posX           = 0;
  myData.posY           = 0;
  myData.velocity       = 0;
  myData.battery        = 100;
  myData.sosFlag        = false;
  myData.mergeStatus    = false;
  myData.pitch          = 0;
  myData.roll           = 0;
  myData.mergePartnerID = 255;
  Serial.println("ESP-NOW mesh ready");
  
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  stopMotors();
  delay(500);
  randomSeed(analogRead(0));
  pickNextMove();
  Serial.println("Lévy walk ready");
}

void loop() {
  imu_read();

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  //complementary filter ─────────────────────────────────────
  AngleRollAcc  = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 57.2958;
  AnglePitchAcc = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 57.2958;
  float tau   = 0.3;
  float alpha = tau / (tau + dt);
  AngleRoll  = alpha * (AngleRoll  + RateRoll  * dt) + (1 - alpha) * AngleRollAcc;
  AnglePitch = alpha * (AnglePitch + RatePitch * dt) + (1 - alpha) * AnglePitchAcc;

  if (!warmedUp) {
    warmedUp     = (millis() - warmupStart) > 1000;
    unstable     = false;
    roughTerrain = false;
    stuck        = false;
  } else {

    if (!unstable) {
      unstable = abs(AngleRoll) > TILT_RAISE || abs(AnglePitch) > TILT_RAISE;
    } else {
      unstable = abs(AngleRoll) > TILT_CLEAR || abs(AnglePitch) > TILT_CLEAR;
    }

    float jerkX = abs(AccX - prevAccX);
    float jerkY = abs(AccY - prevAccY);
    float jerkZ = abs(AccZ - prevAccZ);
    prevAccX = AccX; prevAccY = AccY; prevAccZ = AccZ;

    roughTerrain = (jerkX + jerkY + jerkZ) > JERK_THRESH;

    // stuck: only when wander is active
    static unsigned long lowMotionTimer = 0;
    float motionMag = abs(AccX) + abs(AccY);
    bool  lowMotion = motionMag < MOTION_THRESH;

    if (lowMotion && !unstable && wanderActive) {
      if (lowMotionTimer == 0) lowMotionTimer = millis();
      stuck = (millis() - lowMotionTimer) > 1500;
    } else {
      lowMotionTimer = 0;
      stuck          = false;
    }
  }

  updateHeading(dt);
  updatePosition(dt);
  updateSOS();

  myData.posX     = posX;
  myData.posY     = posY;
  myData.velocity = sqrt(velX * velX + velY * velY);
  myData.pitch    = AnglePitch;
  myData.roll     = AngleRoll;
  // myData.battery = batteryPercent();

  if (millis() - lastBroadcast > 200) {
    broadcastState();
    lastBroadcast = millis();
  }

  if (millis() - lastCheck > DIRECTION_CHECK_MS) {
    lastCheck = millis();

    // pause wander the moment SOS raises
    if (myData.sosFlag && wanderActive) {
      pauseWander();
      Serial.print("SOS active: ");
      Serial.println(sosReason);
    }

    if (!myData.sosFlag && !wanderActive && !myData.mergeStatus && flatTimer == 0) {
      resumeWander();
    }
    // check if any swarm bot needs help
    int nearestSOS = findNearestSOSBot();
    if (nearestSOS != -1 && !myData.mergeStatus && !myData.sosFlag) {
      Serial.print("Nearest bot needing help: ");
      Serial.println(nearestSOS);
    }
  }

  if (wanderActive) {
    if (millis() - moveStart >= moveDuration) {
      pickNextMove();
    }
  }

  for (int i = 0; i < 3; i++) {
    if (!peerSeen[i]) continue;
    Serial.print("Bot");    Serial.print(swarm[i].botID);
    Serial.print(" X:");    Serial.print(swarm[i].posX, 1);
    Serial.print(" Y:");    Serial.print(swarm[i].posY, 1);
    Serial.print(" Bat:");  Serial.print(swarm[i].battery);
    Serial.print("% SOS:"); Serial.print(swarm[i].sosFlag ? "YES" : "no");
    Serial.print(" Mrg:");  Serial.println(swarm[i].mergeStatus ? "YES" : "no");
  }
  Serial.print("Roll: ");   Serial.print(AngleRoll);
  Serial.print(" Pitch: "); Serial.print(AnglePitch);
  Serial.print(" Temp: ");  Serial.print(TempC); Serial.print("C");
  Serial.print(" | X: ");   Serial.print(posX, 2);
  Serial.print(" Y: ");     Serial.print(posY, 2);
  Serial.print(" Hdg: ");   Serial.print(heading, 1);
  Serial.print(" SOS: ");   Serial.print(myData.sosFlag ? "YES" : "no");
  if (myData.sosFlag) { Serial.print(" ("); Serial.print(sosReason); Serial.print(")"); }
  Serial.println();

  delay(500);
}
