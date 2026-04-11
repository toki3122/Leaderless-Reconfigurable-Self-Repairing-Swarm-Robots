#include <Wire.h>

float AccX,AccY,AccZ;
float RateRoll,RatePitch,RateYaw;
float AngleRoll=0;
float AnglePitch=0;
float AngleRollAcc,AnglePitchAcc;
float GyroCalRoll=0;
float GyroCalPitch=0;
float GyroCalYaw=0;
float TempC;
unsigned long prevTime=0;

bool unstable=false;
bool roughTerrain=false;
bool stuck=false;

bool warmedUp=false;
unsigned long warmupStart=0;

float prevAccX=0,prevAccY=0,prevAccZ = 0;

float posX=0,posY=0;
float velX=0,velY=0;
float heading=0;

const float VEL_DECAY=0.98;
const float ACCEL_DEADZONE=0.1;
const float ACCEL_BIAS_TC=0.995;

float accelBiasX=0,accelBiasY=0;

bool sosFlag=false;
String sosReason="";
unsigned long stuckTimer = 0;
const unsigned long STUCK_TIME_MS=3000;

void imu_read() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,14);

  int16_t AccXLSB=Wire.read() << 8 | Wire.read();
  int16_t AccYLSB=Wire.read() << 8 | Wire.read();
  int16_t AccZLSB=Wire.read() << 8 | Wire.read();
  int16_t TempRaw=Wire.read() << 8 | Wire.read();
  int16_t GyroX=Wire.read() << 8 | Wire.read();
  int16_t GyroY=Wire.read() << 8 | Wire.read();
  int16_t GyroZ=Wire.read() << 8 | Wire.read();

  AccX=AccXLSB/4096.0;
  AccY=AccYLSB/4096.0;
  AccZ=AccZLSB/4096.0;
  TempC=(TempRaw/340.0)+36.53;

  RateRoll=(GyroX/65.5)-GyroCalRoll;
  RatePitch=(GyroY/65.5)-GyroCalPitch;
  RateYaw=(GyroZ/65.5)-GyroCalYaw;
}

void updateHeading(float dt) {
  heading+=RateYaw*dt;
  if (heading>360) heading -= 360;
  if (heading<0) heading += 360;
}
// ═══════════════════════════════════════════════════════
// DEAD RECKONING POSITION
// ═══════════════════════════════════════════════════════
void updatePosition(float dt) {
  if (dt<=0 || dt>0.5) return;

  float pitchRad=AnglePitch*0.01745;
  float rollRad  = AngleRoll*0.01745;

  float linAccX =AccX-sin(pitchRad);
  float linAccY =AccY+sin(rollRad);

  // high-pass filter — remove learned DC bias
  accelBiasX=ACCEL_BIAS_TC*accelBiasX+(1-ACCEL_BIAS_TC)*linAccX;
  accelBiasY=ACCEL_BIAS_TC*accelBiasY+(1-ACCEL_BIAS_TC)*linAccY;
  linAccX-=accelBiasX;
  linAccY-=accelBiasY;

  if (abs(linAccX) < ACCEL_DEADZONE) linAccX=0;
  if (abs(linAccY) < ACCEL_DEADZONE) linAccY=0;
  float aX=linAccX*9.81;
  float aY=linAccY*9.81;

  float headRad=heading*0.01745;
  float worldAX=aX*cos(headRad)-aY*sin(headRad);
  float worldAY=aX*sin(headRad)+aY*cos(headRad);

  // only decay velocity when coasting
  float decay=(linAccX==0 && linAccY==0)?VEL_DECAY:1.0;
  velX=(velX+worldAX*dt)*decay;
  velY=(velY+worldAY*dt)*decay;

  posX+=velX*dt;
  posY+=velY*dt;
}

// ═══════════════════════════════════════════════════════
// SOS DETECTION
// ═══════════════════════════════════════════════════════

void updateSOS() {
  unsigned long now=millis();

  if (unstable) {
    sosFlag=true;
    sosReason="TILT";
    return;
  }

  if (roughTerrain && stuck) {
    sosFlag=true;
    sosReason="VIBRATION+STUCK";
    return;
  }

  if (stuck) {
    if (stuckTimer==0) stuckTimer = now;
    if ((now-stuckTimer)>STUCK_TIME_MS) {
      sosFlag=true;
      sosReason="STUCK_3SEC";
      return;
    }
  } else {
    stuckTimer=0;
  }

  // auto-clear SOS when calm and flat for 2 seconds
  if (sosFlag) {
    static unsigned long flatTimer=0;
    bool isCalm=(!unstable && !roughTerrain && !stuck);
    if (isCalm) {
      if (flatTimer==0) flatTimer=now;
      if ((now-flatTimer)>2000) {
        sosFlag=false;
        sosReason="";
        flatTimer=0;
        Serial.println("SOS cleared");
      }
    } else {
      flatTimer=0;
    }
  }
}

// ═══════════════════════════════════════════════════════
// POSITION RESET — call when returning to known point
// ═══════════════════════════════════════════════════════

void resetPosition(float knownX, float knownY) {
  posX=knownX;
  posY=knownY;
  velX=0;
  velY=0;
  Serial.println("Position reset");
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

  for (int i=0;i<4000;i++) {
    imu_read();
    GyroCalRoll+=RateRoll;
    GyroCalPitch+=RatePitch;
    GyroCalYaw+=RateYaw;
    delay(1);
  }
  GyroCalRoll/=4000;
  GyroCalPitch/=4000;
  GyroCalYaw/=4000;
  // warm up sensor 
  for (int i=0;i<10;i++) {
    imu_read();
    delay(20);
  }
  prevAccX=AccX;
  prevAccY=AccY;
  prevAccZ=AccZ;
  warmupStart=millis();
  prevTime=millis();
  Serial.println("IMU ready — calibration done");
}

void loop() {
  imu_read();

  unsigned long currentTime=millis();
  float dt=(currentTime-prevTime)/1000.0;
  prevTime=currentTime;

  // ── complementary filter — roll and pitch ─────────
  AngleRollAcc=atan2(AccY,sqrt(AccX*AccX+AccZ*AccZ))*57.2958;
  AnglePitchAcc=atan2(-AccX,sqrt(AccY*AccY+AccZ*AccZ))*57.2958;
  float tau=0.3;
  float alpha=tau/(tau+dt);
  AngleRoll=alpha*(AngleRoll+RateRoll*dt)+(1-alpha)*AngleRollAcc;
  AnglePitch=alpha*(AnglePitch+RatePitch*dt)+(1-alpha)*AnglePitchAcc;

  // ── status flags ──────────────────────────────────
  if (!warmedUp) {
    // hold all flags false until sensor settles
    warmedUp=(millis()-warmupStart)>1000;
    unstable=false;
    roughTerrain=false;
    stuck=false;

  } else {
    unstable=abs(AngleRoll)>12 || abs(AnglePitch)>12;
    // ROUGH TERRAIN
    float jerkX=abs(AccX-prevAccX);
    float jerkY=abs(AccY-prevAccY);
    float jerkZ=abs(AccZ-prevAccZ);
    prevAccX=AccX;
    prevAccY=AccY;
    prevAccZ=AccZ;
    float jerk=jerkX+jerkY+jerkZ;
    roughTerrain=jerk>0.095;
    // STUCK 
    static unsigned long lowMotionTimer=0;
    float motionMag=abs(AccX)+abs(AccY);
    bool  lowMotion=motionMag<0.095;

    if (lowMotion && !unstable) {
      if (lowMotionTimer==0) lowMotionTimer=millis();
      stuck=(millis()-lowMotionTimer)>1500;
    } else {
      lowMotionTimer=0;
      stuck=false;
    }
  }

  updateHeading(dt);
  updatePosition(dt);
  updateSOS();

  Serial.print("Roll: ");        Serial.print(AngleRoll);
  Serial.print(" Pitch: ");      Serial.print(AnglePitch);
  Serial.print(" Temp: ");       Serial.print(TempC);
  Serial.print("C");
  Serial.print(" | Unstable: "); Serial.print(unstable);
  Serial.print(" Rough: ");      Serial.print(roughTerrain);
  Serial.print(" Stuck: ");      Serial.print(stuck);
  Serial.print(" | X: ");        Serial.print(posX, 2);
  Serial.print(" Y: ");          Serial.print(posY, 2);
  Serial.print(" Hdg: ");        Serial.print(heading, 1);
  Serial.print(" SOS: ");        Serial.print(sosFlag ? "YES" : "no");
  if (sosFlag) {
    Serial.print(" (");
    Serial.print(sosReason);
    Serial.print(")");
  }
  Serial.println();
  delay(100);
}
