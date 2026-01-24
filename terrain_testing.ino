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
void imu_read() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);                 
  Wire.endTransmission();
  Wire.requestFrom(0x68,14);
  int16_t AccXLSB=Wire.read()<<8 | Wire.read();
  int16_t AccYLSB=Wire.read()<<8 | Wire.read();
  int16_t AccZLSB=Wire.read()<<8 | Wire.read();
  int16_t TempRaw=Wire.read()<<8 | Wire.read();
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  AccX=AccXLSB/4096.0;
  AccY=AccYLSB/4096.0;
  AccZ=AccZLSB/4096.0;
  TempC=(TempRaw/340.0)+36.53;
  RateRoll=(GyroX/65.5)-GyroCalRoll;
  RatePitch=(GyroY/65.5)-GyroCalPitch;
  RateYaw=(GyroZ/65.5)-GyroCalYaw;
}

void setup() {
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(200);
  for (int i=0;i<2000;i++) {
    imu_read();
    GyroCalRoll+=RateRoll;
    GyroCalPitch+=RatePitch;
    GyroCalYaw+=RateYaw;
    delay(1);
  }
  GyroCalRoll/=2000;
  GyroCalPitch/=2000;
  GyroCalYaw/=2000;
  prevTime=millis();
}
void loop() {
  imu_read();
  unsigned long currentTime=millis();
  float dt=(currentTime-prevTime)/1000.0;
  prevTime=currentTime;
  AngleRollAcc=atan2(AccY,sqrt(AccX*AccX+AccZ*AccZ))*57.2958;
  AnglePitchAcc=atan2(-AccX,sqrt(AccY*AccY+AccZ*AccZ))*57.2958;
  float tau=0.3;
  float alpha=tau/(tau + dt);
  AngleRoll=alpha*(AngleRoll+RateRoll*dt)+(1-alpha)*AngleRollAcc;
  AnglePitch=alpha*(AnglePitch+RatePitch*dt)+(1-alpha)*AnglePitchAcc;
  unstable=abs(AngleRoll)>12 || abs(AnglePitch)>12;
  float vibration=abs(RateRoll)+abs(RatePitch);
  roughTerrain=vibration > 20;
  stuck=abs(AccX)<0.05;
  Serial.print("Roll: ");
  Serial.print(AngleRoll);
  Serial.print(" Pitch: ");
  Serial.print(AnglePitch);
  Serial.print(" Temp: ");
  Serial.print(TempC);
  Serial.print("C");
  Serial.print(" | Unstable: ");
  Serial.print(unstable);
  Serial.print(" Rough: ");
  Serial.print(roughTerrain);
  Serial.print(" Stuck: ");
  Serial.println(stuck);
  delay(100);
}