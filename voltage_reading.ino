// R1 = 200kΩ (TWO 100k resistors in series between Battery + and ADC pin)
// R2 = 100kΩ (ONE 100k resistor between ADC pin and GND)
const int ADC_PIN = 34;  
// Voltage divider ratio: (R1 + R2) / R2
// (200k + 100k) / 100k = 3.0
const float DIVIDER_RATIO = 3.0;
const float ADC_REF = 3.3;
const int ADC_MAX = 4095;
const float BAT_MAX = 8.4;
const float BAT_MIN = 6.4;
const int SAMPLES = 64;
float readBatteryVoltage() {
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(ADC_PIN);
    delay(2);
  }
  float average=sum/(float)SAMPLES;
  float adcVoltage = (average / ADC_MAX) * ADC_REF;
  float batteryVoltage = adcVoltage * DIVIDER_RATIO;
  return batteryVoltage;
}
int batteryPercent(float voltage) {
  // Clamp voltage to known range
  if (voltage >= BAT_MAX) return 100;
  if (voltage <= BAT_MIN) return 0;
  int percent = (int)((voltage - BAT_MIN) / (BAT_MAX - BAT_MIN) * 100);
  return percent;
}
void setup() {
  Serial.begin(115200);
  // ESP32 C3 ADC attenuation — needed to read up to 3.3v safely
  analogSetAttenuation(ADC_11db);
}
void loop() {
  float voltage = readBatteryVoltage();
  int percent = batteryPercent(voltage);
  Serial.print("Battery voltage: ");
  Serial.print(voltage, 2);
  Serial.print("V Charge: ");
  Serial.print(percent);
  Serial.println("%");
  // botData.battery = percent;
  delay(5000);
}
