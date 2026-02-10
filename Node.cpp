#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <NimBLEDevice.h>

#ifndef NODE_ID
  #define NODE_ID 1
#endif

static const uint8_t MPU_ADDR = 0x68;

// Must match receiver exactly
static const char* SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab";
static const char* ANGLES_UUID  = "12345678-1234-1234-1234-1234567890ac";

static const uint8_t PACKET_VERSION = 1;

// MPU registers (works for many MPU6050 like variants)
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_WHO_AM_I     = 0x75;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

#pragma pack(push, 1)
struct AnglesPacket {
  uint8_t  version;
  uint8_t  node_id;
  uint32_t t_ms;
  float    roll_deg;
  float    pitch_deg;
  uint16_t crc16;
};
#pragma pack(pop)

static NimBLEServer* bleServer = nullptr;
static NimBLECharacteristic* anglesChar = nullptr;

static float roll_deg = 0.0f;
static float pitch_deg = 0.0f;

static float gyro_bias_x_dps = 0.0f;
static float gyro_bias_y_dps = 0.0f;

static uint32_t last_us = 0;

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

static bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* out, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
  if (got != len) return false;

  for (size_t i = 0; i < len; i++) out[i] = Wire.read();
  return true;
}

static bool imuInit() {
  Wire.begin();
  Wire.setClock(100000);
  delay(50);

  uint8_t who = 0;
  if (!i2cReadBytes(MPU_ADDR, REG_WHO_AM_I, &who, 1)) {
    Serial.println("WHO_AM_I read failed");
    return false;
  }

  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);

  // Reset then wake
  if (!i2cWriteByte(MPU_ADDR, REG_PWR_MGMT_1, 0x80)) {
    Serial.println("PWR_MGMT_1 reset write failed");
    return false;
  }
  delay(100);

  if (!i2cWriteByte(MPU_ADDR, REG_PWR_MGMT_1, 0x00)) {
    Serial.println("PWR_MGMT_1 wake write failed");
    return false;
  }
  delay(100);

  // Conservative settings
  i2cWriteByte(MPU_ADDR, REG_CONFIG, 0x03);
  i2cWriteByte(MPU_ADDR, REG_GYRO_CONFIG, 0x00);   // +-250 dps
  i2cWriteByte(MPU_ADDR, REG_ACCEL_CONFIG, 0x00);  // +-2 g

  return true;
}

static bool imuReadRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t buf[14];
  if (!i2cReadBytes(MPU_ADDR, REG_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
  return true;
}

static void calibrateGyroBias() {
  Serial.println("Calibrating gyro bias, keep still");
  const int N = 200;
  int64_t sum_gx = 0;
  int64_t sum_gy = 0;

  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (imuReadRaw(ax, ay, az, gx, gy, gz)) {
      sum_gx += gx;
      sum_gy += gy;
    }
    delay(5);
  }

  // +-250 dps sensitivity
  gyro_bias_x_dps = ((float)sum_gx / (float)N) / 131.0f;
  gyro_bias_y_dps = ((float)sum_gy / (float)N) / 131.0f;

  Serial.print("gyro_bias_x_dps=");
  Serial.println(gyro_bias_x_dps, 6);
  Serial.print("gyro_bias_y_dps=");
  Serial.println(gyro_bias_y_dps, 6);
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s) override {
    (void)s;
    Serial.println("BLE client connected");
  }
  void onDisconnect(NimBLEServer* s) override {
    (void)s;
    Serial.println("BLE client disconnected");
    NimBLEDevice::startAdvertising();
  }
};

static void bleInit() {
  char name[32];
  snprintf(name, sizeof(name), "LEECH_NODE_%d", (int)NODE_ID);

  NimBLEDevice::init(name);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  NimBLEService* svc = bleServer->createService(SERVICE_UUID);

  anglesChar = svc->createCharacteristic(
    ANGLES_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  AnglesPacket pkt{};
  pkt.version = PACKET_VERSION;
  pkt.node_id = (uint8_t)NODE_ID;
  pkt.t_ms = millis();
  pkt.roll_deg = 0.0f;
  pkt.pitch_deg = 0.0f;
  pkt.crc16 = crc16_ccitt((const uint8_t*)&pkt, sizeof(pkt) - 2);

  anglesChar->setValue((uint8_t*)&pkt, sizeof(pkt));
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("BLE advertising started");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("BOOT OK");

  if (!imuInit()) {
    Serial.println("IMU init failed, stopping");
    while (true) delay(1000);
  }

  calibrateGyroBias();

  // Initialise angles from accel
  {
    int16_t ax, ay, az, gx, gy, gz;
    if (imuReadRaw(ax, ay, az, gx, gy, gz)) {
      float axg = (float)ax / 16384.0f;
      float ayg = (float)ay / 16384.0f;
      float azg = (float)az / 16384.0f;

      roll_deg  = atan2f(ayg, azg) * (180.0f / (float)M_PI);
      pitch_deg = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / (float)M_PI);
    }
  }

  last_us = micros();
  bleInit();
}

void loop() {
  uint32_t now_us = micros();
  float dt = (now_us - last_us) / 1000000.0f;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;
  last_us = now_us;

  int16_t axr, ayr, azr, gxr, gyr, gzr;
  if (!imuReadRaw(axr, ayr, azr, gxr, gyr, gzr)) {
    static uint32_t lastFail = 0;
    if (millis() - lastFail > 1000) {
      Serial.println("IMU read failed");
      lastFail = millis();
    }
    delay(10);
    return;
  }

  float axg = (float)axr / 16384.0f;
  float ayg = (float)ayr / 16384.0f;
  float azg = (float)azr / 16384.0f;

  float gx_dps = ((float)gxr / 131.0f) - gyro_bias_x_dps;
  float gy_dps = ((float)gyr / 131.0f) - gyro_bias_y_dps;

  float roll_acc  = atan2f(ayg, azg) * (180.0f / (float)M_PI);
  float pitch_acc = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * (180.0f / (float)M_PI);

  const float alpha = 0.98f;
  roll_deg  = alpha * (roll_deg  + gx_dps * dt) + (1.0f - alpha) * roll_acc;
  pitch_deg = alpha * (pitch_deg + gy_dps * dt) + (1.0f - alpha) * pitch_acc;

  AnglesPacket pkt{};
  pkt.version = PACKET_VERSION;
  pkt.node_id = (uint8_t)NODE_ID;
  pkt.t_ms = millis();
  pkt.roll_deg = roll_deg;
  pkt.pitch_deg = pitch_deg;
  pkt.crc16 = crc16_ccitt((const uint8_t*)&pkt, sizeof(pkt) - 2);

  anglesChar->setValue((uint8_t*)&pkt, sizeof(pkt));
  anglesChar->notify();

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    Serial.print("node ");
    Serial.print((int)NODE_ID);
    Serial.print(" roll=");
    Serial.print(roll_deg, 2);
    Serial.print(" pitch=");
    Serial.println(pitch_deg, 2);
    lastPrint = millis();
  }

  delay(20);
}