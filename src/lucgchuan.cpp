#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define I2C_SDA 21 
#define I2C_SCL 22 
#define BUZZER_PIN 19 
#define MPU_ADDR 0x68
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

const float ACCEL_SCALE = 2048.0; 
const float GYRO_SCALE = 131.0;    

float roll = 0, pitch = 0;
float gx = 0, gy = 0, gz = 0;
float maxShockG = 0;
float smoothedShock = 0;
float totalA = 0;
float totalG = 0;

unsigned long lastTime = 0;
unsigned long lastPrintTime = 0; 
unsigned long lastReadTime = 0;
bool isAccident = false; 
unsigned long alarmStartTime = 0;

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      pServer->getAdvertising()->start();
    }
};

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); 

  BLEDevice::init("ESP32_Tracker");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x80);
  Wire.endTransmission(true);
  delay(100);
  
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x6B); 
  Wire.write(0x00); 
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x18);
  Wire.endTransmission(true); 

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true); 

  lastTime = micros();
}

void loop() {
  if (millis() - lastReadTime >= 10) {
    lastReadTime = millis();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true);
    
    if (Wire.available() == 14) {
      int16_t rawAccX = (Wire.read() << 8 | Wire.read());
      int16_t rawAccY = (Wire.read() << 8 | Wire.read());
      int16_t rawAccZ = (Wire.read() << 8 | Wire.read());
      int16_t rawTemp = (Wire.read() << 8 | Wire.read());
      int16_t rawGyroX = (Wire.read() << 8 | Wire.read());
      int16_t rawGyroY = (Wire.read() << 8 | Wire.read());
      int16_t rawGyroZ = (Wire.read() << 8 | Wire.read());

      float ax = rawAccX / ACCEL_SCALE;
      float ay = rawAccY / ACCEL_SCALE;
      float az = rawAccZ / ACCEL_SCALE;
      float gyroX = rawGyroX / GYRO_SCALE;
      float gyroY = rawGyroY / GYRO_SCALE;

      if (gx == 0 && gy == 0 && gz == 0) {
        gx = ax; gy = ay; gz = az;
      }

      float alphaG = 0.92;
      gx = alphaG * gx + (1.0 - alphaG) * ax;
      gy = alphaG * gy + (1.0 - alphaG) * ay;
      gz = alphaG * gz + (1.0 - alphaG) * az;

      totalA = sqrt(ax * ax + ay * ay + az * az);
      totalG = sqrt(gx * gx + gy * gy + gz * gz);

      float lx = ax - gx;
      float ly = ay - gy;
      float lz = az - gz;

      float rawShock = sqrt(lx * lx + ly * ly + lz * lz);
      
      smoothedShock = 0.6 * smoothedShock + 0.4 * rawShock;
      
      if (smoothedShock > maxShockG) {
        maxShockG = smoothedShock;
      }

      unsigned long currentTime = micros();
      float dt = (currentTime - lastTime) / 1000000.0; 
      lastTime = currentTime;

      float accRoll  = atan2(ay, az) * 180.0 / PI;
      float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

      float alphaAngle = 0.96;
      roll  = alphaAngle * (roll + gyroX * dt) + (1.0 - alphaAngle) * accRoll;
      pitch = alphaAngle * (pitch + gyroY * dt) + (1.0 - alphaAngle) * accPitch;

      float displayRoll = abs(roll);
      if (displayRoll > 90.0) displayRoll = 180.0 - displayRoll;
      float displayPitch = abs(pitch);
      if (displayPitch > 90.0) displayPitch = 180.0 - displayPitch;

      if (maxShockG > 2.5 && (displayRoll > 50 || displayPitch > 50)) {
        if (!isAccident) { 
          isAccident = true;
          alarmStartTime = millis();      
          digitalWrite(BUZZER_PIN, HIGH); 
        }
      }
    }
  }

  if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();
    
    Serial.printf("Total_A:%.2fG | Total_G:%.2fG | Shock_G:%.2f | Tilt: Ox:%.1f Oy:%.1f | ", totalA, totalG, maxShockG, abs(pitch), abs(roll));
    
    String bleData = "";
    if (isAccident) {
      Serial.println("ALERT!");
      bleData = "ACCIDENT! G:" + String(maxShockG, 2);
    } else {
      Serial.println("OK");
      bleData = "A:" + String(totalA, 2) + "G S:" + String(maxShockG, 1) + "G";
    }

    if (deviceConnected) {
      pTxCharacteristic->setValue(bleData.c_str());
      pTxCharacteristic->notify();
    }
    
    maxShockG = 0;
  }

  if (isAccident && (millis() - alarmStartTime >= 2000)) {
    isAccident = false;             
    digitalWrite(BUZZER_PIN, LOW);  
  }
}