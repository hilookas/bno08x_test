// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>
#include "comm.h"
#include <lwip/def.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 5
#define BNO08X_INT 21

// For SPI mode, we also need a RESET
#define BNO08X_RESET 22

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

SPIClass *vspi = NULL;

void setup(void) {
  comm_init();

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  // if (!bno08x.begin_I2C(0x4B, &Wire)) {
  // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer! 
  vspi = new SPIClass(VSPI);
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT, vspi)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  Serial.println("Reading events");
}

sh2_Gyroscope_t gyroscope; 
sh2_Accelerometer_t linearAcceleration; 
sh2_RotationVector_t gameRotationVector; 

void loop() {
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    
    // SPI与串口会竞争条件，需要delay一端时间，否则有些数据会丢失
    Serial.println("Setting desired reports");

    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000)) { // 50Hz
      Serial.println("Could not enable gyroscope");
    }
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 20000)) {
      Serial.println("Could not enable linear acceleration");
    }
    if (!bno08x.enableReport(SH2_ARVR_STABILIZED_GRV, 20000)) {
      Serial.println("Could not enable game rotation vector");
    }
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  float buf[4] = {};
  uint32_t *buf2 = (uint32_t *)&buf;
  switch (sensorValue.sensorId) {
  case SH2_GYROSCOPE_CALIBRATED:
    buf[0] = sensorValue.un.gyroscope.x;
    buf[1] = sensorValue.un.gyroscope.y;
    buf[2] = sensorValue.un.gyroscope.z;
    for (int i = 0; i < 4; ++i) buf2[i] = htonl(buf2[i]);
    comm_send_blocking(COMM_TYPE_GYRO, (uint8_t *)buf2);
    break;

  case SH2_LINEAR_ACCELERATION:
    buf[0] = sensorValue.un.linearAcceleration.x;
    buf[1] = sensorValue.un.linearAcceleration.y;
    buf[2] = sensorValue.un.linearAcceleration.z;
    for (int i = 0; i < 4; ++i) buf2[i] = htonl(buf2[i]);
    comm_send_blocking(COMM_TYPE_ACC, (uint8_t *)buf2);
    break;

  case SH2_ARVR_STABILIZED_GRV:
    buf[0] = sensorValue.un.arvrStabilizedGRV.real;
    buf[1] = sensorValue.un.arvrStabilizedGRV.i;
    buf[2] = sensorValue.un.arvrStabilizedGRV.j;
    buf[3] = sensorValue.un.arvrStabilizedGRV.k;
    for (int i = 0; i < 4; ++i) buf2[i] = htonl(buf2[i]);
    comm_send_blocking(COMM_TYPE_ROTA, (uint8_t *)buf2);
    break;
  }
}