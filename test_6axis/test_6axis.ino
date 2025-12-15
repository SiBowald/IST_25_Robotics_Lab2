#include <Wire.h>
#include "ICM20600.h"
#include "AK09918.h"

ICM20600 icm20600(true);
AK09918 ak09918;

AK09918_err_type_t magErr;
int32_t mx = 0, my = 0, mz = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  icm20600.initialize();

  magErr = ak09918.initialize();
  if (magErr == AK09918_ERR_OK) {
    ak09918.switchMode(AK09918_POWER_DOWN);
    delay(10);
    ak09918.switchMode(AK09918_CONTINUOUS_100HZ);
    delay(10);
  }
}

void loop() {
  int16_t gx = icm20600.getGyroscopeX();
  int16_t gy = icm20600.getGyroscopeY();
  int16_t gz = icm20600.getGyroscopeZ();

  int16_t ax = icm20600.getAccelerationX();
  int16_t ay = icm20600.getAccelerationY();
  int16_t az = icm20600.getAccelerationZ();

  if (magErr == AK09918_ERR_OK) {
    if (ak09918.isDataReady() == AK09918_ERR_OK) {
      (void)ak09918.isDataSkip();
      (void)ak09918.getData(&mx, &my, &mz);
    }
  }

  // Serial Plotter: label:value pairs (tabs between signals)
  Serial.print("gx:"); Serial.print(gx); Serial.print('\t');
  Serial.print("gy:"); Serial.print(gy); Serial.print('\t');
  Serial.print("gz:"); Serial.print(gz); Serial.print('\t');

  Serial.print("ax:"); Serial.print(ax); Serial.print('\t');
  Serial.print("ay:"); Serial.print(ay); Serial.print('\t');
  Serial.print("az:"); Serial.print(az); Serial.print('\t');

  Serial.print("mx:"); Serial.print(mx); Serial.print('\t');
  Serial.print("my:"); Serial.print(my); Serial.print('\t');
  Serial.print("mz:"); Serial.print(mz); Serial.print('\t');
  Serial.println("");

  delay(10);
}
