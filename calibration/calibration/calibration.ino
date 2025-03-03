#include "mpu_calibrate.h"

void setup() {
  Serial.begin(115200);
  imu_setup();
  

}

void loop() {
  unsigned long Time = millis();
    imu_read();
    //compensate_imu_errors();
     Serial.print((float)millis()/1000,3);
    Serial.print(",");
    Serial.print(accel[0],6);
    Serial.print(',');
    Serial.print(accel[1],6);
    Serial.print(',');
    Serial.print(accel[2],6);
    Serial.print(',');
    Serial.print(gyro[0],6);
    Serial.print(',');
    Serial.print(gyro[1],6);
    Serial.print(',');
    Serial.println(gyro[2],6);
    //Serial.print(',');
    
    /*
    Serial.print((float)millis()/1000,3);
    Serial.print(",");
    Serial.print(magnetom[0],3);
    Serial.print(',');
    Serial.print(magnetom[1],3);
    Serial.print(',');
    Serial.print(magnetom[2],3);
    Serial.println();
*/
    //delay(100);
    while(millis()-Time<10);
}
