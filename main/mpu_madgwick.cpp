#include "mpu_madgwick.h"

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float magnetom[3];
float magnetom_tmp[3];
float gyro[3];

float time_now;
float time_former;
float deltat;

Quaternion qua;
//EulerAngles eul;
YPR ypr;
Gravity gravity;


MPU9250 IMU(Wire,0x68);

float yaw_0, yaw_1;
float yaw_linear[2] = {0,0};
bool yaw_linearizer_firstTime = true;

void yaw_linearizer(float yaw_j, float yaw_k) {
    
    yaw_linear[0] += yaw_k-yaw_j;
    yaw_linear[1] = yaw_k - yaw_j;
    

    if(yaw_linear[1] < -360 || yaw_linear[1] > 360) {
        yaw_linear[1] = 999;
    }
    
    else {
        if(yaw_linear[1] > 180) {
            yaw_linear[0] -= 360;
            yaw_linear[1]-= 360;
        }
        
        else if(yaw_linear[1] < -180) {
            yaw_linear[0] += 360;
            yaw_linear[1] += 360;
        }
    }
}

// Read data from MPU9250
void imu_read() {
  IMU.readSensor();
  accel[0] = IMU.getAccelX_mss();
  accel[1] = IMU.getAccelY_mss();
  accel[2] = IMU.getAccelZ_mss();

  magnetom[0] = IMU.getMagX_uT();
  magnetom[1] = IMU.getMagY_uT();
  magnetom[2] = IMU.getMagZ_uT();

  gyro[0] = IMU.getGyroX_rads();
  gyro[1] = IMU.getGyroY_rads();
  gyro[2] = IMU.getGyroZ_rads();
}

// Apply calibration to raw sensor readings
void compensate_imu_errors() {
    // Compensate accelerometer error
    accel[0] = accel[0] - ACCEL_X_OFFSET;
    accel[1] = accel[1] - ACCEL_Y_OFFSET;
    accel[2] = accel[2] - (ACCEL_Z_OFFSET + GRAVITY);

    // Compensate magnetometer error
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);

    // Compensate gyroscope error
    gyro[0] -= GYRO_X_OFFSET;
    gyro[1] -= GYRO_Y_OFFSET;
    gyro[2] -= GYRO_Z_OFFSET;
}




// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
  for(int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

void sendToPC(float* data1, float* data2, float* data3)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 12);
}



EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll_e = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch_e = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch_e = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw_e = atan2(siny_cosp, cosy_cosp);

    return angles;
}

void toYPR(Quaternion q) {
    // yaw: (about Z axis)
    ypr.yaw = atan2(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1);

    gravity.x = 2 * (q.x*q.z - q.w*q.y);
    gravity.y = 2 * (q.w*q.x + q.y*q.z);
    gravity.z = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;

    // pitch: (nose up/down, about Y axis)
    ypr.pitch = atan(gravity.x / sqrt(gravity.y*gravity.y + gravity.z*gravity.z));
    
    // roll: (tilt left/right, about X axis)
    ypr.roll = atan(gravity.y / sqrt(gravity.x*gravity.x + gravity.z*gravity.z));
}


void imu_setup()
{
    IMU.begin();

  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(9);
}

void ins_update_madgwick()
{
      // put your main code here, to run repeatedly:
  imu_read();

  compensate_imu_errors();

  time_now = micros();
  deltat = (float)(time_now - time_former) / 1000000.0f;
  time_former = time_now;
    
  //Serial.println(deltat,5);
    
  MadgwickQuaternionUpdate(accel[0], accel[1], accel[2],
                           gyro[0], gyro[1], gyro[2],
                           magnetom[0], magnetom[1], magnetom[2], deltat);
  qua.w = q[0];
  qua.x = q[1];
  qua.y = q[2];
  qua.z = q[3];
 /* 
  eul = ToEulerAngles(qua);
  eul.yaw_e += 0.8;
  if (eul.yaw_e > PI) {
    eul.yaw_e -= 2 * PI;
  }
*/
    toYPR(qua);
    ypr.yaw = ypr.yaw*180/PI;
    ypr.pitch = ypr.pitch*180/PI;
    ypr.roll = ypr.roll*180/PI;

    yaw_1 = ypr.yaw;
    if(yaw_linearizer_firstTime)
    {
        yaw_0 = yaw_1;
        yaw_linearizer_firstTime = false;
    }
    yaw_linearizer(yaw_0, yaw_1);
    yaw_0 = yaw_1;
}
