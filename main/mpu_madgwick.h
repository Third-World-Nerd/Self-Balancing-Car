#include "MPU9250.h"
#include "MadgwickAHRS.h"



#define GRAVITY 9.802 // The gravity acceleration in New York City

// Calibration outcomes
#define GYRO_X_OFFSET -0.000473712
#define GYRO_Y_OFFSET -0.0000877666
#define GYRO_Z_OFFSET 0.000108979

#define ACCEL_X_OFFSET -1.616102277
#define ACCEL_Y_OFFSET -8.118357627
#define ACCEL_Z_OFFSET -9.279277647


const float magn_ellipsoid_center[3] = {-24.2706, 38.5258, 30.0903};
const float magn_ellipsoid_transform[3][3] = {{0.931626, -0.0172975, 0.0518391}, {-0.0172975, 0.787635, 0.00795392}, {0.0518391, 0.00795392, 0.960569}};

// Set structs for converting result from Quaternion to Euler angles
struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll_e, pitch_e, yaw_e;
};

struct YPR {
  float yaw, pitch, roll;
};

struct Gravity {
  double x,y,z;
};


extern float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
extern float magnetom[3];
extern float gyro[3];


extern Quaternion qua;
//extern EulerAngles eul;
extern YPR ypr;
//extern Gravity gravity;

extern float yaw_linear[2];   // stores linear yaw angle and linear yaw rate


void imu_read();
void compensate_imu_errors();
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3]);
void sendToPC(float* data1, float* data2, float* data3);
EulerAngles ToEulerAngles(Quaternion q);
void toYPR(Quaternion q);
void imu_setup();
void ins_update_madgwick();
void yaw_linearizer(float yaw_j, float yaw_k);