#include "GY_85.h" 
#include <Wire.h>

GY_85 GY85;

// For AHRS 
float samplePeriod = 1.0f / 256.0f;
float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float eInt[3] = {0.0f, 0.0f, 0.0f};
float Kp = 1.0f;
float Ki = 0.0f;                                      // Drift가 있어서 Ki 값 조정해야 할 듯 ################################################
int AccResolution = 10;                               // 전압 해상도 10 bit 
int AccRange = 8;                                     // 중력 범위는 -4g ~ 4g 

int iterate = 100;          
int count = 1;  
int cal_ax = 0; 
int cal_ay = 0;
int cal_az = 0; 

// For Postion Estimation 
float linVel[3] = {0.0f, 0.0f, 0.0f};
float linPos[3] = {0.0f, 0.0f, 0.0f}; 

void setup() {
    Serial.begin(9600); 
    Wire.begin(); 
    GY85.init();
}

void loop() {   

    // Read data from IMU
    int* accelerometerReadings = GY85.readFromAccelerometer();
    int ax = GY85.accelerometer_x(accelerometerReadings);
    int ay = GY85.accelerometer_y(accelerometerReadings);
    int az = GY85.accelerometer_z(accelerometerReadings);
    
    int* compassReadings = GY85.readFromCompass();
    int cx = GY85.compass_x(compassReadings);
    int cy = GY85.compass_y(compassReadings);
    int cz = GY85.compass_z(compassReadings);

    float* gyroReadings = GY85.readGyro();
    float gx = GY85.gyro_x(gyroReadings);
    float gy = GY85.gyro_y(gyroReadings);
    float gz = GY85.gyro_z(gyroReadings);
    float gt = GY85.temp(gyroReadings);

    // Convert to Real Data 
    ax = ax * (AccRange / pow(2, AccResolution)); 
    ay = ay * (AccRange / pow(2, AccResolution)); 
    az = az * (AccRange / pow(2, AccResolution));

    gx = gx / 14.375; 
    gy = gy / 14.375;
    gz = gz / 14.375; 

    cx = cx * 0.92;
    cy = cy * 0.92;
    cz = cz * 0.92;

    // LPF for Accuracy 
    float AccAlpha = 0.1; 
    float GyrAlpha = 0.1; 
    float MagAlpha = 0.1; 

    // ( Tentative)

    // Sensor Calibration
    if(count < iterate){
      cal_ax = ((count - 1) / count) * cal_ax + (1 / count) * ax; 
      cal_ay = ((count - 1) / count) * cal_ay + (1 / count) * ay; 
      cal_az = ((count - 1) / count) * cal_az + (1 / count) * az; 
      count++; 
    }
    else{
      ax = ax - cal_ax;
      ay = ay - cal_ay;
      az = az - cal_az;
    }
    
    // Mahony AHRS 
    mahonyUpdate(gx * (PI / 180), gy * (PI / 180), gz * (PI / 180), ax, ay, az, cx, cy, cz);

    // CheckPoint1 
    Serial.print("Quaternion: ");
    Serial.print(quaternion[0]);
    Serial.print(", ");
    Serial.print(quaternion[1]);    // pitch (normalized)
    Serial.print(", ");
    Serial.print(quaternion[2]);    // roll (normalized)
    Serial.print(", ");
    Serial.println(quaternion[3]);  // yaw (normalized)

    float R[3][3];  // Rotation Matrix (물리량을 IMU 좌표계에서 ECS 좌표계로 변환)

    R[0][0] = 2.0f * quaternion[0] * quaternion[0] - 1.0f + 2.0f * quaternion[1] * quaternion[1];
    R[0][1] = 2.0f * (quaternion[1] * quaternion[2] + quaternion[0] * quaternion[3]);
    R[0][2] = 2.0f * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]);

    R[1][0] = 2.0f * (quaternion[1] * quaternion[2] - quaternion[0] * quaternion[3]);
    R[1][1] = 2.0f * quaternion[0] * quaternion[0] - 1.0f + 2.0f * quaternion[2] * quaternion[2];
    R[1][2] = 2.0f * (quaternion[2] * quaternion[3] + quaternion[0] * quaternion[1]);

    R[2][0] = 2.0f * (quaternion[1] * quaternion[3] + quaternion[0] * quaternion[2]);
    R[2][1] = 2.0f * (quaternion[2] * quaternion[3] - quaternion[0] * quaternion[1]);
    R[2][2] = 2.0f * quaternion[0] * quaternion[0] - 1.0f + 2.0f * quaternion[3] * quaternion[3];

    float linAcc[3] = {0.0f, 0.0f, 0.0f};
    float acc[3];

    acc[0] = ax;
    acc[1] = ay;
    acc[2] = az;

    // Converting Acceleration to ECS Acc 

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        linAcc[i] += R[i][j] * acc[j];
      }
    }

    // CheckPoint2 
    Serial.print("Linear Accleration: ");
    Serial.print(linAcc[0]);                            // x축 
    Serial.print(", ");
    Serial.print(linAcc[1]);                            // y축 
    Serial.print(", ");
    Serial.println(linAcc[2]);                          // z축 

    // Subtracting Gravity 
    linAcc[2] = linAcc[2] - 1; 

    // Calculate LinVel (3축 각각에 대해 1차 적분) 
    for (int i = 0; i < 3; i++) {
      linVel[i] += linAcc[i] * samplePeriod;
    }

    // HPF to remove Drift in Velocity
    float filtCutOffVel = 0.1;                               // HPF Parameter 변경해보기 ######################################## 
    float RCVel = 1.0 / (2.0 * PI * filtCutOffVel);
    float alphaVel = samplePeriod / (RCVel + samplePeriod);   

    float previousOutput[3] = {0.0f, 0.0f, 0.0f};
    float previousInput[3] = {0.0f, 0.0f, 0.0f};
    float linVelHP[3] = {0.0f, 0.0f, 0.0f};

    for(int i = 0; i < 3; i++){
      linVelHP[i] = alphaVel * (previousOutput + linVel[i] - previousInput);
      previousInput[i] = linVel[i];
      previousOutput[i] = linVelHP[i];
    }

    // Calculate LinPos (3축 각각에 대해 1차 적분)
    for (int i = 0; i < 3; i++) {
      linPos[i] += linVelHP[i] * samplePeriod;
    }

    // HPF to remove Drift in Position
    float filtCutOffPos = 0.1;                               // HPF Parameter 변경해보기 ######################################## 
    float RCPos = 1.0 / (2.0 * PI * filtCutOffPos);
    float alphaPos = samplePeriod / (RCPos + samplePeriod);   

    float previousOutputPos[3] = {0.0f, 0.0f, 0.0f};
    float previousInputPos[3] = {0.0f, 0.0f, 0.0f};
    float linPosHP[3] = {0.0f, 0.0f, 0.0f};

    for(int i = 0; i < 3; i++){
      linPosHP[i] = alphaPos * (previousOutputPos + linPos[i] - previousInputPos);
      previousInputPos[i] = linPos[i];
      previousOutputPos[i] = linPosHP[i];
    }

    // Add to Absolute Value of Position 
    float RealPos[3] =  {0.0f, 0.0f, 0.0f};
    
    for(int i = 0; i < 3; i++){
      RealPos[i] = RealPos[i] + linPosHP[i]; 
    } 

    // Final CheckPoint 
    Serial.print("Linear Position: ");
    Serial.print(linPosHP[0]);
    Serial.print(" ");
    Serial.print(linPosHP[1]);
    Serial.print(" ");
    Serial.println(linPosHP[2]);

    delay(100);
}

// Quaternion multiplication
void quaternProd(float *a, float *b, float *result) {
  result[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  result[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  result[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  result[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

// Quaternion conjugate
void quaternConj(float *q, float *result) {
  result[0] = q[0];
  result[1] = -q[1];
  result[2] = -q[2];
  result[3] = -q[3];
}

// Normalize Quaternion
void normalizeQuat(float *q) {
  float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  q[0] /= norm;
  q[1] /= norm;
  q[2] /= norm;
  q[3] /= norm;
}

void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float ANx, ANy, ANz; 
  float AMx, AMy, AMz; 
  float hx, hy, hz, hw; 
  float bx, by, bz, bw; 
  float vx, vy, vz;
  float wx, wy, wz; 
  float ex, ey, ez;
  float qDot[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float NewQ[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float QP[4] = {0.0f, 0.0f, 0.0f, 0.0f}; 

  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) && !((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {

    ANx = ax * 1/sqrt(ax * ax + ay * ay + az * az); 
    ANy = ay * 1/sqrt(ax * ax + ay * ay + az * az); 
    ANz = az * 1/sqrt(ax * ax + ay * ay + az * az);

    AMx = mx * 1/sqrt(mx * mx + my * my + mz * mz);
    AMy = my * 1/sqrt(mx * mx + my * my + mz * mz);
    AMz = mz * 1/sqrt(mx * mx + my * my + mz * mz);

    float QuaternConjx = quaternion[0]; 
    float QuaternConjy = -quaternion[1];
    float QuaternConjz = -quaternion[2]; 
    float QuaternConjw = -quaternion[3]; 

    QP[0] = 0 * QuaternConjx - mx * QuaternConjy - my * QuaternConjz - mz * QuaternConjw;
    QP[1] = 0 * QuaternConjy + mx * QuaternConjx + my * QuaternConjw - mz * QuaternConjz;
    QP[2] = 0 * QuaternConjz - mx * QuaternConjw + my * QuaternConjx + mz * QuaternConjy;
    QP[3] = 0 * QuaternConjw + mx * QuaternConjz - my * QuaternConjy + mz * QuaternConjx;

    hx = quaternion[0] * QP[0] - quaternion[1] * QP[1] - quaternion[2] * QP[3] - quaternion[3] * QuaternConjw;
    hy = quaternion[0] * QP[1] + quaternion[1] * QP[0] + quaternion[2] * QuaternConjw - quaternion[3] * QP[3];
    hz = quaternion[0] * QP[3] - quaternion[1] * QuaternConjw + quaternion[2] * QP[0] + quaternion[3] * QP[1];
    hw = quaternion[0] * QuaternConjw + quaternion[1] * QP[3] - quaternion[2] * QP[1] + quaternion[3] * QP[0]; 

    bx = 0;
    by = sqrt(hy * hy + hz * hz);
    bz = 0;
    bw = hw; 

    vx = 2 * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]); 
    vy = 2 * (quaternion[0] * quaternion[1] - quaternion[2] * quaternion[3]); 
    vz = quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] - quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]; 

    wx = 2 * by * (0.5 - quaternion[2] * quaternion[2] - quaternion[3] * quaternion[3]) + 2 * bw * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]);
    wy = 2 * by * (quaternion[1] * quaternion[2] - quaternion[0] * quaternion[3]) + 2 * bw * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]);
    wz = 2 * by * (quaternion[0] * quaternion[2] + quaternion[1] * quaternion[3]) + 2 * bw * (0.5 - quaternion[1] * quaternion[1] - quaternion[2] * quaternion[2]);

    ex = ANy * vz - ANz * vy + AMy * wz - AMz * wy; 
    ey = - (ANx * vz - ANz * vx) - (AMz * wz - AMz * wx); 
    ez = ANx * vy - ANy * vx + AMz * wy - AMy * wx;

    if(Ki > 0){
      eInt[0] += ex * samplePeriod; 
      eInt[1] += ey * samplePeriod;
      eInt[2] += ez * samplePeriod;
    }
    else{
      eInt[0] = 0.0f;
      eInt[1] = 0.0f;
      eInt[2] = 0.0f; 
    }

    gx += Kp * ex + eInt[0] * Ki; 
    gy += Kp * ey + eInt[1] * Ki;
    gz += Kp * ez + eInt[2] * Ki;

    QP[0] = quaternion[0] * 0 - quaternion[1] * gx - quaternion[2] * gy - quaternion[3] * gz;
    QP[1] = quaternion[0] * gx + quaternion[1] * 0 + quaternion[2] * gz - quaternion[3] * gy;
    QP[2] = quaternion[0] * gy - quaternion[1] * gz + quaternion[2] * 0 + quaternion[3] * gx;
    QP[3] = quaternion[0] * gz + quaternion[1] * gy - quaternion[2] * gx + quaternion[3] * 0;

    for(int i = 0; i <= 3; i++){
      qDot[i] = 0.5 * QP[i];
      NewQ[i] = quaternion[i] + qDot[i] * samplePeriod;
    }

    for(int i = 0; i <= 3; i++){ 
      quaternion[i] = NewQ[i] / sqrt(NewQ[0] * NewQ[0] + NewQ[1] * NewQ[1] + NewQ[2] * NewQ[2]); 
    }
  }
}

void mahonyUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float ANx, ANy, ANz; 
  float vx, vy, vz;
  float ex, ey, ez;
  float qDot[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float NewQ[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float QP[4] = {0.0f, 0.0f, 0.0f, 0.0f}; 

  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    ANx = ax * 1/sqrt(ax * ax + ay * ay + az * az); 
    ANy = ay * 1/sqrt(ax * ax + ay * ay + az * az); 
    ANz = az * 1/sqrt(ax * ax + ay * ay + az * az);

    vx = 2 * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]); 
    vy = 2 * (quaternion[0] * quaternion[1] - quaternion[2] * quaternion[3]); 
    vz = quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] - quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]; 

    ex = ANy * vz - ANz * vy; 
    ey = - (ANx * vz - ANz * vx); 
    ez = ANx * vy - ANy * vx;

    if(Ki > 0){
      eInt[0] += ex * samplePeriod; 
      eInt[1] += ey * samplePeriod;
      eInt[2] += ez * samplePeriod;
    }
    else{
      eInt[0] = 0.0f;
      eInt[1] = 0.0f;
      eInt[2] = 0.0f; 
    }
    
    gx += Kp * ex + eInt[0] * Ki;
    gy += Kp * ey + eInt[1] * Ki;
    gz += Kp * ez + eInt[2] * Ki;

    QP[0] = quaternion[0] * 0 - quaternion[1] * gx - quaternion[2] * gy - quaternion[3] * gz;
    QP[1] = quaternion[0] * gx + quaternion[1] * 0 + quaternion[2] * gz - quaternion[3] * gy;
    QP[2] = quaternion[0] * gy - quaternion[1] * gz + quaternion[2] * 0 + quaternion[3] * gx;
    QP[3] = quaternion[0] * gz + quaternion[1] * gy - quaternion[2] * gx + quaternion[3] * 0;

    for(int i = 0; i <= 3; i++){
      qDot[i] = 0.5 * QP[i];
      NewQ[i] = quaternion[i] + qDot[i] * samplePeriod;
    }

    for(int i = 0; i <= 3; i++){ 
      quaternion[i] = NewQ[i] / sqrt(NewQ[0] * NewQ[0] + NewQ[1] * NewQ[1] + NewQ[2] * NewQ[2]); 
    }
  }
}