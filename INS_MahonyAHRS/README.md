~ INS Using Mahony Filter ~

- Operate by Arduino UNO, written by C

1. Convert Raw Value to Readable Value 
2. Calibration 
(Tentative) Conduct LPF to Remove Error 
3. Change to ECS coordinate using Mahony Filter (AHRS) 
4. Get Rotation Matrix from Quaternion 
5. Convert IMU Acceleration to ECS Acceleration 
6. Integrate Acceleration to Velocity 
7. Conduct HPF to Remove Drift
8. Integrate Velocity to Position 
9. Conduct HPF to Remove Drift

