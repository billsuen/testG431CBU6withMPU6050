#include "kalman.h"

/**
 * @brief 初始化 Kalman 濾波器參數
 */
void Kalman_Init(Kalman_t *Kalman, float Q_angle, float Q_bias, float R_measure) {
    Kalman->Q_angle = Q_angle;
    Kalman->Q_bias = Q_bias;
    Kalman->R_measure = R_measure;

    Kalman->angle = 0.0f;
    Kalman->bias = 0.0f;

    Kalman->P[0][0] = 0.0f;
    Kalman->P[0][1] = 0.0f;
    Kalman->P[1][0] = 0.0f;
    Kalman->P[1][1] = 0.0f;
}

/**
 * @brief Kalman 濾波更新演算法 (1D)
 */
float Kalman_Update(Kalman_t *Kalman, float newAngle, float newRate, float dt) {
    // 1. 預測 (Predict)
    float rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    // 2. 更新 (Update)
    float y = newAngle - Kalman->angle; // 測量殘差
    float S = Kalman->P[0][0] + Kalman->R_measure; // 殘差協方差
    
    float K[2]; // 卡爾曼增益
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}
