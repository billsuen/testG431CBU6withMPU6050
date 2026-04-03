#ifndef __KALMAN_H
#define __KALMAN_H

#include <stdint.h>

typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_t;

void Kalman_Init(Kalman_t *Kalman, float Q_angle, float Q_bias, float R_measure);
float Kalman_Update(Kalman_t *Kalman, float newAngle, float newRate, float dt);

#endif /* __KALMAN_H */
