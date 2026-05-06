//
// Created by lenovo on 2026/4/9.
//
/**
 ******************************************************************************
 * @file    simple_kalman.h
 * @brief   Simplified 1D Kalman Filter for embedded systems
 ******************************************************************************
 */

#ifndef __SIMPLE_KALMAN_H
#define __SIMPLE_KALMAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float xhat;      /* estimated state x(k|k) */
    float xhatminus; /* predicted state x(k|k-1) */
    float z;        /* measurement */
    float P;        /* covariance P(k|k) */
    float Pminus;   /* covariance P(k|k-1) */
    float Q;        /* process noise covariance */
    float R;        /* measurement noise covariance */
    float K;        /* kalman gain */
    float minVariance;
} SimpleKalmanFilter_t;

void SimpleKalman_Init(SimpleKalmanFilter_t *kf, float Q, float R, float initialVariance);
float SimpleKalman_Update(SimpleKalmanFilter_t *kf, float measurement);

typedef struct
{
    float xhat[2];
    float xhatminus[2];
    float P[4];
    float Pminus[4];
    float Q[4];
    float R;
    float K[2];
    float F[4];
    float H[2];
    float minVariance[2];
    uint8_t initialized;
} SimpleKalman2D_t;

void SimpleKalman2D_Init(SimpleKalman2D_t *kf, float Q_pos, float Q_vel, float R, float dt, float initialVariance);
float SimpleKalman2D_Update(SimpleKalman2D_t *kf, float measurement);


#ifdef __cplusplus
}
#endif
#endif
