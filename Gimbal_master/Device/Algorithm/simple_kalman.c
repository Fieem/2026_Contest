//
// Created by lenovo on 2026/4/9.
//
/**
 ******************************************************************************
 * @file    simple_kalman.c
 * @brief   Simplified Kalman Filter implementations for embedded systems
 ******************************************************************************
 */

#include "simple_kalman.h"
#include <string.h>
#include <math.h>

void SimpleKalman_Init(SimpleKalmanFilter_t *kf, float Q, float R, float initialVariance)
{
    memset(kf, 0, sizeof(SimpleKalmanFilter_t));
    kf->Q = Q;
    kf->R = R;
    kf->P = initialVariance;
    kf->Pminus = initialVariance;
    kf->minVariance = 1e-6f;
}

float SimpleKalman_Update(SimpleKalmanFilter_t *kf, float measurement)
{
    kf->z = measurement;

    kf->xhatminus = kf->xhat;
    kf->Pminus = kf->P + kf->Q;

    kf->K = kf->Pminus / (kf->Pminus + kf->R);

    kf->xhat = kf->xhatminus + kf->K * (kf->z - kf->xhatminus);

    kf->P = (1.0f - kf->K) * kf->Pminus;

    if (kf->P < kf->minVariance)
        kf->P = kf->minVariance;

    return kf->xhat;
}

void SimpleKalman2D_Init(SimpleKalman2D_t *kf, float Q_pos, float Q_vel, float R, float dt, float initialVariance)
{
    memset(kf, 0, sizeof(SimpleKalman2D_t));

    kf->F[0] = 1.0f;
    kf->F[1] = dt;
    kf->F[2] = 0.0f;
    kf->F[3] = 1.0f;

    kf->Q[0] = Q_pos;
    kf->Q[1] = 0.0f;
    kf->Q[2] = 0.0f;
    kf->Q[3] = Q_vel;

    kf->R = R;

    kf->P[0] = initialVariance;
    kf->P[1] = 0.0f;
    kf->P[2] = 0.0f;
    kf->P[3] = initialVariance;

    kf->Pminus[0] = initialVariance;
    kf->Pminus[1] = 0.0f;
    kf->Pminus[2] = 0.0f;
    kf->Pminus[3] = initialVariance;

    kf->H[0] = 1.0f;
    kf->H[1] = 0.0f;

    kf->minVariance[0] = 0.001f;
    kf->minVariance[1] = 0.001f;

    kf->initialized = 1;
}

float SimpleKalman2D_Update(SimpleKalman2D_t *kf, float measurement)
{
    if (!kf->initialized)
        return measurement;

    float dt = kf->F[1];   // 从状态转移矩阵读取 dt

    float P11 = kf->Pminus[0];
    float P12 = kf->Pminus[1];
    float P21 = kf->Pminus[2];
    float P22 = kf->Pminus[3];

    float FP0 = kf->F[0] * P11 + kf->F[2] * P21;
    float FP1 = kf->F[1] * P11 + kf->F[3] * P21;
    float FP2 = kf->F[0] * P12 + kf->F[2] * P22;
    float FP3 = kf->F[1] * P12 + kf->F[3] * P22;

    kf->Pminus[0] = FP0 * kf->F[0] + FP2 * kf->F[1] + kf->Q[0];
    kf->Pminus[1] = FP0 * kf->F[2] + FP2 * kf->F[3] + kf->Q[1];
    kf->Pminus[2] = FP1 * kf->F[0] + FP3 * kf->F[1] + kf->Q[2];
    kf->Pminus[3] = FP1 * kf->F[2] + FP3 * kf->F[3] + kf->Q[3];

    float HPHR = kf->Pminus[0] * kf->H[0] * kf->H[0] + kf->R;

    kf->K[0] = (kf->Pminus[0] * kf->H[0] + kf->Pminus[1] * kf->H[1]) / HPHR;
    kf->K[1] = (kf->Pminus[2] * kf->H[0] + kf->Pminus[3] * kf->H[1]) / HPHR;

    /* 预测状态 */
    kf->xhatminus[0] = kf->xhat[0] + dt * kf->xhat[1];
    kf->xhatminus[1] = kf->xhat[1];

    float innovation = measurement - kf->xhatminus[0];

    /* 更新状态 */
    kf->xhat[0] = kf->xhatminus[0] + kf->K[0] * innovation;
    kf->xhat[1] = kf->xhatminus[1] + kf->K[1] * innovation;

    /* 更新协方差 */
    float IKH0 = 1.0f - kf->K[0] * kf->H[0];
    float IKH1 = -kf->K[0] * kf->H[1];
    float IKH2 = -kf->K[1] * kf->H[0];
    float IKH3 = 1.0f - kf->K[1] * kf->H[1];

    float newP0 = IKH0 * kf->Pminus[0] + IKH2 * kf->Pminus[2];
    float newP1 = IKH0 * kf->Pminus[1] + IKH2 * kf->Pminus[3];
    float newP2 = IKH1 * kf->Pminus[0] + IKH3 * kf->Pminus[2];
    float newP3 = IKH1 * kf->Pminus[1] + IKH3 * kf->Pminus[3];

    kf->P[0] = newP0;
    kf->P[1] = newP1;
    kf->P[2] = newP2;
    kf->P[3] = newP3;

    if (kf->P[0] < kf->minVariance[0])
        kf->P[0] = kf->minVariance[0];
    if (kf->P[3] < kf->minVariance[1])
        kf->P[3] = kf->minVariance[1];

    return kf->xhat[0];
}
