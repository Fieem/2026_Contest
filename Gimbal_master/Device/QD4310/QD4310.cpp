#include <algorithm>
#include "QD4310.h"

void QD4310::SendCommand(const Command cmd, const int16_t value) {
    uint8_t TxBuffer[3];

    TxBuffer[0] = static_cast<uint8_t>(cmd);

    TxBuffer[1] = static_cast<uint8_t>(value & 0xFF);
    TxBuffer[2] = static_cast<uint8_t>((value >> 8) & 0xFF);

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 0x400 + id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_3;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxBuffer);
    // while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) != 3);
}

void QD4310::update(const uint8_t feedback[8]) {
    enabled = feedback[0] & 0x01;
    current = *(int16_t *)(feedback + 2) * 10.0f / INT16_MAX;
    speed = *(int16_t *)(feedback + 4) * 1000.0f / INT16_MAX;
    angle = *(uint16_t *)(feedback + 6) * 2 * std::numbers::pi_v<float> / UINT16_MAX;
}

void QD4310::setAngle(const float _angle) {
    std::clamp(_angle, 0.0f, 2 * std::numbers::pi_v<float>); // 限制角度在[0, 2pi]范围内
    SendCommand(Command::ANGLE, _angle / 2 / std::numbers::pi_v<float> * UINT16_MAX);
}

void QD4310::setSpeed(const float _speed) {
    std::clamp(_speed, -1000.0f, 1000.0f); // 限制速度在[-1000, 1000]范围内
    SendCommand(Command::SPEED, _speed / 1000 * INT16_MAX);
}

void QD4310::setLowSpeed(const float _speed) {
    std::clamp(_speed, -1000.0f, 1000.0f); // 限制速度在[-1000, 1000]范围内
    SendCommand(Command::LOW_SPEED, _speed / 1000 * INT16_MAX);
}

void QD4310::setCurrent(const float _current) {
    std::clamp(_current, -10.0f, 10.0f);
    SendCommand(Command::CURRENT, _current / 10 * INT16_MAX);
}
