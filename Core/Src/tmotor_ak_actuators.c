#include "tmotor_ak_actuators.h"

/// AK10-9 V1.1 motor parameters
const MotorParameters ak10_9_v1_1 = {
    -12.5f, 12.5f,  // Position range
    -50.0f, 50.0f,  // Velocity range
    -65.0f, 65.0f,  // Torque range
    0.0f, 500.0f,   // Kp range
    0.0f, 5.0f      // Kd range
};

/// Convert float to uint32_t for CAN transmission
static int32_t convFloatToUint(float val, float min, float max, uint8_t bits)
{
    float span = max - min;
    float normalized = (val - min) / span;
    return (int32_t)(normalized * ((1 << bits) - 1));
}

/// Convert uint32_t to float for CAN reception
static float convUintToFloat(int32_t val, float min, float max, uint8_t bits)
{
    float span = max - min;
    float normalized = (float)val / ((1 << bits) - 1);
    return normalized * span + min;
}

/// Initialize AkActuators object
void AkActuators_init(AkActuators *actuator, uint8_t motorId, MotorParameters motorParas, SendCanDataFunction canSendFunc)
{
    actuator->mId = motorId;
    actuator->mMotorParameters = motorParas;
    actuator->sendCanData = canSendFunc;
}

/// Enable the motor control
void AkActuators_enable(AkActuators *actuator)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    actuator->sendCanData(actuator->mId, 8, data);
}

/// Disable the motor control
void AkActuators_disable(AkActuators *actuator)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    actuator->sendCanData(actuator->mId, 8, data);
}

/// Set the zero position of the motor
void AkActuators_setZeroPosition(AkActuators *actuator)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    actuator->sendCanData(actuator->mId, 8, data);
}

/// Move the motor with given position, velocity, torque, kp, and kd
void AkActuators_move(AkActuators *actuator, float position, float velocity, float torque, float kp, float kd)
{
    uint8_t data[8];
    uint8_t flag;

    int32_t p = convFloatToUint(position, actuator->mMotorParameters.positionMin, actuator->mMotorParameters.positionMax, 16);
    int32_t v = convFloatToUint(velocity, actuator->mMotorParameters.velocityMin, actuator->mMotorParameters.velocityMax, 12);
    int32_t t = convFloatToUint(torque, actuator->mMotorParameters.torqueMin, actuator->mMotorParameters.torqueMax, 12);
    int32_t kp_uint = convFloatToUint(kp, actuator->mMotorParameters.kpMin, actuator->mMotorParameters.kpMax, 16);
    int32_t kd_uint = convFloatToUint(kd, actuator->mMotorParameters.kdMin, actuator->mMotorParameters.kdMax, 16);

    // Pack the data
    data[0] = (p >> 8) & 0xFF;
    data[1] = p & 0xFF;
    data[2] = (v >> 4) & 0xFF;
    data[3] = ((v & 0xF) << 4) | ((kp_uint >> 12) & 0xF);
    data[4] = (kp_uint >> 4) & 0xFF;
    data[5] = ((kp_uint & 0xF) << 4) | ((kd_uint >> 12) & 0xF);
    data[6] = (kd_uint >> 4) & 0xFF;
    data[7] = (kd_uint & 0xF) << 4;

    flag = actuator->sendCanData(actuator->mId, 8, data);
}

/// Parse CAN data to get motor state
MotorState AkActuators_parseMotorState(uint8_t *canData)
{
    MotorState motorState;
    motorState.id = (canData[0] << 8) | canData[1];
    motorState.position = convUintToFloat((canData[2] << 8) | canData[3], -12.5, 12.5, 16);
    motorState.velocity = convUintToFloat((canData[4] << 4) | (canData[5] >> 4), -50, 50, 12);
    motorState.torque = convUintToFloat(((canData[5] & 0xF) << 8) | canData[6], -65, 65, 12);

    return motorState;
}

/// Get the motor ID
uint8_t AkActuators_getId(AkActuators *actuator)
{
    return actuator->mId;
}
