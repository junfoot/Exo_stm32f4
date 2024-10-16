#ifndef TMOTOR_AK_ACTUATORS_H
#define TMOTOR_AK_ACTUATORS_H

#include <stdint.h>

typedef struct
{
    float positionMin;
    float positionMax;

    float velocityMin;
    float velocityMax;

    float torqueMin;
    float torqueMax;

    float kpMin;
    float kpMax;

    float kdMin;
    float kdMax;
} MotorParameters;

/// AK10-9 V1.1 motor parameters
extern const MotorParameters ak10_9_v1_1;

/// Motor state structure
typedef struct
{
    uint16_t id;
    float position;
    float velocity;
    float torque;
} MotorState;

/// Send CAN data function type definition
typedef uint8_t (*SendCanDataFunction)(uint32_t id, uint8_t dlc, uint8_t *data);

/// AkActuators class definition in C
typedef struct
{
    uint8_t mId;
    MotorParameters mMotorParameters;
    SendCanDataFunction sendCanData;
} AkActuators;

/// Function prototypes
void AkActuators_init(AkActuators *actuator, uint8_t motorId, MotorParameters motorParas, SendCanDataFunction canSendFunc);
void AkActuators_enable(AkActuators *actuator);
void AkActuators_disable(AkActuators *actuator);
void AkActuators_setZeroPosition(AkActuators *actuator);
void AkActuators_move(AkActuators *actuator, float position, float velocity, float torque, float kp, float kd);
MotorState AkActuators_parseMotorState(uint8_t *canData);
uint8_t AkActuators_getId(AkActuators *actuator);

#endif // TMOTOR_AK_ACTUATORS_H
