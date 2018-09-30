/* ******************************************************************************************************* */
/**
 * This is an Arduino library for WEMOS Motor Shield
 * (https://wiki.wemos.cc/products:d1_mini_shields:motor_shield).
 * written by : danielfmo
 * source code: https://github.com/danielfmo/WEMOS_Motor_Shield_Arduino_Library
 *
 * This library does implement a reworked version of the original command messages, thus needing
 * a specific version of the firmware available here: https://github.com/danielfmo/wemos_motor_shield
 *
 * This file is part of the WEMOS_Motor_Shield_Arduino_Library
 * Which is release under The MIT License (MIT)
 * Please see LICENSE.md file for details
 *
 * Motor Shield I2C Address:
 * The shield has a matrix in PCB that can be soldered in order to set the shield Address.
 * AD0 | AD1 | Address | MOTORSHIELD_ADDRESS enum
 *  -  | -   | 0x2D    | MOTORSHIELD_AD00
 *  -  | x   | 0x2E    | MOTORSHIELD_AD01
 *  x  | -   | 0x2F    | MOTORSHIELD_AD10
 *  x  | x   | 0x30    | MOTORSHIELD_AD11
 * Where 'x' means that the jumper is soldered and '-' means that the jumper is kept open.
 *
 * Motor Shield PWM resolution and max frequency:
 * The STM32F030 MCU present on the shield have a limitation on the PWM generation, where the PWM frequency
 * times the PWM steps cannot exceed the CPU frequency (8MHz). This means that there is a maximum possible
 * frequency for a given PWM resolution.
 * PWM steps | PWM max frequency | MOTORSHIELD_PWM_RESOLUTION enum
 *        64 |       65'535 (Hz) | MOTORSHIELD_PWM_RES_64STEP or MOTORSHIELD_PWM_RES_6BIT
 *       128 |       62'500 (Hz) | MOTORSHIELD_PWM_RES_128STEP or MOTORSHIELD_PWM_RES_7BIT
 *       256 |       31'250 (Hz) | MOTORSHIELD_PWM_RES_256STEP or MOTORSHIELD_PWM_RES_8BIT
 *       512 |       15'625 (Hz) | MOTORSHIELD_PWM_RES_512STEP or MOTORSHIELD_PWM_RES_9BIT
 *      1024 |        7'812 (Hz) | MOTORSHIELD_PWM_RES_1024STEP or MOTORSHIELD_PWM_RES_10BIT
 *      2048 |        3'906 (Hz) | MOTORSHIELD_PWM_RES_2048STEP or MOTORSHIELD_PWM_RES_11BIT
 *      4096 |        1'953 (Hz) | MOTORSHIELD_PWM_RES_4096STEP or MOTORSHIELD_PWM_RES_12BIT
 *
 * Motor Shield Motor Direction:
 * The shield accepts 5 'directions' to set the motor.
 *         DIRECTION |     MOTORSHIELD_ADDRESS | Description
 *    Brake or Short |   MOTOR_DIRECTION_BRAKE | Stops the motor by shorting connectors (Active brake)
 * Counter-Clockwork |     MOTOR_DIRECTION_CCW | Turns the motor on Counter-Clockwork direction
 *         Clockwork |      MOTOR_DIRECTION_CW | Turns the motor on Clockwork direction
 *     Coast or Open |   MOTOR_DIRECTION_COAST | Stops the motor by opening the connectors, coast.
 *           Standby | MOTOR_DIRECTION_STANDBY | Sets the TB6612FNG on standby mode, impacts all motors.
 *
 */
/* ******************************************************************************************************* */

#ifndef WEMOS_MOTOR_H_
#define WEMOS_MOTOR_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h"

typedef enum : uint8_t {
    MOTORSHIELD_AD00 = 0x2D,  //
    MOTORSHIELD_AD01 = 0x2E,
    MOTORSHIELD_AD10 = 0x2F,
    MOTORSHIELD_AD11 = 0x30
} MOTORSHIELD_ADDRESS;

typedef enum : uint8_t {
    MOTOR_DIRECTION_BRAKE   = 0,  //
    MOTOR_DIRECTION_CCW     = 1,
    MOTOR_DIRECTION_CW      = 2,
    MOTOR_DIRECTION_STOP    = 3,
    MOTOR_DIRECTION_STANDBY = 4,
    MOTOR_DIRECTION_COAST   = 5
} MOTORSHIELD_MOTOR_DIRECTION;

typedef enum : uint8_t {
    MOTORSHIELD_MOTORA = 0,  //
    MOTORSHIELD_MOTORB = 1
} MOTORSHIELD_MOTOR_NUMBER;

typedef enum : uint8_t {
    MOTORSHIELD_PWM_RES_6BIT     = 6,  //
    MOTORSHIELD_PWM_RES_7BIT     = 7,
    MOTORSHIELD_PWM_RES_8BIT     = 8,
    MOTORSHIELD_PWM_RES_9BIT     = 9,
    MOTORSHIELD_PWM_RES_10BIT    = 10,
    MOTORSHIELD_PWM_RES_11BIT    = 11,
    MOTORSHIELD_PWM_RES_12BIT    = 12,
    MOTORSHIELD_PWM_RES_64STEP   = 6,
    MOTORSHIELD_PWM_RES_128STEP  = 7,
    MOTORSHIELD_PWM_RES_256STEP  = 8,
    MOTORSHIELD_PWM_RES_512STEP  = 9,
    MOTORSHIELD_PWM_RES_1024STEP = 10,
    MOTORSHIELD_PWM_RES_2048STEP = 11,
    MOTORSHIELD_PWM_RES_4096STEP = 12,
} MOTORSHIELD_PWM_RESOLUTION;

/* clang-format off */
#define CMD_SIZE        3
#define CMD_SET_PWM     0
#define CMD_SET_MOTOR   1
/* clang-format on */

class Motor {
 public:
    Motor(MOTORSHIELD_ADDRESS address, MOTORSHIELD_MOTOR_NUMBER motor_number);
    explicit Motor(MOTORSHIELD_MOTOR_NUMBER motor_number);
    void drive(MOTORSHIELD_MOTOR_DIRECTION direction, uint16_t pwm_value);
    void driveWithDefaultDirection(MOTORSHIELD_MOTOR_DIRECTION defaultDirection, int32_t pwm_velocity);
    void drive(int32_t pwm_velocity);
    void brake();
    void coast();

 private:
    uint8_t _address;
    uint8_t _motor;
    bool _sendCommandSetMotor(MOTORSHIELD_MOTOR_DIRECTION direction, uint16_t pwm_value);
};

class MotorShield {
 public:
    MotorShield(MOTORSHIELD_ADDRESS address, MOTORSHIELD_PWM_RESOLUTION pwm_resolution, uint16_t pwm_frequency,
                Motor *motorA, Motor *motorB);
    MotorShield(MOTORSHIELD_ADDRESS address, MOTORSHIELD_PWM_RESOLUTION pwm_resolution, uint16_t pwm_frequency);
    explicit MotorShield(MOTORSHIELD_ADDRESS address);
    MotorShield();
    ~MotorShield();

    Motor &getMotorA() { return *motorA; }
    Motor &getMotorB() { return *motorB; }

    bool begin();
    void drive(int32_t pwm_motorA, int32_t pwm_motorB);
    void drive(int32_t pwm_velocity);
    void brake();
    void coast();
    void setStandby();

 private:
    uint8_t _address;
    uint8_t _pwm_resolution;
    uint16_t _pwm_freqency;
    Motor *motorA, *motorB;
    bool _sendCommandConfigurePWM();
};

#endif  // WEMOS_MOTOR_H_
