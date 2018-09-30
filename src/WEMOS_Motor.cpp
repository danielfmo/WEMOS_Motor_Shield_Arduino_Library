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

#include "WEMOS_Motor.h"

/* ******************************************************************************************************* */
/**
 * Motor()
 * Motor class constructor
 * @param  address - address of the motor shield, MOTORSHIELD_ADDRESS enum
 * @param  motor_number - value for Motor PWM's duty cycle
 */
Motor::Motor(MOTORSHIELD_ADDRESS address, MOTORSHIELD_MOTOR_NUMBER motor_number)
    : _address(address), _motor(motor_number) {}

/**
 * Motor()
 * Motor class constructor
 * @param  motor_number - value for Motor PWM's duty cycle
 * Considers the address to be the value MOTORSHIELD_AD00
 */
Motor::Motor(MOTORSHIELD_MOTOR_NUMBER motor_number)
    : Motor(MOTORSHIELD_AD00, motor_number) {}

/**
 * drive()
 * Sets the disired PWM on motor
 * @param  direction - direction of the motor (Brake, CW, CCW, Stop, Standby, Coast)
 * @param  pwm_velocity - value for Motor PWM's duty cycle
 */
void Motor::drive(MOTORSHIELD_MOTOR_DIRECTION direction, uint16_t pwm_value) {
    _sendCommandSetMotor(direction, pwm_value);
}

/**
 * driveWithDefaultDirection()
 * Sets the disired PWM on motor
 * @param  defaultDirection - default direction when pwm_velocity os 0(zero)
 * @param  pwm_velocity - signed value for Motor's PWM's duty cycle
 *                      Positive for CW, negative for CCW, 0 to defaultDirection
 */
void Motor::driveWithDefaultDirection(MOTORSHIELD_MOTOR_DIRECTION defaultDirection, int32_t pwm_velocity) {
    MOTORSHIELD_MOTOR_DIRECTION direction = defaultDirection;
    if (pwm_velocity > 0) {
        direction = MOTOR_DIRECTION_CW;
    } else if (pwm_velocity < 0) {
        direction = MOTOR_DIRECTION_CCW;
    }
    uint16_t pwm_value = abs(pwm_velocity);
    drive(direction, pwm_value);
}

/**
 * drive()
 * Sets the disired PWM on motor
 * @param  pwm_velocity - signed value for Motor's PWM's duty cycle
 *                      Positive for CW, negative for CCW, 0 to brake
 */
void Motor::drive(int32_t pwm_velocity) { driveWithDefaultDirection(MOTOR_DIRECTION_BRAKE, pwm_velocity); }

/**
 * brake()
 * brakes "short" motor
 */
void Motor::brake() { drive(MOTOR_DIRECTION_BRAKE, 0); }

/**
 * coast()
 * stops "coast" motor
 */
void Motor::coast() { drive(MOTOR_DIRECTION_COAST, 0); }

/**
 * _sendCommandSetMotor()
 *  @param  direction - direction of the motor (Brake, CW, CCW, Stop, Standby, Coast)
 *  @param  pwm_value - PWM pulse for motor
 * Sets the motor's PWM duty cycle / pulse and direction
 * |       0001 |      4 bit |     4 bit |        12 bit |
 * |  set motor |      Motor | Direction |          Step |
 * |       0001 |       0001 |      0001 | 0010 00000000 | -> Set MotorB at step 512
 */
bool Motor::_sendCommandSetMotor(MOTORSHIELD_MOTOR_DIRECTION direction, uint16_t pwm_value) {
    Wire.beginTransmission(_address);
    Wire.write((byte)_motor | (byte)0x10);
    Wire.write((byte)(direction << 4) | ((byte)(pwm_value >> 8) & (byte)0x0F));
    Wire.write((byte)pwm_value);
    if (Wire.endTransmission(true) != 0) {
        return false;
    }
    return true;
}

/* ******************************************************************************************************* */
/**
 * MotorShield()
 * MotorShield class constructor
 * @param  address - address of the motor shield, MOTORSHIELD_ADDRESS enum
 * @param  pwm_resolution - resolution of the PWM's generator, MOTORSHIELD_PWM_RESOLUTION enum
 * @param  pwm_frequency - frequency value of the PWM's generator, check header for details
 * @param  *motorA - Reference of the Motor() class
 * @param  *motorB - Reference of the Motor() class
 */
MotorShield::MotorShield(MOTORSHIELD_ADDRESS address, MOTORSHIELD_PWM_RESOLUTION pwm_resolution, uint16_t pwm_frequency,
                         Motor *motorA, Motor *motorB)
    : _address(address),
      _pwm_resolution(pwm_resolution),
      _pwm_freqency(pwm_frequency),
      motorA(motorA),
      motorB(motorB) {}

/**
 * MotorShield()
 * MotorShield class constructor
 * @param  address - address of the motor shield, MOTORSHIELD_ADDRESS enum
 * @param  pwm_resolution - resolution of the PWM's generator, MOTORSHIELD_PWM_RESOLUTION enum
 * @param  pwm_frequency - frequency value of the PWM's generator, check header for details
 */
MotorShield::MotorShield(MOTORSHIELD_ADDRESS address, MOTORSHIELD_PWM_RESOLUTION pwm_resolution, uint16_t pwm_frequency)
    : MotorShield(address, pwm_resolution, pwm_frequency, new Motor(address, MOTORSHIELD_MOTORA),
                  new Motor(address, MOTORSHIELD_MOTORB)) {}

/**
 * MotorShield()
 * MotorShield class constructor
 * @param  address - address of the motor shield, MOTORSHIELD_ADDRESS enum
 * Uses 9 bit or 512 Steps as default PWM resolution and 15KHz as default PWM frequency
 */
MotorShield::MotorShield(MOTORSHIELD_ADDRESS address)
    : MotorShield(address, MOTORSHIELD_PWM_RES_9BIT, 15000, new Motor(address, MOTORSHIELD_MOTORA),
                  new Motor(address, MOTORSHIELD_MOTORB)) {}

/**
 * MotorShield()
 * MotorShield class constructor
 * Use default shield address MOTORSHIELD_AD00
 * Uses 9 bit or 512 Steps as default PWM resolution and 15KHz as default PWM frequency
 */
MotorShield::MotorShield()
    : MotorShield(MOTORSHIELD_AD00, MOTORSHIELD_PWM_RES_9BIT, 15000, new Motor(MOTORSHIELD_AD00, MOTORSHIELD_MOTORA),
                  new Motor(MOTORSHIELD_AD00, MOTORSHIELD_MOTORB)) {}

/**
 * ~MotorShield()
 * MotorShield class destructor
 */
MotorShield::~MotorShield() {
    if (motorA) {
        delete motorA;
    }
    if (motorB) {
        delete motorB;
    }
}

/**
 * begin()
 * Initializes I2C and configures the motor shield, call this function before doing anything else
 * @return (int) - error code:
 *     - 0 success
 *     - 1 data too long to fit in transmit data16
 *     - 2 received NACK on transmit of address
 *     - 3 received NACK on transmit of data
 *     - 4 other error
 */
bool MotorShield::begin() {
    Wire.begin();
    Wire.setClock(400000);
    if (_sendCommandConfigurePWM() == false) {
        return false;
    }
    brake();
    setStandby();
    return true;
}

/**
 * drive()
 * Sets the disired PWM on both motors
 * @param  pwm_motorA - PWM pulse for motor A
 * @param  pwm_motorB - PWM pulse for motor B
 */
void MotorShield::drive(int32_t pwm_motorA, int32_t pwm_motorB) {
    if (motorA) {
        motorA->drive(pwm_motorA);
    }
    if (motorB) {
        motorB->drive(pwm_motorB);
    }
}

/**
 * drive()
 * Sets the disired PWM on both motors
 * @param  pwm_velocity - PWM pulse for motor A and B
 */
void MotorShield::drive(int32_t pwm_velocity) { drive(pwm_velocity, pwm_velocity); }

/**
 * brake()
 * brakes "short" both motors
 */
void MotorShield::brake() {
    if (motorA) {
        motorA->brake();
    }
    if (motorB) {
        motorB->brake();
    }
}

/**
 * coast()
 * stops "coast" both motors
 */
void MotorShield::coast() {
    if (motorA) {
        motorA->coast();
    }
    if (motorB) {
        motorB->coast();
    }
}

/**
 * setStandby()
 * sets the TB6612FNG on standby mode
 */
void MotorShield::setStandby() {
    if (motorA) {
        motorA->drive(MOTOR_DIRECTION_STANDBY, 0);
    }
}

/**
 * _sendCommandConfigurePWM()
 * Sets the MotorShield's PWM resolution and frequency
 * |  4 bit CMD |      4 bit |                    16 bit |
 * | config pwm | Resolution |             PWM Frequency |
 * |       0000 |       1010 |         00010011 10001000 | -> Set 10 bit resolution = 1024 steps and 5KHz Frequency
 */
bool MotorShield::_sendCommandConfigurePWM() {
    Wire.beginTransmission(_address);
    Wire.write(((byte)(_pwm_resolution)) & (byte)0x0F);
    Wire.write((byte)(_pwm_freqency >> 8));
    Wire.write((byte)_pwm_freqency);
    if (Wire.endTransmission(true) != 0) {
        return false;
    }
    return true;
}
/* ******************************************************************************************************* */
