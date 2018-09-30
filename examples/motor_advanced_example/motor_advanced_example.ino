/* ******************************************************************************************************* */
/**
 * This is an Arduino example for the WEMOS Motor Shield
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
#include <Arduino.h>
#include "WEMOS_Motor.h"

MotorShield MS1(MOTORSHIELD_AD10, MOTORSHIELD_PWM_RES_9BIT, 15000);
Motor M1 = MS1.getMotorA();
Motor M2 = MS1.getMotorB();

#define MAX_PULSE 512
#define START_PWM 50
#define INCREMENT 1
#define LOOP_DELAY 50

void loopMotorInc(Motor *motor, MOTORSHIELD_MOTOR_DIRECTION direction) {
    for (uint16_t pwm = START_PWM; pwm <= MAX_PULSE; pwm += INCREMENT) {
        motor->drive(direction, pwm);
        Serial.printf("PWM %d\n", pwm);
        delay(LOOP_DELAY);
    }
}

void loopMotorDec(Motor *motor, MOTORSHIELD_MOTOR_DIRECTION direction) {
    for (uint16_t pwm = MAX_PULSE - START_PWM; pwm <= MAX_PULSE; pwm -= INCREMENT) {
        motor->drive(direction, pwm);
        Serial.printf("PWM %d\n", pwm);
        delay(LOOP_DELAY);
    }
}

void testMotor(Motor *motor) {
    Serial.println("Test Full range CW");
    loopMotorInc(motor, MOTOR_DIRECTION_CW);
    delay(2000);
    Serial.println("Test COAST/STOP");
    motor->coast();
    delay(2000);
    Serial.println("Test Full range CCW");
    loopMotorInc(motor, MOTOR_DIRECTION_CCW);
    delay(2000);
    loopMotorDec(motor, MOTOR_DIRECTION_CCW);
    delay(2000);
    Serial.println("Test BRAKE");
    motor->brake();
    delay(2000);
}

void setup() {
    Serial.begin(115200);
    MS1.begin();
}

void loop() {
    testMotor(&M2);
    testMotor(&M1);
}
