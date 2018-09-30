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

MotorShield MotorShield();

void setup() {
    Serial.begin(115200);
    while (MotorShield.begin() != true) {
        Serial.println(F("Motor Shield is not reachable"));
        delay(5000);
    }
    Serial.println(F("Motor Shield is initialized"));
}

void loop() {
    Serial.println("Set MotorA at 50\% and motorB at 100\% CloclWork direction");
    // By default there are 512 possible steps. 0 means stoped, 512 means 100%
    MotorShield.drive(256, 512);
    delay(5000);

    Serial.println("Coast Motors");
    // coast() lets the motors to stop smoothly by their own, same thing as unpluging power.
    MotorShield.coast();
    delay(2000);

    Serial.println("Set MotorA at 100\% Counter-Clockwork direction and motorB at 20\% CloclWork direction");
    // Setting a negative pwm value makes the motor turn in the other direction
    MotorShield.drive(-512, 100);
    delay(5000);

    Serial.println("BRAKE Motors");
    // Brake does stop the motors immediately.
    MotorShield.brake();
    delay(2000);
}
