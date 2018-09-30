# WEMOS_Motor_Shield_Arduino_Library
This is a Arduino library for the WEMOS Motor Shiled - a shield for D1 mini, i2c interface, based TB6612 [Wemos Motor Shield](https://wiki.wemos.cc/products:d1_mini_shields:motor_shield).
The i2c protocol implemented in this library differs for the original in order to overcome some limitations.
I made a compatible firmware for the shield [here](https://github.com/danielfmo/wemos_motor_shield).

## Motor Shield I2C Address:
The shield has a matrix in PCB that can be soldered in order to set the shield Address.

|  AD0  |  AD1  | Address | `MOTORSHIELD_ADDRESS` enum
| :---: | :---: | :-----: | -----------------------:
|   -   |   -   | 0x2D    | `MOTORSHIELD_AD00`
|   -   |   x   | 0x2E    | `MOTORSHIELD_AD01`
|   x   |   -   | 0x2F    | `MOTORSHIELD_AD10`
|   x   |   x   | 0x30    | `MOTORSHIELD_AD11`
Where 'x' means that the jumper is soldered and '-' means that the jumper is kept open.

## Motor Shield PWM resolution and max frequency:
The STM32F030 MCU present on the shield have a limitation on the PWM generation, where the PWM frequency
times the PWM steps cannot exceed the CPU frequency (8MHz). This means that there is a maximum possible frequency for a given PWM resolution.

| PWM steps | PWM max frequency | `MOTORSHIELD_PWM_RESOLUTION` enum
|      ---: |              ---: | ---
|        64 |       65'535 (Hz) | `MOTORSHIELD_PWM_RES_64STEP` or `MOTORSHIELD_PWM_RES_6BIT`
|       128 |       62'500 (Hz) | `MOTORSHIELD_PWM_RES_128STEP` or `MOTORSHIELD_PWM_RES_7BIT`
|       256 |       31'250 (Hz) | `MOTORSHIELD_PWM_RES_256STEP` or `MOTORSHIELD_PWM_RES_8BIT`
|       512 |       15'625 (Hz) | `MOTORSHIELD_PWM_RES_512STEP` or `MOTORSHIELD_PWM_RES_9BIT`
|      1024 |        7'812 (Hz) | `MOTORSHIELD_PWM_RES_1024STEP` or `MOTORSHIELD_PWM_RES_10BIT`
|      2048 |        3'906 (Hz) | `MOTORSHIELD_PWM_RES_2048STEP` or `MOTORSHIELD_PWM_RES_11BIT`
|      4096 |        1'953 (Hz) | `MOTORSHIELD_PWM_RES_4096STEP` or `MOTORSHIELD_PWM_RES_12BIT`

## Motor Shield Motor Direction:
The shield accepts 5 'directions' to set the motor.

|         DIRECTION |     `MOTORSHIELD_ADDRESS` | Description
| ----------------: | ------------------------: | -------------------------------------------------------
|    Brake or Short |   `MOTOR_DIRECTION_BRAKE` | Stops the motor by shorting connectors (Active brake)
| Counter-Clockwork |     `MOTOR_DIRECTION_CCW` | Turns the motor on Counter-Clockwork direction
|         Clockwork |      `MOTOR_DIRECTION_CW` | Turns the motor on Clockwork direction
|     Coast or Open |   `MOTOR_DIRECTION_COAST` | Stops the motor by opening the connectors, coast.
|           Standby | `MOTOR_DIRECTION_STANDBY` | Sets the TB6612FNG on standby mode, impacts all motors.

## License
This project and all files in this repository are released under MIT License, see the [LICENSE](LICENSE.md) file for more details.

## Credits
This project is based on the firmware released by [pbugalski](https://github.com/pbugalski/wemos_motor_shield) and thank him for that.
This project is as alternative to the original firmware for the [WEMOS motor shield](https://github.com/wemos/Motor_Shield_Firmware).
I would like to thank all Hackaday community for their support and specifically [ꝺeshipu](https://hackaday.io/deshipu) for his [project](https://hackaday.io/project/18439-motor-shield-reprogramming?page=1#discussion-list).