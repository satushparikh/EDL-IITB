#include "DC_MOTOR.h"

const DC_MOTOR_CfgType DC_MOTOR_CfgParam[DC_MOTOR_UNITS] =
{
    // DC MOTOR 1 Configurations
    {
    GPIOB,
	GPIOC,
    GPIO_PIN_4,
	GPIO_PIN_9,
    TIM2,
    TIM_CHANNEL_3,
    64,
    DC_MOTOR_F_PWM,
    DC_MOTOR_PWM_RES
    },
    // DC MOTOR 2 Configurations
    {
    GPIOC,
	GPIOB,
    GPIO_PIN_0,
	GPIO_PIN_5,
    TIM1,
    TIM_CHANNEL_1,
    64,
    DC_MOTOR_F_PWM,
    DC_MOTOR_PWM_RES
    }
};
