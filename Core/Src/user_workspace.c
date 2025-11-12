#include "user_workspace.h"
#include "usart.h"
#include "gpio.h"

#define PI 3.1415926f
#define PWM_MAX 999  

static void SinPWM_Update(void)
{
    static float theta = 0.0f;
    float Ua, Ub, Uc;
    char msg[80];

    Ua = (sinf(theta) + 1.0f) / 2.0f;
    Ub = (sinf(theta + 2.0f * PI / 3.0f) + 1.0f) / 2.0f;
    Uc = (sinf(theta + 4.0f * PI / 3.0f) + 1.0f) / 2.0f;

    int len = sprintf(msg, "Ua=%.3f, Ub=%.3f, Uc=%.3f\r\n", Ua, Ub, Uc);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);

    theta += 0.02f;
    if (theta >= 2.0f * PI)
        theta -= 2.0f;
}

void User_Workspace_Init(void)
{
	// 如果启用 PWM，可以在此启动
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_UART_Transmit(&huart1, (uint8_t*)"User Init OK\r\n", 13, HAL_MAX_DELAY);
}


void User_Workspace_Loop(void)
{
    // 三相GPIO切换测试
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(500);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(500);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(500);

    // 若想输出正弦PWM：
    // SinPWM_Update();
}
