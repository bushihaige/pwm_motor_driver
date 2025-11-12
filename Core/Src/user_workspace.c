#include "user_workspace.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f4xx.h"   // 确保有 DWT 寄存器定义


#define PI 3.1415926f
#define PWM_MAX 999

#define PWM_RES 100
#define DELAY_US 100
#define DUTY_CYCLE 0.2

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

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

void simple_drive_motor(void){
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

}
void fixed_pwm(void){
    for (int i = 0; i < PWM_RES; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (i < DUTY_CYCLE * PWM_RES) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        delay_us(DELAY_US);
    }

}


void User_Workspace_Init(void)
{
	DWT_Init();
    HAL_UART_Transmit(&huart1, (uint8_t*)"User Init OK\r\n", 13, HAL_MAX_DELAY);
}



void User_Workspace_Loop(void)
{
	//simple_drive_motor();
	fixed_pwm();
}
