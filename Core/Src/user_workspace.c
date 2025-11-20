#include "user_workspace.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f4xx.h"   // 确保有 DWT 寄存器定义
#include "i2c.h"
#include "stdio.h"

#define PI 3.1415926f
#define PWM_MAX 999

#define AS5600_ADDR  (0x36 << 1)
#define RAW_ANGLE_REG 0x0C

#define PWM_RES 200
#define DELAY_US 10
#define STEP_INC 0.02f  // 电角度增量（决定转速）
#define DUTY_CYCLE 0.2

#define POLE_PAIRS 7   // 电机极对数

//ADC define
#define ADC_REF_VOLT    3.3f
#define ADC_MAX_CODE    4095.0f
#define VREF_OFFSET     1.65f     // INA199 REF=1.65V
#define CURRENT_GAIN    5.0f      // 1/(0.01Ω * 20V/V)

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

//ADC extern
extern ADC_HandleTypeDef hadc1;
uint16_t adc_buf[3];

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

void motor_spwm(void)
{
	static float theta = 0.0f;   // 电角度
	float Ua = (sinf(POLE_PAIRS * theta) + 1.0f) / 2.0f;
	float Ub = (sinf(POLE_PAIRS * theta + 2.0f * PI / 3.0f) + 1.0f) / 2.0f;
	float Uc = (sinf(POLE_PAIRS * theta + 4.0f * PI / 3.0f) + 1.0f) / 2.0f;

	for (int i = 0; i < PWM_RES; i++) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (i < Ua * PWM_RES) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (i < Ub * PWM_RES) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (i < Uc * PWM_RES) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		delay_us(DELAY_US);
	}

	theta += STEP_INC;
	if (theta > 2 * PI) theta -= 2 * PI;

}


float AS5600_ReadAngle(void)
{
    uint8_t buf[2];
    HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, RAW_ANGLE_REG, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    raw &= 0x0FFF; // 12位数据
    return (raw * 360.0f) / 4096.0f;
}

void AS5600_ReadAndPrint(void)
{
    float angle = AS5600_ReadAngle();

    char msg[64];
    int len = snprintf(msg, sizeof(msg), "Angle=%.2f deg\r\n", angle);

    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
}

void Print_Current(uint8_t ch)
{
    if (ch >= 3) return;  // 防止越界

    char msg[64];
    uint16_t raw = adc_buf[ch];

    // 1) ADC → 电压
    float v = raw * ADC_REF_VOLT / ADC_MAX_CODE;

    // 2) 电压 → 电流
    float current = (v - VREF_OFFSET) * CURRENT_GAIN;

    // 3) 输出
    int len = snprintf(msg, sizeof(msg),
                       "CH%d Current = %.3f A (ADC=%d, V=%.3fV)\r\n",
                       ch, current, raw, v);

    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
}

void User_Workspace_Init(void)
{
	DWT_Init();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 3);  // 告诉DMA你的数组位置
    HAL_UART_Transmit(&huart1, (uint8_t*)"User Init OK\r\n", 13, HAL_MAX_DELAY);
}



void User_Workspace_Loop(void)
{
	motor_spwm();
	//AS5600_ReadAndPrint();
	//Print_Current(0);
	//Print_Current(1);
	//Print_Current(2);


}
