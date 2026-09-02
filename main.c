/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define bool _Bool
#define FILTER_SIZE 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

int pwm;

bool current_mode;
bool previous_mode;
bool unlock_step;

uint32_t adc_buffer[1];

uint8_t usage_count = 0;
uint16_t address_nuse = 0x0005;
uint16_t address_number_of_nuse = 0x000B;
uint8_t lock_stage = 0;

uint8_t filter_index = 0;
uint32_t filtered_value = 0;
uint16_t adc_history[FILTER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void SystemClock_Config(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Update the ADC moving-average filter.
  */
void ADC_Filter_Update(void)
{
    adc_history[filter_index] = adc_buffer[0];
    filter_index = (filter_index + 1) % FILTER_SIZE;

    uint32_t sum = 0;

    for (uint8_t i = 0; i < FILTER_SIZE; i++)
    {
        sum += adc_history[i];
        HAL_Delay(5);
    }

    filtered_value = sum / FILTER_SIZE;
}

/**
  * @brief Gradually change the motor PWM.
  * @param direction Motor movement direction.
  */
void softMove(int direction)
{
    int target_pwm;

    while (1)
    {
        target_pwm = 550 + (0.856 * adc_buffer[0]);
        pwm = pwm + direction * 100;

        HAL_Delay(50);

        if (pwm < 0)
        {
            pwm = 0;
            TIM16->CCR1 = pwm;
            return;
        }
        else if (pwm > target_pwm && direction == 1)
        {
            pwm = target_pwm;
            TIM16->CCR1 = pwm;
            return;
        }
        else
        {
            TIM16->CCR1 = pwm;
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration------------------------------------------------------*/

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM16_Init();
    MX_ADC1_Init();
    MX_I2C2_Init();

    /* Make sure both motor direction outputs are initially disabled. */
    HAL_GPIO_WritePin(Motor_Left_GPIO_Port, Motor_Left_Pin, 0);
    HAL_GPIO_WritePin(Motor_Right_GPIO_Port, Motor_Right_Pin, 0);

    /* Read the current lock stage from external memory. */
    HAL_I2C_Mem_Read(
        &hi2c2,
        0xA0,
        address_number_of_nuse,
        I2C_MEMADD_SIZE_16BIT,
        &lock_stage,
        1,
        1000
    );

    HAL_Delay(100);

    /*
     * Read the number of motor uses only when the system
     * has not already passed the final lock stage.
     */
    if (lock_stage < 2)
    {
        HAL_I2C_Mem_Read(
            &hi2c2,
            0xA0,
            address_nuse,
            I2C_MEMADD_SIZE_16BIT,
            &usage_count,
            1,
            1000
        );

        HAL_Delay(100);
    }

    /* ADC calibration and DMA start. */
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, adc_buffer, 1);

    /* Start PWM output. */
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
    TIM16->CCR1 = 0;

    /* Determine the initial motor direction from the limit switches. */
    if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0)
    {
        current_mode = 0;
        previous_mode = 1;
    }
    else if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0)
    {
        current_mode = 1;
        previous_mode = 0;
    }
    else
    {
        previous_mode = 1;
    }

    /* Initialize the ADC filter buffer. */
    for (uint8_t i = 0; i < FILTER_SIZE; i++)
    {
        adc_history[i] = adc_buffer[0];
        HAL_Delay(5);
    }

    /*
     * First lock stage.
     *
     * Unlock sequence:
     * 1. Low speed/input + SW1 active
     * 2. High speed/input + SW2 active
     */
    if (usage_count > 200 && lock_stage == 0)
    {
        unlock_step = 0;

        while (1)
        {
            if (unlock_step == 0)
            {
                HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                HAL_Delay(1000);

                HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
                HAL_Delay(1000);

                if ((adc_buffer[0] < 100) &&
                    (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0))
                {
                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                    HAL_Delay(500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
                    HAL_Delay(500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                    HAL_Delay(500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
                    HAL_Delay(500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                    HAL_Delay(500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);

                    unlock_step = 1;

                    HAL_Delay(3000);
                }
            }

            if (unlock_step == 1)
            {
                HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                HAL_Delay(250);

                HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
                HAL_Delay(250);

                if ((adc_buffer[0] > 4000) &&
                    (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0))
                {
                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                    HAL_Delay(1500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);

                    usage_count = 0;

                    HAL_I2C_Mem_Write(
                        &hi2c2,
                        0xA0,
                        address_nuse,
                        I2C_MEMADD_SIZE_16BIT,
                        &usage_count,
                        1,
                        1000
                    );

                    HAL_Delay(100);

                    lock_stage++;

                    HAL_I2C_Mem_Write(
                        &hi2c2,
                        0xA0,
                        address_number_of_nuse,
                        I2C_MEMADD_SIZE_16BIT,
                        &lock_stage,
                        1,
                        1000
                    );

                    HAL_Delay(100);

                    break;
                }
            }
        }
    }

    /*
     * Second lock stage.
     *
     * Unlock sequence:
     * 1. High speed/input + both switches inactive
     * 2. Low speed/input + SW1 active
     */
    else if (usage_count > 200 && lock_stage == 1)
    {
        unlock_step = 0;

        while (1)
        {
            if (unlock_step == 0)
            {
                HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                HAL_Delay(1000);

                HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
                HAL_Delay(1000);

                if ((adc_buffer[0] > 4000) &&
                    (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1) &&
                    (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 1))
                {
                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                    HAL_Delay(500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
                    HAL_Delay(500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                    HAL_Delay(500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
                    HAL_Delay(500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                    HAL_Delay(500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);

                    unlock_step = 1;

                    HAL_Delay(3000);
                }
            }

            if (unlock_step == 1)
            {
                HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                HAL_Delay(250);

                HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
                HAL_Delay(250);

                if ((adc_buffer[0] < 100) &&
                    (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0))
                {
                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
                    HAL_Delay(1500);

                    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);

                    usage_count = 0;

                    HAL_I2C_Mem_Write(
                        &hi2c2,
                        0xA0,
                        address_nuse,
                        I2C_MEMADD_SIZE_16BIT,
                        &usage_count,
                        1,
                        1000
                    );

                    HAL_Delay(100);

                    lock_stage++;

                    HAL_I2C_Mem_Write(
                        &hi2c2,
                        0xA0,
                        address_number_of_nuse,
                        I2C_MEMADD_SIZE_16BIT,
                        &lock_stage,
                        1,
                        1000
                    );

                    HAL_Delay(100);

                    break;
                }
            }
        }
    }
    else
    {
        /* Count a normal motor usage and save it to external memory. */
        usage_count++;

        HAL_I2C_Mem_Write(
            &hi2c2,
            0xA0,
            address_nuse,
            I2C_MEMADD_SIZE_16BIT,
            &usage_count,
            1,
            1000
        );

        HAL_Delay(100);
    }

    /* Main motor control loop. */
    while (1)
    {
        /* Update motor direction according to the limit switches. */
        if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0)
        {
            current_mode = 0;
        }
        else if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0)
        {
            current_mode = 1;
        }

        /*
         * If the requested direction has changed,
         * gradually stop the motor, change direction,
         * and gradually increase the PWM again.
         */
        if ((current_mode == 0 && previous_mode == 1) ||
            (current_mode == 1 && previous_mode == 0))
        {
            softMove(-1);

            HAL_Delay(100);

            HAL_GPIO_WritePin(Motor_Left_GPIO_Port, Motor_Left_Pin, 0);
            HAL_GPIO_WritePin(Motor_Right_GPIO_Port, Motor_Right_Pin, 0);

            HAL_Delay(100);

            HAL_GPIO_WritePin(Motor_Left_GPIO_Port, Motor_Left_Pin, current_mode);
            HAL_GPIO_WritePin(Motor_Right_GPIO_Port, Motor_Right_Pin, previous_mode);

            softMove(1);

            HAL_Delay(100);

            previous_mode = current_mode;
        }

        HAL_Delay(50);

        /* Update filtered ADC value and calculate PWM. */
        ADC_Filter_Update();

        pwm = 550 + (0.865 * filtered_value);

        TIM16->CCR1 = pwm;
    }
}