/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 11 Task 2 - PID Controller for Self-Balancing Robot
  ******************************************************************************
  * TIM2 = 200Hz control loop ISR (complementary filter + PID)
  * TIM3 = PWM generation for motors (CH1=left on PC6, CH2=right on PA4)
  * Motor direction pins:
  *   Left  motor: PB12 (IN1), PB13 (IN2)
  *   Right motor: PB14 (IN1), PB15 (IN2)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Sampling Time 
#define DT 0.005f            

// PID Controller Structure 
typedef struct {
    float kp;               // Proportional gain
    float ki;               // Integral gain
    float kd;               // Derivative gain
    float setpoint;         // Target angle (degrees)
    float integral;         // Accumulated integral error
    float prev_error;       // Previous error for derivative
    float output_max;       // Maximum output limit
    float output_min;       // Minimum output limit
    float integral_max;     // Anti-windup limit (positive)
    float integral_min;     // Anti-windup limit (negative)
} PIDController;

// Sensor Fusion Structure 
typedef struct {
    float gyro_rate_dps;    // Gyro angular velocity (deg/s)
    float acc_angle_deg;    // Accelerometer-derived angle (deg)
    float tilt_angle;       // Fused tilt angle estimate (deg)
} AngleEstimate;

// PID Gains
#define KP  30.0f
#define KI  0.00f
#define KD 0.0f

// Setpoint
#define SETPOINT -2.1f

// Output Limits
#define PID_OUT_MAX  999.0f
#define PID_OUT_MIN -999.0f

// Integral Anti-Windup Limit
#define INTEGRAL_MAX  200.0f
#define INTEGRAL_MIN -200.0f

// Complementary Filter Weights
#define COMP_GYRO_WEIGHT  0.98f
#define COMP_ACC_WEIGHT   0.02f

// Sensor Configuration
#define GYRO_SCALE_DPS      0.00875f   
#define GYRO_OUT_Y_L_REG    0x2A      // Y-axis output low byte
#define GYRO_OUT_Y_H_REG    0x2B      // Y-axis output high byte
#define ACC_ADDRESS_WRITE   0x32      // I2C write address
#define ACC_ADDRESS_READ    0x33      // I2C read address
#define ACC_OUT_X_PAGE      0x28      // Output X register 
#define ACC_READ_AUTOINC    0x80      // Auto-increment bit for I2C

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Angle Estimation Variables 
volatile float shared_angle   = 0.0f;   // Bridge between ISR and main
volatile float shared_pid_out = 0.0f;   // PID output for display
volatile uint8_t display_flag = 0;

// Sensor measurements for display/debug
volatile float gyro_rate = 0.0f;
volatile float acc_angle = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t I3G_ReadReg(uint8_t reg_addr) {
    uint8_t tx_data = reg_addr | 0x80;  // Bit 7 = 1 for READ
    uint8_t rx_data;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rx_data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    return rx_data;
}


// Helper Functions for Sensor Data Acquisition

// Read gyroscope Y-axis
float Read_Gyro_Y(void) {
    uint8_t y_low  = I3G_ReadReg(GYRO_OUT_Y_L_REG);
    uint8_t y_high = I3G_ReadReg(GYRO_OUT_Y_H_REG);
    
    int16_t raw_gyro_y = (int16_t)((y_high << 8) | y_low);
    return (float)raw_gyro_y * GYRO_SCALE_DPS;  // Convert to deg/s
}

// Read accelerometer pitch angle
float Read_Acc_Pitch(void) {
    uint8_t acc_reg = ACC_OUT_X_PAGE | ACC_READ_AUTOINC;
    uint8_t acc_buf[6] = {0};

    HAL_I2C_Master_Transmit(&hi2c1, ACC_ADDRESS_WRITE, &acc_reg, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, ACC_ADDRESS_READ, acc_buf, 6, 10);

    int16_t acc_raw_x = (int16_t)((acc_buf[1] << 8) | acc_buf[0]);
    int16_t acc_raw_z = (int16_t)((acc_buf[5] << 8) | acc_buf[4]);

    // Pitch angle
    return atan2f((float)acc_raw_x, (float)acc_raw_z) * (180.0f / 3.14159f);
}

// Initialize PID controller with default
void PID_Init(PIDController *pid) {
    pid->kp = KP;
    pid->ki = KI;
    pid->kd = KD;
    pid->setpoint = SETPOINT;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_max = PID_OUT_MAX;
    pid->output_min = PID_OUT_MIN;
    pid->integral_max = INTEGRAL_MAX;
    pid->integral_min = INTEGRAL_MIN;
}

// Compute PID output given measured error
float PID_Update(PIDController *pid, float error) {
    // P 
    float p_term = pid->kp * error;

    // I 
    pid->integral += pid->ki * error * DT;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    // D 
    float derivative = (error - pid->prev_error) / DT;
    float d_term = pid->kd * derivative;
    pid->prev_error = error;

    // Total PID 
    float output = p_term + pid->integral + d_term;

    // final output
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

// Complementary Filter for Angle Estimation
float Complementary_Filter(float prev_angle, float gyro_rate_dps, float acc_angle_deg) {
    float gyro_contribution = (prev_angle + gyro_rate_dps * DT) * COMP_GYRO_WEIGHT;
    float acc_contribution = acc_angle_deg * COMP_ACC_WEIGHT;
    return gyro_contribution + acc_contribution;
}


// Gyroscope SPI Register Access
void I3G_WriteReg(uint8_t reg_addr, uint8_t data) {
    uint8_t tx_buffer[2];
    tx_buffer[0] = reg_addr & 0x7F; 
    tx_buffer[1] = data;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   // CS HIGH
}


/* 
 *  Motor Control Functions
 * 
 *
 *  Left Motor:
 *    PWM  -> TIM3_CH1 (PC6)   => Shield pin D9
 *    IN1  -> PB12              => Shield pin D6
 *    IN2  -> PB13              => Shield pin D7
 *
 *  Right Motor:
 *    PWM  -> TIM3_CH2 (PA4)   => Shield pin D10
 *    IN1  -> PB14              => Shield pin D8
 *    IN2  -> PB15              => Shield pin D12
 *
 *  Direction logic (L298N style):
 *    Forward  (CW):  IN1=HIGH, IN2=LOW
 *    Backward (CCW): IN1=LOW,  IN2=HIGH
 *    Brake/Stop:     IN1=LOW,  IN2=LOW
 */




void Motor_Left_Forward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);    // IN1 = HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // IN2 = LOW
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
}

void Motor_Left_Backward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  // IN1 = LOW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);    // IN2 = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
} 

// void Motor_Left_Forward(uint16_t speed) {
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);    // IN1 = HIGH
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);  // IN2 = LOW
//     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
// }

// void Motor_Left_Backward(uint16_t speed) {
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);  // IN1 = LOW
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);    // IN2 = HIGH
//     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
// } 

void Motor_Left_Stop(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}

// void Motor_Right_Forward(uint16_t speed) {
//     // SWAP THESE: Original was SET then RESET
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // IN1 = LOW
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // IN2 = HIGH
//     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
// }

// void Motor_Right_Backward(uint16_t speed) {
//     // SWAP THESE: Original was RESET then SET
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // IN1 = HIGH
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // IN2 = LOW
//     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
// }


void Motor_Right_Forward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);    // IN1 = HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);  // IN2 = LOW
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

void Motor_Right_Backward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  // IN1 = LOW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);    // IN2 = HIGH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

void Motor_Right_Stop(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

void Motors_Drive(float pid_output) {
    uint16_t pwm_magnitude = (uint16_t)fabsf(pid_output);
    
    // Clamp to maximum PWM value
    if (pwm_magnitude > (uint16_t)PID_OUT_MAX) {
        pwm_magnitude = (uint16_t)PID_OUT_MAX;
    }

    if (pid_output > 0.0f) {
        // Positive output: drive forward
        Motor_Left_Forward(pwm_magnitude);
        Motor_Right_Forward(pwm_magnitude);
    }
    else if (pid_output < 0.0f) {
        // Negative output: drive backward
        Motor_Left_Backward(pwm_magnitude);
        Motor_Right_Backward(pwm_magnitude);
    }
    else {
        // Zero output: stop motors
        Motor_Left_Stop();
        Motor_Right_Stop();
    }
}

void cout(const char *fmt, ...) {
    char buffer[128]; 
    va_list args;
    va_start(args, fmt);
    int l = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (l > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, l, HAL_MAX_DELAY);
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  PIDController pid_controller;
  /* USER CODE END 1 */

  /* MCU Configuration*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  // 1. Initialize Sensors
  I3G_WriteReg(0x20, 0x4F); // Gyro on
  uint8_t acc_init[2] = {0x20, 0x57}; 
  HAL_I2C_Master_Transmit(&hi2c1, 0x32, acc_init, 2, 50);

  // 2. Start PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // 3. Start Timer Interrupt
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    if (display_flag == 1) 
    {
      display_flag = 0;
      
      // Calculate float parts for printing
      int angle_i = (int)shared_angle;
      int angle_f = (int)(fabsf(shared_angle - angle_i) * 100);
      int acc_i = (int)acc_angle;
      int acc_f = (int)(fabsf(acc_angle - acc_i) * 100);
      int gyro_i = (int)gyro_rate;
      int gyro_f = (int)(fabsf(gyro_rate - gyro_i) * 100);
      int pid_i = (int)shared_pid_out;
      int pid_f = (int)(fabsf(shared_pid_out - pid_i) * 100);

      cout("%d.%02d,%d.%02d,%d.%02d,%d.%02d\r\n", 
           angle_i, angle_f, acc_i, acc_f, gyro_i, gyro_f, pid_i, pid_f);
    }
    /* USER CODE BEGIN 3 */
  } 
  /* USER CODE END 3 */
} // End of main

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ============================================================
 *  TIM2 ISR Callback — runs at 200 Hz (every 5ms)
 *
 *  This is the HEART of the system. It does:
 *   1) Read gyroscope (SPI)
 *   2) Read accelerometer (I2C)
 *   3) Complementary filter -> tilt angle
 *   4) PID controller -> motor command
 *   5) Drive motors
 *   6) Throttle UART display to ~10 Hz
 * ============================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM2) return;

    static PIDController pid_ctrl = {0};
    static float tilt_angle = 0.0f;
    static uint8_t initialized = 0;
    static int uart_counter = 0;

    if (!initialized) {
        PID_Init(&pid_ctrl);
        initialized = 1;
    }

    // Read sensor data using helper functions
    float gyro_rate_dps = Read_Gyro_Y();
    float acc_angle_deg = Read_Acc_Pitch();

    // Apply complementary filter for angle estimation
    tilt_angle = Complementary_Filter(tilt_angle, gyro_rate_dps, acc_angle_deg);

    // Compute PID control output
    float error = pid_ctrl.setpoint - tilt_angle;
    float pid_output = PID_Update(&pid_ctrl, error);

    // Drive motors based on PID output
    Motors_Drive(pid_output);

    // Update shared variables for main loop display
    shared_angle = tilt_angle;
    shared_pid_out = pid_output;
    gyro_rate = gyro_rate_dps;
    acc_angle = acc_angle_deg;

    // Throttle serial display to 10Hz
    if (++uart_counter >= 20) {
        display_flag = 1;
        uart_counter = 0;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
