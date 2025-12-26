/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Automatic Smart Bin with Dual Ultrasonic Sensors
*                   - Sensor 1 (PA8/PA9): Opens lid when hand within 5cm
*                   - Sensor 2 (PB8/PB9): Monitors bin level, locks at ≤3cm
*                   - When bin is full (≤3cm), servo locks at 0° (closed)
******************************************************************************
* @attention
*
* Copyright (c) 2025 STMicroelectronics.
* All rights reserved.
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fonts.h"
#include "ssd1306.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Ultrasonic Sensor 1 - Lid control (Port A)
#define TRIG_PIN_1   GPIO_PIN_9
#define TRIG_PORT_1  GPIOA
#define ECHO_PIN_1   GPIO_PIN_8
#define ECHO_PORT_1  GPIOA
// Ultrasonic Sensor 2 - Bin level measurement (Port B)
#define TRIG_PIN_2   GPIO_PIN_9
#define TRIG_PORT_2  GPIOB
#define ECHO_PIN_2   GPIO_PIN_8
#define ECHO_PORT_2  GPIOB
// Distance thresholds
#define LID_OPEN_DISTANCE  5   // Open lid if hand within 5cm (UPDATED)
#define BIN_FULL_DISTANCE  3   // Bin is full if distance <= 3cm (UPDATED)
// Servo calibration values - adjust these for your specific servo
#define SERVO_MIN_US   500    // Absolute minimum pulse (safety limit)
#define SERVO_0_DEG    1000   // Pulse for 0 degrees (BIN CLOSED)
#define SERVO_90_DEG   1500   // Pulse for 90 degrees (center)
#define SERVO_180_DEG  2000   // Pulse for 180 degrees (BIN OPEN)
#define SERVO_MAX_US   2500   // Absolute maximum pulse (safety limit)
// Bin positions
#define BIN_CLOSED_ANGLE  0     // Lid closed at 0 degrees
#define BIN_OPEN_ANGLE    180   // Lid open at 180 degrees
// Servo movement delay
#define SERVO_MOVE_DELAY  1000  // Time to wait for servo to reach position (ms)
/* USER CODE END PD */
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim1;  // Timer for ultrasonic timing
TIM_HandleTypeDef htim2;  // Timer for servo PWM
/* USER CODE BEGIN PV */
uint16_t distance_sensor1 = 0;  // Distance from sensor 1 (lid control)
uint16_t distance_sensor2 = 0;  // Distance from sensor 2 (bin level)
uint8_t lidIsOpen = 0;           // Track lid state: 0=closed, 1=open
uint8_t binIsFull = 0;           // Track bin full status: 0=not full, 1=full
char strCopy[20];
static uint16_t current_position = SERVO_0_DEG;  // Track current position
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t Read_Ultrasonic_Sensor1(void);
uint16_t Read_Ultrasonic_Sensor2(void);
void Servo_SetPulse_us(uint16_t us);
void Servo_GotoAngle(uint8_t angle);
void Servo_OpenLid(void);
void Servo_CloseLid(void);
void Update_OLED_Display(void);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
/* MCU Configuration--------------------------------------------------------*/
HAL_Init();
SystemClock_Config();
/* Initialize all configured peripherals */
MX_GPIO_Init();
MX_I2C2_Init();
MX_TIM1_Init();
MX_TIM2_Init();
/* USER CODE BEGIN 2 */
// Start TIM1 for ultrasonic timing
HAL_TIM_Base_Start(&htim1);
// Start PWM for servo
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
// Initialize TRIG pins LOW
HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_RESET);
HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_RESET);
// Initialize servo to closed position
Servo_GotoAngle(BIN_CLOSED_ANGLE);
HAL_Delay(SERVO_MOVE_DELAY);
lidIsOpen = 0;
binIsFull = 0;
// Initialize OLED display
SSD1306_Init();
SSD1306_GotoXY(0, 0);
SSD1306_Puts("Smart Bin", &Font_11x18, 1);
SSD1306_GotoXY(0, 25);
SSD1306_Puts("Ready!", &Font_11x18, 1);
SSD1306_UpdateScreen();
HAL_Delay(1000);
/* USER CODE END 2 */
/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
  // Read distance from sensor 2 (bin level measurement) FIRST
  distance_sensor2 = Read_Ultrasonic_Sensor2();

  // Check if bin is full (≤3cm)
  if (distance_sensor2 > 0 && distance_sensor2 <= BIN_FULL_DISTANCE)
  {
    binIsFull = 1;
    // If bin is full and lid is open, close it immediately
    if (lidIsOpen == 1)
    {
      Servo_CloseLid();
      lidIsOpen = 0;
      HAL_Delay(SERVO_MOVE_DELAY);
    }
  }
  else
  {
    binIsFull = 0;
  }

  // Only process hand detection if bin is NOT full
  if (!binIsFull)
  {
    // Read distance from sensor 1 (lid control)
    distance_sensor1 = Read_Ultrasonic_Sensor1();

    // Check if hand is near sensor 1 (within 5cm) to open/close lid
    if (distance_sensor1 > 0 && distance_sensor1 <= LID_OPEN_DISTANCE)
    {
      // Object detected within 5cm range
      if (lidIsOpen == 0)
      {
        // Lid is closed, open it
        Servo_OpenLid();
        lidIsOpen = 1;
        HAL_Delay(SERVO_MOVE_DELAY);  // Wait for servo to reach position
      }
    }
    else
    {
      // No object detected or too far away (>5cm)
      if (lidIsOpen == 1)
      {
        // Lid is open, close it
        Servo_CloseLid();
        lidIsOpen = 0;
        HAL_Delay(SERVO_MOVE_DELAY);  // Wait for servo to reach position
      }
    }
  }
  // If bin is full, servo remains locked at initial position (0°)
  // No servo movement commands are executed

  // Update OLED display with bin level
  Update_OLED_Display();
  HAL_Delay(100);  // Update every 100ms
}
/* USER CODE END 3 */
}
/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
RCC_OscInitTypeDef RCC_OscInitStruct = {0};
RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState = RCC_HSE_ON;
RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
RCC_OscInitStruct.HSIState = RCC_HSI_ON;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
{
  Error_Handler();
}
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
{
  Error_Handler();
}
}
/**
* @brief I2C2 Initialization Function
* @param None
* @retval None
*/
static void MX_I2C2_Init(void)
{
hi2c2.Instance = I2C2;
hi2c2.Init.ClockSpeed = 400000;
hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
hi2c2.Init.OwnAddress1 = 0;
hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
hi2c2.Init.OwnAddress2 = 0;
hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
if (HAL_I2C_Init(&hi2c2) != HAL_OK)
{
  Error_Handler();
}
}
/**
* @brief TIM1 Initialization Function (for ultrasonic timing - 1µs resolution)
* @param None
* @retval None
*/
static void MX_TIM1_Init(void)
{
TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
htim1.Instance = TIM1;
htim1.Init.Prescaler = 71;  // 72MHz / 72 = 1MHz (1µs per tick)
htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
htim1.Init.Period = 65535;
htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim1.Init.RepetitionCounter = 0;
htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
{
  Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
{
  Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
{
  Error_Handler();
}
}
/**
* @brief TIM2 Initialization Function (for servo PWM - 50Hz)
* @param None
* @retval None
*/
static void MX_TIM2_Init(void)
{
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};
htim2.Instance = TIM2;
htim2.Init.Prescaler = 71;     // 72MHz / 72 = 1MHz (1µs per tick)
htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
htim2.Init.Period = 19999;     // 20ms period (50Hz for servo)
htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
{
  Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
{
  Error_Handler();
}
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = SERVO_0_DEG;  // Start at closed position
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
{
  Error_Handler();
}
HAL_TIM_MspPostInit(&htim2);
}
/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};
/* GPIO Ports Clock Enable */
__HAL_RCC_GPIOD_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();
/* Configure Sensor 1 pins (Port A) */
// TRIG pin (PA9) as output
HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_RESET);
GPIO_InitStruct.Pin = TRIG_PIN_1;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(TRIG_PORT_1, &GPIO_InitStruct);
// ECHO pin (PA8) as input
GPIO_InitStruct.Pin = ECHO_PIN_1;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(ECHO_PORT_1, &GPIO_InitStruct);
/* Configure Sensor 2 pins (Port B) */
// TRIG pin (PB9) as output
HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_RESET);
GPIO_InitStruct.Pin = TRIG_PIN_2;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(TRIG_PORT_2, &GPIO_InitStruct);
// ECHO pin (PB8) as input
GPIO_InitStruct.Pin = ECHO_PIN_2;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(ECHO_PORT_2, &GPIO_InitStruct);
}
/* USER CODE BEGIN 4 */
/**
* @brief  Set servo pulse width in microseconds with safety limits
* @param  us: Pulse width in microseconds
* @retval None
*/
void Servo_SetPulse_us(uint16_t us)
{
 // Clamp to safe range
 if (us < SERVO_MIN_US) us = SERVO_MIN_US;
 if (us > SERVO_MAX_US) us = SERVO_MAX_US;
 // With PSC=71 => 1 tick = 1us, so CCR = us
 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, us);
 current_position = us;
}
/**
* @brief  Move servo to specific angle (0-180 degrees)
* @param  angle: Target angle in degrees (0-180)
* @retval None
*/
void Servo_GotoAngle(uint8_t angle)
{
 // Clamp angle to 0-180
 if (angle > 180) angle = 180;
 // Linear interpolation between 0° and 180°
 uint16_t pulse = SERVO_0_DEG + ((uint32_t)angle * (SERVO_180_DEG - SERVO_0_DEG) + 90) / 180;
 Servo_SetPulse_us(pulse);
}
/**
* @brief  Open the lid by moving servo to 180 degrees
* @retval None
*/
void Servo_OpenLid(void)
{
 Servo_GotoAngle(BIN_OPEN_ANGLE);
}
/**
* @brief  Close the lid by moving servo to 0 degrees
* @retval None
*/
void Servo_CloseLid(void)
{
 Servo_GotoAngle(BIN_CLOSED_ANGLE);
}
/**
* @brief  Read distance from Sensor 1 (lid control)
* @retval Distance in centimeters
*/
uint16_t Read_Ultrasonic_Sensor1(void)
{
uint32_t startTick, endTick, timeout;
// Send 10µs trigger pulse
HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_SET);
__HAL_TIM_SET_COUNTER(&htim1, 0);
while (__HAL_TIM_GET_COUNTER(&htim1) < 10);
HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_RESET);
// Wait for ECHO to go HIGH (with timeout)
timeout = HAL_GetTick();
while (!HAL_GPIO_ReadPin(ECHO_PORT_1, ECHO_PIN_1))
{
  if ((HAL_GetTick() - timeout) > 10)
    return 0;
}
startTick = __HAL_TIM_GET_COUNTER(&htim1);
// Wait for ECHO to go LOW (with timeout)
timeout = HAL_GetTick();
while (HAL_GPIO_ReadPin(ECHO_PORT_1, ECHO_PIN_1))
{
  if ((HAL_GetTick() - timeout) > 50)
    return 0;
}
endTick = __HAL_TIM_GET_COUNTER(&htim1);
// Calculate pulse width
uint32_t pulseWidth;
if (endTick >= startTick)
  pulseWidth = endTick - startTick;
else
  pulseWidth = (65535 - startTick) + endTick;
// Convert to distance in cm
uint16_t dist = pulseWidth / 58;
return dist;
}
/**
* @brief  Read distance from Sensor 2 (bin level)
* @retval Distance in centimeters
*/
uint16_t Read_Ultrasonic_Sensor2(void)
{
uint32_t Value1, Value2;
// Send 10µs trigger pulse
HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_SET);
__HAL_TIM_SET_COUNTER(&htim1, 0);
while (__HAL_TIM_GET_COUNTER(&htim1) < 10);
HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_RESET);
// Wait for ECHO to go HIGH
uint32_t timeout = HAL_GetTick();
while (!HAL_GPIO_ReadPin(ECHO_PORT_2, ECHO_PIN_2))
{
  if (HAL_GetTick() - timeout > 50)
    return 0;
}
Value1 = __HAL_TIM_GET_COUNTER(&htim1);
// Wait for ECHO to go LOW
timeout = HAL_GetTick();
while (HAL_GPIO_ReadPin(ECHO_PORT_2, ECHO_PIN_2))
{
  if (HAL_GetTick() - timeout > 50)
    return 0;
}
Value2 = __HAL_TIM_GET_COUNTER(&htim1);
// Calculate distance in cm
uint16_t dist = (Value2 - Value1) * 0.034 / 2;
return dist;
}
/**
* @brief  Update OLED display with bin level information
* @retval None
*/
void Update_OLED_Display(void)
{
SSD1306_Fill(SSD1306_COLOR_BLACK);
// Check if bin is full (distance ≤3 cm)
if (distance_sensor2 > 0 && distance_sensor2 <= BIN_FULL_DISTANCE)
{
  SSD1306_GotoXY(10, 10);
  SSD1306_Puts("Bin is", &Font_11x18, 1);
  SSD1306_GotoXY(10, 35);
  SSD1306_Puts("Full!", &Font_16x26, 1);
}
else
{
  SSD1306_GotoXY(0, 0);
  SSD1306_Puts("Distance:", &Font_11x18, 1);
  sprintf(strCopy, "%d cm   ", distance_sensor2);
  SSD1306_GotoXY(0, 30);
  SSD1306_Puts(strCopy, &Font_16x26, 1);
}
SSD1306_UpdateScreen();
}
/* USER CODE END 4 */
/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
__disable_irq();
while (1)
{
}
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
