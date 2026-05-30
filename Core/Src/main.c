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
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* =========================
   THONG SO PWM + MOTOR
   ========================= */

#define PWM_MAX               999     // Phai khop voi Period cua TIM2
#define PID_INTEGRAL_LIMIT    1.0f

#define MOTOR_FORWARD         1
#define MOTOR_BACKWARD       -1
#define MOTOR_STOP            0

#define MOTOR_DIR             1       // Doi thanh -1 neu motor quay nguoc
#define ENCODER_DIR           1       // Doi thanh -1 neu encoder dem nguoc

/*
  Motor 111 rpm:
  Neu thong so 111 rpm la toc do truc ra sau hop so,
  thi MOTOR_MAX_RPM = 111.0f la dung.
*/
#define MOTOR_MAX_RPM         111.0f

/*
  Thong so banh + encoder
*/
#define WHEEL_DIAMETER_M      0.100f  // 100 mm = 0.100 m
#define ENCODER_PPR           11.0f   // so xung encoder moi vong truc motor
#define GEAR_RATIO            90.0f   // ti so hop so, vi du 1:90
#define ENCODER_MULTIPLIER    4.0f    // encoder mode x4

#define PI_VALUE              3.1415926f

#define WHEEL_CIRCUMFERENCE_M (PI_VALUE * WHEEL_DIAMETER_M)
#define COUNTS_PER_WHEEL_REV  (ENCODER_PPR * ENCODER_MULTIPLIER * GEAR_RATIO)
#define METER_PER_COUNT       (WHEEL_CIRCUMFERENCE_M / COUNTS_PER_WHEEL_REV)

/*
  Toc do max ly thuyet:
  111 rpm, banh 100 mm => ~0.581 m/s
*/
#define MOTOR_MAX_SPEED_MPS   ((MOTOR_MAX_RPM / 60.0f) * WHEEL_CIRCUMFERENCE_M)

/*
  Chu ky PID
*/
#define PID_SAMPLE_TIME_MS    10
#define PID_SAMPLE_TIME_S     ((float)PID_SAMPLE_TIME_MS / 1000.0f)

#define SPEED_DEADBAND_MPS    0.005f

/* =========================
   BIEN DIEU KHIEN MOTOR
   ========================= */

volatile float target_speed_mps = 0.0f;
volatile float current_speed_mps = 0.0f;
volatile float speed_error_mps = 0.0f;

volatile int motor_pwm = 0;
volatile int encoder_delta = 0;

static uint16_t encoder_last = 0;

volatile int target_pwm = 0;          // PWM tuong ung voi target_speed_mps
volatile int pid_pwm_correction = 0;  // PWM PID bu them
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* =========================
   BIEN PID
   ========================= */

/*
  PID da co feed-forward nen Kp khong can qua cao.
  Neu xe len toc do cham: tang Ki len 300-400.
  Neu xe rung/dao dong: giam Kp xuong 250-300.
*/
float Kp = 350.0f;
float Ki = 250.0f;
float Kd = 0.0f;

static float pid_error = 0.0f;
static float pid_last_error = 0.0f;
static float pid_integral = 0.0f;
static float pid_derivative = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

void Motor_Init(void);
void Motor_SetPWM(int pwm);
void Motor_SetDirectionPWM(int direction, int pwm);
void Motor_Stop(void);

int Encoder_GetDelta(void);
float Encoder_DeltaToMPS(int delta);

void Motor_SetSpeedMPS(float speed_mps);
void PID_Reset(void);
int PID_UpdateMPS(float target, float current);
void Motor_PID_Run(void);
void Motor_SendDebugUART(void);

bool SetVelocity(float speed, bool isRight);
int SpeedToTargetPWM(float speed_mps);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* =========================================================
   1. KHOI TAO MOTOR + ENCODER + PWM
   ========================================================= */

void Motor_Init(void)
{
  // Bat encoder TIM1
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  // Bat PWM TIM2 CH3 va CH4
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // PA2 - LPWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // PA3 - RPWM

  // Enable driver motor
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // R_EN + L_EN

  // PWM ban dau = 0
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

  // Reset encoder
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  encoder_last = 0;

  // Reset toc do + PID
  target_speed_mps = 0.0f;
  current_speed_mps = 0.0f;
  speed_error_mps = 0.0f;
  motor_pwm = 0;
  encoder_delta = 0;

  PID_Reset();
}

/* =========================================================
   2. SET PWM CHO MOTOR
   pwm > 0: quay thuan
   pwm < 0: quay nghich
   pwm = 0: dung
   ========================================================= */

void Motor_SetPWM(int pwm)
{
  pwm = pwm * MOTOR_DIR;

  if (pwm > PWM_MAX)
    pwm = PWM_MAX;

  if (pwm < -PWM_MAX)
    pwm = -PWM_MAX;

  if (pwm > 0)
  {
    // Quay thuan: RPWM chay, LPWM tat
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm); // RPWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);   // LPWM
  }
  else if (pwm < 0)
  {
    // Quay nghich: LPWM tat, RPWM tat nguoc
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, -pwm);
  }
  else
  {
    // Dung motor
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  }
}

/* =========================================================
   3. SET CHIEU + PWM
   direction = MOTOR_FORWARD / MOTOR_BACKWARD / MOTOR_STOP
   ========================================================= */

void Motor_SetDirectionPWM(int direction, int pwm)
{
  if (pwm < 0)
    pwm = -pwm;

  if (pwm > PWM_MAX)
    pwm = PWM_MAX;

  if (direction == MOTOR_FORWARD)
  {
    Motor_SetPWM(pwm);
  }
  else if (direction == MOTOR_BACKWARD)
  {
    Motor_SetPWM(-pwm);
  }
  else
  {
    Motor_SetPWM(0);
  }
}

/* =========================================================
   4. DUNG MOTOR
   ========================================================= */

void Motor_Stop(void)
{
  Motor_SetPWM(0);

  target_speed_mps = 0.0f;
  current_speed_mps = 0.0f;
  speed_error_mps = 0.0f;
  motor_pwm = 0;
  encoder_delta = 0;

  PID_Reset();
}

/* =========================================================
   5. DOC ENCODER DELTA
   Tra ve so xung encoder trong 1 chu ky dieu khien
   Ham nay xu ly duoc tran counter 16-bit
   ========================================================= */

int Encoder_GetDelta(void)
{
  uint16_t encoder_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

  int16_t delta = (int16_t)(encoder_now - encoder_last);

  encoder_last = encoder_now;

  return ENCODER_DIR * delta;
}

/* =========================================================
   6. DOI ENCODER DELTA SANG m/s
   delta: so xung encoder trong PID_SAMPLE_TIME_MS
   return: toc do hien tai, don vi m/s
   ========================================================= */

float Encoder_DeltaToMPS(int delta)
{
  float distance_m = (float)delta * METER_PER_COUNT;
  float speed_mps = distance_m / PID_SAMPLE_TIME_S;

  return speed_mps;
}

/* =========================================================
   7. SET TOC DO MONG MUON THEO m/s
   Vi du:
   Motor_SetSpeedMPS(0.30f);   // 0.30 m/s
   Motor_SetSpeedMPS(-0.30f);  // -0.30 m/s
   Motor_SetSpeedMPS(0.0f);    // dung
   ========================================================= */

void Motor_SetSpeedMPS(float speed_mps)
{
  /*
    Gioi han toc do theo motor 111 rpm.
    Max ly thuyet voi banh 100 mm la ~0.581 m/s.
  */
  if (speed_mps > MOTOR_MAX_SPEED_MPS)
    speed_mps = MOTOR_MAX_SPEED_MPS;

  if (speed_mps < -MOTOR_MAX_SPEED_MPS)
    speed_mps = -MOTOR_MAX_SPEED_MPS;

  target_speed_mps = speed_mps;
}

/* =========================================================
   8. RESET PID
   ========================================================= */

void PID_Reset(void)
{
  pid_error = 0.0f;
  pid_last_error = 0.0f;
  pid_integral = 0.0f;
  pid_derivative = 0.0f;

  target_pwm = 0;
  pid_pwm_correction = 0;
}

/* =========================================================
   9. TINH PID THEO DON VI m/s
   Co feed-forward theo motor 111 rpm.
   target: toc do mong muon, m/s
   current: toc do thuc te, m/s
   return: gia tri PWM
   ========================================================= */
int PID_UpdateMPS(float target, float current)
{
  if (fabsf(target) < SPEED_DEADBAND_MPS)
  {
    speed_error_mps = 0.0f;
    pid_error = 0.0f;
    pid_last_error = 0.0f;
    pid_integral = 0.0f;
    pid_derivative = 0.0f;

    target_pwm = 0;
    pid_pwm_correction = 0;

    return 0;
  }

  pid_error = target - current;
  speed_error_mps = pid_error;

  pid_integral += pid_error * PID_SAMPLE_TIME_S;

  if (pid_integral > PID_INTEGRAL_LIMIT)
    pid_integral = PID_INTEGRAL_LIMIT;

  if (pid_integral < -PID_INTEGRAL_LIMIT)
    pid_integral = -PID_INTEGRAL_LIMIT;

  pid_derivative = (pid_error - pid_last_error) / PID_SAMPLE_TIME_S;
  pid_last_error = pid_error;

  float pwm_feedforward = (float)SpeedToTargetPWM(target);

  float pid_correction = (Kp * pid_error)
                       + (Ki * pid_integral)
                       + (Kd * pid_derivative);

  float output = pwm_feedforward + pid_correction;

  target_pwm = (int)pwm_feedforward;
  pid_pwm_correction = (int)pid_correction;

  if (output > PWM_MAX)
    output = PWM_MAX;

  if (output < -PWM_MAX)
    output = -PWM_MAX;

  return (int)output;
}

/* =========================================================
   10. CHAY 1 CHU KY PID MOTOR
   Goi ham nay moi PID_SAMPLE_TIME_MS
   ========================================================= */

void Motor_PID_Run(void)
{
  // Doc encoder trong 1 chu ky
  encoder_delta = Encoder_GetDelta();

  // Doi sang toc do m/s
  current_speed_mps = Encoder_DeltaToMPS(encoder_delta);

  // Tinh PWM bang PID + feed-forward
  motor_pwm = PID_UpdateMPS(target_speed_mps, current_speed_mps);

  // Xuat PWM ra motor
  Motor_SetPWM(motor_pwm);
}

/* =========================================================
   11. GUI DEBUG QUA UART
   Goi ham nay trong main loop, khong nen goi lien tuc trong interrupt
   In mm/s de khong can bat printf float trong STM32
   ========================================================= */

void Motor_SendDebugUART(void)
{
  char msg[220];

  int target_mmps  = (int)(target_speed_mps * 1000.0f);
  int current_mmps = (int)(current_speed_mps * 1000.0f);
  int error_mmps   = (int)(speed_error_mps * 1000.0f);

  int max_mmps     = (int)(MOTOR_MAX_SPEED_MPS * 1000.0f);

  snprintf(msg, sizeof(msg),
           "Target:%d mm/s, Current:%d mm/s, Error:%d mm/s, TargetPWM:%d, PID_PWM:%d, PWM:%d, Delta:%d, Max:%d mm/s\r\n",
           target_mmps,
           current_mmps,
           error_mmps,
           target_pwm,
           pid_pwm_correction,
           motor_pwm,
           encoder_delta,
           max_mmps);

  HAL_UART_Transmit(&huart3,
                    (uint8_t *)msg,
                    strlen(msg),
                    HAL_MAX_DELAY);
}

/* =========================================================
   12. HAM SET VELOCITY DANG FLOAT
   speed: m/s
   isRight hien tai chua dung vi code nay moi cho 1 motor
   ========================================================= */

bool SetVelocity(float speed, bool isRight)
{
  (void)isRight;

  Motor_SetSpeedMPS(speed);

  return true;
}

int SpeedToTargetPWM(float speed_mps)
{
  if (speed_mps > MOTOR_MAX_SPEED_MPS)
    speed_mps = MOTOR_MAX_SPEED_MPS;

  if (speed_mps < -MOTOR_MAX_SPEED_MPS)
    speed_mps = -MOTOR_MAX_SPEED_MPS;

  return (int)((speed_mps / MOTOR_MAX_SPEED_MPS) * PWM_MAX);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  Motor_Init();

  /*
    Toc do muc tieu theo m/s.
    Motor 111 rpm + banh 100 mm:
    Max ly thuyet ~0.581 m/s.
    Nen 0.30 m/s la hop ly.
  */
  Motor_SetSpeedMPS(0.56f);

  uint32_t last_pid_time = 0;
  uint32_t last_debug_time = 0;

  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */

    uint32_t now = HAL_GetTick();

    /*
      Chay PID dung moi 10 ms.
      Khong dung HAL_Delay de tranh sai chu ky.
    */
    if (now - last_pid_time >= PID_SAMPLE_TIME_MS)
    {
      last_pid_time = now;
      Motor_PID_Run();
    }

    /*
      Gui debug moi 200 ms.
    */
    if (now - last_debug_time >= 200)
    {
      last_debug_time = now;
      Motor_SendDebugUART();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

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
  * @param  line: assert_param line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  (void)file;
  (void)line;

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
