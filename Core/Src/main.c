/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


/// high-bandwidth 3-phase motor control for robots
/// Written by Ben Katz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "structs.h"
#include <stdio.h>
#include <string.h>

#include "position_sensor.h"
#include "preference_writer.h"
#include "hw_config.h"
#include "user_config.h"
#include "fsm.h"
#include "drv8323.h"
#include "foc.h"
#include "math_ops.h"
#include "calibration.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define VERSION_NUM 2.0f


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Flash Registers */
float __float_reg[64];
int __int_reg[256];
PreferenceWriter prefs;

int count = 0;

/* Structs for control, etc */

ControllerStruct controller;
ObserverStruct observer;
FSMStruct state;
EncoderStruct comm_encoder;
DRVStruct drv;
CalStruct comm_encoder_cal;
CANTxMessage can_tx;
CANRxMessage can_rx;

uint8_t Serial2RxBuffer[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void watchdog_init(void){
	/* Independent watchdog, kicked by the commutation interrupt.  If the control
	 * loop ever stops running (wedged SPI, stuck ADC poll, crash), this resets
	 * the chip instead of leaving the PWM frozen with the gate drive enabled.
	 *
	 * Timeout is ~4 s nominal.  It can't be much shorter: saving preferences
	 * erases a 128 KB flash sector, which stalls the whole CPU (including the
	 * kicking interrupt) for up to ~2 s, and the LSI clock feeding the watchdog
	 * is loose (17-47 kHz across parts and temperature). */
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;	// pause the watchdog while halted on a breakpoint
	IWDG->KR = 0x5555;								// enable register access
	IWDG->PR = 0x3;									// prescaler /32: ~1 kHz tick from the ~32 kHz LSI
	IWDG->RLR = 4000;								// ~4 s nominal timeout
	IWDG->KR = 0xAAAA;								// latch the reload value
	IWDG->KR = 0xCCCC;								// start the watchdog.  No way to stop it now
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

  /* Start the watchdog first, so a hang anywhere - including during init -
   * resets the chip.  The commutation interrupt kicks it once running;
   * all of init takes well under the ~4 s timeout */
  watchdog_init();

  /* Load settings from flash */
  preference_writer_load(&prefs);

  /* Sanitize configs in case flash is empty*/
  if(E_ZERO==-1){E_ZERO = 0;}
  if(M_ZERO==-1){M_ZERO = 0;}
  if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
  if(isnan(I_MAX) || I_MAX ==-1){I_MAX=40;}
  if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=0;}
  if((CAN_ID < 0) || (CAN_ID > 127)){CAN_ID = 1;}			// 7-bit node ID - the upper 4 CAN ID bits are the opcode
  if((CAN_MASTER < 0) || (CAN_MASTER > 127)){CAN_MASTER = 0;}
  if(CAN_TIMEOUT==-1){CAN_TIMEOUT = 1000;}
  if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
  if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
  if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
  if(isnan(I_CAL)||I_CAL==-1){I_CAL = 5.0f;}
  if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 21.0f;}
  if(isnan(GR) || GR==-1){GR = 1.0f;}
  if(isnan(KT) || KT==-1){KT = 1.0f;}
  if(isnan(KP_MAX) || KP_MAX==-1){KP_MAX = 500.0f;}
  if(isnan(KD_MAX) || KD_MAX==-1){KD_MAX = 5.0f;}
  if(isnan(P_MAX)){P_MAX = 12.5f;}
  if(isnan(P_MIN)){P_MIN = -12.5f;}
  if(isnan(V_MAX)){V_MAX = 65.0f;}
  if(isnan(V_MIN)){V_MIN = -65.0f;}

  printf("\r\nFirmware Version Number: %.2f\r\n", VERSION_NUM);
  if(RCC->CSR & RCC_CSR_IWDGRSTF){
	  printf("!!! Last reset caused by the watchdog - control loop stopped running !!!\r\n");
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* Controller Setup */
  init_controller_params(&controller);

  /* calibration "encoder" zeroing */
  memset(&comm_encoder_cal.cal_position, 0, sizeof(EncoderStruct));

  /* commutation encoder setup */
  comm_encoder.m_zero = M_ZERO;
  comm_encoder.e_zero = E_ZERO;
  comm_encoder.ppairs = PPAIRS;

  if(EN_ENC_LINEARIZATION){memcpy(&comm_encoder.offset_lut, &ENCODER_LUT, sizeof(comm_encoder.offset_lut));}	// Copy the linearization lookup table
  else{memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));}
  ps_warmup(&comm_encoder, 100);			// clear the noisy data when the encoder first turns on

  //for(int i = 0; i<128; i++){printf("%d\r\n", comm_encoder.offset_lut[i]);}

  /* Turn on ADCs.  Injected conversions on all three are triggered by the
   * PWM timer update event (triple injected simultaneous mode) */
  HAL_ADCEx_InjectedStart(&hadc2);
  HAL_ADCEx_InjectedStart(&hadc3);
  HAL_ADCEx_InjectedStart(&hadc1);	// master last: arms the TIM1 TRGO trigger

  /* Turn on PWM.  The timer update event triggers current/voltage sampling,
   * so it must be running before zero_current() below can sample anything */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  controller.dtc_u = 0.0f;
  controller.dtc_v = 0.0f;
  controller.dtc_w = 0.0f;
  set_dtc(&controller);				// command all phases low before the gate driver wakes up

  /* DRV8323 setup */
  HAL_GPIO_WritePin(DRV_CS, GPIO_PIN_SET ); 	// CS high
  HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_SET );
  HAL_Delay(1);
  //drv_calibrate(drv);
  HAL_Delay(1);
  drv_write_DCR(drv, 0x0, DIS_GDF_EN, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
  HAL_Delay(1);
  int CSA_GAIN;
  if(I_MAX <= 40.0f){CSA_GAIN = CSA_GAIN_40;}	// Up to 40A use 40X amplifier gain
  else{CSA_GAIN = CSA_GAIN_20;}					// From 40-60A use 20X amplifier gain.  (Make this generic in the future)
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x1, 0x1, 0x1, SEN_LVL_0_25);
  HAL_Delay(1);
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN, 0x1, 0x0, 0x0, 0x0, SEN_LVL_0_25);
  HAL_Delay(1);
  zero_current(&controller);
  HAL_Delay(1);
  drv_write_OCPCR(drv, TRETRY_50US, DEADTIME_50NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_0_45);
  HAL_Delay(1);
  drv_disable_gd(drv);
  HAL_Delay(1);
  printf("ADC A OFFSET: %d     ADC B OFFSET: %d\r\n", controller.adc_a_offset, controller.adc_b_offset);

  /* CAN setup */
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);
  HAL_CAN_Start(&CAN_H); //start CAN.  RX is polled from the commutation interrupt

  /* Set Interrupt Priorities */
  HAL_NVIC_SetPriority(PWM_ISR, 0x0,0x0); // commutation > communication

  /* Start the FSM */
  state.state = MENU_MODE;
  state.next_state = MENU_MODE;
  state.ready = 1;
  state.print_pending = 1;	// print the menu on power-up


  /* Turn on interrupts */
  HAL_UART_Receive_IT(&huart2, (uint8_t *)Serial2RxBuffer, 1);
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int main_loop_count = 0;
  while (1)
  {
	  HAL_Delay(1);

	  /* Slow state machine work: printing, flash saves.  All of it was moved
	   * out of the interrupts so it can't stall commutation */
	  run_fsm_slow(&state);

	  /* Check the DRV8323 fault registers every ~100 ms.  The SPI reads happen
	   * with the commutation interrupt masked, so they can't collide with the
	   * gate enable/disable SPI writes done on state transitions.  Printing
	   * happens after the interrupt is re-enabled, since it's slow. */
	  if(++main_loop_count >= 100){
		  main_loop_count = 0;
		  HAL_NVIC_DisableIRQ(PWM_ISR);
		  uint16_t drv_fsr1 = drv_read_FSR1(drv);
		  uint16_t drv_fsr2 = drv_read_FSR2(drv);
		  HAL_NVIC_EnableIRQ(PWM_ISR);
		  drv_print_fault_codes(drv_fsr1, drv_fsr2);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /* Get the power stage into a safe state, then wait.  If the watchdog is
   * running it will reset the chip; if this happens during init (before the
   * watchdog starts), hang here with the gate drive off. */
  __disable_irq();
  fault_shutdown();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
