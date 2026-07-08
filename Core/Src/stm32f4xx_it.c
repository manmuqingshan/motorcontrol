/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "structs.h"
#include "math_ops.h"
#include "usart.h"
#include "fsm.h"
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "foc.h"
#include "can.h"
#include "position_sensor.h"
#include "hw_config.h"
#include "user_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void fault_shutdown(void){
	/* Called from fault handlers and Error_Handler: get the power stage into a
	 * safe state using only register writes.  The DRV8323 enable pin is pulled
	 * low by GPIO (not SPI - the SPI bus may be wedged or the fault may have
	 * happened mid-transaction), which puts it to sleep and turns off all six
	 * FETs.  PWM outputs are also disabled at the timer.  The watchdog will
	 * eventually reset the chip. */
	HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_RESET);
	TIM1->BDTR &= ~TIM_BDTR_MOE;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  fault_shutdown();
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  fault_shutdown();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  fault_shutdown();
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  fault_shutdown();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  fault_shutdown();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	//HAL_GPIO_WritePin(LED, GPIO_PIN_SET );	// Useful for timing

	/* Kick the watchdog - if this loop ever stops running, the chip resets */
	IWDG->KR = 0xAAAA;

	/* Sample ADCs */
	analog_sample(&controller);

	/* Sample position sensor */
	ps_sample(&comm_encoder, DT);

	/* Run Finite State Machine */
	run_fsm(&state);

	/* Check for CAN messages */
	can_tx_rx();

	/* increment loop count */
	controller.loop_count++;
	//HAL_GPIO_WritePin(LED, GPIO_PIN_RESET );

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* Received bytes are handled in HAL_UART_RxCpltCallback (usart.c), so that only
   * real receive events update the FSM - not error interrupts replaying stale data */
  /* USER CODE END USART2_IRQn 1 */
}


/* USER CODE BEGIN 1 */

void can_tx_rx(void){

	if(HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data) != HAL_OK){
		return;		// no message pending
	}

	/* The 11-bit CAN ID is (opcode << 7) | node_id - see the table in can.h.
	 * The hardware filter only accepts this node's ID, so just dispatch on
	 * the opcode.  Malformed frames (wrong length, NaN floats) are ignored */
	uint32_t TxMailbox;
	uint32_t opcode = CAN_OPCODE(can_rx.rx_header.StdId);
	uint32_t dlc = can_rx.rx_header.DLC;
	int send_telemetry_reply = 0;
	float f_a, f_b;

	switch(opcode){

		case CAN_OP_LEGACY:
			if(dlc == 8){
				/* Legacy protocol: all 8-byte frames.  Reply with the packed telemetry */
				pack_reply(&can_tx, CAN_ID,  comm_encoder.angle_multiturn/GR, comm_encoder.velocity/GR, controller.i_q_filt*KT*GR, controller.v_bus_filt);	// Pack response
				HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

				/* Check for special Commands: bytes 0-6 all 0xFF, command in byte 7 */
				int special = (can_rx.data[0] & can_rx.data[1] & can_rx.data[2] & can_rx.data[3] & can_rx.data[4] & can_rx.data[5] & can_rx.data[6]) == 0xFF;
				if(special && (can_rx.data[7] == 0xFC)){
					update_fsm(&state, MOTOR_CMD);
				}
				else if(special && (can_rx.data[7] == 0xFD)){
					update_fsm(&state, MENU_CMD);
				}
				else if(special && (can_rx.data[7] == 0xFE)){
					update_fsm(&state, ZERO_CMD);
				}
				else{
					unpack_cmd(can_rx, controller.commands);	// Unpack commands
					controller.timeout = 0;						// Reset timeout counter
				}
			}
			else if((dlc == 2) && (can_rx.data[0] == CAN_LEGACY_READ_VAR)){
				/* Legacy single float read */
				pack_float_reply(&can_tx, can_rx.data[1], can_read_variable(can_rx.data[1]));
				HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);
			}
			else if((dlc == 3) && (can_rx.data[0] == CAN_LEGACY_READ_VAR)){
				/* Legacy float pair read */
				pack_float_pair_reply(&can_tx, can_read_variable(can_rx.data[1]), can_read_variable(can_rx.data[2]));
				HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);
			}
			break;

		case CAN_OP_SET_PV:
			if((dlc == 8) && can_unpack_two_floats(&can_rx, &f_a, &f_b)){
				controller.p_des = fast_fmaxf(fast_fminf(f_a, P_MAX), P_MIN);
				controller.v_des = fast_fmaxf(fast_fminf(f_b, V_MAX), V_MIN);
				controller.timeout = 0;
				send_telemetry_reply = 1;
			}
			break;

		case CAN_OP_SET_GAINS:
			if((dlc == 8) && can_unpack_two_floats(&can_rx, &f_a, &f_b)){
				controller.kp = fast_fmaxf(fast_fminf(f_a, KP_MAX), KP_MIN);
				controller.kd = fast_fmaxf(fast_fminf(f_b, KD_MAX), KD_MIN);
				controller.timeout = 0;
				send_telemetry_reply = 1;
			}
			break;

		case CAN_OP_SET_TFF:
			if(dlc == 4){
				memcpy(&f_a, &can_rx.data[0], sizeof(float));
				if(!isnan(f_a)){
					controller.t_ff = fast_fmaxf(fast_fminf(f_a, I_MAX*KT*GR), -I_MAX*KT*GR);
					controller.timeout = 0;
					send_telemetry_reply = 1;
				}
			}
			break;

		case CAN_OP_SET_REPLY_FORMAT:
			/* Select which two variables come back in telemetry replies.
			 * Stored in RAM - re-send after a power cycle.  Doesn't reset the
			 * command timeout.  The reply acts as an ack, in the new format */
			if(dlc == 2){
				can_tx.reply_var_a = can_rx.data[0];
				can_tx.reply_var_b = can_rx.data[1];
				send_telemetry_reply = 1;
			}
			break;

		case CAN_OP_READ_VARS:
			/* One-shot read of one or two variables.  Doesn't change the
			 * configured reply format, and doesn't reset the command timeout -
			 * telemetry polling shouldn't keep a stale command alive */
			if(dlc == 2){
				pack_telemetry_reply(&can_tx, can_rx.data[0], can_rx.data[1]);
				HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);
			}
			else if(dlc == 1){
				pack_telemetry_reply(&can_tx, can_rx.data[0], 0xFF);	// second slot reads back NaN
				HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);
			}
			break;

		case CAN_OP_CONTROL:
			if(dlc == 1){
				if(can_rx.data[0] == CAN_CTRL_MOTOR_MODE){update_fsm(&state, MOTOR_CMD);}
				else if(can_rx.data[0] == CAN_CTRL_MENU_MODE){update_fsm(&state, MENU_CMD);}
				else if(can_rx.data[0] == CAN_CTRL_ZERO){update_fsm(&state, ZERO_CMD);}
			}
			break;

		/* Unknown opcodes are ignored */
	}

	if(send_telemetry_reply){
		pack_telemetry_reply(&can_tx, can_tx.reply_var_a, can_tx.reply_var_b);
		HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);
	}

}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
