/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include "hw_config.h"
#include "user_config.h"
#include "math_ops.h"
#include "structs.h"
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN RX is polled from the commutation interrupt (can_tx_rx), not interrupt-driven,
     * so the CAN1_RX0 interrupt is left disabled */
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void can_rx_init(CANRxMessage *msg){
	msg->filter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 	// set fifo assignment
	msg->filter.FilterIdHigh=CAN_ID<<5; 				// standard CAN ID, in FxR1[15:5]
	msg->filter.FilterIdLow=0x0;						// IDE = 0 (standard frame), RTR = 0 (data frame)
	msg->filter.FilterMaskIdHigh=0x7F<<5;				// match the 7 node ID bits, accept any of the 16 opcode blocks
	msg->filter.FilterMaskIdLow=0x6;					// match the IDE and RTR bits
	msg->filter.FilterMode = CAN_FILTERMODE_IDMASK;
	msg->filter.FilterScale=CAN_FILTERSCALE_32BIT;
	msg->filter.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&CAN_H, &msg->filter);
}

void can_tx_init(CANTxMessage *msg){
	msg->tx_header.DLC = 7; 			// overwritten by each pack function
	msg->tx_header.IDE=CAN_ID_STD; 		// set identifier to standard
	msg->tx_header.RTR=CAN_RTR_DATA; 	// data frame
	msg->tx_header.StdId = CAN_MASTER;  // overwritten by each pack function
	msg->reply_var_a = CAN_VAR_POSITION;	// default telemetry reply until CAN_OP_SET_REPLY_FORMAT changes it
	msg->reply_var_b = CAN_VAR_VELOCITY;
}

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t, float vb){
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t, -(I_MAX+SENSE_BUFFER)*KT*GR, (I_MAX+SENSE_BUFFER)*KT*GR, 12);
    int vb_int = float_to_uint(vb, VB_MIN, VB_MAX, 8);
    msg->tx_header.StdId = CAN_MASTER;
    msg->tx_header.DLC = 7;
    msg->data[0] = id;
    msg->data[1] = p_int>>8;
    msg->data[2] = p_int&0xFF;
    msg->data[3] = v_int>>4;
    msg->data[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg->data[5] = t_int&0xFF;
    msg->data[6] = vb_int;
    }

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void unpack_cmd(CANRxMessage msg, float *commands){// ControllerStruct * controller){
        int p_int = (msg.data[0]<<8)|msg.data[1];
        int v_int = (msg.data[2]<<4)|(msg.data[3]>>4);
        int kp_int = ((msg.data[3]&0xF)<<8)|msg.data[4];
        int kd_int = (msg.data[5]<<4)|(msg.data[6]>>4);
        int t_int = ((msg.data[6]&0xF)<<8)|msg.data[7];

        commands[0] = uint_to_float(p_int, P_MIN, P_MAX, 16);
        commands[1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
        commands[2] = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
        commands[3] = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
        commands[4] = uint_to_float(t_int, -I_MAX*KT*GR, I_MAX*KT*GR, 12);
    //printf("Received   ");
    //printf("%.3f  %.3f  %.3f  %.3f  %.3f   %.3f", controller->p_des, controller->v_des, controller->kp, controller->kd, controller->t_ff, controller->i_q_ref);
    //printf("\n\r");
    }

/// Float Variable Read Reply ///
/// Response to a CAN_OP_READ_VAR request
/// 0:   [CAN ID]
/// 1:   [variable ID]
/// 2-5: [float32, little-endian]
void pack_float_reply(CANTxMessage *msg, uint8_t var_id, float value){
    msg->tx_header.StdId = CAN_MASTER;
    msg->tx_header.DLC = 6;
    msg->data[0] = CAN_ID;
    msg->data[1] = var_id;
    memcpy(&msg->data[2], &value, sizeof(float));	// Cortex-M4 is little-endian
    }

/// Float Variable Pair Read Reply ///
/// Response to a 3-byte CAN_OP_READ_VAR request: two variables in one frame.
/// 0-3: [float32_a, little-endian]
/// 4-7: [float32_b, little-endian]
/// No room for the CAN_ID/var_id echo - the master matches replies to requests by order
void pack_float_pair_reply(CANTxMessage *msg, float value_a, float value_b){
    msg->tx_header.StdId = CAN_MASTER;
    msg->tx_header.DLC = 8;
    memcpy(&msg->data[0], &value_a, sizeof(float));
    memcpy(&msg->data[4], &value_b, sizeof(float));
    }

/// Telemetry Reply ///
/// Reply to the ID-opcode protocol (opcodes 1-5): two variables as float32.
/// Sent with ID = (CAN_OP_TELEMETRY<<7) | CAN_ID, so the master identifies
/// the sender from the ID bits and the full payload carries data.
/// 0-3: [float32 var_a, little-endian]
/// 4-7: [float32 var_b, little-endian]
void pack_telemetry_reply(CANTxMessage *msg, uint8_t var_a, uint8_t var_b){
    float value_a = can_read_variable(var_a);
    float value_b = can_read_variable(var_b);
    msg->tx_header.StdId = (CAN_OP_TELEMETRY<<7) | CAN_ID;
    msg->tx_header.DLC = 8;
    memcpy(&msg->data[0], &value_a, sizeof(float));
    memcpy(&msg->data[4], &value_b, sizeof(float));
    }

/// Unpack two float32s from a command frame ///
/// Returns 0 (and leaves the outputs unusable) if either is NaN,
/// so garbage frames can't inject NaN into the control loop
int can_unpack_two_floats(CANRxMessage *msg, float *a, float *b){
    memcpy(a, &msg->data[0], sizeof(float));
    memcpy(b, &msg->data[4], sizeof(float));
    return !(isnan(*a) || isnan(*b));
    }

/// Look up a variable by ID for the float read protocol ///
/// Called from the commutation interrupt, so everything here must be fast
float can_read_variable(uint8_t var_id){
    switch(var_id){
        case CAN_VAR_POSITION:		return comm_encoder.angle_multiturn/GR;
        case CAN_VAR_VELOCITY:		return comm_encoder.velocity/GR;
        case CAN_VAR_I_Q:			return controller.i_q_filt;
        case CAN_VAR_I_D:			return controller.i_d_filt;
        case CAN_VAR_TORQUE:		return controller.i_q_filt*KT*GR;
        case CAN_VAR_V_BUS:			return controller.v_bus_filt;
        case CAN_VAR_V_Q:			return controller.v_q;
        case CAN_VAR_V_D:			return controller.v_d;
        case CAN_VAR_I_A:			return controller.i_a;
        case CAN_VAR_I_B:			return controller.i_b;
        case CAN_VAR_I_C:			return controller.i_c;
        case CAN_VAR_THETA_ELEC:	return comm_encoder.elec_angle;
        case CAN_VAR_FW_INT:		return controller.fw_int;
        case CAN_VAR_FAULT:			return (float)controller.fault;

        /* Read-only config and status.  All plain register/struct loads, so this
         * costs the interrupt nothing beyond the wider jump table */
        case CAN_VAR_P_MAX:			return P_MAX;
        case CAN_VAR_V_MAX:			return V_MAX;
        case CAN_VAR_KP_MAX:		return KP_MAX;
        case CAN_VAR_KD_MAX:		return KD_MAX;
        case CAN_VAR_I_MAX:			return I_MAX;
        case CAN_VAR_KT:			return KT;
        case CAN_VAR_GR:			return GR;
        case CAN_VAR_CAN_TIMEOUT:	return (float)CAN_TIMEOUT;
        case CAN_VAR_VERSION:		return VERSION_NUM;
        case CAN_VAR_MODE:			return (float)state.state;

        /* Bus health, read straight from the error status register */
        case CAN_VAR_CAN_TEC:		return (float)((CAN_H.Instance->ESR >> 16) & 0xFF);
        case CAN_VAR_CAN_REC:		return (float)((CAN_H.Instance->ESR >> 24) & 0xFF);
        case CAN_VAR_CAN_LEC:		return (float)((CAN_H.Instance->ESR >> 4) & 0x7);

        default:					return NAN;		// unknown ID reads back NaN
        }
    }

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
