/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
//#define P_MIN -12.5f
//#define P_MAX 12.5f
//#define V_MIN -65.0f
//#define V_MAX 65.0f
#define KP_MIN 0.0f
//#define KP_MAX 500.0f
#define KD_MIN 0.0f
//#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
#define VB_MIN 0.0f
#define VB_MAX 40.0f
#define SENSE_BUFFER 0.0f

/* Extended protocol:
 * All legacy frames (commands, FF... special commands) are 8 bytes long, so
 * any frame shorter than 8 bytes is an extended-protocol frame, with the
 * first byte as an opcode.  Frames with unknown opcodes/lengths are ignored. */
#define CAN_OP_READ_VAR		0x01	// single: request [CAN_OP_READ_VAR, var_id], 2 bytes
									//         reply   [CAN_ID, var_id, float32 little-endian], 6 bytes
									// pair:   request [CAN_OP_READ_VAR, var_id_a, var_id_b], 3 bytes
									//         reply   [float32_a, float32_b, little-endian], 8 bytes
									// The pair reply has no room for the CAN_ID/var_id echo, so
									// don't interleave pair requests to multiple motors sharing
									// one master ID - the replies are told apart by request order

/* Variable IDs readable with CAN_OP_READ_VAR.  Unknown IDs read back NaN */
#define CAN_VAR_POSITION	0		// multiturn position (rad, output shaft)
#define CAN_VAR_VELOCITY	1		// velocity (rad/s, output shaft)
#define CAN_VAR_I_Q			2		// q-axis current (A, filtered)
#define CAN_VAR_I_D			3		// d-axis current (A, filtered)
#define CAN_VAR_TORQUE		4		// torque estimate (N-m, output shaft)
#define CAN_VAR_V_BUS		5		// bus voltage (V, filtered)
#define CAN_VAR_V_Q			6		// q-axis output voltage (V)
#define CAN_VAR_V_D			7		// d-axis output voltage (V)
#define CAN_VAR_I_A			8		// phase A current (A)
#define CAN_VAR_I_B			9		// phase B current (A)
#define CAN_VAR_I_C			10		// phase C current (A)
#define CAN_VAR_THETA_ELEC	11		// electrical angle (rad)
#define CAN_VAR_FW_INT		12		// field weakening current (A)

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct{
	uint8_t id;
	uint8_t data[8];
	CAN_RxHeaderTypeDef rx_header;
	CAN_FilterTypeDef filter;
}CANRxMessage ;

typedef struct{
	uint8_t id;
	uint8_t data[8];
	CAN_TxHeaderTypeDef tx_header;
}CANTxMessage ;

void can_rx_init(CANRxMessage *msg);
void can_tx_init(CANTxMessage *msg);
void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t, float vb);
void unpack_cmd(CANRxMessage msg, float *commands);
void pack_float_reply(CANTxMessage *msg, uint8_t var_id, float value);
void pack_float_pair_reply(CANTxMessage *msg, float value_a, float value_b);
float can_read_variable(uint8_t var_id);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
