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

/* ------------------------------------------------------------------------
 * CAN protocol
 *
 * The 11-bit CAN ID is split into a 4-bit opcode and a 7-bit node ID:
 *
 *     ID = (opcode << 7) | node_id          node_id = CAN_ID, 0-127
 *
 * Opcode 0 is the legacy protocol, so plain-ID frames from old masters work
 * unchanged.  Lower opcodes win bus arbitration, so commands beat telemetry.
 * All floats are float32, little-endian.  Frames with unknown opcodes or
 * lengths, or NaN command values, are ignored.
 *
 * op  name              dir     data                            notes
 * 0x0 LEGACY            M->S    8-byte packed command           replies with 7-byte packed telemetry to CAN_MASTER
 *                               [FF x7, FC/FD/FE]               enter motor mode / menu mode / zero
 *                               [CAN_LEGACY_READ_VAR, id]       single float read, reply [CAN_ID, id, f32] to CAN_MASTER
 *                               [CAN_LEGACY_READ_VAR, a, b]     pair float read, reply [f32, f32] to CAN_MASTER
 * 0x1 SET_PV            M->S    [p_des f32][v_des f32]          clamped to P/V limits; resets timeout; telemetry reply
 * 0x2 SET_GAINS         M->S    [kp f32][kd f32]                clamped to 0..KP_MAX/KD_MAX; resets timeout; telemetry reply
 * 0x3 SET_TFF           M->S    [t_ff f32]                      clamped to +/-I_MAX*KT*GR; resets timeout; telemetry reply
 * 0x4 SET_REPLY_FORMAT  M->S    [var_id_a][var_id_b]            sets telemetry reply content (RAM, not saved); telemetry reply
 * 0x5 READ_VARS         M->S    [var_id_a][var_id_b] or [var_id] one-shot read; telemetry reply; does NOT reset timeout
 * 0x6 CONTROL           M->S    [CAN_CTRL_x]                    enter motor mode / menu mode / zero
 * 0xF TELEMETRY         S->M    [f32 a][f32 b]                  reply to opcodes 1-5, sent with ID = (0xF<<7)|CAN_ID,
 *                                                               so the sender is identified by the ID bits
 * ------------------------------------------------------------------------ */
#define CAN_OPCODE(stdid)		((stdid) >> 7)
#define CAN_NODE_ID(stdid)		((stdid) & 0x7F)

#define CAN_OP_LEGACY			0x0
#define CAN_OP_SET_PV			0x1
#define CAN_OP_SET_GAINS		0x2
#define CAN_OP_SET_TFF			0x3
#define CAN_OP_SET_REPLY_FORMAT	0x4
#define CAN_OP_READ_VARS		0x5
#define CAN_OP_CONTROL			0x6
#define CAN_OP_TELEMETRY		0xF

/* CAN_OP_CONTROL command bytes */
#define CAN_CTRL_MOTOR_MODE		1
#define CAN_CTRL_MENU_MODE		2
#define CAN_CTRL_ZERO			3

/* In-payload opcode for float reads within the legacy protocol (any legacy
 * frame shorter than 8 bytes; kept for compatibility with READ_VARS masters
 * from before the ID-opcode scheme) */
#define CAN_LEGACY_READ_VAR		0x01

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
#define CAN_VAR_FAULT		13		// latched fault bits (FAULT_x in foc.h), as a float.  0.0 = no fault

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
	uint8_t reply_var_a, reply_var_b;	// telemetry reply content, set by CAN_OP_SET_REPLY_FORMAT
	CAN_TxHeaderTypeDef tx_header;
}CANTxMessage ;

void can_rx_init(CANRxMessage *msg);
void can_tx_init(CANTxMessage *msg);
void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t, float vb);
void unpack_cmd(CANRxMessage msg, float *commands);
void pack_float_reply(CANTxMessage *msg, uint8_t var_id, float value);
void pack_float_pair_reply(CANTxMessage *msg, float value_a, float value_b);
void pack_telemetry_reply(CANTxMessage *msg, uint8_t var_a, uint8_t var_b);
int can_unpack_two_floats(CANRxMessage *msg, float *a, float *b);
float can_read_variable(uint8_t var_id);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
