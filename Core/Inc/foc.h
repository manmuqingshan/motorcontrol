/*
 * foc.h
 *
 *  Created on: Aug 2, 2020
 *      Author: ben
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#include <stdint.h>
#include "position_sensor.h"

/* Latched fault bits.  Any nonzero controller.fault forces the FSM into
 * FAULT_MODE (gates off) until explicitly cleared by exiting to the menu */
#define FAULT_OV		(1<<0)		// bus overvoltage
#define FAULT_OC		(1<<1)		// phase overcurrent


typedef struct{
    int adc_a_raw, adc_b_raw, adc_vbus_raw;                 // Raw ADC Values
    float i_a, i_b, i_c;                                    // Phase currents
    float v_bus, v_bus_filt;                                // DC link voltage
    float theta_mech, theta_elec;                           // Rotor mechanical and electrical angle
    float dtheta_mech, dtheta_elec;                         // Rotor mechanical and electrical angular velocity
    float i_d, i_q, i_q_filt, i_d_filt;                     // D/Q currents
    float v_d, v_q;                                         // D/Q voltages
    float dtc_u, dtc_v, dtc_w;                              // Terminal duty cycles
    float v_u, v_v, v_w;                                    // Terminal voltages
    float i_scale, k_d, k_q, ki_d, ki_q, ki_fw;             // Current loop gains
    float flux_linkage;
    float d_int, q_int;                                     // Current error integrals
    int adc_a_offset, adc_b_offset;                         // ADC offsets
    float i_d_des, i_q_des, t_ff_filt;                      // Current references
    int loop_count;                                         // Degubbing counter
    int timeout;                                            // Watchdog counter
    union{
    	float commands[5];									// Making this easier to pass around without including foc.h everywhere
    	struct{
    		float p_des, v_des, kp, kd, t_ff;                   // Desired position, velocity, gains, torque
    	};
    };
    float v_max, v_ref, fw_int;                             // output voltage magnitude, field-weakening integral
    int otw_flag;                                           // Over-temp warning
    float i_max;											// Maximum current
    uint32_t fault;                                         // Latched fault bits (FAULT_x)
    float i_oc_limit;                                       // Overcurrent fault threshold
    float fault_vbus, fault_iabs;                           // Values captured when a fault first latched
    } ControllerStruct;

typedef struct{
    double temperature;                                     // Estimated temperature
    float temp_measured;									// "Measured" temperature computed from resistance
    float qd_in, qd_out;									// Thermal power in and out
    float resistance;										// Motor resistance
    float k;												// Temperature observer gain
    float trust;											// Temperature observer "trust' (kind of like 1/covariance)
    float delta_t;											// Temperature rise
    }   ObserverStruct;

void set_dtc(ControllerStruct *controller);
void analog_sample(ControllerStruct *controller);
void abc(float theta, float d, float q, float *a, float *b, float *c);
void dq0(float theta, float a, float b, float c, float *d, float *q);
void svm(float v_max, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w);
void zero_current(ControllerStruct *controller);
void reset_foc(ControllerStruct *controller);
void reset_observer(ObserverStruct *observer);
void init_controller_params(ControllerStruct *controller);
void commutate(ControllerStruct *controller, EncoderStruct *encoder);
void torque_control(ControllerStruct *controller);
void check_faults(ControllerStruct *controller);
void update_observer(ControllerStruct *controller, ObserverStruct *observer);
void field_weaken(ControllerStruct *controller);
void zero_commands(ControllerStruct * controller);


#endif /* INC_FOC_H_ */
