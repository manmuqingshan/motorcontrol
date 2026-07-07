/*
 * position_sensor.c
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */
#include <stdio.h>
#include <string.h>
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"

void ps_warmup(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
	for(int i = 0; i<n; i++){
		encoder->spi_tx_word = 0x0000;
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
		HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
		while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	}
}

void ps_sample(EncoderStruct * encoder, float dt){
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt */

	/* Shift around previous samples.  The buffer holds multi-turn positions in
	 * integer counts, so velocity resolution doesn't degrade as the float
	 * multi-turn angle gets large */
	encoder->old_angle = encoder->angle_singleturn;
	for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->count_buff[i] = encoder->count_buff[i-1];}

	/* SPI read/write */
	encoder->spi_tx_word = ENC_READ_WORD;
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
	HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
	while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	encoder->raw = encoder ->spi_rx_word;

	/* Linearization */
	int off_1 = encoder->offset_lut[(encoder->raw)>>9];				// lookup table lower entry
	int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];		// lookup table higher entry
	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
	encoder->count = encoder->raw + off_interp;


	/* Single-turn count relative to the zero position, wrapped to [0, ENC_CPR) */
	int count_wrapped = (encoder->count - M_ZERO) % ENC_CPR;
	if(count_wrapped < 0){count_wrapped += ENC_CPR;}
	encoder->angle_singleturn = TWO_PI_F*((float)count_wrapped)/((float)ENC_CPR);

	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
	int int_angle = (int)encoder->elec_angle;
	encoder->elec_angle = TWO_PI_F*(encoder->elec_angle - (float)int_angle);
	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers

	/* Rollover */
	int rollover = 0;
	float angle_diff = encoder->angle_singleturn - encoder->old_angle;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	encoder->turns += rollover;
	int first_sample = !encoder->first_sample;
	if(first_sample){
		/* On the first sample, pick the turn count so the multi-turn angle starts
		 * within (-3*pi/2, pi/2] of the zero position, like the old mbed firmware.
		 * angle_singleturn is in [0, 2*pi) here, so it can never be negative */
		encoder->turns = 0;
		if(encoder->angle_singleturn > PI_OVER_2_F){encoder->turns = -1;}
		encoder->first_sample = 1;
	}

	/* Multi-turn position, exact in integer counts (good for +/-32768 turns
	 * before overflowing), and in radians for the rest of the firmware */
	encoder->count_buff[0] = count_wrapped + ENC_CPR*encoder->turns;
	encoder->angle_multiturn = TWO_PI_F*((float)encoder->count_buff[0])/((float)ENC_CPR);
	if(first_sample){	// fill the buffer on the first sample so velocity starts at zero, not with a spike
		for(int i = 1; i<N_POS_SAMPLES; i++){encoder->count_buff[i] = encoder->count_buff[0];}
	}

	/* Velocity: finite difference of the integer counts.  Differencing counts
	 * instead of the float angles keeps velocity resolution independent of how
	 * far the multi-turn position is from zero */
	/* (Tried a moving least-squares fit here; it wasn't any better than the simple difference) */
	encoder->velocity = TWO_PI_F*((float)(encoder->count_buff[0] - encoder->count_buff[N_POS_SAMPLES-1]))
			/((float)ENC_CPR*dt*(float)(N_POS_SAMPLES-1));
	encoder->elec_velocity = encoder->ppairs*encoder->velocity;

}

void ps_print(EncoderStruct * encoder, int dt_ms){
	printf("Raw: %d", encoder->raw);
	printf("   Linearized Count: %d", encoder->count);
	printf("   Single Turn: %f", encoder->angle_singleturn);
	printf("   Multiturn: %f", encoder->angle_multiturn);
	printf("   Electrical: %f", encoder->elec_angle);
	printf("   Turns:  %d\r\n", encoder->turns);
	//HAL_Delay(dt_ms);
}
