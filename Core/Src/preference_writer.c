/*
 * preference_writer.c
 *
 *  Created on: Apr 13, 2020
 *      Author: ben
 */

#include <string.h>
#include "preference_writer.h"
#include "user_config.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_flash.h"

/* The two 128 KB sectors at the top of the STM32F446RE's 512 KB flash */
#define N_SECTORS			2
static const uint32_t sector_addr[N_SECTORS] = {0x08040000, 0x08060000};	// sectors 6 and 7
static const uint16_t sector_id[N_SECTORS]   = {FLASH_Sector_6, FLASH_Sector_7};

#define CONFIG_MAGIC		0x434F4E46	// 'CONF'
#define CONFIG_VERSION		1
#define N_INT_REG			256
#define N_FLOAT_REG			64

/* Word offsets within a stored block */
#define OFFS_MAGIC			0
#define OFFS_VERSION		1
#define OFFS_SEQUENCE		2
#define OFFS_CRC			3
#define OFFS_PAYLOAD		4
#define N_PAYLOAD_WORDS		(N_INT_REG + N_FLOAT_REG)

/* Hardware CRC unit (fixed CRC-32 polynomial, word-wise) */
static void crc_begin(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
	CRC->CR = CRC_CR_RESET;
}

static uint32_t float_bits(float f){
	uint32_t u;
	memcpy(&u, &f, sizeof(u));
	return u;
}

static uint32_t crc_of_ram_config(uint32_t sequence){
	crc_begin();
	CRC->DR = sequence;
	for(int i = 0; i < N_INT_REG; i++){CRC->DR = (uint32_t)__int_reg[i];}
	for(int i = 0; i < N_FLOAT_REG; i++){CRC->DR = float_bits(__float_reg[i]);}
	return CRC->DR;
}

static int block_valid(int sector, uint32_t *sequence){
	const uint32_t *b = (const uint32_t *)sector_addr[sector];
	if(b[OFFS_MAGIC] != CONFIG_MAGIC){return 0;}
	if(b[OFFS_VERSION] != CONFIG_VERSION){return 0;}
	crc_begin();
	CRC->DR = b[OFFS_SEQUENCE];
	for(int i = 0; i < N_PAYLOAD_WORDS; i++){CRC->DR = b[OFFS_PAYLOAD + i];}
	if(CRC->DR != b[OFFS_CRC]){return 0;}
	*sequence = b[OFFS_SEQUENCE];
	return 1;
}

void preference_writer_load(PreferenceWriter *pr){
	uint32_t seq[N_SECTORS];
	int valid[N_SECTORS];
	for(int s = 0; s < N_SECTORS; s++){valid[s] = block_valid(s, &seq[s]);}

	int src = -1;
	if(valid[0] && valid[1]){src = (seq[1] > seq[0]) ? 1 : 0;}	// newest block wins
	else if(valid[0]){src = 0;}
	else if(valid[1]){src = 1;}

	if(src >= 0){
		const uint32_t *b = (const uint32_t *)sector_addr[src];
		for(int i = 0; i < N_INT_REG; i++){__int_reg[i] = (int)b[OFFS_PAYLOAD + i];}
		memcpy(__float_reg, (const void *)&b[OFFS_PAYLOAD + N_INT_REG], N_FLOAT_REG*sizeof(float));
		pr->active_sector = src;
		pr->sequence = seq[src];
		return;
	}

	/* No valid block.  If there's a legacy (pre-CRC) config in the first
	 * sector - raw registers starting at the sector base, written by firmware
	 * from before this format - import it so calibration survives the
	 * firmware upgrade.  It isn't re-saved here; the first save writes the
	 * new format to the OTHER sector, so the legacy data stays intact as a
	 * fallback until the save after that. */
	const uint32_t *legacy = (const uint32_t *)sector_addr[0];
	if(legacy[0] != 0xFFFFFFFF){
		for(int i = 0; i < N_INT_REG; i++){__int_reg[i] = (int)legacy[i];}
		memcpy(__float_reg, (const void *)&legacy[N_INT_REG], N_FLOAT_REG*sizeof(float));
	}
	/* Otherwise the chip is erased: the boot-time sanitization in main()
	 * supplies defaults */
	pr->active_sector = 0;		// so the first save targets sector 1
	pr->sequence = 0;
}

int preference_writer_save(PreferenceWriter *pr){
	/* Write the RAM config to the sector NOT holding the newest valid block.
	 * The sector erase stalls the whole CPU for up to ~2 s - the caller
	 * (save_preferences in fsm.c) kicks the watchdog first and only runs
	 * from the main loop, in states where the gate drive is off. */
	int target = !pr->active_sector;
	uint32_t base = sector_addr[target];
	uint32_t sequence = pr->sequence + 1;

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	if(FLASH_EraseSector(sector_id[target], VoltageRange_3) != FLASH_COMPLETE2){
		FLASH_Lock();
		return 0;
	}
	FLASH_ProgramWord(base + 4*OFFS_MAGIC, CONFIG_MAGIC);
	FLASH_ProgramWord(base + 4*OFFS_VERSION, CONFIG_VERSION);
	FLASH_ProgramWord(base + 4*OFFS_SEQUENCE, sequence);
	FLASH_ProgramWord(base + 4*OFFS_CRC, crc_of_ram_config(sequence));
	for(int i = 0; i < N_INT_REG; i++){FLASH_ProgramWord(base + 4*(OFFS_PAYLOAD + i), (uint32_t)__int_reg[i]);}
	for(int i = 0; i < N_FLOAT_REG; i++){FLASH_ProgramWord(base + 4*(OFFS_PAYLOAD + N_INT_REG + i), float_bits(__float_reg[i]));}
	FLASH_Lock();

	/* Verify what actually landed in flash before pointing at it */
	uint32_t check_seq;
	if(!block_valid(target, &check_seq) || (check_seq != sequence)){return 0;}
	pr->active_sector = target;
	pr->sequence = sequence;
	return 1;
}
