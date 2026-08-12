/*
 * preference_writer.h
 *
 *  Created on: Apr 13, 2020
 *      Author: ben
 */

#ifndef INC_PREFERENCE_WRITER_H_
#define INC_PREFERENCE_WRITER_H_

#include <stdint.h>

/* Flash-backed config storage.
 *
 * The user config registers (user_config.h: 256 ints + 64 floats) are stored
 * as a block with a magic number, layout version, sequence counter, and CRC,
 * ping-ponged between two flash sectors.  Each save erases and rewrites the
 * sector NOT holding the newest valid block, so a power loss during a save
 * can never lose the previous config.  At load time, the valid block with
 * the highest sequence number wins.  Legacy (pre-CRC) configs are imported
 * automatically on first boot after a firmware upgrade. */

typedef struct{
	uint32_t sequence;			// sequence number of the loaded / last-saved block
	uint32_t active_sector;		// 0 or 1: which sector holds the newest valid block
	int loaded;					// 1 if a config was found, 0 if the chip is erased.
								// The registers are zero-initialised, and zero is a
								// legal-looking value for most of them, so "erased"
								// cannot be detected from their contents alone
}PreferenceWriter;

void preference_writer_load(PreferenceWriter *pr);
int preference_writer_save(PreferenceWriter *pr);	// 1 on success, verified by CRC read-back

extern PreferenceWriter prefs;

#endif /* INC_PREFERENCE_WRITER_H_ */
