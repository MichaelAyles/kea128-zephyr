/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/linker/sections.h>

/*
 * KEA non-volatile flash configuration field at 0x00000400.
 * Keep the device unsecured and unprotected for bring-up.
 */
uint8_t __used __kinetis_flash_config_section __kea_flash_config[] = {
	/* Backdoor comparison key (unused). */
	0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,

	/* Reserved bytes 0x408..0x40C. */
	0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,

	/* FPROT, FSEC, FOPT */
	0xFFu, /* Unprotect flash regions */
	0xFEu, /* SEC=0b10 (unsecured), KEYEN disabled */
	0xFFu,
};
