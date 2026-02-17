/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kea_ftmre_flash_controller

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>

#include "kea_regs.h"

#define KEA_FTMRE_FLASH_CLOCK_HZ 1000000u
#define KEA_FTMRE_CMD_WAIT_LOOPS 1000000u
#define KEA_FTMRE_CMD_PROGRAM_LONGWORD 0x06u
#define KEA_FTMRE_CMD_ERASE_SECTOR 0x0Au

#define KEA_FLASH_NODE DT_CHOSEN(zephyr_flash)

#if !DT_NODE_EXISTS(KEA_FLASH_NODE)
#error "KEA FTMRE flash driver requires chosen zephyr,flash"
#endif

struct kea_ftmre_flash_cfg {
	kea_ftmre_t *base;
	uintptr_t flash_base;
	size_t flash_size;
	size_t erase_block_size;
	size_t write_block_size;
};

struct kea_ftmre_flash_data {
	struct k_sem lock;
};

static bool kea_ftmre_is_range_valid(const struct kea_ftmre_flash_cfg *cfg, off_t offset, size_t len)
{
	uint64_t end;

	if (offset < 0) {
		return false;
	}

	end = (uint64_t)offset + len;
	if (end > cfg->flash_size) {
		return false;
	}

	return true;
}

static int kea_ftmre_wait_ready(kea_ftmre_t *base)
{
	uint32_t timeout = KEA_FTMRE_CMD_WAIT_LOOPS;

	while (((base->FSTAT & KEA_FTMRE_FSTAT_CCIF_MASK) == 0u) && (timeout > 0u)) {
		timeout--;
	}

	if (timeout == 0u) {
		return -ETIMEDOUT;
	}

	return 0;
}

static void kea_ftmre_clear_errors(kea_ftmre_t *base)
{
	base->FSTAT = KEA_FTMRE_FSTAT_ACCERR_MASK | KEA_FTMRE_FSTAT_FPVIOL_MASK;
}

static __ramfunc int kea_ftmre_launch_command(kea_ftmre_t *base)
{
	uint32_t timeout = KEA_FTMRE_CMD_WAIT_LOOPS;
	uint8_t status;

	/* Write 1 to CCIF launches the command currently loaded in FCCOB. */
	base->FSTAT = KEA_FTMRE_FSTAT_CCIF_MASK;

	while (((base->FSTAT & KEA_FTMRE_FSTAT_CCIF_MASK) == 0u) && (timeout > 0u)) {
		timeout--;
	}

	if (timeout == 0u) {
		return -ETIMEDOUT;
	}

	status = base->FSTAT;
	if ((status & (KEA_FTMRE_FSTAT_ACCERR_MASK | KEA_FTMRE_FSTAT_FPVIOL_MASK)) != 0u) {
		return -EIO;
	}

	if ((status & KEA_FTMRE_FSTAT_MGSTAT_MASK) != 0u) {
		return -EIO;
	}

	return 0;
}

static int kea_ftmre_execute_command(kea_ftmre_t *base)
{
	unsigned int key;
	int ret;

	key = irq_lock();
	ret = kea_ftmre_launch_command(base);
	irq_unlock(key);

	return ret;
}

static void kea_ftmre_load_addr_cmd(kea_ftmre_t *base, uint8_t cmd, uint32_t addr)
{
	base->FCCOBIX = 0u;
	base->FCCOBHI = cmd;
	base->FCCOBLO = (uint8_t)((addr >> 16) & 0xFFu);

	base->FCCOBIX = 1u;
	base->FCCOBHI = (uint8_t)((addr >> 8) & 0xFFu);
	base->FCCOBLO = (uint8_t)(addr & 0xFFu);
}

static int kea_ftmre_program_longword(kea_ftmre_t *base, uint32_t addr, uint32_t data)
{
	int ret;

	ret = kea_ftmre_wait_ready(base);
	if (ret != 0) {
		return ret;
	}

	kea_ftmre_clear_errors(base);
	kea_ftmre_load_addr_cmd(base, KEA_FTMRE_CMD_PROGRAM_LONGWORD, addr);

	base->FCCOBIX = 2u;
	base->FCCOBHI = (uint8_t)((data >> 8) & 0xFFu);
	base->FCCOBLO = (uint8_t)(data & 0xFFu);

	base->FCCOBIX = 3u;
	base->FCCOBHI = (uint8_t)((data >> 24) & 0xFFu);
	base->FCCOBLO = (uint8_t)((data >> 16) & 0xFFu);

	return kea_ftmre_execute_command(base);
}

static int kea_ftmre_erase_sector(kea_ftmre_t *base, uint32_t addr)
{
	int ret;

	ret = kea_ftmre_wait_ready(base);
	if (ret != 0) {
		return ret;
	}

	kea_ftmre_clear_errors(base);
	kea_ftmre_load_addr_cmd(base, KEA_FTMRE_CMD_ERASE_SECTOR, addr);

	return kea_ftmre_execute_command(base);
}

static int kea_ftmre_flash_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct kea_ftmre_flash_cfg *cfg = dev->config;

	if (len == 0u) {
		return 0;
	}

	if ((data == NULL) || !kea_ftmre_is_range_valid(cfg, offset, len)) {
		return -EINVAL;
	}

	memcpy(data, (const void *)(cfg->flash_base + (uintptr_t)offset), len);

	return 0;
}

static int kea_ftmre_flash_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	const struct kea_ftmre_flash_cfg *cfg = dev->config;
	struct kea_ftmre_flash_data *drv_data = dev->data;
	const uint8_t *src = data;
	uint32_t addr;
	int ret = 0;

	if (len == 0u) {
		return 0;
	}

	if ((data == NULL) || !kea_ftmre_is_range_valid(cfg, offset, len)) {
		return -EINVAL;
	}

	if (((offset % cfg->write_block_size) != 0u) || ((len % cfg->write_block_size) != 0u)) {
		return -EINVAL;
	}

	ret = k_sem_take(&drv_data->lock, K_FOREVER);
	if (ret != 0) {
		return ret;
	}

	addr = (uint32_t)(cfg->flash_base + (uintptr_t)offset);

	while (len > 0u) {
		uint32_t word = ((uint32_t)src[0]) |
				((uint32_t)src[1] << 8) |
				((uint32_t)src[2] << 16) |
				((uint32_t)src[3] << 24);

		ret = kea_ftmre_program_longword(cfg->base, addr, word);
		if (ret != 0) {
			break;
		}

		addr += (uint32_t)cfg->write_block_size;
		src += cfg->write_block_size;
		len -= cfg->write_block_size;
	}

	k_sem_give(&drv_data->lock);

	return ret;
}

static int kea_ftmre_flash_erase(const struct device *dev, off_t offset, size_t size)
{
	const struct kea_ftmre_flash_cfg *cfg = dev->config;
	struct kea_ftmre_flash_data *drv_data = dev->data;
	uint32_t addr;
	int ret = 0;

	if (size == 0u) {
		return 0;
	}

	if (!kea_ftmre_is_range_valid(cfg, offset, size)) {
		return -EINVAL;
	}

	if (((offset % cfg->erase_block_size) != 0u) || ((size % cfg->erase_block_size) != 0u)) {
		return -EINVAL;
	}

	ret = k_sem_take(&drv_data->lock, K_FOREVER);
	if (ret != 0) {
		return ret;
	}

	addr = (uint32_t)(cfg->flash_base + (uintptr_t)offset);

	while (size > 0u) {
		ret = kea_ftmre_erase_sector(cfg->base, addr);
		if (ret != 0) {
			break;
		}

		addr += (uint32_t)cfg->erase_block_size;
		size -= cfg->erase_block_size;
	}

	k_sem_give(&drv_data->lock);

	return ret;
}

static int kea_ftmre_flash_get_size(const struct device *dev, uint64_t *size)
{
	const struct kea_ftmre_flash_cfg *cfg = dev->config;

	if (size == NULL) {
		return -EINVAL;
	}

	*size = cfg->flash_size;

	return 0;
}

static const struct flash_parameters *kea_ftmre_flash_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	static const struct flash_parameters params = {
		.write_block_size = DT_PROP(KEA_FLASH_NODE, write_block_size),
		.erase_value = 0xFFu,
	};

	return &params;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void kea_ftmre_flash_page_layout(const struct device *dev,
					const struct flash_pages_layout **layout,
					size_t *layout_size)
{
	ARG_UNUSED(dev);

	static const struct flash_pages_layout flash_layout = {
		.pages_count = DT_REG_SIZE(KEA_FLASH_NODE) / DT_PROP(KEA_FLASH_NODE, erase_block_size),
		.pages_size = DT_PROP(KEA_FLASH_NODE, erase_block_size),
	};

	*layout = &flash_layout;
	*layout_size = 1u;
}
#endif

static int kea_ftmre_flash_init(const struct device *dev)
{
	const struct kea_ftmre_flash_cfg *cfg = dev->config;
	struct kea_ftmre_flash_data *drv_data = dev->data;
	uint32_t fdiv;
	int ret;

	k_sem_init(&drv_data->lock, 1, 1);

	/* Ensure flash controller clock gate is on. */
	KEA_SIM->SCGC |= KEA_SIM_SCGC_FLASH_MASK;

	/* Program divider once if it has not been loaded yet. */
	if ((cfg->base->FCLKDIV & KEA_FTMRE_FCLKDIV_FDIVLD_MASK) == 0u) {
		fdiv = (DT_PROP(DT_NODELABEL(simclk), clock_frequency) / KEA_FTMRE_FLASH_CLOCK_HZ);
		if (fdiv > 0u) {
			fdiv -= 1u;
		}
		if (fdiv > KEA_FTMRE_FCLKDIV_FDIV_MASK) {
			fdiv = KEA_FTMRE_FCLKDIV_FDIV_MASK;
		}

		cfg->base->FCLKDIV = (uint8_t)fdiv;
	}

	ret = kea_ftmre_wait_ready(cfg->base);
	if (ret != 0) {
		return ret;
	}

	kea_ftmre_clear_errors(cfg->base);

	return 0;
}

static DEVICE_API(flash, kea_ftmre_flash_api) = {
	.read = kea_ftmre_flash_read,
	.write = kea_ftmre_flash_write,
	.erase = kea_ftmre_flash_erase,
	.get_parameters = kea_ftmre_flash_get_parameters,
	.get_size = kea_ftmre_flash_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = kea_ftmre_flash_page_layout,
#endif
};

static struct kea_ftmre_flash_data kea_ftmre_flash_data_0;

static const struct kea_ftmre_flash_cfg kea_ftmre_flash_cfg_0 = {
	.base = (kea_ftmre_t *)DT_INST_REG_ADDR(0),
	.flash_base = DT_REG_ADDR(KEA_FLASH_NODE),
	.flash_size = DT_REG_SIZE(KEA_FLASH_NODE),
	.erase_block_size = DT_PROP(KEA_FLASH_NODE, erase_block_size),
	.write_block_size = DT_PROP(KEA_FLASH_NODE, write_block_size),
};

DEVICE_DT_INST_DEFINE(0, kea_ftmre_flash_init, NULL, &kea_ftmre_flash_data_0,
		      &kea_ftmre_flash_cfg_0, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &kea_ftmre_flash_api);
