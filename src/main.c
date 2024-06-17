/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_qspi.h>
// #include <drivers/flash/nrf_qspi_nor.h>
#if defined(CONFIG_BOARD_ADAFRUIT_FEATHER_STM32F405)
#define SPI_FLASH_TEST_REGION_OFFSET 0xf000
#elif defined(CONFIG_BOARD_ARTY_A7_DESIGNSTART_FPGA_CORTEX_M1) || \
	defined(CONFIG_BOARD_ARTY_A7_DESIGNSTART_FPGA_CORTEX_M3)
/* The FPGA bitstream is stored in the lower 536 sectors of the flash. */
#define SPI_FLASH_TEST_REGION_OFFSET \
	DT_REG_SIZE(DT_NODE_BY_FIXED_PARTITION_LABEL(fpga_bitstream))
#elif defined(CONFIG_BOARD_NPCX9M6F_EVB) || \
	defined(CONFIG_BOARD_NPCX7M6FB_EVB)
#define SPI_FLASH_TEST_REGION_OFFSET 0x7F000
#else
#define SPI_FLASH_TEST_REGION_OFFSET 0xff000
#endif
#define SPI_FLASH_SECTOR_SIZE        4096

#if defined(CONFIG_FLASH_STM32_OSPI) || defined(CONFIG_FLASH_STM32_QSPI)
#define SPI_FLASH_MULTI_SECTOR_TEST
#endif

void single_sector_test(const struct device *flash_dev)
{
	const uint8_t expected[] = { 0x55, 0xaa, 0x66, 0x99 };
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];
	int rc;

	printk("\nPerform test on single sector");
	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */
	printk("\nTest 1: Flash erase\n");

	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 */
	rc = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET,
			 SPI_FLASH_SECTOR_SIZE);
	if (rc != 0) {
		printk("Flash erase failed! %d\n", rc);
	} else {
		printk("Flash erase succeeded!\n");
	}

	printk("\nTest 2: Flash write\n");

	printk("Attempting to write %zu bytes\n", len);
	rc = flash_write(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, expected, len);
	if (rc != 0) {
		printk("Flash write failed! %d\n", rc);
		return;
	}

	memset(buf, 0, len);
	rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buf, len);
	if (rc != 0) {
		printk("Flash read failed! %d\n", rc);
		return;
	}

	if (memcmp(expected, buf, len) == 0) {
		printk("Data read matches data written. Good!!\n");
	} else {
		const uint8_t *wp = expected;
		const uint8_t *rp = buf;
		const uint8_t *rpe = rp + len;

		printk("Data read does not match data written!!\n");
		while (rp < rpe) {
			printk("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(SPI_FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			++rp;
			++wp;
		}
	}
}

#if defined SPI_FLASH_MULTI_SECTOR_TEST
void multi_sector_test(const struct device *flash_dev)
{
	const uint8_t expected[] = { 0x55, 0xaa, 0x66, 0x99 };
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];
	int rc;

	printk("\nPerform test on multiple consequtive sectors");

	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */
	printk("\nTest 1: Flash erase\n");

	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 * Erase 2 sectors for check for erase of consequtive sectors
	 */
	rc = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, SPI_FLASH_SECTOR_SIZE * 2);
	if (rc != 0) {
		printk("Flash erase failed! %d\n", rc);
	} else {
		/* Read the content and check for erased */
		memset(buf, 0, len);
		size_t offs = SPI_FLASH_TEST_REGION_OFFSET;

		while (offs < SPI_FLASH_TEST_REGION_OFFSET + 2 * SPI_FLASH_SECTOR_SIZE) {
			rc = flash_read(flash_dev, offs, buf, len);
			if (rc != 0) {
				printk("Flash read failed! %d\n", rc);
				return;
			}
			if (buf[0] != 0xff) {
				printk("Flash erase failed at offset 0x%x got 0x%x\n",
				offs, buf[0]);
				return;
			}
			offs += SPI_FLASH_SECTOR_SIZE;
		}
		printk("Flash erase succeeded!\n");
	}

	printk("\nTest 2: Flash write\n");

	size_t offs = SPI_FLASH_TEST_REGION_OFFSET;

	while (offs < SPI_FLASH_TEST_REGION_OFFSET + 2 * SPI_FLASH_SECTOR_SIZE) {
		printk("Attempting to write %zu bytes at offset 0x%x\n", len, offs);
		rc = flash_write(flash_dev, offs, expected, len);
		if (rc != 0) {
			printk("Flash write failed! %d\n", rc);
			return;
		}

		memset(buf, 0, len);
		rc = flash_read(flash_dev, offs, buf, len);
		if (rc != 0) {
			printk("Flash read failed! %d\n", rc);
			return;
		}

		if (memcmp(expected, buf, len) == 0) {
			printk("Data read matches data written. Good!!\n");
		} else {
			const uint8_t *wp = expected;
			const uint8_t *rp = buf;
			const uint8_t *rpe = rp + len;

			printk("Data read does not match data written!!\n");
			while (rp < rpe) {
				printk("%08x wrote %02x read %02x %s\n",
					(uint32_t)(offs + (rp - buf)),
					*wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
				++rp;
				++wp;
			}
		}
		offs += SPI_FLASH_SECTOR_SIZE;
	}
}
#endif

int main(void)
{
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
	uint8_t jid[3] = {0}; 
	// int rd = qspi_read_jedec_id((flash_dev, jid));
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,21));
	nrf_gpio_pin_write(NRF_GPIO_PIN_MAP(0,21),0);
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,29));
	nrf_gpio_pin_write(NRF_GPIO_PIN_MAP(0,29),1);
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,24));
	nrf_gpio_pin_write(NRF_GPIO_PIN_MAP(0,24),1);
	if (!device_is_ready(flash_dev)) {
		printk("%s: device not ready.\n", flash_dev->name);
		// return 0;
	}

	printk("\n%s SPI flash testing\n", flash_dev->name);
	printk("==========================\n");

	single_sector_test(flash_dev);
#if defined SPI_FLASH_MULTI_SECTOR_TEST
	multi_sector_test(flash_dev);
#endif
	return 0;
}
