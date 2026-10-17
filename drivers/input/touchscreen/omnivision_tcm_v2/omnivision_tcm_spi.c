/*
 * omnivision TCM touchscreen driver
 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND omnivision
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL omnivision BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF omnivision WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, omnivision'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "omnivision_tcm_core.h"

/*
 * SPI bus glue.  The SPI client creates a child platform device which carries
 * the board data and the bus accessors to the (bus independent) core.
 *
 * All transfers are bounced through kmalloc()ed buffers so that callers may
 * safely pass stack or vmalloc memory (SPI controllers may DMA directly).
 */

static unsigned char *buf;	/* TX bounce buffer */
static unsigned char *rbuf;	/* RX bounce buffer */
static unsigned int buf_size;
static unsigned int xfer_count;
static struct spi_transfer *xfer;

static void ovt_tcm_set_delay(struct spi_transfer *t, unsigned int usecs)
{
	t->delay.value = usecs;
	t->delay.unit = SPI_DELAY_UNIT_USECS;
}

static int ovt_tcm_spi_alloc_mem(struct ovt_tcm_hcd *tcm_hcd,
		unsigned int count, unsigned int size)
{
	struct device *dev = tcm_hcd->pdev->dev.parent;

	if (count > xfer_count) {
		kfree(xfer);
		xfer = kcalloc(count, sizeof(*xfer), GFP_KERNEL);
		if (!xfer) {
			LOGE(dev, "Failed to allocate memory for xfer\n");
			xfer_count = 0;
			return -ENOMEM;
		}
		xfer_count = count;
	} else {
		memset(xfer, 0, count * sizeof(*xfer));
	}

	if (size > buf_size) {
		kfree(buf);
		kfree(rbuf);
		buf = kmalloc(size, GFP_KERNEL);
		rbuf = kmalloc(size, GFP_KERNEL);
		if (!buf || !rbuf) {
			LOGE(dev, "Failed to allocate memory for buf\n");
			kfree(buf);
			kfree(rbuf);
			buf = NULL;
			rbuf = NULL;
			buf_size = 0;
			return -ENOMEM;
		}
		buf_size = size;
	}

	return 0;
}

static int ovt_tcm_spi_rmi_read(struct ovt_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval;
	unsigned int idx;
	unsigned int byte_count;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(tcm_hcd->pdev->dev.parent);
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	spi_message_init(&msg);

	byte_count = length + 2;

	retval = ovt_tcm_spi_alloc_mem(tcm_hcd,
			bdata->ubl_byte_delay_us ? byte_count : 2,
			max(byte_count, 3U));
	if (retval < 0) {
		LOGE(&spi->dev, "Failed to allocate memory\n");
		goto exit;
	}

	buf[0] = (unsigned char)(addr >> 8) | 0x80;
	buf[1] = (unsigned char)addr;

	if (bdata->ubl_byte_delay_us == 0) {
		xfer[0].len = 2;
		xfer[0].tx_buf = buf;
		xfer[0].speed_hz = bdata->ubl_max_freq;
		spi_message_add_tail(&xfer[0], &msg);

		memset(&buf[2], 0xff, length);
		xfer[1].len = length;
		xfer[1].tx_buf = &buf[2];
		xfer[1].rx_buf = rbuf;
		if (bdata->block_delay_us)
			ovt_tcm_set_delay(&xfer[1], bdata->block_delay_us);
		xfer[1].speed_hz = bdata->ubl_max_freq;
		spi_message_add_tail(&xfer[1], &msg);
	} else {
		buf[2] = 0xff;
		for (idx = 0; idx < byte_count; idx++) {
			xfer[idx].len = 1;
			if (idx < 2) {
				xfer[idx].tx_buf = &buf[idx];
			} else {
				xfer[idx].tx_buf = &buf[2];
				xfer[idx].rx_buf = &rbuf[idx - 2];
			}
			ovt_tcm_set_delay(&xfer[idx], bdata->ubl_byte_delay_us);
			if (bdata->block_delay_us && idx == byte_count - 1)
				ovt_tcm_set_delay(&xfer[idx],
						bdata->block_delay_us);
			xfer[idx].speed_hz = bdata->ubl_max_freq;
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}

	retval = spi_sync(spi, &msg);
	if (retval == 0) {
		memcpy(data, rbuf, length);
		retval = length;
	} else {
		LOGE(&spi->dev, "Failed to complete SPI transfer, error = %d\n",
				retval);
	}

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static int ovt_tcm_spi_rmi_write(struct ovt_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval;
	unsigned int byte_count;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(tcm_hcd->pdev->dev.parent);
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	spi_message_init(&msg);

	byte_count = length + 2;

	retval = ovt_tcm_spi_alloc_mem(tcm_hcd, 1, byte_count);
	if (retval < 0) {
		LOGE(&spi->dev, "Failed to allocate memory\n");
		goto exit;
	}

	buf[0] = (unsigned char)(addr >> 8) & ~0x80;
	buf[1] = (unsigned char)addr;
	memcpy(&buf[2], data, length);

	xfer[0].len = byte_count;
	xfer[0].tx_buf = buf;
	if (bdata->block_delay_us)
		ovt_tcm_set_delay(&xfer[0], bdata->block_delay_us);
	spi_message_add_tail(&xfer[0], &msg);

	retval = spi_sync(spi, &msg);
	if (retval == 0)
		retval = length;
	else
		LOGE(&spi->dev, "Failed to complete SPI transfer, error = %d\n",
				retval);

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static int ovt_tcm_spi_read(struct ovt_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval;
	unsigned int idx;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(tcm_hcd->pdev->dev.parent);
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	spi_message_init(&msg);

	retval = ovt_tcm_spi_alloc_mem(tcm_hcd,
			bdata->byte_delay_us ? length : 1, length);
	if (retval < 0) {
		LOGE(&spi->dev, "Failed to allocate memory\n");
		goto exit;
	}

	memset(buf, 0xff, length);

	if (bdata->byte_delay_us == 0) {
		xfer[0].len = length;
		xfer[0].tx_buf = buf;
		xfer[0].rx_buf = rbuf;
		if (bdata->block_delay_us)
			ovt_tcm_set_delay(&xfer[0], bdata->block_delay_us);
		spi_message_add_tail(&xfer[0], &msg);
	} else {
		for (idx = 0; idx < length; idx++) {
			xfer[idx].len = 1;
			xfer[idx].tx_buf = &buf[idx];
			xfer[idx].rx_buf = &rbuf[idx];
			ovt_tcm_set_delay(&xfer[idx], bdata->byte_delay_us);
			if (bdata->block_delay_us && idx == length - 1)
				ovt_tcm_set_delay(&xfer[idx],
						bdata->block_delay_us);
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}

	retval = spi_sync(spi, &msg);
	if (retval == 0) {
		memcpy(data, rbuf, length);
		retval = length;
	} else {
		LOGE(&spi->dev, "Failed to complete SPI transfer, error = %d\n",
				retval);
	}

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static int ovt_tcm_spi_write(struct ovt_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval;
	unsigned int idx;
	struct spi_message msg;
	struct spi_device *spi = to_spi_device(tcm_hcd->pdev->dev.parent);
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	spi_message_init(&msg);

	retval = ovt_tcm_spi_alloc_mem(tcm_hcd,
			bdata->byte_delay_us ? length : 1, length);
	if (retval < 0) {
		LOGE(&spi->dev, "Failed to allocate memory\n");
		goto exit;
	}

	memcpy(buf, data, length);

	if (bdata->byte_delay_us == 0) {
		xfer[0].len = length;
		xfer[0].tx_buf = buf;
		if (bdata->block_delay_us)
			ovt_tcm_set_delay(&xfer[0], bdata->block_delay_us);
		spi_message_add_tail(&xfer[0], &msg);
	} else {
		for (idx = 0; idx < length; idx++) {
			xfer[idx].len = 1;
			xfer[idx].tx_buf = &buf[idx];
			ovt_tcm_set_delay(&xfer[idx], bdata->byte_delay_us);
			if (bdata->block_delay_us && idx == length - 1)
				ovt_tcm_set_delay(&xfer[idx],
						bdata->block_delay_us);
			spi_message_add_tail(&xfer[idx], &msg);
		}
	}

	retval = spi_sync(spi, &msg);
	if (retval == 0)
		retval = length;
	else
		LOGE(&spi->dev, "Failed to complete SPI transfer, error = %d\n",
				retval);

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static const struct ovt_tcm_bus_io ovt_tcm_spi_bus_io = {
	.type = BUS_SPI,
	.read = ovt_tcm_spi_read,
	.write = ovt_tcm_spi_write,
	.rmi_read = ovt_tcm_spi_rmi_read,
	.rmi_write = ovt_tcm_spi_rmi_write,
};

/* ------------------------------------------------------------------ */
/* Device tree                                                          */
/* ------------------------------------------------------------------ */

static const char * const ovt_tcm_variants[] = {
	[0] = "default",
	[1] = "dsjm-boe",
	[2] = "tm",
	[3] = "dsjm-nairobi",
};

static int ovt_tcm_parse_dt(struct device *dev,
		struct ovt_tcm_board_data *bdata)
{
	const char *str;
	int i, ret;

	/* GPIOs (polarity comes from the DT flags) */
	bdata->reset_gpiod = devm_gpiod_get_optional(dev, "reset",
			GPIOD_OUT_LOW);
	if (IS_ERR(bdata->reset_gpiod))
		return dev_err_probe(dev, PTR_ERR(bdata->reset_gpiod),
				"Failed to get reset GPIO\n");

	bdata->power_gpiod = devm_gpiod_get_optional(dev, "power",
			GPIOD_OUT_LOW);
	if (IS_ERR(bdata->power_gpiod))
		return dev_err_probe(dev, PTR_ERR(bdata->power_gpiod),
				"Failed to get power GPIO\n");

	bdata->tpio_reset_gpiod = devm_gpiod_get_optional(dev, "tpio-reset",
			GPIOD_OUT_LOW);
	if (IS_ERR(bdata->tpio_reset_gpiod))
		return dev_err_probe(dev, PTR_ERR(bdata->tpio_reset_gpiod),
				"Failed to get tpio-reset GPIO\n");

	/* Supplies: "vdd" is the bus/IO rail, "avdd" the analog power rail */
	bdata->bus_reg = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(bdata->bus_reg)) {
		if (PTR_ERR(bdata->bus_reg) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(bdata->bus_reg),
					"Failed to get vdd supply\n");
		bdata->bus_reg = NULL;
	}

	bdata->pwr_reg = devm_regulator_get_optional(dev, "avdd");
	if (IS_ERR(bdata->pwr_reg)) {
		if (PTR_ERR(bdata->pwr_reg) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(bdata->pwr_reg),
					"Failed to get avdd supply\n");
		bdata->pwr_reg = NULL;
	}

	/* Timings */
	device_property_read_u32(dev, "ovti,power-delay-ms",
			&bdata->power_delay_ms);
	device_property_read_u32(dev, "ovti,reset-active-ms",
			&bdata->reset_active_ms);
	device_property_read_u32(dev, "ovti,reset-delay-ms",
			&bdata->reset_delay_ms);
	device_property_read_u32(dev, "ovti,byte-delay-us",
			&bdata->byte_delay_us);
	device_property_read_u32(dev, "ovti,block-delay-us",
			&bdata->block_delay_us);
	device_property_read_u32(dev, "ovti,ubl-max-frequency-hz",
			&bdata->ubl_max_freq);
	device_property_read_u32(dev, "ovti,ubl-byte-delay-us",
			&bdata->ubl_byte_delay_us);

	/* Axis handling, standard touchscreen.yaml names */
	bdata->x_flip = device_property_read_bool(dev,
			"touchscreen-inverted-x");
	bdata->y_flip = device_property_read_bool(dev,
			"touchscreen-inverted-y");
	bdata->swap_axes = device_property_read_bool(dev,
			"touchscreen-swapped-x-y");

	bdata->rainstorm_mode_support = device_property_read_bool(dev,
			"ovti,rainstorm-mode-support");

	/* Firmware image set (replaces the downstream lcd_name= bootarg) */
	g_firmware_version_v2 = 0;
	ret = device_property_read_string(dev, "ovti,panel-variant", &str);
	if (!ret) {
		for (i = 0; i < ARRAY_SIZE(ovt_tcm_variants); i++) {
			if (!strcmp(str, ovt_tcm_variants[i])) {
				g_firmware_version_v2 = i;
				break;
			}
		}
		if (i == ARRAY_SIZE(ovt_tcm_variants))
			return dev_err_probe(dev, -EINVAL,
					"Unknown ovti,panel-variant \"%s\"\n",
					str);
	}

	return 0;
}

static int ovt_tcm_spi_probe(struct spi_device *spi)
{
	int retval;
	struct ovt_tcm_hw_interface *hw_if;
	struct ovt_tcm_board_data *bdata;
	struct platform_device *pdev;

	if (spi->controller->flags & SPI_CONTROLLER_HALF_DUPLEX) {
		dev_err(&spi->dev, "Full duplex not supported by host\n");
		return -EIO;
	}

	if (spi->irq <= 0)
		return dev_err_probe(&spi->dev, spi->irq ? spi->irq : -EINVAL,
				"No interrupt specified\n");

	hw_if = devm_kzalloc(&spi->dev, sizeof(*hw_if), GFP_KERNEL);
	bdata = devm_kzalloc(&spi->dev, sizeof(*bdata), GFP_KERNEL);
	if (!hw_if || !bdata)
		return -ENOMEM;

	retval = ovt_tcm_parse_dt(&spi->dev, bdata);
	if (retval)
		return retval;

	bdata->irq = spi->irq;
	hw_if->bdata = bdata;
	hw_if->bus_io = &ovt_tcm_spi_bus_io;

	/*
	 * The controller is clocked in SPI mode 3 (spi-cpol + spi-cpha in DT).
	 * Warn when the board description disagrees, the hardware won't talk.
	 */
	if ((spi->mode & SPI_MODE_X_MASK) != SPI_MODE_3)
		dev_warn(&spi->dev,
			 "SPI mode %u requested, device needs mode 3 (spi-cpol, spi-cpha)\n",
			 (unsigned int)(spi->mode & SPI_MODE_X_MASK));

	spi->bits_per_word = 8;
	retval = spi_setup(spi);
	if (retval < 0)
		return dev_err_probe(&spi->dev, retval,
				"Failed to set up SPI protocol driver\n");

	/* platform_data is copied; the pointers inside stay valid (devm) */
	pdev = platform_device_register_data(&spi->dev, PLATFORM_DRIVER_NAME,
			PLATFORM_DEVID_NONE, hw_if, sizeof(*hw_if));
	if (IS_ERR(pdev))
		return dev_err_probe(&spi->dev, PTR_ERR(pdev),
				"Failed to add platform device\n");

	spi_set_drvdata(spi, pdev);

	return 0;
}

static void ovt_tcm_spi_remove(struct spi_device *spi)
{
	struct platform_device *pdev = spi_get_drvdata(spi);

	platform_device_unregister(pdev);
}

static const struct spi_device_id ovt_tcm_id_table[] = {
	{ "td4160b" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ovt_tcm_id_table);

static const struct of_device_id ovt_tcm_of_match_table[] = {
	{ .compatible = "ovti,td4160b" },
	{ }
};
MODULE_DEVICE_TABLE(of, ovt_tcm_of_match_table);

static struct spi_driver ovt_tcm_spi_driver = {
	.driver = {
		.name = SPI_MODULE_NAME,
		.of_match_table = ovt_tcm_of_match_table,
	},
	.probe = ovt_tcm_spi_probe,
	.remove = ovt_tcm_spi_remove,
	.id_table = ovt_tcm_id_table,
};

int ovt_tcm_bus_init_v2(void)
{
	return spi_register_driver(&ovt_tcm_spi_driver);
}

void ovt_tcm_bus_exit_v2(void)
{
	spi_unregister_driver(&ovt_tcm_spi_driver);

	kfree(buf);
	kfree(rbuf);
	kfree(xfer);
	buf = NULL;
	rbuf = NULL;
	xfer = NULL;
	buf_size = 0;
	xfer_count = 0;
}

MODULE_AUTHOR("omnivision, Inc.");
MODULE_DESCRIPTION("omnivision TCM SPI Bus Module");
MODULE_LICENSE("GPL");
