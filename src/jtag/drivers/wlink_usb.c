/***************************************************************************
 *   Copyright (C) 2016-2017 by Nuvoton                                    *
 *   Zale Yu <cyyu@nuvoton.com>                                            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <helper/binarybuffer.h>
#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>
#include <target/target.h>

#include <target/cortex_m.h>

#include "libusb_helper.h"

#define WLINK_READ_TIMEOUT  1000
#define WLINK_WRITE_TIMEOUT 1000

#define WLINK_BUFF_SIZE   (64)

struct wlink_usb_handle_s {
	libusb_device_handle *dev_handle;
	uint8_t ep;
	uint8_t cmdidx;
	uint8_t cmdbuf[WLINK_BUFF_SIZE];
};

#define EP_CMD_TX 0x01
#define EP_CMD_RX 0x81
#define EP_DAT_TX 0x02
#define EP_DAT_RX 0x82
#define EP_SER_TX 0x03
#define EP_SER_RX 0x83

/* ICE Command */
#define CMD_INIT1 0x810d0101
#define CMD_INIT2 0x810d0102
#define CMD_INIT3 0x810d0103
#define CMD_MCU_RESET 0x810b0100


static int wlink_usb_xfer_rw(void *handle, uint8_t *buf, int size)
{
	struct wlink_usb_handle_s *h = handle;
	int tr, ret;

	assert(handle);

        ret = jtag_libusb_bulk_write(h->dev_handle, EP_CMD_TX, (char *)h->cmdbuf,
                                     h->cmdidx, WLINK_WRITE_TIMEOUT, &tr);
        if (ret || tr != h->cmdidx)
                return ERROR_FAIL;

        if (h->ep & 0x80) {
                ret = jtag_libusb_bulk_read(h->dev_handle, h->ep, (char *)buf,
                                             size, WLINK_WRITE_TIMEOUT, &tr);
                if (ret || tr != size) {
                        LOG_DEBUG("bulk read failed");
                        return ERROR_FAIL;
                }
        } else if (h->ep != 0x00) {
                ret = jtag_libusb_bulk_write(h->dev_handle, h->ep, (char *)buf,
                                            size, WLINK_READ_TIMEOUT, &tr);
                if (ret || tr != size) {
                        LOG_DEBUG("bulk write failed");
                        return ERROR_FAIL;
                }
        }

        return ERROR_OK;
}

static void wlink_usb_init_buffer(void *handle, uint32_t size)
{
	struct wlink_usb_handle_s *h = handle;

	h->cmdidx = 0;

	memset(h->cmdbuf, 0, WLINK_BUFF_SIZE);
	//memset(h->tempbuf, 0, h->max_packet_size);
	//memset(h->databuf, 0, h->max_packet_size);
}

static int wlink_usb_reset(void *handle)
{
	struct wlink_usb_handle_s *h = handle;

	LOG_DEBUG("wlink_usb_reset");

	assert(handle);

	wlink_usb_init_buffer(handle, 4 * 4);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_RESET);
	//return wlink_usb_xfer(handle, h->databuf, 4);
	return wlink_usb_xfer_rw(h->dev_handle, NULL, 0);
}

static int wlink_usb_halt(void *handle)
{
	struct wlink_usb_handle_s *h = handle;
	int res;
	LOG_DEBUG("wlink_usb_halt");

	assert(handle);

	wlink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_INIT1);
	res = wlink_usb_xfer_rw(handle, NULL, 0);
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_INIT2);
	res = wlink_usb_xfer_rw(handle, NULL, 0);
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_INIT3);
	res = wlink_usb_xfer_rw(handle, NULL, 0);

	//LOG_DEBUG("Nu-Link stop_pc 0x%08" PRIx32, le_to_h_u32(h->databuf + 4));

	return res;
}

static int wlink_usb_override_target(const char *targetname)
{
        return !strcmp(targetname, "riscv");
}

static int wlink_usb_close(void *handle)
{
	struct wlink_usb_handle_s *h = handle;

	LOG_DEBUG("wlink_usb_close");

	if (h && h->dev_handle)
		jtag_libusb_close(h->dev_handle);

	free(h);

	//hid_exit();

	return ERROR_OK;
}

static int wlink_usb_open(struct hl_interface_param_s *param, void **fd)
{
	LOG_DEBUG("wlink_usb_open");

	if (param->transport != HL_TRANSPORT_JTAG)
		return TARGET_UNKNOWN;

	if (!param->vid[0] && !param->pid[0]) {
		LOG_ERROR("Missing vid/pid");
		return ERROR_FAIL;
	}

	struct wlink_usb_handle_s *h = calloc(1, sizeof(*h));
	if (!h) {
		LOG_ERROR("Out of memory");
		goto error_open;
	}

	jtag_libusb_open(param->vid, param->pid, &h->dev_handle, NULL);
	if (!h->dev_handle) {
		//LOG_ERROR("unable to open Nu-Link device 0x%" PRIx16 ":0x%" PRIx16, target_vid, target_pid);
		goto error_open;
	}

	h->cmdidx = 0;


	/* get cpuid, so we can determine the max page size
	 * start with a safe default */
	//h->max_mem_packet = (1 << 10);

	LOG_DEBUG("wlink_usb_open: we manually perform wlink_usb_reset");
	wlink_usb_reset(h);

	*fd = h;

	//free(target_serial);
	return ERROR_OK;

error_open:
	wlink_usb_close(h);
	//free(target_serial);

	return ERROR_FAIL;
}

struct hl_layout_api_s wlink_usb_layout_api = {
	.open = wlink_usb_open,
	.close = wlink_usb_close,
	.reset = wlink_usb_reset,
	.halt = wlink_usb_halt,
	.override_target = wlink_usb_override_target,
};

struct adapter_driver wlink_dap_adapter_driver = {
	.name = "wlink",
	.transports = jtag_only,
	//.dap_jtag_ops = &stlink_dap_ops,
};
