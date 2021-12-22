/***************************************************************************
 *   Copyright (C) 2021 by Xu Ruijun                                       *
 *   Xu Ruijun <1687701765@qq.com>                                         *
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
	int cmd_size_tx;
	int cmd_size_rx;
	uint8_t cmdbuf[WLINK_BUFF_SIZE];
	int data_size;
	uint8_t databuf[WLINK_BUFF_SIZE];
};

static struct wlink_usb_handle_s *wlink_usb;

const uint16_t wlink_vids[] = {0x1a86, 0x0000};
const uint16_t wlink_pids[] = {0x8010, 0x0000};


#define EP_IN        0x80
#define EP_OUT       0x00

#define EP_CMD_TX    (0x01 | EP_OUT)
#define EP_CMD_RX    (0x01 | EP_IN)
#define EP_DAT_TX    (0x02 | EP_OUT)
#define EP_DAT_RX    (0x02 | EP_IN)
#define EP_SER_TX    (0x03 | EP_OUT)
#define EP_SER_RX    (0x03 | EP_IN)

/* ICE Command */
#define CMD_GET_VERSION  0x810d0101
#define CMD_CONN_TARGET  0x810d0102
#define CMD_INIT3        0x810d0103  //未知
#define CMD_MCU_RESET    0x810b0100


static int wlink_cmd_rw(void)
{
	int tr, ret;

        ret = jtag_libusb_bulk_write(wlink_usb->dev_handle, EP_CMD_TX, (char *)wlink_usb->cmdbuf,
                                     wlink_usb->cmd_size_tx, WLINK_WRITE_TIMEOUT, &tr);
        if (ret || tr != wlink_usb->cmd_size_tx)
                return ERROR_FAIL;
	ret = jtag_libusb_bulk_read(wlink_usb->dev_handle, EP_CMD_RX, (char *)wlink_usb->cmdbuf,
				    wlink_usb->cmd_size_rx, WLINK_READ_TIMEOUT, &tr);
	if (ret || tr != wlink_usb->cmd_size_rx)
		return ERROR_FAIL;

        return ERROR_OK;
}


static int wlink_usb_close(void)
{
	free(wlink_usb);
	return ERROR_OK;
}

static int wlink_usb_open(void)
{
	wlink_usb = calloc(1, sizeof(*wlink_usb));
	LOG_DEBUG("wlink_usb_open");

	if (!wlink_usb) {
		LOG_ERROR("Out of memory");
		goto error_open;
	}

	jtag_libusb_open(wlink_vids, wlink_pids, &wlink_usb->dev_handle, NULL);
	if (!wlink_usb->dev_handle) {
		LOG_ERROR("unable to open WCH-Link device 0x%" PRIx16 ":0x%" PRIx16, wlink_vids[0], wlink_pids[0]);
		goto error_open;
	}

	jtag_libusb_set_configuration(wlink_usb->dev_handle, 0);

	if (libusb_claim_interface(wlink_usb->dev_handle, 0) != ERROR_OK) {
		LOG_DEBUG("claim interface failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;

error_open:
	wlink_usb_close();
	return ERROR_FAIL;
}


static int wlink_init(void)
{
	wlink_usb_open();

	h_u32_to_be(wlink_usb->cmdbuf, CMD_GET_VERSION);
	wlink_usb->cmd_size_tx = 4;
	wlink_usb->cmd_size_rx = 5;
	wlink_cmd_rw();
	LOG_INFO("WCH-Link version %d.%d", wlink_usb->cmdbuf[3], wlink_usb->cmdbuf[4]);
	return ERROR_OK;
}


static int wlink_execute_command(struct jtag_command *cmd)
{
        switch (cmd->type) {
	       case JTAG_RESET:
		 //TODO: add RESET
                case JTAG_STABLECLOCKS:
                        LOG_ERROR("JTAG_STABLECLOCKS");
                        break;
                case JTAG_RUNTEST:
                        LOG_ERROR("JTAG_RUNTEST");
                        break;
                case JTAG_TLR_RESET:
		        LOG_ERROR("JTAG_TLR_REST");
                        //jlink_execute_statemove(cmd);
                        break;
                case JTAG_PATHMOVE:
		        LOG_ERROR("JTAG_PATHMOVE");
                        //jlink_execute_pathmove(cmd);
                        break;
                case JTAG_SCAN:
		        LOG_ERROR("JTAG_SCAN");
                        //jlink_execute_scan(cmd);
                        break;
                case JTAG_SLEEP:
		        LOG_ERROR("JTAG_SLEEP");
		        //jlink_execute_sleep(cmd);
                        break;
                default:
                        LOG_ERROR("BUG: Unknown JTAG command type encountered");
                        return ERROR_JTAG_QUEUE_FAILED;
        }

        return ERROR_OK;
}

static int wlink_execute_queue(void)
{
        int ret;
        struct jtag_command *cmd = jtag_command_queue;

        while (cmd) {
                ret = wlink_execute_command(cmd);

                if (ret != ERROR_OK)
                        return ret;

                cmd = cmd->next;
        }

        return ERROR_OK;
}

//static const struct command_registran

static struct jtag_interface wlink_jtag_ops = {
        .execute_queue = &wlink_execute_queue,
};

struct adapter_driver wlink_adapter_driver = {
	.name = "wlink",
	.transports = jtag_only,
	//.commands = 

	.init = &wlink_init,
	//.quit = &wlink_quit,
	//.reset = &wlink_reset,
	
	.jtag_ops = &wlink_jtag_ops,
};
