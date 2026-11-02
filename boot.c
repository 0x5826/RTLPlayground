#pragma codeseg BANK3
#pragma constseg BANK3

#include <8051.h>
#include <stdint.h>
#include <stdbool.h>

#include "rtl837x_sfr.h"
#include "rtl837x_regs.h"
#include "rtl837x_common.h"
#include "rtl837x_flash.h"
#include "rtl837x_pins.h"
#include "rtl837x_phy.h"
#include "rtl837x_port.h"
#include "rtl837x_init.h"
#include "machine.h"
#include "phy.h"
#include "boot.h"
#include "rtl837x_nic.h"
#include "uip/uip.h"

extern __code const struct machine machine;
extern __xdata struct machine_runtime machine_detected;
extern __xdata uint8_t sfr_data[4];
extern volatile __xdata uint32_t ticks;
__xdata uint32_t rtl8224_release_tick;
extern __xdata uint8_t tx_seq;
extern __code const uint8_t * __code const hex;
extern __xdata uint8_t uip_buf[];
extern __xdata uint8_t rx_headers[];
extern void tcpip_output(void);



void sds_config_mac(uint8_t sds, uint8_t mode) __banked
{
	reg_read_m(RTL837X_REG_SDS_MODES);
	sfr_data[0] = 0;
	sfr_data[1] = 0;
	switch (sds) {
	case 0:
		sfr_mask_data(0, 0x1f, mode);
		break;
	case 1:
		sfr_mask_data(0, 0xe0, mode << 5);
		sfr_mask_data(1, 0x03, mode >> 3);
		break;
	case 2:
		sfr_mask_data(1, 0xfc, 0x02 << 2);
	}
	if (machine_detected.isRTL8373) // Set 3rd SERDES Mode to 0x2 for RTL8224
		sfr_mask_data(1, 0xfc, 0x02 << 2);
	else
		sfr_data[2] &= 0x03;
	reg_write_m(RTL837X_REG_SDS_MODES);
	print_string("\nRTL837X_REG_SDS_MODES: ");
	print_reg(RTL837X_REG_SDS_MODES);
	print_string("\n");
}



void early_boot_handle_button(void) __banked
{
	if (machine.reset_pin == GPIO_NA)
		return;

	gpio_input_setup(machine.reset_pin);

	// Debounce after init
	delay(100);
	// If the button is not already held at boot, continue normally.
	if (gpio_pin_test(machine.reset_pin))
		return;

	set_sys_led_state(SYS_LED_FAST);
	print_string("\n[Reset button held at boot]\n");

	if (gpio_pin_test(machine.reset_pin))
		return;

	const __xdata uint32_t min_hold_ticks = 10UL * SYS_TICK_HZ;
	const __xdata uint32_t max_hold_ticks = 30UL * SYS_TICK_HZ;
	const __xdata uint32_t blink_ticks = SYS_TICK_HZ / 10;      // 100 ms
	const __xdata uint32_t pause_ticks = SYS_TICK_HZ / 2;       // 500 ms
	__xdata uint32_t start_ticks = ticks;
	__xdata uint32_t last_blink_step = start_ticks;
	__xdata uint8_t blink_step = 0;

	set_sys_led_state(SYS_LED_ON);

	while (!gpio_pin_test(machine.reset_pin)) {
		__xdata uint32_t held_ticks = ticks - start_ticks;

		if (held_ticks > max_hold_ticks) {
			print_string("[Button held >30s at boot; continuing normal boot]\n");
			return;
		}

		// Double blink pattern while button is held:
		// ON (100ms), OFF (100ms), ON (100ms), OFF (500ms)
		__xdata uint32_t step_ticks = (blink_step == 3) ? pause_ticks : blink_ticks;
		if ((ticks - last_blink_step) >= step_ticks) {
			blink_step = (blink_step + 1) & 0x3;
			set_sys_led_state((blink_step == 0 || blink_step == 2) ? SYS_LED_ON : SYS_LED_OFF);
			last_blink_step = ticks;
		}

		PCON |= 1;
	}

	set_sys_led_state(SYS_LED_ON);

	if ((ticks - start_ticks) >= min_hold_ticks) {
		print_string("[Button held 10s-30s at boot; restoring default config]\n");
		set_sys_led_state(SYS_LED_FAST);
		flash_default_config();
		delay(3UL * SYS_TICK_HZ);
	}
}

static bool check_magic(void)
{
	for (uint8_t k = 12; k < 40; k++) {
		if (uip_buf[k] == 'R' && uip_buf[k + 1] == 'T' &&
		    uip_buf[k + 2] == 'L' && uip_buf[k + 3] == 'D' &&
		    uip_buf[k + 4] == 'E' && uip_buf[k + 5] == 'B' &&
		    uip_buf[k + 6] == 'R' && uip_buf[k + 7] == 'K')
			return true;
	}
	return false;
}

static bool probe_loopback(uint8_t p1, uint8_t p2)
{
	bool matched = false;
	// Purge existing RX ring entries
	while (1) {
		reg_read(RTL837X_REG_NIC_RX_BUFF_DATA);
		if (!SFR_DATA_U16)
			break;
		REG_SET(RTL837X_REG_NIC_RXCMD, 1);
	}

	// Prepare probe frame in uip_buf (offset by RTL_FRAME_DESC_SIZE for TX descriptor)
	__xdata uint8_t *pkt = &uip_buf[RTL_FRAME_DESC_SIZE];

	// DST MAC (broadcast) + SRC MAC (uip_ethaddr)
	for (uint8_t i = 0; i < 6; i++) {
		pkt[i] = 0xff;
		pkt[i + 6] = uip_ethaddr.addr[i];
	}
	// RTL CPU Tag (8 bytes): send only to physical port p1
	pkt[12] = 0x88; pkt[13] = 0x99; // RTL_FRAME_TAG_ID
	pkt[14] = 0x04; pkt[15] = 0x00; // version 4, reason 0
	pkt[16] = 0x00; pkt[17] = 0x20; // flags: RTL_TAG_LEARN_DIS
	pkt[18] = 0x00; pkt[19] = (1 << p1); // pmask: port p1

	// Custom EtherType and Magic string
	pkt[20] = 0xff; pkt[21] = 0xfe;
	pkt[22] = 'R'; pkt[23] = 'T'; pkt[24] = 'L'; pkt[25] = 'D';
	pkt[26] = 'E'; pkt[27] = 'B'; pkt[28] = 'R'; pkt[29] = 'K';

	for (uint8_t i = 30; i < 60; i++)
		pkt[i] = 0;

	uip_len = 60;
	tcpip_output();

	// Wait up to 50 ms for frame arrival on port p2
	__xdata uint32_t t_start = ticks;
	while (!matched && ((ticks - t_start) < (SYS_TICK_HZ / 20))) {
		reg_read(RTL837X_REG_NIC_RX_BUFF_DATA);
		if (SFR_DATA_U16) {
			reg_read(RTL837X_REG_CPU_RX_CURR_PKT);
			uint16_t ring_ptr = SFR_DATA_U16 << 3;
			if (nic_rx_header(ring_ptr)) {
				uint8_t rx_port = rx_headers[3] & 0x0f;
				if (rx_port == p2) {
					if (nic_rx_packet(uip_buf, ring_ptr + 8)) {
						if (check_magic())
							matched = true;
					}
				}
			}
			REG_SET(RTL837X_REG_NIC_RXCMD, 1);
		}
	}
	return matched;
}

void early_boot_handle_recovery_loop(void) __banked
{
	uint8_t p1 = 0xff, p2 = 0xff;
	for (uint8_t i = machine.min_port; i <= machine.max_port; i++) {
		if (machine.is_sfp[i])
			continue;
		if (p1 == 0xff) {
			p1 = i;
		} else if (p2 == 0xff) {
			p2 = i;
		}
	}
	if (p1 == 0xff || p2 == 0xff)
		return;

	// Fast link check: both ports must be Up
	reg_read_m(RTL837X_REG_LINKS_STS);
	uint16_t link_mask = (((uint16_t)sfr_data[2]) << 8) | sfr_data[1];
	if (!((link_mask >> p1) & 1) || !((link_mask >> p2) & 1))
		return;

	// No other ports may be connected (avoids misdetection in operational network)
	for (uint8_t i = machine.min_port; i <= machine.max_port; i++) {
		if (i != p1 && i != p2 && ((link_mask >> i) & 1))
			return;
	}

	// Verify loopback by probing packet transfer between p1 and p2
	if (!probe_loopback(p1, p2))
		return;

	print_string("\n[Port loopback detected (port 1 <-> port 2)]\n");
	print_string("[Keep loopback connected for 10s to restore factory defaults]\n");

	const __xdata uint32_t min_hold_ticks = 10UL * SYS_TICK_HZ;
	const __xdata uint32_t max_hold_ticks = 30UL * SYS_TICK_HZ;
	const __xdata uint32_t blink_ticks = SYS_TICK_HZ / 10;      // 100 ms
	const __xdata uint32_t pause_ticks = SYS_TICK_HZ / 2;       // 500 ms
	__xdata uint32_t start_ticks = ticks;
	__xdata uint32_t last_blink_step = start_ticks;
	__xdata uint32_t last_check_ticks = start_ticks;
	__xdata uint8_t blink_step = 0;

	set_sys_led_state(SYS_LED_ON);

	while (1) {
		__xdata uint32_t held_ticks = ticks - start_ticks;

		if (held_ticks > max_hold_ticks) {
			print_string("[Loopback held >30s; continuing normal boot]\n");
			set_sys_led_state(SYS_LED_ON);
			return;
		}

		if (held_ticks >= min_hold_ticks) {
			print_string("[Loopback held 10s; restoring factory default config]\n");
			set_sys_led_state(SYS_LED_FAST);
			flash_default_config();
			print_string("Resetting chip...\n");
			delay(1000);
			reset_chip();
			return;
		}

		// Periodic link verification every 500 ms
		if ((ticks - last_check_ticks) >= (SYS_TICK_HZ / 2)) {
			last_check_ticks = ticks;
			reg_read_m(RTL837X_REG_LINKS_STS);
			link_mask = (((uint16_t)sfr_data[2]) << 8) | sfr_data[1];
			if (!((link_mask >> p1) & 1) || !((link_mask >> p2) & 1)) {
				print_string("[Loopback disconnected; continuing normal boot]\n");
				set_sys_led_state(SYS_LED_ON);
				return;
			}
		}

		// Double blink pattern while loopback is held
		__xdata uint32_t step_ticks = (blink_step == 3) ? pause_ticks : blink_ticks;
		if ((ticks - last_blink_step) >= step_ticks) {
			blink_step = (blink_step + 1) & 0x3;
			set_sys_led_state((blink_step == 0 || blink_step == 2) ? SYS_LED_ON : SYS_LED_OFF);
			last_blink_step = ticks;
		}

		PCON |= 1;
	}
}


/*
 * Configure the SerDes of the SoC for a particular mode
 * to connect to an SFP module or a PHY
 * Valid modes are SDS_10GR, SDS_QXGMII, SDS_HISGMII, SDS_HSG, SDS_SGMII and SDS_1000BX_FIBER
 * The SerDes ID may be 0 or 1 for RTL8272 and 0-2 for RTL8373
 * SDS_QXGMII is used for 10G Fiber, RTL8224 and RTL8261BE
 */
void sds_config(uint8_t sds, uint8_t mode) __banked
{
	print_string("sds_config sds: "); print_byte(sds); print_string(", mode: "); print_byte(mode); write_char('\n');

	uint8_t page = 0;
	uint16_t v = 0;

	switch (mode) {
	case SDS_SGMII:
	case SDS_1000BX_FIBER:
		v = 0x0300;
		page = 0x24;
		break;
	case SDS_HISGMII:
	case SDS_HSG:
		v = 0x0200;
		page = 0x28;
		break;
	case SDS_10GR:
	case SDS_QXGMII:
		v = 0x0200;
		page = 0x2e;
		break;
	case SDS_100FX:
		v = 0x0200;
		page = 0x26;
		break;
	default:
		print_string("Error in SDS Mode\n");
		return;
	}

	sds_config_mac(sds, mode);

	if (mode == SDS_10GR || mode == SDS_QXGMII)
		sds_write_v(sds, 0x21, 0x10, 0x4480); // Q002110:6480
	else
		sds_write_v(sds, 0x21, 0x10, 0x6480); // Q002110:6480

	sds_write_v(sds, 0x21, 0x13, 0x0400); // Q002113:0400
	sds_write_v(sds, 0x21, 0x18, 0x6d02); // Q002118:6d02
	sds_write_v(sds, 0x21, 0x1b, 0x424e); // Q00211b:424e
	sds_write_v(sds, 0x21, 0x1d, 0x0002); // Q00211d:0002
	sds_write_v(sds, 0x36, 0x1c, 0x1390); // Q00361c:1390
	sds_write_v(sds, 0x36, 0x14, 0x003f); // Q003614:003f

	sds_write_v(sds, 0x36, 0x10, v); // Q003610:0200

	if (page == 0x2e) {  // 10G Fiber / SDS_QXGMII
		sds_write_v(sds, page, 0x04, 0x0080); // Q012e04:0080
		sds_write_v(sds, page, 0x06, 0x0408); // Q012e06:0408
		sds_write_v(sds, page, 0x07, 0x020d); // Q012e07:020d
		sds_write_v(sds, page, 0x09, 0x0601); // Q012e09:0601
		sds_write_v(sds, page, 0x0b, 0x222c); // Q012e0b:222c
		sds_write_v(sds, page, 0x0c, 0xa217); // Q012e0c:a217
		sds_write_v(sds, page, 0x0d, 0xfe40); // Q012e0d:fe40
		sds_write_v(sds, page, 0x15, 0xf5c1); // Q012e15:f5c1
	} else {
		sds_write_v(sds, page, 0x04, 0x0080); // Q002804:0080
		sds_write_v(sds, page, 0x07, 0x1201); // Q002807:1201
		sds_write_v(sds, page, 0x09, 0x0601); // Q002809:0601
		sds_write_v(sds, page, 0x0b, 0x232c); // Q00280b:232c
		sds_write_v(sds, page, 0x0c, 0x9217); // Q00280c:9217
		sds_write_v(sds, page, 0x0f, 0x5b50); // Q00280f:5b50
		sds_write_v(sds, page, 0x15, 0xe7c1); // Q002815:e7f1 BUG !
	}

	sds_write_v(sds, page, 0x16, 0x0443); // Q002816:0443 / Q012e16:0443
	sds_write_v(sds, page, 0x1d, 0xabb0); // Q00281d:abb0 / Q012e1d:abb0

	sds_write_v(sds, 0x06, 0x12, 0x5078); // Q000612:5078
	sds_write_v(sds, 0x07, 0x06, 0x9401); // Q000706:9401
	sds_write_v(sds, 0x07, 0x08, 0x9401); // Q000708:9401
	sds_write_v(sds, 0x07, 0x0a, 0x9401); // Q00070a:9401
	sds_write_v(sds, 0x07, 0x0c, 0x9401); // Q00070c:9401
	sds_write_v(sds, 0x1f, 0x0b, 0x0003); // Q001f0b:0003
	sds_write_v(sds, 0x06, 0x03, 0xc45c); // Q000603:c45c

	// RTL8261BE
	if (machine.n_10g && mode == SDS_QXGMII) {
		sds_write_v(sds, 0x06, 0x1f, 0x2100); // Q00061f:2100
		sds_write_v(sds, 0x07, 0x11, 0x054f); // Q000711:054f
		sds_write_v(sds, 0x20, 0x00, 0x0030); // Q002000:0030
		sds_write_v(sds, 0x20, 0x00, 0x0010); // Q002000:0010
		sds_write_v(sds, 0x20, 0x00, 0x0050); // Q002000:0050
		sds_write_v(sds, 0x20, 0x00, 0x00d0); // Q002000:00d0
		sds_write_v(sds, 0x20, 0x00, 0x0cd0); // Q002000:0cd0
		sds_write_v(sds, 0x20, 0x00, 0x04d0); // Q002000:04d0
		sds_write_v(sds, 0x20, 0x00, 0x04d0); // Q002000:04d0
		sds_write_v(sds, 0x20, 0x00, 0x0cd0); // Q002000:0cd0
		sds_write_v(sds, 0x20, 0x00, 0x00d0); // Q002000:00d0
		sds_write_v(sds, 0x20, 0x00, 0x00d0); // Q002000:00d0
		sds_write_v(sds, 0x20, 0x00, 0x0050); // Q002000:0050
		sds_write_v(sds, 0x20, 0x00, 0x0010); // Q002000:0010
		sds_write_v(sds, 0x20, 0x00, 0x0010); // Q002000:0010
		sds_write_v(sds, 0x20, 0x00, 0x0030); // Q002000:0030
		sds_write_v(sds, 0x20, 0x00, 0x0000); // Q002000:0000
		sds_write_v(sds, 0x1f, 0x00, 0x000b); // Q001f00:000b
		sds_write_v(sds, 0x1f, 0x00, 0x0000); // Q001f00:0000
		return;
	}
	if (mode != SDS_QXGMII)
		sds_write_v(sds, 0x06, 0x1f, 0x2100); // Q00061f:2100

	if (machine_detected.isRTL8373 && sds == 1) {
		sds_write_v(1, 0x21, 0x00, 0x4906);
		sds_write_v(1, 0x36, 0x05, 0x4000);
		sds_write_v(1, 0x1f, 0x02, 0x001f);
		if (machine_detected.isN) {
			if (mode == SDS_10GR) {
				sds_read(1, 6, 2);
				sds_write_v(1, 6, 2, SFR_DATA_U16 | 0x2000);
			} else if (mode == SDS_1000BX_FIBER) {
				sds_read(1, 0, 0);
				sds_write_v(1, 0, 0, SFR_DATA_U16 | 0x200);
			}
		}
	}

	if (mode == SDS_1000BX_FIBER) {
		sds_write_v(sds, 0x02, 0x04, 0x0020); 	// Q000204:0020
		sds_write_v(sds, 0x00, 0x02, 0x73d0); 	// Q000002:73d0
		sds_write_v(sds, 0x00, 0x04, 0x074d); 	// Q000004:074d
		sds_write_v(sds, 0x20, 0x04, 0x0000); 	// Q002000:0000
		sds_write_v(sds, 0x1f, 0x00, 0x0000); 	// Q001f00:0000
	}
}




void rtl8224_enable(void) __banked
{
	// Set Pin 4 low
	reg_bit_clear(RTL837X_REG_GPIO_32_63_OUTPUT, 4);
	// Configure Pin as output
	reg_bit_set(RTL837X_REG_GPIO_32_63_DIRECTION, 4);
	delay(4);
	// Set pin 4 high
	reg_bit_set(RTL837X_REG_GPIO_32_63_OUTPUT, 4);
	rtl8224_release_tick = ticks;
}



void nic_setup(void) __banked
{
	// Enable NIC
	// r6040:00000100 R6040-00001100
	reg_bit_set(RTL837X_REG_HW_CONF, 0xc);

	// This sets the size of the RX buffer, the filling level is in 0x7874
	// R7848-000004ff
	REG_SET(RTL837X_REG_NIC_RXBUFF_RX, RTL837X_NIC_RX_LAST_SLOT);

	// R7844-000007fe
	REG_SET(RTL837X_REG_NIC_BUFFSIZE_TX, 0x7fe);

	// Configure NIC RX to receive various types of packets
	// RTL837X_REG_RX_CTRL: Set bits 24-31 to 0x4, clear bits 16/17
	reg_read_m(RTL837X_REG_RX_CTRL);
	sfr_mask_data(3, 0xff, 0x04);
	sfr_mask_data(2, 0x03, 0);
	reg_write_m(RTL837X_REG_RX_CTRL);

	// Enable NIC TX (set bit 0)
	reg_bit_set(RTL837X_REG_TX_CTRL, 0);

	// Enable NIC RX (set bit 0)
	reg_bit_set(RTL837X_REG_RX_CTRL, 0);

	// Drop packets with invalid CRC
	reg_bit_clear(RTL837X_REG_RX_CTRL, 2);

	// R603c-00000200
	// CPU-port is CPU-Tag aware (bit 9)
	REG_SET(RTL837X_REG_CPU_TAG_AWARE_PMASK, 0x200);

	// Insert CPU-tag for internally received packets (bit 0), MODE is 0, i.e. ALL packets (bits 8-9)
	reg_read_m(RTL837X_REG_CPU_TAG);
	sfr_mask_data(0, 1, 1);
	sfr_mask_data(1, 3, 0);
	reg_write_m(RTL837X_REG_CPU_TAG);

	// Force MAC mode of the CPU port (port 9)
	// r6368:00000194 R6368-00000197
	reg_read_m(RTL837X_REG_MAC_FORCE_MODE + 9 * 4);
	sfr_mask_data(0, 0, 3); // Set bits 0, 1: Force link
	reg_write_m(RTL837X_REG_MAC_FORCE_MODE+ 9 * 4);

	// Sequence number of TX packets
	tx_seq = 0;
	REG_SET(RTL837X_NIC_INT_MSK, NIC_INT_RXIE);
}



void rtl8373_revision(void) __banked
{
	reg_read_m(RTL837X_REG_CHIP_INFO);
	sfr_mask_data(2, 0x0a, 0x0a); 	// Enable reading version
	reg_write_m(RTL837X_REG_CHIP_INFO);
	delay(50);

	reg_read_m(RTL837X_REG_CHIP_INFO);
	print_string("CPU revision: "); print_byte(sfr_data[2]); print_byte(sfr_data[2]); write_char('\n');
	sfr_mask_data(2, 0x0a, 0x00); 	// Enable reading version
	reg_write_m(RTL837X_REG_CHIP_INFO);
}


/*
 * The SoC manages Link-State for steering the LEDs and can set PHY-settings
 * automatically through Realtek's SMI (Simple Managagement) Interface, a
 * proprietary version of MDIO which for example allows for more PHYs on the same
 * bus.
 * Configure polling via SMI and the interface setup during boot.
 */
void init_smi(void) __banked
{
	print_string("\ninit_switch called\n");

	/* Set the SMI(i.e.I2C) type for PHY polling, 0b01 is 2.5/10G PHY. Disable (0b00) for the SFP-ports
	 * which are at port 8 and additionally at port 3 for a dual SFP device
	 */

	// Default: 0x00005555
	// Workaround for SDCC BUG 4070: SFR_DATA_U32 = 0x00005555;
	SFR_DATA_U16_UPPER = 0x0000;
	SFR_DATA_U16 = 0x5555;
	if (machine.n_10g == 2) {
		// 0x00015555, only change the bytes that differs from the default.
		SFR_DATA_16 = 0x01;
	} else if (machine.n_sfp == 2)
		// 0x00005515
		SFR_DATA_0 = 0x15;
	reg_write(RTL837X_REG_SMI_MAC_TYPE);

	// Configure polling of all PHYs by the MAC to detect link-state changes
	// Default: 0x000000ff
	// Workaround for SDCC BUG 4070: SFR_DATA_U32 = 0x000000ff;
	SFR_DATA_U16_UPPER = 0x0000;
	SFR_DATA_U16 = 0x00ff;
	if (!machine_detected.isRTL8373) {
		if (machine.n_sfp == 2) {
			// 0x000000f0, only change the bytes that differs from the default.
			SFR_DATA_0 = 0xf0;
		} else {
			// 0x000001f8, only change the bytes that differs from the default.
			SFR_DATA_8 = 0x01;
			SFR_DATA_0 = 0xf8;
		}
	}
	reg_write(RTL837X_REG_SMI_PORT_POLLING);
	// Enable MDC
	reg_read_m(RTL837X_REG_SMI_CTRL);
	sfr_mask_data(1, 0, 0x70); 	// Set bits 12-14 to enable MDC for SMI0-SMI2
	reg_write_m(RTL837X_REG_SMI_CTRL);
	delay(50);

	if (!machine_detected.isRTL8373) {
		// Change I2C addresses for SMI of the non-existent PHYs
		// r6450:000020e6 R6450-000000e6
		reg_read_m(RTL837X_REG_SMI_PORT6_9_ADDR);
		sfr_mask_data(1, 0x7c, 0);
		reg_write_m(RTL837X_REG_SMI_PORT6_9_ADDR);

		// r644c:0a418820 R644c-0a400820
		reg_read_m(RTL837X_REG_SMI_PORT0_5_ADDR);
		sfr_mask_data(2, 0x0f, 0);
		sfr_mask_data(1, 0x80, 0);
		reg_write_m(RTL837X_REG_SMI_PORT0_5_ADDR);
	}

	if (machine.n_10g == 2) {
		// Set address of second external PHY on port 8
		REG_SET(RTL837X_REG_SMI_PORT6_9_ADDR, 0x000040e6);
	}
}




void setup_i2c(void) __banked
{
	REG_SET(RTL837X_REG_I2C_MST_IF_CTRL, 0);
	// Configure SFP EEPROM address (0x50) as I2C device address
	// Configure SFP readings address (0x51) as I2C device address
	REG_WRITE(RTL837X_REG_I2C_CTRL, 0x00, 0x1 << (I2C_MEM_ADDR_WIDTH-16),  0x50 >> 5, (0x50 << 3) & 0xff);

	REG_SET(RTL837X_REG_I2C_CTRL2, 0);

	// HW Control register, enable I2C depending on PIN configuration
	reg_read_m(RTL837X_PIN_MUX_1);
	for (uint8_t sfp = 0; sfp < machine.n_sfp; sfp++) {
		uint8_t i2c = machine.sfp_port[sfp].i2c;
		uint8_t scl_bus = (i2c >> RTL837X_REG_I2C_SCL_SHIFT) & RTL837X_REG_I2C_SCL_MASK;
		uint8_t sda_bus = (i2c >> RTL837X_REG_I2C_SDA_SHIFT) & RTL837X_REG_I2C_SDA_MASK;
		print_string("Configuring I2C for SFP idx="); print_byte(sfp); print_string(" SCL="); print_byte(scl_bus); print_string(", SDA="); print_byte(sda_bus); write_char('\n');
		// `& RTL837X_REG_I2C_SCL_MASK` is needed to silens a compiler warning 110.
		switch (scl_bus & RTL837X_REG_I2C_SCL_MASK) {
			case 3:
				// Bit 5-6 0b10 -> SCL (implies enabled SDA on bus 3)
				sfr_mask_data(0, 0x60, 0x40);
				break;
			case 2: 
				// Bit 15-16 0b01 -> SCL
				sfr_mask_data(1, 0x80, 0x80);
				sfr_mask_data(2, 0x01, 0x00);
				break;
			case 1:
				// Bit 11-12 0b01 -> SCL
				sfr_mask_data(1, 0x18, 0x08);
				break;
			case 0:
				// Bit 7-8 0b01 -> SCL
				sfr_mask_data(0, 0x80, 0x80);
				sfr_mask_data(1, 0x01, 0x00);
				break;
		}

		switch (sda_bus) {
			case 4:
				// Bit 29 0b0 -> SDA
				sfr_mask_data(3, 0x20, 0x00);
				break;
			case 3:
				// Bit 5-6 0b10 -> SDA (implies enabled SCL on bus 3)
				sfr_mask_data(0, 0x60, 0x40);
				break;
			case 2:
				// Bit 17-18 0b01 -> SDA
				sfr_mask_data(2, 0x06, 0x02);
				break;
			case 1:
				// Bit 13-14 0b01 -> SDA
				sfr_mask_data(1, 0x60, 0x20);
				break;
			case 0:
				// Bit 9-10 0b01 -> SDA
				sfr_mask_data(1, 0x06, 0x02);
				break;
		}
	}
	reg_write_m(RTL837X_PIN_MUX_1);	
}


/* Give the switch a name carrying the tail of its MAC, so several of them on
 * one network are distinguishable out of the box. Called after the startup
 * config has been replayed and returns at once if that config already set a
 * name, so a configured switch does no work for it (suggested in review).
 *
 * Written without a loop on purpose. Locals - counters and pointers alike -
 * land in the 8051's internal-RAM overlay, and on an image with LACP and STP
 * both enabled that overlay is exhausted: a loop here makes the linker fail
 * with "Could not get 8 consecutive bytes in internal RAM for area OSEG".
 * Moving the code into its own function does not help; the overlay is shared
 * across the whole image. Hoisting the locals to xdata does not help either,
 * because itohex() is inline and brings its own frame. */
void set_hostname_default(void) __banked
{
	if (hostname[0] != NUL)
		return;

	strcpy((__xdata uint8_t *)hostname, "RTLPlayground-");
	hostname[14] = hex[uip_ethaddr.addr[3] >> 4];
	hostname[15] = hex[uip_ethaddr.addr[3] & 0xf];
	hostname[16] = hex[uip_ethaddr.addr[4] >> 4];
	hostname[17] = hex[uip_ethaddr.addr[4] & 0xf];
	hostname[18] = hex[uip_ethaddr.addr[5] >> 4];
	hostname[19] = hex[uip_ethaddr.addr[5] & 0xf];
	hostname[20] = NUL;
}
