#ifndef __SIFIVE_UART_H_
#define __SIFVE_UART_H_

#include <vmm_types.h>

/*
 * Register offsets
 */

/* TXDATA */
#define SIFIVE_SERIAL_TXDATA_OFFS		0x0
#define SIFIVE_SERIAL_TXDATA_FULL_SHIFT		31
#define SIFIVE_SERIAL_TXDATA_FULL_MASK		(1 << SIFIVE_SERIAL_TXDATA_FULL_SHIFT)
#define SIFIVE_SERIAL_TXDATA_DATA_SHIFT		0
#define SIFIVE_SERIAL_TXDATA_DATA_MASK		(0xff << SIFIVE_SERIAL_TXDATA_DATA_SHIFT)

/* RXDATA */
#define SIFIVE_SERIAL_RXDATA_OFFS		0x4
#define SIFIVE_SERIAL_RXDATA_EMPTY_SHIFT	31
#define SIFIVE_SERIAL_RXDATA_EMPTY_MASK		(1 << SIFIVE_SERIAL_RXDATA_EMPTY_SHIFT)
#define SIFIVE_SERIAL_RXDATA_DATA_SHIFT		0
#define SIFIVE_SERIAL_RXDATA_DATA_MASK		(0xff << SIFIVE_SERIAL_RXDATA_DATA_SHIFT)

/* TXCTRL */
#define SIFIVE_SERIAL_TXCTRL_OFFS		0x8
#define SIFIVE_SERIAL_TXCTRL_TXCNT_SHIFT	16
#define SIFIVE_SERIAL_TXCTRL_TXCNT_MASK		(0x7 << SIFIVE_SERIAL_TXCTRL_TXCNT_SHIFT)
#define SIFIVE_SERIAL_TXCTRL_NSTOP_SHIFT	1
#define SIFIVE_SERIAL_TXCTRL_NSTOP_MASK		(1 << SIFIVE_SERIAL_TXCTRL_NSTOP_SHIFT)
#define SIFIVE_SERIAL_TXCTRL_TXEN_SHIFT		0
#define SIFIVE_SERIAL_TXCTRL_TXEN_MASK		(1 << SIFIVE_SERIAL_TXCTRL_TXEN_SHIFT)

/* RXCTRL */
#define SIFIVE_SERIAL_RXCTRL_OFFS		0xC
#define SIFIVE_SERIAL_RXCTRL_RXCNT_SHIFT	16
#define SIFIVE_SERIAL_RXCTRL_RXCNT_MASK		(0x7 << SIFIVE_SERIAL_TXCTRL_TXCNT_SHIFT)
#define SIFIVE_SERIAL_RXCTRL_RXEN_SHIFT		0
#define SIFIVE_SERIAL_RXCTRL_RXEN_MASK		(1 << SIFIVE_SERIAL_RXCTRL_RXEN_SHIFT)

/* IE */
#define SIFIVE_SERIAL_IE_OFFS			0x10
#define SIFIVE_SERIAL_IE_RXWM_SHIFT		1
#define SIFIVE_SERIAL_IE_RXWM_MASK		(1 << SIFIVE_SERIAL_IE_RXWM_SHIFT)
#define SIFIVE_SERIAL_IE_TXWM_SHIFT		0
#define SIFIVE_SERIAL_IE_TXWM_MASK		(1 << SIFIVE_SERIAL_IE_TXWM_SHIFT)

/* IP */
#define SIFIVE_SERIAL_IP_OFFS			0x14
#define SIFIVE_SERIAL_IP_RXWM_SHIFT		1
#define SIFIVE_SERIAL_IP_RXWM_MASK		(1 << SIFIVE_SERIAL_IP_RXWM_SHIFT)
#define SIFIVE_SERIAL_IP_TXWM_SHIFT		0
#define SIFIVE_SERIAL_IP_TXWM_MASK		(1 << SIFIVE_SERIAL_IP_TXWM_SHIFT)

/* DIV */
#define SIFIVE_SERIAL_DIV_OFFS			0x18
#define SIFIVE_SERIAL_DIV_DIV_SHIFT		0
#define SIFIVE_SERIAL_DIV_DIV_MASK		(0xffff << SIFIVE_SERIAL_IP_DIV_SHIFT)

/*
 * Config macros
 */

/*
 * SIFIVE_SERIAL_MAX_PORTS: maximum number of UARTs on a device that can
 *                          host a serial console
 */
#define SIFIVE_SERIAL_MAX_PORTS			8

/*
 * SIFIVE_DEFAULT_BAUD_RATE: default baud rate that the driver should
 *                           configure itself to use
 */
#define SIFIVE_DEFAULT_BAUD_RATE		921600

/* SIFIVE_SERIAL_NAME: our driver's name that we pass to the operating system */
#define SIFIVE_SERIAL_NAME			"sifive-serial"

/* SIFIVE_TTY_PREFIX: tty name prefix for SiFive serial ports */
#define SIFIVE_TTY_PREFIX			"ttySIF"

/* SIFIVE_TX_FIFO_DEPTH: depth of the TX FIFO (in bytes) */
#define SIFIVE_TX_FIFO_DEPTH			1024

/* SIFIVE_RX_FIFO_DEPTH: depth of the TX FIFO (in bytes) */
#define SIFIVE_RX_FIFO_DEPTH			1024

#if (SIFIVE_TX_FIFO_DEPTH != SIFIVE_RX_FIFO_DEPTH)
#error Driver does not support configurations with different TX, RX FIFO sizes
#endif

struct sifive_uart {
	u32 txdata;
	u32 rxdata;
	u32 txctrl;
	u32 rxctrl;
	u32 ie;
	u32 ip;
	u32 div;
};

struct sifive_uart_port {
	struct serial *p;
	struct sifive_uart *regs;
	bool skip_baudrate_config;
	u32 baudrate;
	u32 input_clock;
	u32 irq;
};

bool sifive_uart_lowlevel_can_getc(struct sifive_uart *regs);
u8 sifive_uart_lowlevel_getc(struct sifive_uart *regs);
bool sifive_uart_lowlevel_can_putc(struct sifive_uart *regs);
void sifive_uart_lowlevel_putc(struct sifive_uart *regs, u8 ch);
void sifive_uart_lowlevel_init(struct sifive_uart_port *port);

#endif