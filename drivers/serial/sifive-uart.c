#include <vmm_error.h>
#include <vmm_heap.h>
#include <vmm_host_io.h>
#include <vmm_host_irq.h>
#include <vmm_modules.h>
#include <vmm_devtree.h>
#include <vmm_devdrv.h>
#include <libs/stringlib.h>
#include <libs/mathlib.h>
#include <drv/clk.h>
#include <drv/serial.h>
#include <drv/serial/sifive-uart.h>

#define MODULE_DESC                     "Sifive Serial Driver"
#define MODULE_AUTHOR                   "Benshan Mei"
#define MODULE_LICENSE                  "GPL"
#define MODULE_IPRIORITY                (SERIAL_IPRIORITY+1)
#define MODULE_INIT                     sifive_uart_driver_init
#define MODULE_EXIT                     sifive_uart_driver_exit

static void sifive_uart_set_stop_bits(struct sifive_uart *regs, char nstop)
{
	u32 v;

	if (nstop < 1 || nstop > 2) {
		WARN_ON(1);
		return;
	}

	v = vmm_readl((void *)&regs->txctrl);
	v &= ~SIFIVE_SERIAL_TXCTRL_NSTOP_MASK;
	v |= (nstop - 1) << SIFIVE_SERIAL_TXCTRL_NSTOP_SHIFT;
	vmm_writel(v, (void *)&regs->txctrl);
}

bool sifive_uart_lowlevel_can_getc(struct sifive_uart *regs)
{
	u32 v = vmm_readl((void *)&regs->rxdata);

	return (v & SIFIVE_SERIAL_RXDATA_EMPTY_MASK) ? FALSE : TRUE;
}

u8 sifive_uart_lowlevel_getc(struct sifive_uart *regs)
{
	/* Wait until there is no data in the FIFO */
	while (!sifive_uart_lowlevel_can_getc(regs)) ;

	/* Read IO register */
	u32 v = vmm_readl((void *)&regs->rxdata);

	return (u8)(v & SIFIVE_SERIAL_RXDATA_DATA_MASK);
}

bool sifive_uart_lowlevel_can_putc(struct sifive_uart *regs)
{
	u32 v = vmm_readl((void *)&regs->txdata);

	return (v & SIFIVE_SERIAL_TXDATA_FULL_MASK ? FALSE : TRUE);
}

void sifive_uart_lowlevel_putc(struct sifive_uart *regs, u8 ch)
{
#if __riscv_atomic
	int r;
	do {
		asm volatile (
		"amoor.w %0, %2, %1\n"
		: "=r" (r), "+A" (*(volatile u32 *)(&regs->txdata))
		: "r" (ch)
		);
	} while (r < 0);
#else
    /* Wait until there is space in the FIFO */
    while (!sifive_uart_lowlevel_can_putc(regs))
		arch_cpu_relax();

	/* Send the character */
	vmm_writel(ch, (void *)&regs->txdata);
#endif
}

void sifive_uart_lowlevel_init(struct sifive_uart_port *port) {

    u16 div;
	u32 txctrl;

	struct sifive_uart *regs;

	regs = port->regs;

	if (!port->skip_baudrate_config) {
		// f_baud = f_in / (div + 1) => div = (f_in / f_baud) - 1
		// div = (f_in / f_baud) - 1
		//
		// The nearest integer solution for div requires rounding up as to not exceed
		// max_target_hz.
		//
		// div = ceil(f_in / f_baud) - 1
		//     = floor((f_in - 1 + f_baud) / f_baud) - 1
		//
		// This should not overflow as long as (f_in - 1 + f_baud) does not exceed
		// 2^32 - 1, which is unlikely since we represent frequencies in kHz.
        div = DIV_ROUND_UP(port->input_clock + port->baudrate - 1, port->baudrate);
		vmm_writel(div == 0 ? 0 : div - 1, (void *)&regs->div);
	}

	sifive_uart_set_stop_bits(regs, 1);

	/* Ignore all characters if CREAD is not set */ 
	txctrl = vmm_readl((void *)&regs->txctrl);
	txctrl &= SIFIVE_SERIAL_RXCTRL_RXEN_MASK;
	vmm_writel(txctrl, (void *)&regs->txctrl);
}

static vmm_irq_return_t sifive_uart_irq_handler(int irq, void *dev_id)
{
	u8 ch;
	u32 ip;
	struct sifive_uart_port *port = (struct sifive_uart_port *)dev_id;

	/* Get interrupt status */
	ip = vmm_readl((void *)&port->regs->ip);
	if (!ip) {
		return VMM_IRQ_NONE;
    }

	/* Handle RX interrupt */
    if (ip & SIFIVE_SERIAL_IP_RXWM_MASK) {
		/* Pull-out bytes from RX FIFO */
		while (sifive_uart_lowlevel_can_getc(port->regs)) {
			ch = sifive_uart_lowlevel_getc(port->regs);
			serial_rx(port->p, &ch, 1);
		}
	}

	/* Clear Interupt status */
	vmm_writel(ip, &port->regs->ip);

	return VMM_IRQ_HANDLED;
}

static u32 sifive_uart_tx(struct serial *p, u8 *src, size_t len)
{
	u32 i;
	struct sifive_uart_port *port = serial_tx_priv(p);

	for (i = 0; i < len; i++) {
		if (!sifive_uart_lowlevel_can_putc(port->regs)) {
			break;
		}
		sifive_uart_lowlevel_putc(port->regs, src[i]);
	}

	return i;
}

static int sifive_uart_driver_probe(struct vmm_device *dev)
{

	u32 ie;
	int rc;
	struct sifive_uart_port *port;

	port = vmm_zalloc(sizeof(struct sifive_uart_port));
	if (!port) {
		rc = VMM_ENOMEM;
		goto free_nothing;
	}

	rc = vmm_devtree_request_regmap(dev->of_node,
					(virtual_addr_t*)&port->regs,
					0, "SIFIVE UART");
	if (rc) {
		goto free_port;
	}

	if (vmm_devtree_read_u32(dev->of_node, "baudrate",
				 &port->baudrate)) {
		port->baudrate = SIFIVE_DEFAULT_BAUD_RATE;
	}

	rc = vmm_devtree_clock_frequency(dev->of_node, &port->input_clock);
	if (rc) {
		port->skip_baudrate_config = TRUE;
	} else {
		port->skip_baudrate_config = FALSE;
	}

	port->irq = vmm_devtree_irq_parse_map(dev->of_node, 0);
	if (!port->irq) {
		rc = VMM_ENODEV;
		goto free_reg;
	}
	if ((rc = vmm_host_irq_register(port->irq, dev->name,
					sifive_uart_irq_handler, port))) {
		goto free_reg;
	}

	/* Call low-level init function */
	sifive_uart_lowlevel_init(port);

	/* Create Serial Port */
	port->p = serial_create(dev, 256, sifive_uart_tx, port);
	if (VMM_IS_ERR_OR_NULL(port->p)) {
		rc = VMM_PTR_ERR(port->p);
		goto free_irq;
	}

	/* Save port pointer */
	dev->priv = port;

	/* clear all interrupts */
	vmm_writel(vmm_readl((void *)&port->regs->ip), &port->regs->ip);

	/* Enable interrupt generation when the transmit/receive FIFO watermark
	 * is reached on the SiFive UART
	 */
	ie = vmm_readl((void *)&port->regs->ie);
	vmm_writel(ie | SIFIVE_SERIAL_IE_TXWM_MASK | SIFIVE_SERIAL_IE_RXWM_MASK, &port->regs->ie);

	/* Enable transmits and set the watermark level to 1 */
    vmm_writel((1 << SIFIVE_SERIAL_TXCTRL_TXCNT_SHIFT) | SIFIVE_SERIAL_TXCTRL_TXEN_MASK,
                       (void *)&port->regs->txctrl);

    /* Enable receives and set the watermark level to 0 */
    vmm_writel((0 << SIFIVE_SERIAL_RXCTRL_RXCNT_SHIFT) | SIFIVE_SERIAL_RXCTRL_RXEN_MASK,
                       (void *)&port->regs->rxctrl);

	return VMM_OK;

free_irq:
	vmm_host_irq_unregister(port->irq, port);
free_reg:
	vmm_devtree_regunmap_release(dev->of_node,
				     (virtual_addr_t)port->regs, 0);
free_port:
	vmm_free(port);
free_nothing:
	return rc;
}

static int sifive_uart_driver_remove(struct vmm_device *dev)
{
	struct sifive_uart_port *port = dev->priv;

	if (!port) {
		return VMM_OK;
	}

	/* Mask RX interrupts, Disable interrupt generation when the receive 
	 * FIFO watermark is reached on the UART
 	 */
	port->regs->ie &= ~SIFIVE_SERIAL_IE_RXWM_MASK;
	vmm_writel(port->regs->ie, (void *)&port->regs->ie);

	/* Free-up resources */
	serial_destroy(port->p);
	vmm_host_irq_unregister(port->irq, port);
	vmm_devtree_regunmap_release(dev->of_node,
				     (virtual_addr_t)port->regs, 0);
	vmm_free(port);
	dev->priv = NULL;

	return VMM_OK;
}

static struct vmm_devtree_nodeid sifive_uart_devid_table[] = {
	{ .compatible = "sifive,fu540-c000-uart0" },
	{ .compatible = "sifive,uart0" },
	{ /* end of list */ },
};

static struct vmm_driver sifive_uart_driver = {
	.name = "sifive_uart_serial",
	.match_table = sifive_uart_devid_table,
	.probe = sifive_uart_driver_probe,
	.remove = sifive_uart_driver_remove,
};

static int __init sifive_uart_driver_init(void)
{
	return vmm_devdrv_register_driver(&sifive_uart_driver);
}

static void __exit sifive_uart_driver_exit(void)
{
	vmm_devdrv_unregister_driver(&sifive_uart_driver);
}

VMM_DECLARE_MODULE(MODULE_DESC,
		   MODULE_AUTHOR,
		   MODULE_LICENSE,
		   MODULE_IPRIORITY,
		   MODULE_INIT,
		   MODULE_EXIT);