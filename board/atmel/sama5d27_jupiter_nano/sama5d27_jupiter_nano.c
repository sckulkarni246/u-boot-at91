// SPDX-License-Identifier: GPL-2.0+
/*
 * Configuration file for the SAMA5D27 Jupiter Nano.
 * This is based on SAMA5D27 SOM1 EK board.
 * 
 * Contributed by Shashank Kulkarni.
 * 
 * The Jupiter Nano is an open-source hardware board
 * based on Microchip's SAMA5D27 1 Gbit LPDDR2 processor.
 *
 * Copyright (C) 2017 Microchip Corporation
 *		      Wenyou.Yang <wenyou.yang@microchip.com>
 */

#include <common.h>
#include <debug_uart.h>
#include <fdtdec.h>
#include <init.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch/at91_common.h>
#include <asm/arch/atmel_pio4.h>
#include <asm/arch/atmel_mpddrc.h>
#include <asm/arch/atmel_sdhci.h>
#include <asm/arch/clk.h>
#include <asm/arch/gpio.h>
#include <asm/arch/sama5d2.h>

extern void at91_pda_detect(void);

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_CMD_USB
static void board_usb_hw_init(void)
{
	atmel_pio4_set_pio_output(AT91_PIO_PORTA, 27, 1);
}
#endif

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#ifdef CONFIG_DM_VIDEO
	at91_video_show_board_info();
#endif
	at91_pda_detect();
	return 0;
}
#endif

#ifdef CONFIG_DEBUG_UART_BOARD_INIT
static void board_uart1_hw_init(void)
{
	atmel_pio4_set_a_periph(AT91_PIO_PORTD, 2, ATMEL_PIO_PUEN_MASK);	/* URXD1 */
	atmel_pio4_set_a_periph(AT91_PIO_PORTD, 3, 0);	/* UTXD1 */

	at91_periph_clk_enable(ATMEL_ID_UART1);
}

void board_debug_uart_init(void)
{
	board_uart1_hw_init();
}
#endif

#ifdef CONFIG_BOARD_EARLY_INIT_F
int board_early_init_f(void)
{
#ifdef CONFIG_DEBUG_UART
	debug_uart_init();
#endif

	return 0;
}
#endif

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = gd->bd->bi_dram[0].start + 0x100;

#ifdef CONFIG_CMD_USB
	board_usb_hw_init();
#endif

	return 0;
}

int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}

int dram_init(void)
{
	return fdtdec_setup_mem_size_base();
}

#define MAC24AA_MAC_OFFSET	0xfa

#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
#ifdef CONFIG_I2C_EEPROM
	at91_set_ethaddr(MAC24AA_MAC_OFFSET);
#endif
	return 0;
}
#endif

/* SPL */
#ifdef CONFIG_SPL_BUILD
void spl_board_init(void)
{
}

static void ddrc_conf(struct atmel_mpddrc_config *ddrc)
{
	ddrc->md = (ATMEL_MPDDRC_MD_DBW_32_BITS | ATMEL_MPDDRC_MD_LPDDR2_SDRAM);

	ddrc->cr = (ATMEL_MPDDRC_CR_NC_COL_9 |
		    ATMEL_MPDDRC_CR_NR_ROW_13 |
		    ATMEL_MPDDRC_CR_CAS_DDR_CAS3 |
		    ATMEL_MPDDRC_CR_ZQ_SHORT |
		    ATMEL_MPDDRC_CR_NB_8BANKS |
		    ATMEL_MPDDRC_CR_DECOD_INTERLEAVED |
		    ATMEL_MPDDRC_CR_UNAL_SUPPORTED);

	ddrc->lpddr23_lpr = ATMEL_MPDDRC_LPDDR23_LPR_DS(0x3);

	ddrc->rtr = 0x288;
	/* Enable Adjust Refresh Rate */
	ddrc->rtr |= ATMEL_MPDDRC_RTR_ADJ_REF;

	ddrc->tpr0 = ((7 << ATMEL_MPDDRC_TPR0_TRAS_OFFSET) |
		      (3 << ATMEL_MPDDRC_TPR0_TRCD_OFFSET) |
		      (4 << ATMEL_MPDDRC_TPR0_TWR_OFFSET) |
		      (11 << ATMEL_MPDDRC_TPR0_TRC_OFFSET) |
		      (4 << ATMEL_MPDDRC_TPR0_TRP_OFFSET) |
		      (2 << ATMEL_MPDDRC_TPR0_TRRD_OFFSET) |
		      (2 << ATMEL_MPDDRC_TPR0_TWTR_OFFSET) |
		      (5 << ATMEL_MPDDRC_TPR0_TMRD_OFFSET));

	ddrc->tpr1 = ((21 << ATMEL_MPDDRC_TPR1_TRFC_OFFSET) |
		      (0 << ATMEL_MPDDRC_TPR1_TXSNR_OFFSET) |
		      (23 << ATMEL_MPDDRC_TPR1_TXSRD_OFFSET) |
		      (2 << ATMEL_MPDDRC_TPR1_TXP_OFFSET));

	ddrc->tpr2 = ((0 << ATMEL_MPDDRC_TPR2_TXARD_OFFSET) |
		      (0 << ATMEL_MPDDRC_TPR2_TXARDS_OFFSET) |
		      (4 << ATMEL_MPDDRC_TPR2_TRPA_OFFSET) |
		      (2 << ATMEL_MPDDRC_TPR2_TRTP_OFFSET) |
		      (10 << ATMEL_MPDDRC_TPR2_TFAW_OFFSET));

	ddrc->tim_cal = ATMEL_MPDDRC_CALR_ZQCS(15);

	/*
	 * According to the sama5d2 datasheet and the following values:
	 * T Sens = 0.75%/C, V Sens = 0.2%/mV, T driftrate = 1C/sec and V driftrate = 15 mV/s
	 * Warning: note that the values T driftrate and V driftrate are dependent on
	 * the application environment.
	 * ZQCS period is 1.5 / ((0.75 x 1) + (0.2 x 15)) = 0.4s
	 * If Trefi is 3.9us, we have: 400000 / 3.9 = 102564: we can maximize
	 * this timer to 0xFFFE.
	 */
	ddrc->cal_mr4 = ATMEL_MPDDRC_CAL_MR4_COUNT_CAL(0xFFFE);

	/*
	 * MR4 Read interval is dependent on the application environment.
	 * Here, we want to maximize this value as temperature is supposed
	 * to vary slowly in the application chosen.
	 * If Trefi is 3.9us, we have:
	 * (0xFFFE) 65534 x 3.9 = 0.25s between MR4 reads.
	 */
	ddrc->cal_mr4 |= ATMEL_MPDDRC_CAL_MR4_MR4R(0xFFFE);
}

void mem_init(void)
{
	struct at91_pmc *pmc = (struct at91_pmc *)ATMEL_BASE_PMC;
	struct atmel_mpddr *mpddrc = (struct atmel_mpddr *)ATMEL_BASE_MPDDRC;
	struct atmel_mpddrc_config ddrc_config;
	u32 reg;

	at91_periph_clk_enable(ATMEL_ID_MPDDRC);
	writel(AT91_PMC_DDR, &pmc->scer);

	ddrc_conf(&ddrc_config);

	reg = readl(&mpddrc->io_calibr);
	reg &= ~ATMEL_MPDDRC_IO_CALIBR_RDIV;
	reg |= ATMEL_MPDDRC_IO_CALIBR_LPDDR2_RZQ_48;
	reg &= ~ATMEL_MPDDRC_IO_CALIBR_TZQIO;
	reg |= ATMEL_MPDDRC_IO_CALIBR_TZQIO_(100);
	writel(reg, &mpddrc->io_calibr);

	writel(ATMEL_MPDDRC_RD_DATA_PATH_SHIFT_ONE_CYCLE,
	       &mpddrc->rd_data_path);

	lpddr2_init(ATMEL_BASE_MPDDRC, ATMEL_BASE_DDRCS, &ddrc_config);
}

void at91_pmc_init(void)
{
	u32 tmp;
	struct at91_pmc *pmc = (struct at91_pmc *)ATMEL_BASE_PMC;

	tmp = AT91_PMC_MCKR_PLLADIV_2 |
	      AT91_PMC_MCKR_MDIV_3 |
	      AT91_PMC_MCKR_CSS_MAIN;
	at91_mck_init_down(tmp);

	tmp = AT91_PMC_PLLAR_29 |
	      AT91_PMC_PLLXR_PLLCOUNT(0x3f) |
	      AT91_PMC_PLLXR_MUL(82) |
	      AT91_PMC_PLLXR_DIV(1);
	at91_plla_init(tmp);

	writel(0x0 << 8, &pmc->pllicpr);

	tmp = AT91_PMC_MCKR_H32MXDIV |
	      AT91_PMC_MCKR_PLLADIV_2 |
	      AT91_PMC_MCKR_MDIV_3 |
	      AT91_PMC_MCKR_CSS_PLLA;
	at91_mck_init(tmp);
}
#endif
