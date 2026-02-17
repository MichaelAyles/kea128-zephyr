/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_NXP_KEA_REGS_H_
#define ZEPHYR_SOC_NXP_KEA_REGS_H_

#include <stdint.h>

typedef struct {
	volatile uint32_t SRSID;
	volatile uint32_t SOPT0;
	volatile uint32_t SOPT1;
	volatile uint32_t PINSEL;
	volatile uint32_t PINSEL1;
	volatile uint32_t SCGC;
	volatile uint32_t UUIDL;
	volatile uint32_t UUIDML;
	volatile uint32_t UUIDMH;
	volatile uint32_t CLKDIV;
} kea_sim_t;

#define KEA_SIM_BASE 0x40048000u
#define KEA_SIM ((kea_sim_t *)KEA_SIM_BASE)
#define KEA_SIM_CLKDIV_OUTDIV3_MASK (1u << 20)
#define KEA_SIM_CLKDIV_OUTDIV3_SHIFT 20u
#define KEA_SIM_CLKDIV_OUTDIV2_MASK (1u << 24)
#define KEA_SIM_CLKDIV_OUTDIV2_SHIFT 24u
#define KEA_SIM_CLKDIV_OUTDIV1_MASK (3u << 28)
#define KEA_SIM_CLKDIV_OUTDIV1_SHIFT 28u

/* SIM SCGC clock gates */
#define KEA_SIM_SCGC_PIT_MASK (1u << 1)
#define KEA_SIM_SCGC_FTM0_MASK (1u << 5)
#define KEA_SIM_SCGC_FTM1_MASK (1u << 6)
#define KEA_SIM_SCGC_FTM2_MASK (1u << 7)
#define KEA_SIM_SCGC_MSCAN_MASK (1u << 15)
#define KEA_SIM_SCGC_I2C0_MASK (1u << 16)
#define KEA_SIM_SCGC_I2C1_MASK (1u << 17)
#define KEA_SIM_SCGC_SPI0_MASK (1u << 18)
#define KEA_SIM_SCGC_SPI1_MASK (1u << 19)
#define KEA_SIM_SCGC_UART0_MASK (1u << 20)
#define KEA_SIM_SCGC_UART1_MASK (1u << 21)
#define KEA_SIM_SCGC_UART2_MASK (1u << 22)
#define KEA_SIM_SCGC_KBI0_MASK (1u << 24)
#define KEA_SIM_SCGC_KBI1_MASK (1u << 25)
#define KEA_SIM_SCGC_ADC_MASK (1u << 29)

/* SIM PINSEL1 routing */
#define KEA_SIM_PINSEL_I2C0PS_MASK (1u << 5)
#define KEA_SIM_PINSEL_SPI0PS_MASK (1u << 6)
#define KEA_SIM_PINSEL1_UART2PS_MASK (1u << 13)
#define KEA_SIM_PINSEL1_MSCANPS_MASK (1u << 16)
#define KEA_SIM_PINSEL1_I2C1PS_MASK (1u << 10)
#define KEA_SIM_PINSEL1_SPI1PS_MASK (1u << 11)

typedef struct {
	volatile uint8_t C1;
	volatile uint8_t C2;
	volatile uint8_t C3;
	volatile uint8_t C4;
	volatile uint8_t S;
} kea_ics_t;

#define KEA_ICS_BASE 0x40064000u
#define KEA_ICS ((kea_ics_t *)KEA_ICS_BASE)

#define KEA_ICS_C1_CLKS_MASK 0xC0u
#define KEA_ICS_C1_CLKS_SHIFT 6u
#define KEA_ICS_C2_BDIV_MASK 0xE0u
#define KEA_ICS_C2_BDIV_SHIFT 5u
#define KEA_ICS_S_CLKST_MASK 0x0Cu
#define KEA_ICS_S_CLKST_SHIFT 2u

typedef struct {
	volatile uint8_t BDH;
	volatile uint8_t BDL;
	volatile uint8_t C1;
	volatile uint8_t C2;
	volatile uint8_t S1;
	volatile uint8_t S2;
	volatile uint8_t C3;
	volatile uint8_t D;
} kea_uart_t;

#define KEA_UART0_BASE 0x4006A000u
#define KEA_UART1_BASE 0x4006B000u
#define KEA_UART2_BASE 0x4006C000u

#define KEA_UART_BDH_SBR_MASK 0x1Fu
#define KEA_UART_C2_RE_MASK 0x04u
#define KEA_UART_C2_TE_MASK 0x08u
#define KEA_UART_C2_RIE_MASK 0x20u
#define KEA_UART_C2_TIE_MASK 0x80u
#define KEA_UART_S1_PF_MASK 0x01u
#define KEA_UART_S1_FE_MASK 0x02u
#define KEA_UART_S1_NF_MASK 0x04u
#define KEA_UART_S1_OR_MASK 0x08u
#define KEA_UART_S1_RDRF_MASK 0x20u
#define KEA_UART_S1_TC_MASK 0x40u
#define KEA_UART_S1_TDRE_MASK 0x80u

typedef struct {
	volatile uint32_t SC1;
	volatile uint32_t SC2;
	volatile uint32_t SC3;
	volatile uint32_t SC4;
	volatile uint32_t R;
	volatile uint32_t CV;
	volatile uint32_t APCTL1;
	volatile uint32_t SC5;
} kea_adc_t;

#define KEA_ADC_BASE 0x4003B000u
#define KEA_ADC ((kea_adc_t *)KEA_ADC_BASE)

#define KEA_ADC_SC1_ADCH_MASK 0x1Fu
#define KEA_ADC_SC1_ADCH_SHIFT 0
#define KEA_ADC_SC1_ADCO_MASK 0x20u
#define KEA_ADC_SC1_AIEN_MASK 0x40u
#define KEA_ADC_SC1_COCO_MASK 0x80u
#define KEA_ADC_SC3_ADICLK_MASK 0x03u
#define KEA_ADC_SC3_MODE_MASK 0x0Cu
#define KEA_ADC_SC3_MODE_SHIFT 2

typedef struct {
	volatile uint32_t MCR;
	uint8_t reserved0[252];
	struct {
		volatile uint32_t LDVAL;
		volatile uint32_t CVAL;
		volatile uint32_t TCTRL;
		volatile uint32_t TFLG;
	} CHANNEL[2];
} kea_pit_t;

#define KEA_PIT_BASE 0x40037000u
#define KEA_PIT ((kea_pit_t *)KEA_PIT_BASE)

#define KEA_PIT_MCR_FRZ_MASK 0x1u
#define KEA_PIT_MCR_MDIS_MASK 0x2u
#define KEA_PIT_TCTRL_TEN_MASK 0x1u
#define KEA_PIT_TCTRL_TIE_MASK 0x2u
#define KEA_PIT_TFLG_TIF_MASK 0x1u

typedef struct {
	volatile uint32_t SC;
	volatile uint32_t CNT;
	volatile uint32_t MOD;
	struct {
		volatile uint32_t CnSC;
		volatile uint32_t CnV;
	} CONTROLS[6];
	uint8_t reserved0[16];
	volatile uint32_t CNTIN;
	volatile uint32_t STATUS;
	volatile uint32_t MODE;
	volatile uint32_t SYNC;
	volatile uint32_t OUTINIT;
	volatile uint32_t OUTMASK;
	volatile uint32_t COMBINE;
	volatile uint32_t DEADTIME;
	volatile uint32_t EXTTRIG;
	volatile uint32_t POL;
	volatile uint32_t FMS;
	volatile uint32_t FILTER;
	volatile uint32_t FLTCTRL;
	uint8_t reserved1[4];
	volatile uint32_t CONF;
	volatile uint32_t FLTPOL;
	volatile uint32_t SYNCONF;
	volatile uint32_t INVCTRL;
	volatile uint32_t SWOCTRL;
	volatile uint32_t PWMLOAD;
} kea_ftm_t;

#define KEA_FTM0_BASE 0x40038000u
#define KEA_FTM1_BASE 0x40039000u
#define KEA_FTM2_BASE 0x4003A000u

#define KEA_FTM_SC_PS_MASK 0x7u
#define KEA_FTM_SC_CLKS_MASK 0x18u
#define KEA_FTM_SC_CLKS_SHIFT 3
#define KEA_FTM_SC_TOF_MASK 0x80u
#define KEA_FTM_CNSC_ELSA_MASK 0x4u
#define KEA_FTM_CNSC_ELSB_MASK 0x8u
#define KEA_FTM_CNSC_MSB_MASK 0x20u
#define KEA_FTM_MODE_FTMEN_MASK 0x1u
#define KEA_FTM_MODE_WPDIS_MASK 0x4u

typedef struct {
	volatile uint8_t CANCTL0;
	volatile uint8_t CANCTL1;
	volatile uint8_t CANBTR0;
	volatile uint8_t CANBTR1;
	volatile uint8_t CANRFLG;
	volatile uint8_t CANRIER;
	volatile uint8_t CANTFLG;
	volatile uint8_t CANTIER;
	volatile uint8_t CANTARQ;
	volatile uint8_t CANTAAK;
	volatile uint8_t CANTBSEL;
	volatile uint8_t CANIDAC;
	uint8_t reserved0[1];
	volatile uint8_t CANMISC;
	volatile uint8_t CANRXERR;
	volatile uint8_t CANTXERR;
	volatile uint8_t CANIDAR_BANK_1[4];
	volatile uint8_t CANIDMR_BANK_1[4];
	volatile uint8_t CANIDAR_BANK_2[4];
	volatile uint8_t CANIDMR_BANK_2[4];
	union {
		volatile uint8_t REIDR0;
		volatile uint8_t RSIDR0;
	};
	union {
		volatile uint8_t REIDR1;
		volatile uint8_t RSIDR1;
	};
	volatile uint8_t REIDR2;
	volatile uint8_t REIDR3;
	volatile uint8_t REDSR[8];
	volatile uint8_t RDLR;
	uint8_t reserved1[1];
	volatile uint8_t RTSRH;
	volatile uint8_t RTSRL;
	union {
		volatile uint8_t TEIDR0;
		volatile uint8_t TSIDR0;
	};
	union {
		volatile uint8_t TEIDR1;
		volatile uint8_t TSIDR1;
	};
	volatile uint8_t TEIDR2;
	volatile uint8_t TEIDR3;
	volatile uint8_t TEDSR[8];
	volatile uint8_t TDLR;
	volatile uint8_t TBPR;
	volatile uint8_t TTSRH;
	volatile uint8_t TTSRL;
} kea_mscan_t;

#define KEA_MSCAN_BASE 0x40024000u
#define KEA_MSCAN ((kea_mscan_t *)KEA_MSCAN_BASE)

#define KEA_MSCAN_CANCTL0_INITRQ_MASK 0x01u
#define KEA_MSCAN_CANCTL0_SLPRQ_MASK 0x02u
#define KEA_MSCAN_CANCTL1_INITAK_MASK 0x01u
#define KEA_MSCAN_CANCTL1_SLPAK_MASK 0x02u
#define KEA_MSCAN_CANCTL1_LISTEN_MASK 0x10u
#define KEA_MSCAN_CANCTL1_LOOPB_MASK 0x20u
#define KEA_MSCAN_CANCTL1_CLKSRC_MASK 0x40u
#define KEA_MSCAN_CANCTL1_CANE_MASK 0x80u
#define KEA_MSCAN_CANBTR0_BRP_MASK 0x3Fu
#define KEA_MSCAN_CANBTR0_BRP_SHIFT 0
#define KEA_MSCAN_CANBTR0_SJW_MASK 0xC0u
#define KEA_MSCAN_CANBTR0_SJW_SHIFT 6
#define KEA_MSCAN_CANBTR1_TSEG1_MASK 0x0Fu
#define KEA_MSCAN_CANBTR1_TSEG1_SHIFT 0
#define KEA_MSCAN_CANBTR1_TSEG2_MASK 0x70u
#define KEA_MSCAN_CANBTR1_TSEG2_SHIFT 4
#define KEA_MSCAN_CANBTR1_SAMP_MASK 0x80u
#define KEA_MSCAN_CANRFLG_RXF_MASK 0x01u
#define KEA_MSCAN_CANRFLG_OVRIF_MASK 0x02u
#define KEA_MSCAN_CANRFLG_CSCIF_MASK 0x40u
#define KEA_MSCAN_CANRFLG_WUPIF_MASK 0x80u
#define KEA_MSCAN_CANRIER_RXFIE_MASK 0x01u
#define KEA_MSCAN_CANRIER_OVRIE_MASK 0x02u
#define KEA_MSCAN_CANTFLG_TXE_MASK 0x07u
#define KEA_MSCAN_CANTIER_TXEIE_MASK 0x07u
#define KEA_MSCAN_CANTBSEL_TX_MASK 0x07u
#define KEA_MSCAN_CANIDAC_IDAM_MASK 0x30u
#define KEA_MSCAN_CANIDAC_IDAM_SHIFT 4
#define KEA_MSCAN_RSIDR1_RSIDE_MASK 0x08u
#define KEA_MSCAN_RSIDR1_RSRTR_MASK 0x10u
#define KEA_MSCAN_RSIDR1_RSID2_0_MASK 0xE0u
#define KEA_MSCAN_TSIDR1_TSIDE_MASK 0x08u
#define KEA_MSCAN_TSIDR1_TSRTR_MASK 0x10u
#define KEA_MSCAN_TSIDR1_TSID2_0_MASK 0xE0u
#define KEA_MSCAN_TDLR_TDLC_MASK 0x0Fu

typedef struct {
	volatile uint8_t A1;
	volatile uint8_t F;
	volatile uint8_t C1;
	volatile uint8_t S;
	volatile uint8_t D;
	volatile uint8_t C2;
	volatile uint8_t FLT;
	volatile uint8_t RA;
	volatile uint8_t SMB;
	volatile uint8_t A2;
	volatile uint8_t SLTH;
	volatile uint8_t SLTL;
} kea_i2c_t;

#define KEA_I2C0_BASE 0x40066000u
#define KEA_I2C1_BASE 0x40067000u

#define KEA_I2C_C1_RSTA_MASK 0x04u
#define KEA_I2C_C1_TXAK_MASK 0x08u
#define KEA_I2C_C1_TX_MASK 0x10u
#define KEA_I2C_C1_MST_MASK 0x20u
#define KEA_I2C_C1_IICIE_MASK 0x40u
#define KEA_I2C_C1_IICEN_MASK 0x80u
#define KEA_I2C_S_RXAK_MASK 0x01u
#define KEA_I2C_S_IICIF_MASK 0x02u
#define KEA_I2C_S_SRW_MASK 0x04u
#define KEA_I2C_S_ARBL_MASK 0x10u
#define KEA_I2C_S_BUSY_MASK 0x20u
#define KEA_I2C_S_IAAS_MASK 0x40u

typedef struct {
	volatile uint8_t C1;
	volatile uint8_t C2;
	volatile uint8_t BR;
	volatile uint8_t S;
	uint8_t reserved0[1];
	volatile uint8_t D;
	uint8_t reserved1[1];
	volatile uint8_t M;
} kea_spi_t;

#define KEA_SPI0_BASE 0x40076000u
#define KEA_SPI1_BASE 0x40077000u

#define KEA_SPI_C1_LSBFE_MASK 0x01u
#define KEA_SPI_C1_CPHA_MASK 0x04u
#define KEA_SPI_C1_CPOL_MASK 0x08u
#define KEA_SPI_C1_MSTR_MASK 0x10u
#define KEA_SPI_C1_SPE_MASK 0x40u
#define KEA_SPI_BR_SPR_MASK 0x0Fu
#define KEA_SPI_BR_SPPR_MASK 0x70u
#define KEA_SPI_BR_SPPR_SHIFT 4
#define KEA_SPI_S_SPTEF_MASK 0x20u
#define KEA_SPI_S_SPRF_MASK 0x80u

typedef struct {
	volatile uint32_t PE;
	volatile uint32_t ES;
	volatile uint32_t SC;
	volatile uint32_t SP;
} kea_kbi_t;

#define KEA_KBI0_BASE 0x40079000u
#define KEA_KBI1_BASE 0x4007A000u
#define KEA_KBI0 ((kea_kbi_t *)KEA_KBI0_BASE)
#define KEA_KBI1 ((kea_kbi_t *)KEA_KBI1_BASE)

#define KEA_KBI_SC_KBMOD_MASK 0x1u
#define KEA_KBI_SC_KBIE_MASK 0x2u
#define KEA_KBI_SC_KBACK_MASK 0x4u
#define KEA_KBI_SC_KBF_MASK 0x8u
#define KEA_KBI_SC_KBSPEN_MASK 0x10u
#define KEA_KBI_SC_RSTKBSP_MASK 0x20u

typedef struct {
	volatile uint32_t PDOR;
	volatile uint32_t PSOR;
	volatile uint32_t PCOR;
	volatile uint32_t PTOR;
	volatile uint32_t PDIR;
	volatile uint32_t PDDR;
	volatile uint32_t PIDR;
} kea_gpio_t;

#define KEA_GPIOA_BASE 0x400FF000u
#define KEA_GPIOB_BASE 0x400FF040u
#define KEA_GPIOC_BASE 0x400FF080u

typedef struct {
	volatile uint8_t CS1;
	volatile uint8_t CS2;
	volatile uint16_t CNT;
	volatile uint16_t TOVAL;
	volatile uint16_t WIN;
} kea_wdog_t;

#define KEA_WDOG_BASE 0x40052000u
#define KEA_WDOG ((kea_wdog_t *)KEA_WDOG_BASE)

#define KEA_WDOG_CS1_STOP_MASK 0x01u
#define KEA_WDOG_CS1_WAIT_MASK 0x02u
#define KEA_WDOG_CS1_DBG_MASK 0x04u
#define KEA_WDOG_CS1_TST_MASK 0x18u
#define KEA_WDOG_CS1_UPDATE_MASK 0x20u
#define KEA_WDOG_CS1_INT_MASK 0x40u
#define KEA_WDOG_CS1_EN_MASK 0x80u

#define KEA_WDOG_CS2_CLK_MASK 0x03u
#define KEA_WDOG_CS2_CLK_SHIFT 0u
#define KEA_WDOG_CS2_CLK_BUS 0x00u
#define KEA_WDOG_CS2_CLK_LPO 0x01u
#define KEA_WDOG_CS2_CLK_ICSIRCLK 0x02u
#define KEA_WDOG_CS2_CLK_ERCLK 0x03u
#define KEA_WDOG_CS2_PRES_MASK 0x10u
#define KEA_WDOG_CS2_FLG_MASK 0x40u
#define KEA_WDOG_CS2_WIN_MASK 0x80u

#define KEA_WDOG_REFRESH_KEY1 0x02A6u
#define KEA_WDOG_REFRESH_KEY2 0x80B4u
#define KEA_WDOG_UNLOCK_KEY1 0x20C5u
#define KEA_WDOG_UNLOCK_KEY2 0x28D9u

#endif /* ZEPHYR_SOC_NXP_KEA_REGS_H_ */
