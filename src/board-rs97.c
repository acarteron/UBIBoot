
#include <string.h>
#include <stdint.h>

#include "board.h"
#include "config.h"
#include "sdram.h"
#include "serial.h"
#include "utils.h"
#include "jz.h"
#include "jz4770-cpm.h"
#include "jz4770-ddrc.h"
#include "jz4740-gpio.h"

#define PIN_A (32*4 + 29)		/* Port 4 pin 29: A button */
#define PIN_X (32*4 + 28)		/* Port 4 pin 28: X button */
#define PIN_Y (32*4 + 27)		/* Port 4 pin 27: Y button */

/* Authorized values: 1 2 3 4 6 8 12 */
#define CFG_CDIV  1
#define CFG_HDIV 2
#define CFG_H2DIV 4
#define CFG_PDIV 4
#define CFG_MDIV  4
#define CFG_SDIV 4


#define FVCO_MIN  500*1000*1000	/* 500 MHz */
#define FVCO_MAX  1500*1000*1000	/* 1.5 GHz */

/* The frequency after the input divider must be between 10 and 50 MHz.
The highest divider yields the best resolution. */
#define INDIV(ext) (ext / 10000000)

#define OUTDIV(f) \
  (((f) >= FVCO_MIN) ? 1 : \
   ((f) >= (FVCO_MIN / 2)) ? 2 : \
   ((f) >= (FVCO_MIN / 4)) ? 4 : 8)

#define RATE(f) \
  ((f) < (FVCO_MIN / 8) ? FVCO_MIN : \
	(((f) > FVCO_MAX) ? FVCO_MAX : (f)))

#define FEEDBACK(f, ext) \
  ((((RATE(f) * OUTDIV(f)) / 1000) * INDIV(ext)) / (ext / 1000))

#define MHZ (1000 * 1000)
static inline unsigned int pll_calc_m_n_od(unsigned int speed, unsigned int xtal)
{
	const int pll_m_max = 0x7f, pll_m_min = 4;
	const int pll_n_max = 0x0f, pll_n_min = 2;

	int od[] = {1, 2, 4, 8};

	unsigned int plcr_m_n_od = 0;
	unsigned int distance;
	unsigned int tmp, raw;

	int i, j, k;
	int m, n;

	distance = 0xFFFFFFFF;

	for (i = 0; i < sizeof (od) / sizeof(int); i++) {
		/* Limit: 500MHZ <= CLK_OUT * OD <= 1500MHZ */
		if ((speed * od[i]) < FVCO_MIN || (speed * od[i]) > FVCO_MAX)
			continue;
		for (k = pll_n_min; k <= pll_n_max; k++) {
			n = k;
			
			/* Limit: 1MHZ <= XIN/N <= 50MHZ */
			if ((xtal / n) < (1 * MHZ))
				break;
			if ((xtal / n) > (50 * MHZ))
				continue;

			for (j = pll_m_min; j <= pll_m_max; j++) {
				m = j*2;

				raw = xtal * m / n;
				tmp = raw / od[i];

				tmp = (tmp > speed) ? (tmp - speed) : (speed - tmp);

				if (tmp < distance) {
					distance = tmp;
					
					plcr_m_n_od = (j << CPM_CPPCR_PLLM_BIT) 
						| (k << CPM_CPPCR_PLLN_BIT)
						| (i << CPM_CPPCR_PLLOD_BIT);

					if (!distance)	/* Match. */
						return plcr_m_n_od;
				}
			}
		}
	}
	return plcr_m_n_od;
}

static void pll_init(void)
{
	uint32_t cppcr;
	unsigned int i;
	static const uint8_t n2FR[13] = {
		0, 0, 1, 2, 3, 0, 4, 0, 5, 0, 0, 0, 6,
	};

	/* write REG_DDRC_CTRL 8 times to clear ddr fifo */
	for (i = 0; i < 8; i++)
		REG_DDRC_CTRL = 0;

	REG_CPM_CPCCR = ((CFG_CDIV - 1) << CPM_CPCCR_CDIV_BIT) |
	  ((unsigned int) n2FR[CFG_HDIV] << CPM_CPCCR_H0DIV_BIT) |
	  ((unsigned int) n2FR[CFG_H2DIV] << CPM_CPCCR_H2DIV_BIT) |
	  ((unsigned int) n2FR[CFG_PDIV] << CPM_CPCCR_PDIV_BIT) |
	  ((unsigned int) n2FR[CFG_MDIV]  << CPM_CPCCR_C1DIV_BIT)  |
	  ((unsigned int) n2FR[CFG_SDIV] << CPM_CPCCR_H1DIV_BIT) |
	  CPM_CPCCR_MEM | CPM_CPCCR_CE;

	cppcr = pll_calc_m_n_od(CFG_CPU_SPEED, CFG_EXTAL);
	cppcr |= CPM_CPPCR_PLLEN;

	REG_CPM_CPPCR = cppcr;

	/* Wait for a stable output */
	while (!__cpm_pll_is_on());
	while (!(REG_CPM_CPPCR & CPM_CPPCR_PLLS));
}

#if 0
void ddr_mem_init(int msel, int hl, int tsel, int arg)
{
	volatile int tmp_cnt;
	register unsigned int cpu_clk, ddr_twr;
	register unsigned int ddrc_cfg_reg=0, init_ddrc_mdelay=0;

	cpu_clk = CFG_CPU_SPEED;

	ddrc_cfg_reg = DDRC_CFG_TYPE_DDR2 | (DDR_ROW-12)<<10
		| (DDR_COL-8)<<8 | DDR_CS1EN<<7 | DDR_CS0EN<<6
		| ((DDR_CL-1) | 0x8)<<2 | DDR_BANK8<<1 | DDR_DW32;

	ddrc_cfg_reg |= DDRC_CFG_MPRT;

	init_ddrc_mdelay= tsel<<18 | msel<<16 | hl<<15 | arg << 14;
	ddr_twr = ((REG_DDRC_TIMING1 & DDRC_TIMING1_TWR_MASK) >> DDRC_TIMING1_TWR_BIT) + 1;
	REG_DDRC_CFG     = ddrc_cfg_reg;

	REG_DDRC_MDELAY = init_ddrc_mdelay  | 1 << 6;
	/***** init ddrc registers & ddr memory regs ****/
	/* AR: auto refresh */
	REG_DDRC_LMR = DDRC_LMR_CMD_AUREF | DDRC_LMR_START; //0x11;
	/* Wait for DDR_tRP */
	tmp_cnt = (cpu_clk / 1000000) * 1;
	while (tmp_cnt--);

	/* Wait for number of auto-refresh cycles */
	tmp_cnt = (cpu_clk / 1000000) * 10;
	while (tmp_cnt--);

	/* Set CKE High */
	REG_DDRC_CTRL = DDRC_CTRL_CKE; // ?

	/* Wait for number of auto-refresh cycles */
	tmp_cnt = (cpu_clk / 1000000) * 1;
	while (tmp_cnt--);

	/* PREA */
	REG_DDRC_LMR =  DDRC_LMR_CMD_PREC | DDRC_LMR_START; //0x1;

	/* Wait for DDR_tRP */
	tmp_cnt = (cpu_clk / 1000000) * 1;
	while (tmp_cnt--);

	/* EMR2: extend mode register2 */
	REG_DDRC_LMR = DDRC_LMR_BA_EMRS2 | DDRC_LMR_CMD_LMR | DDRC_LMR_START;//0x221;

	/* EMR3: extend mode register3 */
	REG_DDRC_LMR = DDRC_LMR_BA_EMRS3 | DDRC_LMR_CMD_LMR | DDRC_LMR_START;//0x321;

	/* EMR1: extend mode register1 */
	REG_DDRC_LMR = (DDR_EMRS1_DQS_DIS << 16) | DDRC_LMR_BA_EMRS1 | DDRC_LMR_CMD_LMR | DDRC_LMR_START;

	/* wait DDR_tMRD */
	tmp_cnt = (cpu_clk / 1000000) * 1;
	while (tmp_cnt--);

	/* MR - DLL Reset A1A0 burst 2 */
	REG_DDRC_LMR = ((ddr_twr-1)<<9 | DDR2_MRS_DLL_RST | DDR_CL<<4 | DDR_MRS_BL_4)<< 16
		| DDRC_LMR_BA_MRS | DDRC_LMR_CMD_LMR | DDRC_LMR_START;

	/* wait DDR_tMRD */
	tmp_cnt = (cpu_clk / 1000000) * 1;
	while (tmp_cnt--);

	/* PREA */
	REG_DDRC_LMR =  DDRC_LMR_CMD_PREC | DDRC_LMR_START; //0x1;

	/* Wait for DDR_tRP */
	tmp_cnt = (cpu_clk / 1000000) * 1;
	while (tmp_cnt--);

	/* AR: auto refresh */
	REG_DDRC_LMR = DDRC_LMR_CMD_AUREF | DDRC_LMR_START; //0x11;
	/* Wait for DDR_tRP */
	tmp_cnt = (cpu_clk / 1000000) * 1;
	while (tmp_cnt--);

	REG_DDRC_LMR = DDRC_LMR_CMD_AUREF | DDRC_LMR_START; //0x11;

	/* Wait for DDR_tRP */
	tmp_cnt = (cpu_clk / 1000000) * 1;
	while (tmp_cnt--);

	/* MR - DLL Reset End */
	REG_DDRC_LMR = ((ddr_twr-1)<<9 | DDR_CL<<4 | DDR_MRS_BL_4)<< 16
		| DDRC_LMR_BA_MRS | DDRC_LMR_CMD_LMR | DDRC_LMR_START;

	/* wait 200 tCK */
	tmp_cnt = (cpu_clk / 1000000) * 2;
	while (tmp_cnt--);

	/* EMR1 - OCD Default */
	REG_DDRC_LMR = (DDR_EMRS1_DQS_DIS | DDR_EMRS1_OCD_DFLT) << 16
		| DDRC_LMR_BA_EMRS1 | DDRC_LMR_CMD_LMR | DDRC_LMR_START;

	/* EMR1 - OCD Exit */
	REG_DDRC_LMR = (DDR_EMRS1_DQS_DIS << 16) | DDRC_LMR_BA_EMRS1 | DDRC_LMR_CMD_LMR | DDRC_LMR_START;

	/* wait DDR_tMRD */
	tmp_cnt = (cpu_clk / 1000000) * 1;
	while (tmp_cnt--);
}

int initdram(int board_type)
{
	u32 ddr_cfg;
	u32 rows, cols, dw, banks;
	unsigned long size;
	ddr_cfg = REG_DDRC_CFG;
	rows = 12 + ((ddr_cfg & DDRC_CFG_ROW_MASK) >> DDRC_CFG_ROW_BIT);
	cols = 8 + ((ddr_cfg & DDRC_CFG_COL_MASK) >> DDRC_CFG_COL_BIT);
	
	dw = (ddr_cfg & DDRC_CFG_DW) ? 4 : 2;
	banks = (ddr_cfg & DDRC_CFG_BA) ? 8 : 4;
	
	size = (1 << (rows + cols)) * dw * banks;
	size *= (DDR_CS1EN + DDR_CS0EN);

	return size;
}

#define DEF_DDR_CVT 0
#define DDR_USE_FIRST_ARGS 0
/* DDR sdram init */
void sdram_init(void)
{
	//add driver power
	REG_EMC_PMEMPS2 |= (3 << 18);

	REG_DMAC_DMADCKE(0) = 0x3f;
	REG_DMAC_DMADCKE(1) = 0x3f;
	int i, num = 0, tsel = 0, msel, hl;
	volatile unsigned int tmp_cnt;
	register unsigned int tmp, cpu_clk, mem_clk, ddr_twr, ns, ns_int;
	register unsigned int ddrc_timing1_reg=0, ddrc_timing2_reg=0;
	register unsigned int init_ddrc_refcnt=0, init_ddrc_dqs=0, init_ddrc_ctrl=0;

	register unsigned int memsize, ddrc_mmap0_reg, ddrc_mmap1_reg;
	register unsigned int mem_base0, mem_base1, mem_mask0, mem_mask1;

	cpu_clk = CFG_CPU_SPEED;

	mem_clk = __cpm_get_mclk();
	ns = 1000000000 / mem_clk; /* ns per tck ns <= real value */

	/* ACTIVE to PRECHARGE command period */
	tmp = (DDR_tRAS%ns == 0) ? (DDR_tRAS/ns) : (DDR_tRAS/ns+1);
	if (tmp < 1) tmp = 1;
	if (tmp > 31) tmp = 31;
	ddrc_timing1_reg = ((tmp/2) << DDRC_TIMING1_TRAS_BIT);

	/* READ to PRECHARGE command period. */
	tmp = (DDR_tRTP%ns == 0) ? (DDR_tRTP/ns) : (DDR_tRTP/ns+1);
	if (tmp < 1) tmp = 1;
	if (tmp > 4) tmp = 4;
	ddrc_timing1_reg |= ((tmp-1) << DDRC_TIMING1_TRTP_BIT);
	
	/* PRECHARGE command period. */
	tmp = (DDR_tRP%ns == 0) ? DDR_tRP/ns : (DDR_tRP/ns+1);
	if (tmp < 1) tmp = 1;
	if (tmp > 8) tmp = 8;
	ddrc_timing1_reg |= ((tmp-1) << DDRC_TIMING1_TRP_BIT);

	/* ACTIVE to READ or WRITE command period. */
	tmp = (DDR_tRCD%ns == 0) ? DDR_tRCD/ns : (DDR_tRCD/ns+1);
	if (tmp < 1) tmp = 1;
	if (tmp > 8) tmp = 8;
	ddrc_timing1_reg |= ((tmp-1) << DDRC_TIMING1_TRCD_BIT);

	/* ACTIVE to ACTIVE command period. */
	tmp = (DDR_tRC%ns == 0) ? DDR_tRC/ns : (DDR_tRC/ns+1);
	if (tmp < 3) tmp = 3;
	if (tmp > 31) tmp = 31;
	ddrc_timing1_reg |= ((tmp/2) << DDRC_TIMING1_TRC_BIT);

	/* ACTIVE bank A to ACTIVE bank B command period. */
	tmp = (DDR_tRRD%ns == 0) ? DDR_tRRD/ns : (DDR_tRRD/ns+1);
	if (tmp < 2) tmp = 2;
	if (tmp > 4) tmp = 4;
	ddrc_timing1_reg |= ((tmp-1) << DDRC_TIMING1_TRRD_BIT);


	/* WRITE Recovery Time defined by register MR of DDR2 memory */
	tmp = (DDR_tWR%ns == 0) ? DDR_tWR/ns : (DDR_tWR/ns+1);
	tmp = (tmp < 1) ? 1 : tmp;
	tmp = (tmp < 2) ? 2 : tmp;
	tmp = (tmp > 6) ? 6 : tmp;
	ddrc_timing1_reg |= ((tmp-1) << DDRC_TIMING1_TWR_BIT);
	ddr_twr = tmp; 

	/* WRITE to READ command delay. */ 
	tmp = (DDR_tWTR%ns == 0) ? DDR_tWTR/ns : (DDR_tWTR/ns+1);
	if (tmp > 4) tmp = 4;
	ddrc_timing1_reg |= ((tmp-1) << DDRC_TIMING1_TWTR_BIT);


	/* WRITE to READ command delay. */ 
	tmp = DDR_tWTR/ns;
	if (tmp < 1) tmp = 1;
	if (tmp > 4) tmp = 4;
	ddrc_timing1_reg |= ((tmp-1) << DDRC_TIMING1_TWTR_BIT);

	/* AUTO-REFRESH command period. */
	tmp = (DDR_tRFC%ns == 0) ? DDR_tRFC/ns : (DDR_tRFC/ns+1);
	if (tmp > 31) tmp = 31;
	ddrc_timing2_reg = ((tmp/2) << DDRC_TIMING2_TRFC_BIT);

	/* Minimum Self-Refresh / Deep-Power-Down time */
	tmp = DDR_tMINSR/ns;
	if (tmp < 9) tmp = 9;
	if (tmp > 129) tmp = 129;
	ddrc_timing2_reg |= (((tmp-1)/8-1) << DDRC_TIMING2_TMINSR_BIT);
	ddrc_timing2_reg |= (DDR_tXP-1)<<4 | (DDR_tMRD-1);

	init_ddrc_refcnt = DDR_CLK_DIV << 1 | DDRC_REFCNT_REF_EN;

	ns_int = (1000000000%mem_clk == 0) ?
		(1000000000/mem_clk) : (1000000000/mem_clk+1);
	tmp = DDR_tREFI/ns_int;
	tmp = tmp / (16 * (1 << DDR_CLK_DIV)) - 1;
	if (tmp > 0xfff)
		tmp = 0xfff;
	if (tmp < 1)
		tmp = 1;

	init_ddrc_refcnt |= tmp << DDRC_REFCNT_CON_BIT;
	init_ddrc_dqs = DDRC_DQS_AUTO | DDRC_DQS_DET;

	/* precharge power down, disable power down */
	/* precharge power down, if set active power down, |= DDRC_CTRL_ACTPD */
	init_ddrc_ctrl = DDRC_CTRL_PDT_DIS | DDRC_CTRL_PRET_8 | DDRC_CTRL_UNALIGN | DDRC_CTRL_CKE;
	/* Add Jz4760 chip here. Jz4760 chip have no cvt */
#define MAX_TSEL_VALUE 4
#define MAX_DELAY_VALUES 16 /* quars (2) * hls (2) * msels (4) */
	int index = 0, quar;

	msel = index/4;
	hl = ((index/2)&1)^1;
	quar = index&1;

	/* reset ddrc_controller */
	REG_DDRC_CTRL = DDRC_CTRL_RESET;
	
	/* Wait for precharge, > 200us */
	tmp_cnt = (cpu_clk / 1000000) * 200;
	while (tmp_cnt--);
	
	REG_DDRC_CTRL = 0x0;
	REG_DDRC_TIMING1 = ddrc_timing1_reg;
	REG_DDRC_TIMING2 = ddrc_timing2_reg;

	ddr_mem_init(msel, hl, tsel, quar);

	memsize = initdram(0);
	mem_base0 = DDR_MEM_PHY_BASE >> 24;
	mem_base1 = (DDR_MEM_PHY_BASE + memsize / (DDR_CS1EN + DDR_CS0EN)) >> 24;
	mem_mask1 = mem_mask0 = 0xff &
		~(((memsize/(DDR_CS1EN+DDR_CS0EN) >> 24)
		   - 1) & DDRC_MMAP_MASK_MASK);
	
	ddrc_mmap0_reg = mem_base0 << DDRC_MMAP_BASE_BIT | mem_mask0;
	ddrc_mmap1_reg = mem_base1 << DDRC_MMAP_BASE_BIT | mem_mask1;

	REG_DDRC_MMAP0 = ddrc_mmap0_reg;
	REG_DDRC_MMAP1 = ddrc_mmap1_reg;
	REG_DDRC_REFCNT = init_ddrc_refcnt;

	/* Enable DLL Detect */
	REG_DDRC_DQS    = init_ddrc_dqs;
			
	/* Set CKE High */
	REG_DDRC_CTRL = init_ddrc_ctrl;

	/* Wait for number of auto-refresh cycles */
	tmp_cnt = (cpu_clk / 1000000) * 10;
	while (tmp_cnt--);
	
	/* Auto Refresh */
	REG_DDRC_LMR = DDRC_LMR_CMD_AUREF | DDRC_LMR_START; //0x11;
	
	/* Wait for number of auto-refresh cycles */
	tmp_cnt = (cpu_clk / 1000000) * 10;
	while (tmp_cnt--);
}
#else
static void ddr_mem_init(void)
{
	/* TODO: Add support for CL=1.5 CL=2.5 CL=3.5 CL=4.5 */
	static const uint8_t ddr_cl_lookup[5] = {
		0, 9, 10, 11, 12,
	};

	unsigned int ddr_twr =
#if defined(CONFIG_SDRAM_DDR2)
	  ((REG_DDRC_TIMING1 & DDRC_TIMING1_TWR_MASK) >> DDRC_TIMING1_TWR_BIT) + 1;
#else
	  1;
#endif

	REG_DDRC_CFG =
#if defined(CONFIG_SDRAM_DDR1)
	  DDRC_CFG_TYPE_DDR1 | DDRC_CFG_BTRUN |
#elif defined(CONFIG_SDRAM_MDDR)
	  DDRC_CFG_TYPE_MDDR | DDRC_CFG_BTRUN |
#elif defined(CONFIG_SDRAM_DDR2)
	  DDRC_CFG_TYPE_DDR2 |
#else
#error No supported SDRAM type defined
#endif
	  DDRC_CFG_MPRT |
	  ((DDR_ROW - 12) << DDRC_CFG_ROW_BIT) |
	  ((DDR_COL - 8) << DDRC_CFG_COL_BIT) |
	  (DDR_CS1EN << 7) |
	  (DDR_CS0EN << 6) |
	  ((unsigned int) ddr_cl_lookup[DDR_CL - 1] << DDRC_CFG_CL_BIT) |
	  (DDR_BANK8 << 1) |
	  DDR_DW32;

	REG_DDRC_MDELAY = DDRC_MDELAY_MAUTO |
	  (DDR_TSEL << DDRC_CFG_TSEL_BIT) |
	  (DDR_MSEL << DDRC_CFG_MSEL_BIT) |
	  (DDR_HL << 15) | (DDR_QUAR << 14);
	udelay(10);

#if defined(CONFIG_SDRAM_DDR2)
	/* PREA */
	REG_DDRC_LMR =  DDRC_LMR_CMD_PREC | DDRC_LMR_START;
	udelay(1);

	/* EMR2: extend mode register2 */
	REG_DDRC_LMR = DDRC_LMR_BA_EMRS2 | DDRC_LMR_CMD_LMR | DDRC_LMR_START;

	/* EMR3: extend mode register3 */
	REG_DDRC_LMR = DDRC_LMR_BA_EMRS3 | DDRC_LMR_CMD_LMR | DDRC_LMR_START;

	/* EMR1: extend mode register1 */
	REG_DDRC_LMR = ((DDR_EMRS1_DIC_HALF) << 16) | DDRC_LMR_BA_EMRS1 | DDRC_LMR_CMD_LMR | DDRC_LMR_START;
	udelay(1);

	/* MR - DLL Reset A1A0 burst 2 */
	REG_DDRC_LMR = ((ddr_twr - 1) << 9 | DDR2_MRS_DLL_RST | DDR_CL << 4 | DDR_MRS_BL_4) << 16
		| DDRC_LMR_BA_MRS | DDRC_LMR_CMD_LMR | DDRC_LMR_START;
	udelay(1);
#endif

	/* PREA */
	REG_DDRC_LMR =  DDRC_LMR_CMD_PREC | DDRC_LMR_START; //0x1;
	udelay(1);

	/* AR: auto refresh */
	REG_DDRC_LMR = DDRC_LMR_CMD_AUREF | DDRC_LMR_START; //0x11;
	udelay(1);
	REG_DDRC_LMR = DDRC_LMR_CMD_AUREF | DDRC_LMR_START; //0x11;
	udelay(1);

	/* MR - DLL Reset End */
	REG_DDRC_LMR = ((ddr_twr-1)<<9 | DDR_CL<<4 | DDR_MRS_BL_4)<< 16
		| DDRC_LMR_BA_MRS | DDRC_LMR_CMD_LMR | DDRC_LMR_START;
	udelay(2);

#if defined(CONFIG_SDRAM_DDR2)
	/* EMR1 - OCD Default */
	REG_DDRC_LMR = (DDR_EMRS1_DIC_HALF | DDR_EMRS1_OCD_DFLT) << 16 | DDRC_LMR_BA_EMRS1 | DDRC_LMR_CMD_LMR | DDRC_LMR_START;

	/* EMR1 - OCD Exit */
	REG_DDRC_LMR = ((DDR_EMRS1_DIC_HALF) << 16) | DDRC_LMR_BA_EMRS1 | DDRC_LMR_CMD_LMR | DDRC_LMR_START;
#else
	/* EMR: extend mode register */
	REG_DDRC_LMR = (DDR_EMRS_DS_FULL | DDR_EMRS_PRSR_ALL) << 16
		| DDRC_LMR_BA_M_EMRS | DDRC_LMR_CMD_LMR | DDRC_LMR_START;
#endif
	udelay(1);
}

static unsigned int constrain(unsigned int x,
			unsigned int min, unsigned int max)
{
	if (x < min)
		x = min;
	if (x > max)
		x = max;
	return x;
}

static void sdram_init(void)
{
	unsigned int mem_mask, mem_base, mem_clk, ps, timing, tmp, ns_int;
	unsigned int mem_size = get_memory_size();

	__cpm_start_ddr();

	mem_clk = __cpm_get_mclk();
	SERIAL_PUTS_ARGI("SDRAM running at ", mem_clk / 1000000, " MHz.\n");

	ps = 1000000000 / (mem_clk / 1000); /* ns per tck ns <= real value */
	ns_int = (1000000000 % mem_clk == 0) ?
		(1000000000 / mem_clk) : (1000000000 / mem_clk + 1);

	/* reset ddrc_controller */
	REG_DDRC_CTRL = DDRC_CTRL_RESET;
	udelay(300);

	REG_DDRC_CTRL = DDRC_CTRL_PDT_DIS | DDRC_CTRL_PRET_8 |
	  DDRC_CTRL_UNALIGN | DDRC_CTRL_CKE;

	/* ACTIVE to PRECHARGE command period */
	tmp = DDR_GET_VALUE(DDR_tRAS, ps);
	tmp = constrain(tmp, 1, 31);
	timing = (((tmp) / 2) << DDRC_TIMING1_TRAS_BIT);

	/* READ to PRECHARGE command period. */
	tmp = DDR_GET_VALUE(DDR_tRTP, ps);
	tmp = constrain(tmp, 1, 4);
	timing |= ((tmp - 1) << DDRC_TIMING1_TRTP_BIT);

	/* PRECHARGE command period. */
	tmp = DDR_GET_VALUE(DDR_tRP, ps);
	tmp = constrain(tmp, 1, 8);
	timing |= ((tmp - 1) << DDRC_TIMING1_TRP_BIT);

	/* ACTIVE to READ or WRITE command period. */
	tmp = DDR_GET_VALUE(DDR_tRCD, ps);
	tmp = constrain(tmp, 1, 8);
	timing |= ((tmp - 1) << DDRC_TIMING1_TRCD_BIT);

	/* ACTIVE to ACTIVE command period. */
	tmp = DDR_GET_VALUE(DDR_tRC, ps);
	tmp = constrain(tmp, 3, 31);
	timing |= ((tmp / 2) << DDRC_TIMING1_TRC_BIT);

	/* ACTIVE bank A to ACTIVE bank B command period. */
	tmp = DDR_GET_VALUE(DDR_tRRD, ps);
	tmp = constrain(tmp, 2, 4);
	timing |= ((tmp - 1) << DDRC_TIMING1_TRRD_BIT);

	/* WRITE Recovery Time defined by register MR of DDR2 memory */
	tmp = DDR_GET_VALUE(DDR_tWR, ps);
	tmp = constrain(tmp, 2, 6);
	timing |= ((tmp - 1) << DDRC_TIMING1_TWR_BIT);

	/* WRITE to READ command delay. */
	tmp = DDR_tWTR < 5
		? DDR_tWTR /* unit is tCK */
		: DDR_GET_VALUE(DDR_tWTR, ps); /* unit is ps */
	tmp = constrain(tmp, 1, 4);
	timing |= ((tmp - 1) << DDRC_TIMING1_TWTR_BIT);

	REG_DDRC_TIMING1 = timing;

	/* AUTO-REFRESH command period. */
	tmp = DDR_GET_VALUE(DDR_tRFC, ps);
	tmp = constrain(tmp, 0, 31);
	timing |= ((tmp / 2) << DDRC_TIMING2_TRFC_BIT);

	/* Minimum Self-Refresh / Deep-Power-Down time */
	tmp = DDR_tMINSR;
	tmp = constrain(tmp, 9, 129);
	tmp = ((tmp - 1) % 8 == 0) ? ((tmp - 1) / 8 - 1) : ((tmp - 1) / 8);
	timing |= (tmp << DDRC_TIMING2_TMINSR_BIT);
	timing |= (DDR_tXP - 1) << 4 | (DDR_tMRD - 1);

	REG_DDRC_TIMING2 = timing;
	REG_DDRC_DQS_ADJ = 0;

	ddr_mem_init();

	mem_mask = 0xff &
		~(((mem_size / (DDR_CS1EN + DDR_CS0EN) >> 24) - 1) &
		  DDRC_MMAP_MASK_MASK);

	mem_base = DDR_MEM_PHY_BASE >> 24;
	REG_DDRC_MMAP0 = mem_base << DDRC_MMAP_BASE_BIT | mem_mask;

	mem_base = (DDR_MEM_PHY_BASE + mem_size / (DDR_CS1EN + DDR_CS0EN)) >> 24;
	REG_DDRC_MMAP1 = mem_base << DDRC_MMAP_BASE_BIT | mem_mask;

	tmp = (DDR_tREFI / ns_int) / (16 * (1 << DDR_CLK_DIV)) - 1;
	tmp = constrain(tmp, 1, 0xfff);

	REG_DDRC_REFCNT = (tmp << DDRC_REFCNT_CON_BIT) |
	  (DDR_CLK_DIV << 1) | DDRC_REFCNT_REF_EN;

	/* Enable DLL Detect */
	REG_DDRC_DQS = DDRC_DQS_AUTO | DDRC_DQS_DET | DDRC_DQS_SRDET;
	while (!(REG_DDRC_DQS & DDRC_DQS_READY));

	/* Auto Refresh */
	REG_DDRC_LMR = DDRC_LMR_CMD_AUREF | DDRC_LMR_START;
	udelay(500);
}
#endif

void board_init(void)
{
#ifdef USE_SERIAL
	/* UART0 */
	__gpio_as_func(GPIOF, 0, 0);
	__gpio_as_func(GPIOF, 3, 0);

	/* UART1 */
	__gpio_as_func(GPIOD, 26, 0);
	__gpio_as_func(GPIOD, 28, 0);

	/* UART2 */
	__gpio_as_func(GPIOC, 28, 0);
	__gpio_as_func(GPIOC, 30, 0);

	/* UART3 */
	__gpio_as_func(GPIOD, 12, 0);
	__gpio_as_func(GPIOE, 5, 1);

	REG_CPM_CLKGR0 &= ~BIT(15 + LOG_UART);
	serial_init();
#endif

#ifdef BKLIGHT_ON
	light(1);
#endif

	//__gpio_as_msc0_4bit();
	__cpm_start_msc0();
	__cpm_select_msc_clk(0, 1);

	pll_init();
	sdram_init();

	/* This is magic. If you know what Cop0 register $5 config 4 does,
	 * please tell us. */
	asm volatile ("li $2, 0xa9000000\n\t"
			"mtc0 $2, $5, 4\n\t"
			"nop\n\t"
			::"r"(2));
}

void light(int en)
{
	__gpio_as_output(GPIOD, 11);

	if (en)
		__gpio_set_pin(GPIOD, 11);
	else
		__gpio_clear_pin(GPIOD, 11);
}

int alt_key_pressed(void)
{
#if 0
	__gpio_as_input(PIN_X);
	return !__gpio_get_pin(PIN_X);
#else
	return 0;
#endif
}

int alt2_key_pressed(void)
{
#if 0
	__gpio_as_input(PIN_Y);
	return !__gpio_get_pin(PIN_Y);
#else
	return 0;
#endif
}

int alt3_key_pressed(void)
{
#if 0
	__gpio_as_input(PIN_A);
	return !__gpio_get_pin(PIN_A);
#else
	return 0;
#endif
}

unsigned int get_memory_size(void)
{
	return (1 << (DDR_ROW + DDR_COL + DDR_DW32 + DDR_BANK8 + 3))
			* (DDR_CS1EN + DDR_CS0EN);
}
