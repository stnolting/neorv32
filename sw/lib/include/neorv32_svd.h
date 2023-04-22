#ifndef neorv32_svd_h
#define neorv32_svd_h

#ifdef NEORV32_SVD_HEADER
#pragma message "*** using 'neorv32_svd_h'"
#endif

/* ---- CFS ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t REG0;
    uint32_t REG1;
    uint32_t REG2;
    uint32_t REG3;
    uint32_t REG4;
    uint32_t REG5;
    uint32_t REG6;
    uint32_t REG7;
    uint32_t REG8;
    uint32_t REG9;
    uint32_t REG10;
    uint32_t REG11;
    uint32_t REG12;
    uint32_t REG13;
    uint32_t REG14;
    uint32_t REG15;
    uint32_t REG16;
    uint32_t REG17;
    uint32_t REG18;
    uint32_t REG19;
    uint32_t REG20;
    uint32_t REG21;
    uint32_t REG22;
    uint32_t REG23;
    uint32_t REG24;
    uint32_t REG25;
    uint32_t REG26;
    uint32_t REG27;
    uint32_t REG28;
    uint32_t REG29;
    uint32_t REG30;
    uint32_t REG31;
    uint32_t REG32;
    uint32_t REG33;
    uint32_t REG34;
    uint32_t REG35;
    uint32_t REG36;
    uint32_t REG37;
    uint32_t REG38;
    uint32_t REG39;
    uint32_t REG40;
    uint32_t REG41;
    uint32_t REG42;
    uint32_t REG43;
    uint32_t REG44;
    uint32_t REG45;
    uint32_t REG46;
    uint32_t REG47;
    uint32_t REG48;
    uint32_t REG49;
    uint32_t REG50;
    uint32_t REG51;
    uint32_t REG52;
    uint32_t REG53;
    uint32_t REG54;
    uint32_t REG55;
    uint32_t REG56;
    uint32_t REG57;
    uint32_t REG58;
    uint32_t REG59;
    uint32_t REG60;
    uint32_t REG61;
    uint32_t REG62;
    uint32_t REG63;
} neorv32_cfs_t;

#define NEORV32_CFS_BASE 0xFFFFFE00
#define NEORV32_CFS ((neorv32_cfs_t*) (NEORV32_CFS_BASE))

/* ---- SDI ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
    uint32_t DATA;
} neorv32_sdi_t;

#define NEORV32_SDI_BASE 0xFFFFFF00
#define NEORV32_SDI ((neorv32_sdi_t*) (NEORV32_SDI_BASE))

enum NEORV32_SDI_CTRL_enum {
    SDI_CTRL_EN = 0,
    SDI_CTRL_CLR_RX = 1,
    SDI_CTRL_FIFO_LSB = 4,
    SDI_CTRL_FIFO_MSB = 7,
    SDI_CTRL_FIFO0 = 4,
    SDI_CTRL_FIFO1 = 5,
    SDI_CTRL_FIFO2 = 6,
    SDI_CTRL_FIFO3 = 7,
    SDI_CTRL_IRQ_RX_AVAIL = 15,
    SDI_CTRL_IRQ_RX_HALF = 16,
    SDI_CTRL_IRQ_RX_FULL = 17,
    SDI_CTRL_IRQ_TX_EMPTY = 18,
    SDI_CTRL_RX_AVAIL = 23,
    SDI_CTRL_RX_HALF = 24,
    SDI_CTRL_RX_FULL = 25,
    SDI_CTRL_TX_EMPTY = 26,
    SDI_CTRL_TX_FULL = 27,
};

/* ---- PWM ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
    uint32_t DC[3];
} neorv32_pwm_t;

#define NEORV32_PWM_BASE 0xFFFFFF50
#define NEORV32_PWM ((neorv32_pwm_t*) (NEORV32_PWM_BASE))

enum NEORV32_PWM_CTRL_enum {
    PWM_CTRL_EN = 0,
    PWM_CTRL_PRSC0 = 1,
    PWM_CTRL_PRSC1 = 2,
    PWM_CTRL_PRSC2 = 3,
};

/* ---- XIP ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
    uint32_t DATA_LO;
    uint32_t DATA_HI;
} neorv32_xip_t;

#define NEORV32_XIP_BASE 0xFFFFFF40
#define NEORV32_XIP ((neorv32_xip_t*) (NEORV32_XIP_BASE))

enum NEORV32_XIP_CTRL_enum {
    XIP_CTRL_EN = 0,
    XIP_CTRL_PRSC0 = 1,
    XIP_CTRL_PRSC1 = 2,
    XIP_CTRL_PRSC2 = 3,
    XIP_CTRL_CPOL = 4,
    XIP_CTRL_CPHA = 5,
    XIP_CTRL_SPI_NBYTES_LSB = 6,
    XIP_CTRL_SPI_NBYTES_MSB = 9,
    XIP_CTRL_XIP_EN = 10,
    XIP_CTRL_XIP_ABYTES_LSB = 11,
    XIP_CTRL_XIP_ABYTES_MSB = 12,
    XIP_CTRL_RD_CMD_LSB = 13,
    XIP_CTRL_RD_CMD_MSB = 20,
    XIP_CTRL_PAGE_LSB = 21,
    XIP_CTRL_PAGE_MSB = 24,
    XIP_CTRL_SPI_CSEN = 25,
    XIP_CTRL_HIGHSPEED = 26,
    XIP_CTRL_BURST_EN = 27,
    XIP_CTRL_PHY_BUSY = 30,
    XIP_CTRL_XIP_BUSY = 31,
};

/* ---- GPTMR ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
    uint32_t THRES;
    uint32_t COUNT;
} neorv32_gptmr_t;

#define NEORV32_GPTMR_BASE 0xFFFFFF60
#define NEORV32_GPTMR ((neorv32_gptmr_t*) (NEORV32_GPTMR_BASE))

enum NEORV32_GPTMR_CTRL_enum {
    GPTMR_CTRL_EN = 0,
    GPTMR_CTRL_PRSC0 = 1,
    GPTMR_CTRL_PRSC1 = 2,
    GPTMR_CTRL_PRSC2 = 3,
    GPTMR_CTRL_MODE = 4,
};

/* ---- ONEWIRE ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
    uint32_t DATA;
} neorv32_onewire_t;

#define NEORV32_ONEWIRE_BASE 0xFFFFFF70
#define NEORV32_ONEWIRE ((neorv32_onewire_t*) (NEORV32_ONEWIRE_BASE))

enum NEORV32_ONEWIRE_CTRL_enum {
    ONEWIRE_CTRL_EN = 0,
    ONEWIRE_CTRL_PRSC0 = 1,
    ONEWIRE_CTRL_PRSC1 = 2,
    ONEWIRE_CTRL_CLKDIV0 = 3,
    ONEWIRE_CTRL_CLKDIV1 = 4,
    ONEWIRE_CTRL_CLKDIV2 = 5,
    ONEWIRE_CTRL_CLKDIV3 = 6,
    ONEWIRE_CTRL_CLKDIV4 = 7,
    ONEWIRE_CTRL_CLKDIV5 = 8,
    ONEWIRE_CTRL_CLKDIV6 = 9,
    ONEWIRE_CTRL_CLKDIV7 = 10,
    ONEWIRE_CTRL_TRIG_RST = 11,
    ONEWIRE_CTRL_TRIG_BIT = 12,
    ONEWIRE_CTRL_TRIG_BYTE = 13,
    ONEWIRE_CTRL_SENSE = 29,
    ONEWIRE_CTRL_PRESENCE = 30,
    ONEWIRE_CTRL_BUSY = 31,
};

enum NEORV32_ONEWIRE_DATA_enum {
    ONEWIRE_DATA0 = 0,
    ONEWIRE_DATA1 = 1,
    ONEWIRE_DATA2 = 2,
    ONEWIRE_DATA3 = 3,
    ONEWIRE_DATA4 = 4,
    ONEWIRE_DATA5 = 5,
    ONEWIRE_DATA6 = 6,
    ONEWIRE_DATA7 = 7,
};

/* ---- BUSKEEPER ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
} neorv32_buskeeper_t;

#define NEORV32_BUSKEEPER_BASE 0xFFFFFF78
#define NEORV32_BUSKEEPER ((neorv32_buskeeper_t*) (NEORV32_BUSKEEPER_BASE))

enum NEORV32_BUSKEEPER_CTRL_enum {
    BUSKEEPER_ERR_TYPE = 0,
    BUSKEEPER_ERR_FLAG = 31,
};

/* ---- XIRQ ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t EIE;
    uint32_t EIP;
    uint32_t ESC;
} neorv32_xirq_t;

#define NEORV32_XIRQ_BASE 0xFFFFFF80
#define NEORV32_XIRQ ((neorv32_xirq_t*) (NEORV32_XIRQ_BASE))

/* ---- MTIME ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t TIME_LO;
    uint32_t TIME_HI;
    uint32_t TIMECMP_LO;
    uint32_t TIMECMP_HI;
} neorv32_mtime_t;

#define NEORV32_MTIME_BASE 0xFFFFFF90
#define NEORV32_MTIME ((neorv32_mtime_t*) (NEORV32_MTIME_BASE))

/* ---- UART0 ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
    uint32_t DATA;
} neorv32_uart_t;

#define NEORV32_UART0_BASE 0xFFFFFFA0
#define NEORV32_UART0 ((neorv32_uart_t*) (NEORV32_UART0_BASE))

enum NEORV32_UART_CTRL_enum {
    UART_CTRL_EN = 0,
    UART_CTRL_SIM_MODE = 1,
    UART_CTRL_HWFC_EN = 2,
    UART_CTRL_PRSC0 = 3,
    UART_CTRL_PRSC1 = 4,
    UART_CTRL_PRSC2 = 5,
    UART_CTRL_BAUD0 = 6,
    UART_CTRL_BAUD1 = 7,
    UART_CTRL_BAUD2 = 8,
    UART_CTRL_BAUD3 = 9,
    UART_CTRL_BAUD4 = 10,
    UART_CTRL_BAUD5 = 11,
    UART_CTRL_BAUD6 = 12,
    UART_CTRL_BAUD7 = 13,
    UART_CTRL_BAUD8 = 14,
    UART_CTRL_BAUD9 = 15,
    UART_CTRL_RX_NEMPTY = 16,
    UART_CTRL_RX_HALF = 17,
    UART_CTRL_RX_FULL = 18,
    UART_CTRL_TX_EMPTY = 19,
    UART_CTRL_TX_NHALF = 20,
    UART_CTRL_TX_FULL = 21,
    UART_CTRL_IRQ_RX_NEMPTY = 22,
    UART_CTRL_IRQ_RX_HALF = 23,
    UART_CTRL_IRQ_RX_FULL = 24,
    UART_CTRL_IRQ_TX_EMPTY = 25,
    UART_CTRL_IRQ_TX_NHALF = 26,
    UART_CTRL_RX_OVER = 30,
    UART_CTRL_TX_BUSY = 31,
};

enum NEORV32_UART_DATA_enum {
    UART_DATA_RTX_LSB = 0,
    UART_DATA_RTX_MSB = 7,
    UART_DATA_RX_FIFO_SIZE_LSB = 8,
    UART_DATA_RX_FIFO_SIZE_MSB = 11,
    UART_DATA_TX_FIFO_SIZE_LSB = 12,
    UART_DATA_TX_FIFO_SIZE_MSB = 15,
};

/* ---- UART1 ---- */

#define NEORV32_UART1_BASE 0xFFFFFFD0
#define NEORV32_UART1 ((neorv32_uart_t*) (NEORV32_UART1_BASE))

/* ---- SPI ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
    uint32_t DATA;
} neorv32_spi_t;

#define NEORV32_SPI_BASE 0xFFFFFFA8
#define NEORV32_SPI ((neorv32_spi_t*) (NEORV32_SPI_BASE))

enum NEORV32_SPI_CTRL_enum {
    SPI_CTRL_EN = 0,
    SPI_CTRL_CPHA = 1,
    SPI_CTRL_CPOL = 2,
    SPI_CTRL_CS_SEL0 = 3,
    SPI_CTRL_CS_SEL1 = 4,
    SPI_CTRL_CS_SEL2 = 5,
    SPI_CTRL_CS_EN = 6,
    SPI_CTRL_PRSC0 = 7,
    SPI_CTRL_PRSC1 = 8,
    SPI_CTRL_PRSC2 = 9,
    SPI_CTRL_CDIV0 = 10,
    SPI_CTRL_CDIV1 = 11,
    SPI_CTRL_CDIV2 = 12,
    SPI_CTRL_CDIV3 = 13,
    SPI_CTRL_RX_AVAIL = 16,
    SPI_CTRL_TX_EMPTY = 17,
    SPI_CTRL_TX_NHALF = 18,
    SPI_CTRL_TX_FULL = 19,
    SPI_CTRL_IRQ_RX_AVAIL = 20,
    SPI_CTRL_IRQ_TX_EMPTY = 21,
    SPI_CTRL_IRQ_TX_NHALF = 22,
    SPI_CTRL_FIFO_LSB = 23,
    SPI_CTRL_FIFO_MSB = 26,
    SPI_CTRL_BUSY = 31,
};

/* ---- TWI ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
    uint32_t DATA;
} neorv32_twi_t;

#define NEORV32_TWI_BASE 0xFFFFFFB0
#define NEORV32_TWI ((neorv32_twi_t*) (NEORV32_TWI_BASE))

enum NEORV32_TWI_CTRL_enum {
    TWI_CTRL_EN = 0,
    TWI_CTRL_START = 1,
    TWI_CTRL_STOP = 2,
    TWI_CTRL_MACK = 3,
    TWI_CTRL_CSEN = 4,
    TWI_CTRL_PRSC0 = 5,
    TWI_CTRL_PRSC1 = 6,
    TWI_CTRL_PRSC2 = 7,
    TWI_CTRL_CDIV0 = 8,
    TWI_CTRL_CDIV1 = 9,
    TWI_CTRL_CDIV2 = 10,
    TWI_CTRL_CDIV3 = 11,
    TWI_CTRL_CLAIMED = 29,
    TWI_CTRL_ACK = 30,
    TWI_CTRL_BUSY = 31,
};

enum NEORV32_TWI_DATA_enum {
    TWI_DATA0 = 0,
    TWI_DATA1 = 1,
    TWI_DATA2 = 2,
    TWI_DATA3 = 3,
    TWI_DATA4 = 4,
    TWI_DATA5 = 5,
    TWI_DATA6 = 6,
    TWI_DATA7 = 7,
};

/* ---- TRNG ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
} neorv32_trng_t;

#define NEORV32_TRNG_BASE 0xFFFFFFB8
#define NEORV32_TRNG ((neorv32_trng_t*) (NEORV32_TRNG_BASE))

enum NEORV32_TRNG_CTRL_enum {
    TRNG_CTRL_DATA_LSB = 0,
    TRNG_CTRL_DATA_MSB = 7,
    TRNG_CTRL_FIFO_CLR = 28,
    TRNG_CTRL_SIM_MODE = 29,
    TRNG_CTRL_EN = 30,
    TRNG_CTRL_VALID = 31,
};

/* ---- WDT ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
} neorv32_wdt_t;

#define NEORV32_WDT_BASE 0xFFFFFFBC
#define NEORV32_WDT ((neorv32_wdt_t*) (NEORV32_WDT_BASE))

enum NEORV32_WDT_CTRL_enum {
    WDT_CTRL_EN = 0,
    WDT_CTRL_LOCK = 1,
    WDT_CTRL_DBEN = 2,
    WDT_CTRL_SEN = 3,
    WDT_CTRL_RESET = 4,
    WDT_CTRL_RCAUSE = 5,
    WDT_CTRL_TIMEOUT_LSB = 8,
    WDT_CTRL_TIMEOUT_MSB = 31,
};

/* ---- GPIO ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    const uint32_t INPUT_LO;
    const uint32_t INPUT_HI;
    uint32_t OUTPUT_LO;
    uint32_t OUTPUT_HI;
} neorv32_gpio_t;

#define NEORV32_GPIO_BASE 0xFFFFFFc0
#define NEORV32_GPIO ((neorv32_gpio_t*) (NEORV32_GPIO_BASE))

/* ---- NEOLED ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    uint32_t CTRL;
    uint32_t DATA;
} neorv32_neoled_t;

#define NEORV32_NEOLED_BASE 0xFFFFFFD8
#define NEORV32_NEOLED ((neorv32_neoled_t*) (NEORV32_NEOLED_BASE))

enum NEORV32_NEOLED_CTRL_enum {
    NEOLED_CTRL_EN = 0,
    NEOLED_CTRL_MODE = 1,
    NEOLED_CTRL_STROBE = 2,
    NEOLED_CTRL_PRSC0 = 3,
    NEOLED_CTRL_PRSC1 = 4,
    NEOLED_CTRL_PRSC2 = 5,
    NEOLED_CTRL_BUFS_0 = 6,
    NEOLED_CTRL_BUFS_1 = 7,
    NEOLED_CTRL_BUFS_2 = 8,
    NEOLED_CTRL_BUFS_3 = 9,
    NEOLED_CTRL_T_TOT_0 = 10,
    NEOLED_CTRL_T_TOT_1 = 11,
    NEOLED_CTRL_T_TOT_2 = 12,
    NEOLED_CTRL_T_TOT_3 = 13,
    NEOLED_CTRL_T_TOT_4 = 14,
    NEOLED_CTRL_T_ZERO_H_0 = 15,
    NEOLED_CTRL_T_ZERO_H_1 = 16,
    NEOLED_CTRL_T_ZERO_H_2 = 17,
    NEOLED_CTRL_T_ZERO_H_3 = 18,
    NEOLED_CTRL_T_ZERO_H_4 = 19,
    NEOLED_CTRL_T_ONE_H_0 = 20,
    NEOLED_CTRL_T_ONE_H_1 = 21,
    NEOLED_CTRL_T_ONE_H_2 = 22,
    NEOLED_CTRL_T_ONE_H_3 = 23,
    NEOLED_CTRL_T_ONE_H_4 = 24,
    NEOLED_CTRL_IRQ_CONF = 27,
    NEOLED_CTRL_TX_EMPTY = 28,
    NEOLED_CTRL_TX_HALF = 29,
    NEOLED_CTRL_TX_FULL = 30,
    NEOLED_CTRL_TX_BUSY = 31,
};

/* ---- SYSINFO ---- */

typedef volatile struct __attribute__((packed,aligned(4))) {
    const uint32_t CLK;
    const uint32_t CUSTOM_ID;
    const uint32_t SOC;
    const uint32_t CACHE;
    const uint32_t ISPACE_BASE;
    const uint32_t DSPACE_BASE;
    const uint32_t IMEM_SIZE;
    const uint32_t DMEM_SIZE;
} neorv32_sysinfo_t;

#define NEORV32_SYSINFO_BASE 0xFFFFFFE0
#define NEORV32_SYSINFO ((neorv32_sysinfo_t*) (NEORV32_SYSINFO_BASE))

enum NEORV32_SYSINFO_SOC_enum {
    SYSINFO_SOC_BOOTLOADER = 0,
    SYSINFO_SOC_MEM_EXT = 1,
    SYSINFO_SOC_MEM_INT_IMEM = 2,
    SYSINFO_SOC_MEM_INT_DMEM = 3,
    SYSINFO_SOC_MEM_EXT_ENDIAN = 4,
    SYSINFO_SOC_ICACHE = 5,
    SYSINFO_SOC_DCACHE = 6,
    SYSINFO_SOC_IS_SIM = 13,
    SYSINFO_SOC_OCD = 14,
    SYSINFO_SOC_IO_GPIO = 16,
    SYSINFO_SOC_IO_MTIME = 17,
    SYSINFO_SOC_IO_UART0 = 18,
    SYSINFO_SOC_IO_SPI = 19,
    SYSINFO_SOC_IO_TWI = 20,
    SYSINFO_SOC_IO_PWM = 21,
    SYSINFO_SOC_IO_WDT = 22,
    SYSINFO_SOC_IO_CFS = 23,
    SYSINFO_SOC_IO_TRNG = 24,
    SYSINFO_SOC_IO_SDI = 25,
    SYSINFO_SOC_IO_UART1 = 26,
    SYSINFO_SOC_IO_NEOLED = 27,
    SYSINFO_SOC_IO_XIRQ = 28,
    SYSINFO_SOC_IO_GPTMR = 29,
    SYSINFO_SOC_IO_XIP = 30,
    SYSINFO_SOC_IO_ONEWIRE = 31,
};

enum NEORV32_SYSINFO_CACHE_enum {
    SYSINFO_CACHE_IC_BLOCK_SIZE_0 = 0,
    SYSINFO_CACHE_IC_BLOCK_SIZE_1 = 1,
    SYSINFO_CACHE_IC_BLOCK_SIZE_2 = 2,
    SYSINFO_CACHE_IC_BLOCK_SIZE_3 = 3,
    SYSINFO_CACHE_IC_NUM_BLOCKS_0 = 4,
    SYSINFO_CACHE_IC_NUM_BLOCKS_1 = 5,
    SYSINFO_CACHE_IC_NUM_BLOCKS_2 = 6,
    SYSINFO_CACHE_IC_NUM_BLOCKS_3 = 7,
    SYSINFO_CACHE_IC_ASSOCIATIVITY_0 = 8,
    SYSINFO_CACHE_IC_ASSOCIATIVITY_1 = 9,
    SYSINFO_CACHE_IC_ASSOCIATIVITY_2 = 10,
    SYSINFO_CACHE_IC_ASSOCIATIVITY_3 = 11,
    SYSINFO_CACHE_IC_REPLACEMENT_0 = 12,
    SYSINFO_CACHE_IC_REPLACEMENT_1 = 13,
    SYSINFO_CACHE_IC_REPLACEMENT_2 = 14,
    SYSINFO_CACHE_IC_REPLACEMENT_3 = 15,
    SYSINFO_CACHE_DC_BLOCK_SIZE_0 = 16,
    SYSINFO_CACHE_DC_BLOCK_SIZE_1 = 17,
    SYSINFO_CACHE_DC_BLOCK_SIZE_2 = 18,
    SYSINFO_CACHE_DC_BLOCK_SIZE_3 = 19,
    SYSINFO_CACHE_DC_NUM_BLOCKS_0 = 20,
    SYSINFO_CACHE_DC_NUM_BLOCKS_1 = 21,
    SYSINFO_CACHE_DC_NUM_BLOCKS_2 = 22,
    SYSINFO_CACHE_DC_NUM_BLOCKS_3 = 23,
    SYSINFO_CACHE_DC_ASSOCIATIVITY_0 = 24,
    SYSINFO_CACHE_DC_ASSOCIATIVITY_1 = 25,
    SYSINFO_CACHE_DC_ASSOCIATIVITY_2 = 26,
    SYSINFO_CACHE_DC_ASSOCIATIVITY_3 = 27,
    SYSINFO_CACHE_DC_REPLACEMENT_0 = 28,
    SYSINFO_CACHE_DC_REPLACEMENT_1 = 29,
    SYSINFO_CACHE_DC_REPLACEMENT_2 = 30,
    SYSINFO_CACHE_DC_REPLACEMENT_3 = 31,
};

#endif // neorv32_svd_h
