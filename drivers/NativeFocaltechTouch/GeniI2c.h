// SPDX-License-Identifier: BSD-2-Clause-Patent
// Qualcomm GENI serial-engine I2C master, FIFO (PIO) mode. Register layout and sequences follow the study of the
// Linux i2c-qcom-geni / qcom-geni-se drivers (no code copied). No board addresses, clocks or pins here.
// Sibling of drivers/NativeNovatekTouch/GeniSpi.{c,h}; the serial-engine bring-up is shared, the transfers and
// the protocol configuration are I2C-specific.
#ifndef GENI_I2C_H
#define GENI_I2C_H
#include <stdint.h>
#include <stddef.h>

// Serial-engine registers (offsets from the SE base; qcom-geni-se.h).
#define GENI_FORCE_DEFAULT_REG 0x20
#define GENI_OUTPUT_CTRL 0x24
#define SE_GENI_CGC_CTRL 0x28
#define SE_GENI_STATUS 0x40
#define GENI_SER_M_CLK_CFG 0x48
#define GENI_IF_DISABLE_RO 0x64
#define GENI_FW_REVISION_RO 0x68
#define SE_GENI_CLK_SEL 0x7c
#define SE_GENI_BYTE_GRAN 0x254
#define SE_GENI_DMA_MODE_EN 0x258
#define SE_GENI_TX_PACKING_CFG0 0x260
#define SE_GENI_TX_PACKING_CFG1 0x264
#define SE_GENI_RX_PACKING_CFG0 0x284
#define SE_GENI_RX_PACKING_CFG1 0x288
#define SE_I2C_TX_TRANS_LEN 0x26c
#define SE_I2C_RX_TRANS_LEN 0x270
#define SE_I2C_SCL_COUNTERS 0x278
#define SE_GENI_M_CMD0 0x600
#define SE_GENI_M_CMD_CTRL_REG 0x604
#define SE_GENI_M_IRQ_STATUS 0x610
#define SE_GENI_M_IRQ_EN 0x614
#define SE_GENI_M_IRQ_CLEAR 0x618
#define SE_GENI_S_IRQ_EN 0x644
#define SE_GENI_S_IRQ_CLEAR 0x648
#define SE_DMA_TX_IRQ_CLR 0xc44
#define SE_DMA_RX_IRQ_CLR 0xd44
#define SE_GENI_TX_FIFOn 0x700
#define SE_GENI_RX_FIFOn 0x780
#define SE_GENI_TX_FIFO_STATUS 0x800
#define SE_GENI_RX_FIFO_STATUS 0x804
#define SE_GENI_TX_WATERMARK_REG 0x80c
#define SE_GENI_RX_WATERMARK_REG 0x810
#define SE_GENI_RX_RFR_WATERMARK_REG 0x814
#define SE_IRQ_EN 0xe1c
#define SE_HW_PARAM_0 0xe24
#define SE_HW_PARAM_1 0xe28
#define SE_DMA_GENERAL_CFG 0xe30
#define SE_GSI_EVENT_EN 0xe18
#define QUPV3_HW_VER_REG 0x4

// GENI_SE_I2C in the Linux protocol enum (GENI_SE_NONE 0, SPI 1, UART 2, I2C 3).
#define GENI_PROTO_I2C 3

// M_CMD OP codes for I2C (i2c-qcom-geni.c).
#define I2C_WRITE 0x1
#define I2C_READ 0x2
// M_CMD params for I2C (i2c-qcom-geni.c).
#define I2C_M_STOP_STRETCH (1u << 2)      // hold the bus (no STOP) after this message
#define I2C_M_SLV_ADDR_MSK (0x7fu << 9)
#define I2C_M_SLV_ADDR_SHFT 9

// Register field values (qcom-geni-se.h).
#define GENI_FORCE_DEFAULT (1u << 0)
#define GENI_SER_CLK_EN (1u << 0)
#define GENI_CLK_DIV_SHIFT 4
#define GENI_CLK_DIV_MAX 0xfffu
#define GENI_FIFO_IF_DISABLE (1u << 0)
#define GENI_FW_PROTO_SHIFT 8
#define GENI_DMA_MODE_EN (1u << 0)
#define GENI_DEFAULT_CGC_EN 0x7fu
#define GENI_DEFAULT_IO_OUTPUT_CTRL 0x7fu
#define GENI_M_OPCODE_SHIFT 27
#define GENI_M_CMD_CANCEL (1u << 2)
#define GENI_M_CMD_ABORT (1u << 1)
#define GENI_STATUS_M_ACTIVE (1u << 0)
// SE_IRQ_EN
#define GENI_IRQ_DMA_RX (1u << 0)
#define GENI_IRQ_DMA_TX (1u << 1)
#define GENI_IRQ_M (1u << 2)
#define GENI_IRQ_S (1u << 3)
// SE_DMA_GENERAL_CFG
#define GENI_DMA_RX_CGC_ON (1u << 0)
#define GENI_DMA_TX_CGC_ON (1u << 1)
#define GENI_DMA_AHB_SLV_CGC_ON (1u << 2)
#define GENI_AHB_SEC_SLV_CGC_ON (1u << 3)
// M_IRQ bits
#define GENI_M_CMD_DONE (1u << 0)
#define GENI_M_CMD_OVERRUN (1u << 1)
#define GENI_M_ILLEGAL_CMD (1u << 2)
#define GENI_M_CMD_FAILURE (1u << 3)
#define GENI_M_CMD_CANCEL_DONE (1u << 4)
#define GENI_M_CMD_ABORT_DONE (1u << 5)
#define GENI_M_GP_IRQ_1 (1u << 6)         // SCL stuck low (i2c-qcom-geni M_GP_IRQ_1_EN)
#define GENI_M_GP_IRQ_3 (1u << 8)         // bus NACK (ADDR_NACK)
#define GENI_M_GP_IRQ_4 (1u << 9)         // bus error
#define GENI_M_IO_DATA_DEASSERT (1u << 22)
#define GENI_M_IO_DATA_ASSERT (1u << 23)
#define GENI_M_RX_FIFO_RD_ERR (1u << 24)
#define GENI_M_RX_FIFO_WR_ERR (1u << 25)
#define GENI_M_RX_FIFO_WATERMARK (1u << 26)
#define GENI_M_RX_FIFO_LAST (1u << 27)
#define GENI_M_TX_FIFO_RD_ERR (1u << 28)
#define GENI_M_TX_FIFO_WR_ERR (1u << 29)
#define GENI_M_TX_FIFO_WATERMARK (1u << 30)
#define GENI_M_COMMON_IRQ_EN (0x7eu | GENI_M_IO_DATA_DEASSERT | GENI_M_IO_DATA_ASSERT | GENI_M_RX_FIFO_RD_ERR | \
                              GENI_M_RX_FIFO_WR_ERR | GENI_M_TX_FIFO_RD_ERR | GENI_M_TX_FIFO_WR_ERR)
#define GENI_M_ERRORS (GENI_M_CMD_OVERRUN | GENI_M_ILLEGAL_CMD | GENI_M_CMD_FAILURE | GENI_M_GP_IRQ_1 | \
                       GENI_M_GP_IRQ_3 | GENI_M_GP_IRQ_4 | GENI_M_RX_FIFO_RD_ERR | GENI_M_RX_FIFO_WR_ERR | \
                       GENI_M_TX_FIFO_RD_ERR | GENI_M_TX_FIFO_WR_ERR)
#define GENI_TX_WC_MASK 0xfffffffu
#define GENI_RX_WC_MASK 0x1ffffffu
#define GENI_RX_LAST (1u << 31)
#define GENI_RX_LAST_VALID_SHIFT 28
#define GENI_RX_LAST_VALID_MASK 7u
#define GENI_PACK_START_SHIFT 5
#define GENI_PACK_DIR_SHIFT 4
#define GENI_PACK_LEN_SHIFT 1
#define GENI_PACK_STOP (1u << 0)
#define GENI_PACK_VECTOR_SHIFT 10
#define GENI_PACK_VECTORS 4

#define GENI_ERR_PARAM (-22)
#define GENI_ERR_PROTO (-71)
#define GENI_ERR_NODEV (-19)
#define GENI_ERR_NACK (-16)               // address NACKed: no controller at that address
#define GENI_ERR_TIMEOUT (-110)
#define GENI_ERR_IO (-5)

struct geni_io {
    void *ctx;
    uint32_t (*rd)(void *ctx, uint32_t reg);
    void (*wr)(void *ctx, uint32_t reg, uint32_t value);
    void (*delay_us)(void *ctx, unsigned us);
};

struct geni_i2c_info {
    uint32_t fw_revision, protocol, if_disable, geni_status, hw_param0, hw_param1, qup_hw_version;
    unsigned tx_fifo_depth, rx_fifo_depth, fifo_width_bits;
    int fifo_disabled;
};

struct geni_i2c {
    struct geni_io se;                  // serial-engine registers
    struct geni_io wrapper;             // QUP wrapper registers (hardware version only)
    struct geni_i2c_info info;
    unsigned cur_speed_hz;              // 0 until set_clock ran
    unsigned poll_limit;                // iterations of the completion poll before cancel/abort
    // Diagnostic snapshot taken when a transfer fails or times out.
    uint32_t snap_irq, snap_status, snap_txfs, snap_rxfs, snap_mcmd;
    uint32_t snap_tx_rem, snap_rx_rem;
    int snap_valid;
};

// Read-only identity: protocol, FIFO geometry, FIFO-disable flag. Safe before any register write; only valid when
// the SE clocks already run. Does not fail on a non-I2C protocol: the caller decides (diagnostics want the value).
int geni_i2c_read_info(struct geni_i2c *s);
// Select FIFO mode and program the controller defaults. Refuses engines whose firmware is not I2C or whose FIFO
// interface is disabled.
int geni_i2c_init(struct geni_i2c *s);
// Program the serial clock: the GENI i2c source-map index (0 = 19.2 MHz) and the target SCL rate from the
// hard-coded 19.2 MHz divider table (100 kHz / 400 kHz).
int geni_i2c_set_clock(struct geni_i2c *s, unsigned speed_hz);
// One message: write len bytes (stop_stretch holds the bus afterwards), or read len bytes into rx. The 7-bit
// slave address goes into the M_CMD params of every message, exactly like i2c-qcom-geni.
int geni_i2c_write(struct geni_i2c *s, const uint8_t *tx, size_t len, int stop_stretch);
int geni_i2c_read(struct geni_i2c *s, uint8_t *rx, size_t len);
#endif
