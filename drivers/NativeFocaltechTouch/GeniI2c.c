// SPDX-License-Identifier: BSD-2-Clause-Patent
#include "GeniI2c.h"
#include "FtBoard.h"

static uint32_t rd(struct geni_i2c *s, uint32_t reg) { return s->se.rd(s->se.ctx, reg); }
static void wr(struct geni_i2c *s, uint32_t reg, uint32_t v) { s->se.wr(s->se.ctx, reg, v); }
static void rmw_set(struct geni_i2c *s, uint32_t reg, uint32_t bits) { wr(s, reg, rd(s, reg) | bits); }
static void stall(struct geni_i2c *s) { if (s->se.delay_us) s->se.delay_us(s->se.ctx, 1); }

static void irq_clear(struct geni_i2c *s) {
    wr(s, SE_GSI_EVENT_EN, 0);
    wr(s, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
    wr(s, SE_GENI_S_IRQ_CLEAR, 0xffffffffu);
    wr(s, SE_DMA_TX_IRQ_CLR, 0xffffffffu);
    wr(s, SE_DMA_RX_IRQ_CLR, 0xffffffffu);
    wr(s, SE_IRQ_EN, 0xffffffffu);
}

static unsigned bytes_per_word(const struct geni_i2c *s) { return s->info.fifo_width_bits / 8; }

int geni_i2c_read_info(struct geni_i2c *s) {
    if (!s || !s->se.rd || !s->wrapper.rd) return GENI_ERR_PARAM;
    uint32_t fw = rd(s, GENI_FW_REVISION_RO);
    s->info.fw_revision = fw;
    s->info.protocol = (fw >> GENI_FW_PROTO_SHIFT) & 0xff;
    s->info.if_disable = rd(s, GENI_IF_DISABLE_RO);
    s->info.fifo_disabled = (s->info.if_disable & GENI_FIFO_IF_DISABLE) != 0;
    s->info.geni_status = rd(s, SE_GENI_STATUS);
    s->info.hw_param0 = rd(s, SE_HW_PARAM_0);
    s->info.hw_param1 = rd(s, SE_HW_PARAM_1);
    s->info.qup_hw_version = s->wrapper.rd(s->wrapper.ctx, QUPV3_HW_VER_REG);
    uint32_t major = s->info.qup_hw_version >> 28, minor = (s->info.qup_hw_version >> 16) & 0xfff;
    uint32_t mask = ((major == 3 && minor >= 10) || major > 3) ? 0xff0000u : 0x3f0000u;
    s->info.tx_fifo_depth = (s->info.hw_param0 & mask) >> 16;
    s->info.rx_fifo_depth = (s->info.hw_param1 & mask) >> 16;
    if (s->info.rx_fifo_depth < 4) s->info.rx_fifo_depth = 4;
    s->info.fifo_width_bits = (s->info.hw_param0 >> 24) & 0x3f;
    return 0;
}

// Packing for 8-bit words, 4 per FIFO word; the same values the vendor sequence programs (qcom-geni-se
// geni_se_config_packing, shared by the SPI and I2C engines).
static void config_packing(struct geni_i2c *s) {
    const unsigned bpw = 8, pack_words = s->info.fifo_width_bits / bpw;
    uint32_t cfg[GENI_PACK_VECTORS] = {0};
    int idx = 0;
    unsigned iter = bpw * pack_words / 8;
    if (iter == 0 || iter > GENI_PACK_VECTORS) return;
    for (unsigned i = 0; i < iter; i++) {
        cfg[i] = ((uint32_t)idx << GENI_PACK_START_SHIFT) | (0u << GENI_PACK_DIR_SHIFT) |
                 ((uint32_t)(bpw - 1) << GENI_PACK_LEN_SHIFT);
        idx = (int)(i + 1) * 8;
    }
    cfg[iter - 1] |= GENI_PACK_STOP;
    wr(s, SE_GENI_TX_PACKING_CFG0, cfg[0] | (cfg[1] << GENI_PACK_VECTOR_SHIFT));
    wr(s, SE_GENI_TX_PACKING_CFG1, cfg[2] | (cfg[3] << GENI_PACK_VECTOR_SHIFT));
    wr(s, SE_GENI_RX_PACKING_CFG0, cfg[0] | (cfg[1] << GENI_PACK_VECTOR_SHIFT));
    wr(s, SE_GENI_RX_PACKING_CFG1, cfg[2] | (cfg[3] << GENI_PACK_VECTOR_SHIFT));
    if (pack_words) wr(s, SE_GENI_BYTE_GRAN, bpw / 16);
}

int geni_i2c_init(struct geni_i2c *s) {
    int r = geni_i2c_read_info(s);
    if (r) return r;
    if (s->info.protocol != GENI_PROTO_I2C) return GENI_ERR_PROTO;
    if (s->info.fifo_disabled) return GENI_ERR_NODEV;   // GPI DMA path not implemented
    if (s->info.tx_fifo_depth < 4 || s->info.fifo_width_bits != 32) return GENI_ERR_NODEV;
    // geni_se_init
    irq_clear(s);
    rmw_set(s, SE_GENI_CGC_CTRL, GENI_DEFAULT_CGC_EN);
    rmw_set(s, SE_DMA_GENERAL_CFG, GENI_AHB_SEC_SLV_CGC_ON | GENI_DMA_AHB_SLV_CGC_ON | GENI_DMA_TX_CGC_ON | GENI_DMA_RX_CGC_ON);
    wr(s, GENI_OUTPUT_CTRL, GENI_DEFAULT_IO_OUTPUT_CTRL);
    wr(s, GENI_FORCE_DEFAULT_REG, GENI_FORCE_DEFAULT);
    rmw_set(s, SE_IRQ_EN, GENI_IRQ_M | GENI_IRQ_S | GENI_IRQ_DMA_TX | GENI_IRQ_DMA_RX);
    wr(s, SE_GENI_DMA_MODE_EN, rd(s, SE_GENI_DMA_MODE_EN) & ~GENI_DMA_MODE_EN);
    wr(s, SE_GSI_EVENT_EN, 0);
    wr(s, SE_GENI_RX_WATERMARK_REG, s->info.rx_fifo_depth / 2);
    wr(s, SE_GENI_RX_RFR_WATERMARK_REG, s->info.rx_fifo_depth - 2);
    rmw_set(s, SE_GENI_M_IRQ_EN, GENI_M_COMMON_IRQ_EN);
    // FIFO mode selection (geni_se_select_fifo_mode): FIFO interrupt sources on, DMA off.
    irq_clear(s);
    rmw_set(s, SE_GENI_M_IRQ_EN, GENI_M_CMD_DONE | GENI_M_TX_FIFO_WATERMARK | GENI_M_RX_FIFO_WATERMARK | GENI_M_RX_FIFO_LAST);
    wr(s, SE_GENI_DMA_MODE_EN, rd(s, SE_GENI_DMA_MODE_EN) & ~GENI_DMA_MODE_EN);
    config_packing(s);
    s->cur_speed_hz = 0;
    if (!s->poll_limit) s->poll_limit = 1000000;
    return 0;
}

int geni_i2c_set_clock(struct geni_i2c *s, unsigned speed_hz) {
    // Pick the closest table entry not above the request (i2c-qcom-geni geni_i2c_clk_map_19p2mhz, source map 0 =
    // 19.2 MHz; the 32 MHz map only applies where the DFS table offers it, which this board's SE7 does not).
    const struct tb_i2c_clock_fld *pick = 0;
    for (unsigned i = 0; i < sizeof(TB_I2C_CLOCKS) / sizeof(TB_I2C_CLOCKS[0]); ++i) {
        if (TB_I2C_CLOCKS[i].hz <= speed_hz && (!pick || TB_I2C_CLOCKS[i].hz > pick->hz)) pick = &TB_I2C_CLOCKS[i];
    }
    if (!pick) return GENI_ERR_PARAM;
    // qcom_geni_i2c_conf: SE_GENI_CLK_SEL = source-map index 0, SER_M_CLK_CFG = (div << 4) | SER_CLK_EN,
    // SE_I2C_SCL_COUNTERS = t_high << 20 | t_low << 10 | t_cycle.
    wr(s, SE_GENI_CLK_SEL, 0);
    wr(s, GENI_SER_M_CLK_CFG, (pick->div << GENI_CLK_DIV_SHIFT) | GENI_SER_CLK_EN);
    wr(s, SE_I2C_SCL_COUNTERS, (pick->t_high << 20) | (pick->t_low << 10) | pick->t_cycle);
    s->cur_speed_hz = pick->hz;
    return 0;
}

static void snapshot(struct geni_i2c *s, uint32_t irq, size_t tx_rem, size_t rx_rem) {
    s->snap_irq = irq;
    s->snap_status = rd(s, SE_GENI_STATUS);
    s->snap_txfs = rd(s, SE_GENI_TX_FIFO_STATUS);
    s->snap_rxfs = rd(s, SE_GENI_RX_FIFO_STATUS);
    s->snap_mcmd = rd(s, SE_GENI_M_CMD0);
    s->snap_tx_rem = (uint32_t)tx_rem;
    s->snap_rx_rem = (uint32_t)rx_rem;
    s->snap_valid = 1;
}

static void recover(struct geni_i2c *s) {
    wr(s, SE_GENI_TX_WATERMARK_REG, 0);
    wr(s, SE_GENI_M_CMD_CTRL_REG, GENI_M_CMD_CANCEL);
    uint32_t st = 0;
    for (unsigned n = 0; n < s->poll_limit; ++n) {
        st = rd(s, SE_GENI_M_IRQ_STATUS);
        if (st & (GENI_M_CMD_CANCEL_DONE | GENI_M_CMD_DONE)) { wr(s, SE_GENI_M_IRQ_CLEAR, st); return; }
        stall(s);
    }
    wr(s, SE_GENI_M_CMD_CTRL_REG, GENI_M_CMD_ABORT);
    for (unsigned n = 0; n < s->poll_limit; ++n) {
        st = rd(s, SE_GENI_M_IRQ_STATUS);
        if (st & (GENI_M_CMD_ABORT_DONE | GENI_M_CMD_DONE)) { wr(s, SE_GENI_M_IRQ_CLEAR, st); return; }
        stall(s);
    }
}

// Shared poll: feed the TX FIFO on watermark, drain the RX FIFO on watermark/last, wait for CMD_DONE.
// op is I2C_WRITE or I2C_READ; m_param carries the slave address and the stop-stretch flag.
static int transfer(struct geni_i2c *s, unsigned op, const uint8_t *tx, uint8_t *rx, size_t len, uint32_t m_param) {
    struct { const uint8_t *tx; uint8_t *rx; size_t tx_rem, rx_rem; } x = {tx, rx, tx ? len : 0, rx ? len : 0};
    const unsigned bpw = bytes_per_word(s);
    if (op == I2C_WRITE) {
        wr(s, SE_I2C_TX_TRANS_LEN, (uint32_t)len);
    } else {
        wr(s, SE_I2C_RX_TRANS_LEN, (uint32_t)len);
    }
    wr(s, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
    wr(s, SE_GENI_M_CMD0, (op << GENI_M_OPCODE_SHIFT) | m_param);
    // Fill the FIFO once up front; arm the TX watermark only if a remainder stayed behind.
    if (tx) {
        const unsigned wc = rd(s, SE_GENI_TX_FIFO_STATUS) & GENI_TX_WC_MASK;
        unsigned free_words = wc < s->info.tx_fifo_depth ? s->info.tx_fifo_depth - wc : 0;
        size_t max = (size_t)free_words * bpw;
        if (x.tx_rem < max) max = x.tx_rem;
        size_t i = 0;
        while (i < max) {
            uint32_t word = 0;
            size_t n2 = max - i < bpw ? max - i : bpw;
            for (size_t j = 0; j < n2; j++) word |= (uint32_t)tx[i++] << (8 * j);
            wr(s, SE_GENI_TX_FIFOn, word);
        }
        x.tx_rem -= max;
        if (x.tx_rem) wr(s, SE_GENI_TX_WATERMARK_REG, 1);
    }
    int status = GENI_ERR_TIMEOUT;
    for (unsigned n = 0; n < s->poll_limit; n++) {
        uint32_t st = rd(s, SE_GENI_M_IRQ_STATUS);
        if (!st) { stall(s); continue; }
        wr(s, SE_GENI_M_IRQ_CLEAR, st);
        if (st & GENI_M_ERRORS) {
            snapshot(s, st, x.tx_rem, x.rx_rem);
            status = (st & GENI_M_GP_IRQ_3) ? GENI_ERR_NACK : GENI_ERR_IO;
            break;
        }
        if ((st & GENI_M_TX_FIFO_WATERMARK) && tx && x.tx_rem) {
            const unsigned wc = rd(s, SE_GENI_TX_FIFO_STATUS) & GENI_TX_WC_MASK;
            unsigned free_words = wc < s->info.tx_fifo_depth ? s->info.tx_fifo_depth - wc : 0;
            size_t max = (size_t)free_words * bpw;
            if (x.tx_rem < max) max = x.tx_rem;
            size_t i = 0;
            while (i < max) {
                uint32_t word = 0;
                size_t n2 = max - i < bpw ? max - i : bpw;
                for (size_t j = 0; j < n2; j++) word |= (uint32_t)tx[i++] << (8 * j);
                wr(s, SE_GENI_TX_FIFOn, word);
            }
            x.tx_rem -= max;
            if (!x.tx_rem) wr(s, SE_GENI_TX_WATERMARK_REG, 0);
        }
        if ((st & (GENI_M_RX_FIFO_WATERMARK | GENI_M_RX_FIFO_LAST)) && rx) {
            uint32_t rs = rd(s, SE_GENI_RX_FIFO_STATUS);
            size_t bytes = (size_t)(rs & GENI_RX_WC_MASK) * bpw;
            if (rs & GENI_RX_LAST) {
                unsigned valid = (rs >> GENI_RX_LAST_VALID_SHIFT) & GENI_RX_LAST_VALID_MASK;
                if (valid && valid < 4) bytes -= bpw - valid;
            }
            if (bytes > x.rx_rem) bytes = x.rx_rem;
            size_t i = 0;
            while (i < bytes) {
                uint32_t word = rd(s, SE_GENI_RX_FIFOn);
                size_t n2 = bytes - i < bpw ? bytes - i : bpw;
                for (size_t j = 0; j < n2; j++) rx[i++] = (uint8_t)(word >> (8 * j));
            }
            x.rx_rem -= bytes;
        }
        if (st & GENI_M_CMD_DONE) {
            // Drain whatever the FIFO still holds after the command finished.
            if (rx && x.rx_rem) {
                uint32_t rs = rd(s, SE_GENI_RX_FIFO_STATUS);
                size_t bytes = (size_t)(rs & GENI_RX_WC_MASK) * bpw;
                if (rs & GENI_RX_LAST) {
                    unsigned valid = (rs >> GENI_RX_LAST_VALID_SHIFT) & GENI_RX_LAST_VALID_MASK;
                    if (valid && valid < 4) bytes -= bpw - valid;
                }
                if (bytes > x.rx_rem) bytes = x.rx_rem;
                size_t i = 0;
                while (i < bytes) {
                    uint32_t word = rd(s, SE_GENI_RX_FIFOn);
                    size_t n2 = bytes - i < bpw ? bytes - i : bpw;
                    for (size_t j = 0; j < n2; j++) rx[i++] = (uint8_t)(word >> (8 * j));
                }
                x.rx_rem -= bytes;
            }
            status = (x.tx_rem || x.rx_rem) ? GENI_ERR_IO : 0;
            if (status) snapshot(s, st, x.tx_rem, x.rx_rem);
            if (x.tx_rem) wr(s, SE_GENI_TX_WATERMARK_REG, 0);
            return status;
        }
    }
    snapshot(s, rd(s, SE_GENI_M_IRQ_STATUS), x.tx_rem, x.rx_rem);
    recover(s);
    return status;
}

static uint32_t addr_param(void) { return ((uint32_t)TB_I2C_ADDR << I2C_M_SLV_ADDR_SHFT) & I2C_M_SLV_ADDR_MSK; }

int geni_i2c_write(struct geni_i2c *s, const uint8_t *tx, size_t len, int stop_stretch) {
    if (!s || !s->cur_speed_hz || !len || len > 0xffffffu || !tx) return GENI_ERR_PARAM;
    const uint32_t param = addr_param() | (stop_stretch ? I2C_M_STOP_STRETCH : 0);
    return transfer(s, I2C_WRITE, tx, 0, len, param);
}

int geni_i2c_read(struct geni_i2c *s, uint8_t *rx, size_t len) {
    if (!s || !s->cur_speed_hz || !len || len > 0xffffffu || !rx) return GENI_ERR_PARAM;
    return transfer(s, I2C_READ, 0, rx, len, addr_param());
}
