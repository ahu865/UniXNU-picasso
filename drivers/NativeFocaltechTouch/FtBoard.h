// SPDX-License-Identifier: BSD-2-Clause-Patent
// vangogh (Qualcomm SM7250 "lito", Xiaomi Mi 10 Youth) FocalTech FT3518 board facts and the pure helpers around
// them. Every constant states its source. Nothing here touches hardware; FtProbe.cpp owns the MMIO.
// Sibling of drivers/NativeNovatekTouch/TouchBoard.h (picasso): same SoC, same SE7 engine, different panel bus.
#ifndef NATIVE_FOCALTECH_TOUCH_BOARD_H
#define NATIVE_FOCALTECH_TOUCH_BOARD_H
#include <stdint.h>

// ---------------------------------------------------------------------------------------------------------------
// QUPv3 wrapper 1 and serial engine 7. The vangogh DT puts the touch on qupv3_se7_i2c (vangogh-sm7250.dtsi:397);
// the engine base is the same cell picasso uses for its SE7 SPI: qupv3_se7_spi@984000, wrapper qupv3_1@9c0000.
// ---------------------------------------------------------------------------------------------------------------
#define TB_QUP_WRAPPER_BASE 0x009c0000ULL
#define TB_QUP_WRAPPER_SIZE 0x2000ULL
#define TB_SE_BASE 0x00984000ULL
#define TB_SE_SIZE 0x4000ULL

// DMA path (not used by the FIFO-mode probe). qupv3_1 iommus = <&apps_smmu 0x23 0> (picasso.dts:14481).
#define TB_QUP_SMMU_SID 0x23

// Interrupts. The SE7 cell interrupt is GIC SPI 354 (picasso.dts:14853); the touch IRQ is TLMM GPIO5 whose pin
// interrupts are summarised as GIC SPI 208 (lito-pinctrl.dtsi), so the summary INTID is 32 + 208 = 240.
#define TB_SE_IRQ_SPI 354
#define TB_TLMM_SUMMARY_SPI 208
#define TB_TLMM_SUMMARY_INTID (32 + TB_TLMM_SUMMARY_SPI)

// GCC: gcc@100000, reg = <0x100000 0x1f0000> (picasso.dts:3427). The SE7 clock specifiers are the same cell
// values picasso reads: se-clk = 0x4b, m-ahb = 0x57, s-ahb = 0x58.
#define TB_GCC_BASE 0x00100000ULL
#define TB_GCC_SIZE 0x001f0000ULL
#define TB_CLOCK_SE_CLK 0x4b
#define TB_CLOCK_M_AHB 0x57
#define TB_CLOCK_S_AHB 0x58
// GCC branch offsets (SM7250/lito), as validated on picasso by the Novatek probe:
// QUPv3 wrapper 1: core 0x23138 (vote bit 19), core_2x 0x23140 (bit 18), m_ahb 0x18004 (bit 20),
// s_ahb 0x18008 (bit 21); wrapper-1 engine s1 (= global SE7, DT clock ID 0x4b) branch 0x1813c (bit 23).
#define TB_GCC_GPLL0_MODE 0x00000000
#define TB_GCC_GPLL0_ENABLE 0x0052010
#define TB_GCC_VOTE_ENABLE 0x0052008
#define TB_GCC_BRANCH_M_AHB 0x00018004
#define TB_GCC_BRANCH_S_AHB 0x00018008
#define TB_GCC_BRANCH_S1 0x0001813c
#define TB_GCC_BRANCH_CORE 0x00023138
#define TB_GCC_BRANCH_CORE_2X 0x00023140
#define TB_GCC_DFS_S1_RCG 0x00018140

#define TB_VOTE_CORE_2X (1u << 18)
#define TB_VOTE_CORE (1u << 19)
#define TB_VOTE_M_AHB (1u << 20)
#define TB_VOTE_S_AHB (1u << 21)
#define TB_VOTE_S1 (1u << 23)
#define TB_VOTE_ALL (TB_VOTE_CORE_2X | TB_VOTE_CORE | TB_VOTE_M_AHB | TB_VOTE_S_AHB | TB_VOTE_S1)
#define TB_CBCR_CLK_OFF (1u << 31)
#define TB_GPLL0_LOCK (1u << 31)
#define TB_GCC_PAGE_SIZE 0x1000
#define TB_GCC_LOW_PAGE 0x0               // GPLL0 mode
#define TB_GCC_QUPV3_PAGE 0x18000         // m_ahb/s_ahb/s1 branches and the s1 DFS RCG
#define TB_GCC_CORE_PAGE 0x23000          // core and core_2x branches
#define TB_GCC_VOTE_PAGE 0x52000          // GPLL0 and QUP branch votes

// FocalTech reset pulse. Vendor fts_reset_proc: reset low 5 ms, high, then wait hdelayms and poll the chip id
// 10 times at 15 ms intervals (focaltech_core.c:301-321; the probe path passes hdelayms 75).
#define TB_RESET_LOW_MS 5u
#define TB_RESET_SETTLE_MS 75u
#define TB_ID_POLLS 10u
#define TB_ID_POLL_GAP_MS 15u

// RCG/DFS register layout (clk-rcg2.c: cmd, cfg, M, N, D, DFSR_CMD, then per-level CFG/M/N tables).
#define TB_DFS_LEVELS 8
#define TB_XO_HZ 19200000ULL
#define TB_RCG_CMD 0x0
#define TB_RCG_CFG 0x4
#define TB_RCG_M 0x8
#define TB_RCG_N 0xc
#define TB_RCG_DFSR_CMD 0x14
#define TB_RCG_PERF_CFG(l) (0x1c + 4 * (l))
#define TB_RCG_PERF_M(l) (0x5c + 4 * (l))
#define TB_RCG_PERF_N(l) (0x9c + 4 * (l))

// TLMM: pinctrl@f000000, four tiles (pinctrl-lito.c: NORTH 0x0, SOUTH 0x100000, WEST 0x500000, EAST 0x900000);
// a pin's register is tile_base + 0x1000 * gpio. GPIO 4/5 are EAST, 6..10 WEST (same SoC pin table as picasso).
#define TB_TLMM_BASE 0x0f000000ULL
#define TB_TLMM_NORTH 0x0000000ULL
#define TB_TLMM_SOUTH 0x0100000ULL
#define TB_TLMM_WEST 0x0500000ULL
#define TB_TLMM_EAST 0x0900000ULL
#define TB_GPIO_STRIDE 0x1000
#define TB_TLMM_IO_OFFSET 0x4
#define TB_TLMM_INTR_CFG_OFFSET 0x8
#define TB_TLMM_INTR_STATUS_OFFSET 0xc

// Pins. The vangogh DT (vangogh-sm7250.dtsi:399-419) gives focaltech@38: irq-gpio <&tlmm 5>, reset-gpio
// <&tlmm 4>, vdd-gpio <&tlmm 8>; the parent qupv3_se7_i2c muxes gpio6/gpio7 to function "qup11"
// (lito-pinctrl.dtsi qupv3_se7_i2c_active, drive-strength 2, bias-disable).
#define TB_GPIO_I2C_SDA 6
#define TB_GPIO_I2C_SCL 7
#define TB_GPIO_VDD 8
#define TB_GPIO_RESET 4
#define TB_GPIO_IRQ 5
#define TB_FUNC_GPIO 0
#define TB_FUNC_QUP11 1                   // pinctrl-lito.c lito_pins: pin 6 starts with qup11, so its slot is 1
#define TB_PULL_NONE 0
#define TB_PULL_DOWN 1
#define TB_PULL_KEEPER 2
#define TB_PULL_UP 3
#define TB_DRIVE_2MA 0                    // drive-strength = <2> encoded as (mA / 2) - 1
#define TB_DRIVE_6MA 2

// pinctrl-msm TLMM interrupt config bits.
#define TB_TLMM_INTR_ENABLE (1u << 0)
#define TB_TLMM_INTR_POL_HIGH (1u << 1)
#define TB_TLMM_INTR_DETECT_RISING (1u << 2)
#define TB_TLMM_INTR_RAW_STATUS (1u << 4)
#define TB_TLMM_INTR_TARGET_APPS (3u << 5)
#define TB_TLMM_INTR_CFG (TB_TLMM_INTR_ENABLE | TB_TLMM_INTR_POL_HIGH | TB_TLMM_INTR_DETECT_RISING | \
                          TB_TLMM_INTR_RAW_STATUS | TB_TLMM_INTR_TARGET_APPS)

// I2C. The DT node has no clock-frequency, so the vendor adapter runs the GENI i2c default 100 kHz
// (I2C_MAX_STANDARD_MODE_FREQ); sheng-touch-hz may raise it, capped at fast-mode 400 kHz.
#define TB_I2C_ADDR 0x38                  // focaltech@38, 7-bit
#define TB_I2C_DEFAULT_HZ 100000u
#define TB_I2C_MAX_HZ 400000u
// i2c-qcom-geni.c geni_i2c_clk_map_19p2mhz: {freq, clk_div, t_high_cnt, t_low_cnt, t_cycle_cnt}.
static const struct tb_i2c_clock_fld {
    unsigned hz, div, t_high, t_low, t_cycle;
} TB_I2C_CLOCKS[2] = {
    {100000u, 7u, 10u, 12u, 26u},
    {400000u, 2u, 5u, 11u, 22u},
};

// Panel: focaltech,display-coords = <0 0 1080 2400> (vangogh-sm7250.dtsi:411), max-touch-number 10.
#define TB_PANEL_WIDTH_PX 1080
#define TB_PANEL_HEIGHT_PX 2400
#define TB_PANEL_SHORT_TENTHS (TB_PANEL_WIDTH_PX * 10)
#define TB_PANEL_LONG_TENTHS (TB_PANEL_HEIGHT_PX * 10)
#define TB_MAX_CONTACTS 10

// FocalTech report offsets inside the 6-byte contact record (focaltech_core.h:82-88; the record area starts at
// byte 3 of the point buffer). Event flag: 0 down, 1 up, 2 contact.
#define FTS_X_OFF 3
#define FTS_Y_OFF 5
#define FTS_ID_OFF 5
#define FTS_REG_CHIP_ID_A 0xA3             // expect 0x54 (focaltech_core.c fts_wait_tp_to_valid)
#define FTS_REG_INT_ACK 0x3E               // focaltech_common.h:113

// HID publication contract (NativeHidInput.cpp normalises xs by 30480 and ys by 20320; portrait, as picasso).
#define TB_HID_X_MAX 30480
#define TB_HID_Y_MAX 20320
#define TB_HID_MIRROR_X 0
#define TB_HID_MIRROR_Y 0

static inline uint32_t tb_tlmm_ctl(unsigned gpio) { return (uint32_t)gpio * TB_GPIO_STRIDE; }
static inline uint32_t tb_tlmm_tile(unsigned gpio) { return gpio <= TB_GPIO_IRQ ? TB_TLMM_EAST : TB_TLMM_WEST; }
static inline uintptr_t tb_tlmm_pa(unsigned gpio) { return TB_TLMM_BASE + tb_tlmm_tile(gpio) + tb_tlmm_ctl(gpio); }
static inline uint32_t tb_tlmm_io(unsigned gpio) { return tb_tlmm_ctl(gpio) + TB_TLMM_IO_OFFSET; }
static inline uint32_t tb_tlmm_intr_cfg(unsigned gpio) { return tb_tlmm_ctl(gpio) + TB_TLMM_INTR_CFG_OFFSET; }
static inline uint32_t tb_tlmm_intr_status(unsigned gpio) { return tb_tlmm_ctl(gpio) + TB_TLMM_INTR_STATUS_OFFSET; }

// One TLMM control value: function 2:0, pull 5:4, drive strength 8:6, output enable 9 (pinctrl-msm).
static inline uint32_t tb_pin_cfg(unsigned function, unsigned drive_code, unsigned pull, int output) {
    return ((function & 7u) << 2) | ((pull & 3u) << 4) | ((drive_code & 7u) << 6) | (output ? (1u << 9) : 0u);
}
#define TB_PIN_CFG_MASK (((7u) << 2) | ((3u) << 4) | ((7u) << 6) | (1u << 9))
#define TB_PIN_CFG_I2C (tb_pin_cfg(TB_FUNC_QUP11, TB_DRIVE_2MA, TB_PULL_NONE, 0))
#define TB_PIN_CFG_RESET (tb_pin_cfg(TB_FUNC_GPIO, TB_DRIVE_6MA, TB_PULL_UP, 1))
#define TB_PIN_CFG_IRQ (tb_pin_cfg(TB_FUNC_GPIO, TB_DRIVE_6MA, TB_PULL_DOWN, 0))
#define TB_PIN_CFG_VDD (tb_pin_cfg(TB_FUNC_GPIO, TB_DRIVE_6MA, TB_PULL_NONE, 1))

// GCC page/offset split: register constants above are offsets from TB_GCC_BASE; the probe maps 4 KiB pages.
static inline uint64_t tb_gcc_page(uint32_t reg) { return TB_GCC_BASE + (reg & ~0xfffu); }
static inline uint32_t tb_gcc_offset(uint32_t reg) { return reg & 0xfffu; }

// Source clock rates by RCG source-select value (gcc_parent_map_0: BI_TCXO = 0, GPLL0 = 1, GPLL0_EVEN = 6).
static inline uint64_t tb_src_hz(unsigned sel) {
    return sel == 0 ? 19200000ULL : sel == 1 ? 600000000ULL : sel == 6 ? 300000000ULL : 0;
}
static inline uint64_t tb_calc_rate(uint64_t prate, uint32_t m, uint32_t n, unsigned mode, unsigned hid) {
    if (hid) { prate *= 2; prate /= hid + 1; }
    if (mode && n) { prate *= m; prate /= n; }
    return prate;
}
static inline uint64_t tb_dfs_level_hz(uint32_t cfg, uint32_t mreg, uint32_t nreg) {
    unsigned src = (cfg >> 8) & 7, mode = (cfg >> 12) & 3, hid = cfg & 0x1f;
    uint32_t m = 0, n = 0;
    if (mode) { m = mreg & 0xffff; n = ((~nreg) & 0xffff) + m; }
    return tb_calc_rate(tb_src_hz(src), m, n, mode, hid);
}

// Core (x = short axis, y = long axis, both 0.1 panel pixels) to the HID raw convention used by NativeHidInput.cpp.
static inline int tb_clamp(int value, int low, int high) { return value < low ? low : value > high ? high : value; }
static inline int tb_hid_x_from_short(int short_tenths) {
    long long v = tb_clamp(short_tenths, 0, TB_PANEL_SHORT_TENTHS - 1);
    v = v * TB_HID_X_MAX / TB_PANEL_SHORT_TENTHS;
    if (TB_HID_MIRROR_X) v = TB_HID_X_MAX - 1 - v;
    return tb_clamp((int)v, 0, TB_HID_X_MAX - 1);
}
static inline int tb_hid_y_from_long(int long_tenths) {
    long long v = tb_clamp(long_tenths, 0, TB_PANEL_LONG_TENTHS - 1);
    v = v * TB_HID_Y_MAX / TB_PANEL_LONG_TENTHS;
    if (TB_HID_MIRROR_Y) v = TB_HID_Y_MAX - 1 - v;
    return tb_clamp((int)v, 0, TB_HID_Y_MAX - 1);
}
struct tb_hid_point { int x, y; };
static inline struct tb_hid_point tb_hid_from_panel(int short_tenths, int long_tenths) {
    const struct tb_hid_point point = {tb_hid_x_from_short(short_tenths), tb_hid_y_from_long(long_tenths)};
    return point;
}

#endif
