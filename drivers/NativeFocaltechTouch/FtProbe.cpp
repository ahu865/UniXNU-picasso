// SPDX-License-Identifier: BSD-2-Clause-Patent
// vangogh's FocalTech FT3518 touch probe. `sheng-touch=<n>` (the same contract the boot monitor uses for the
// picasso Novatek probe):
//   1  read-only snapshot of the GCC branches, the SE7 DFS levels and the touch pins
//   2  vote the QUP clocks, wait for them, configure the pins (I2C 6/7, reset 4, IRQ 5, vdd 8)
//   3  initialize GENI I2C, choose the serial clock, pulse reset, read the chip id
//   4  read contact reports and publish contacts into the shared HID snapshot
// Every stage reports through native_focaltech_touch_* globals and a PICASSO_FTS line; the first failure stops the
// sequence. Board constants and their sources are in FtBoard.h. The FT3518 keeps its firmware in internal flash,
// so there is no download stage (the vendor probe never upgrades this chip: focaltech_flash.c upgrade_func_list).
#include <IOKit/IOLib.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <libkern/c++/OSSharedPtr.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <string.h>
extern "C" {
#include "GeniI2c.h"
#include "FtBoard.h"
#include "TouchPublish.h"
}

extern "C" {
// The HID digitizer reads this snapshot under the seqlock (PicassoBootProgressStubs.c owns the storage).
extern volatile uint32_t native_touch_seq;
extern volatile uint32_t native_touch_count;
extern volatile uint32_t native_touch_writer_lock;
extern volatile int native_touch_xs[12], native_touch_ys[12], native_touch_ids[12];
void native_touch_notify(void);
void picasso_hid_physical_contact(void);
static void notifyThunk(void *published) {
    if (published) *static_cast<bool *>(published) = true;
    native_touch_notify();
}

// Entry points the boot monitor weak-imports (same names as the picasso Novatek probe: only one touch driver is
// ever linked, so reusing the contract needs no kernel-side change).
void native_novatek_touch_probe(void);
void native_novatek_touch_boot_report(unsigned now);

// Probe diagnostics.
volatile uint32_t native_focaltech_touch_stage_req, native_focaltech_touch_stage, native_focaltech_touch_step;
volatile uint32_t native_focaltech_touch_frames, native_focaltech_touch_valid, native_focaltech_touch_contacts;
volatile int native_focaltech_touch_error;
volatile int native_focaltech_touch_last_x = -1, native_focaltech_touch_last_y = -1;
volatile uint32_t native_focaltech_touch_chip_id[2];
volatile uint32_t native_focaltech_touch_irq_stats[3];   // [0] armed/failed, [1] handled, [2] spurious
}

namespace {
struct Region {
    OSSharedPtr<IOMemoryMap> keep;
    uintptr_t va;
};

bool mapRegion(uint64_t phys, uint64_t length, Region &region) {
    IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
    if (!memory) return false;
    region.keep = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
    memory->release();
    region.va = region.keep ? static_cast<uintptr_t>(region.keep->getVirtualAddress()) : 0;
    return region.va != 0;
}

// Warm-reset marker in IMEM (the USB console owns 0x65c; 0x800 is this driver's separate word).
#define TB_IMEM_BASE 0x146ab000ULL
#define TB_IMEM_MARKER 0x800
#define TB_MARKER_TAG 0x46540000u          // 'FT'
static uintptr_t gMarkerVa;
static bool markerMap(void) {
    if (gMarkerVa) return true;
    IODeviceMemory *memory = IODeviceMemory::withRange(TB_IMEM_BASE, 0x1000);
    if (!memory) return false;
    IOMemoryMap *map = memory->map(kIOMapAnywhere | kIOMapInhibitCache);
    memory->release();
    if (!map) return false;
    gMarkerVa = static_cast<uintptr_t>(map->getVirtualAddress());
    return gMarkerVa != 0;
}
static void marker(uint32_t code) {
    if (!gMarkerVa && !markerMap()) return;
    *reinterpret_cast<volatile uint32_t *>(gMarkerVa + TB_IMEM_MARKER) = TB_MARKER_TAG | (code & 0xffffu);
    __asm__ volatile("dsb sy" ::: "memory");
}

uint32_t rd32(uintptr_t base, uint32_t reg) { return *reinterpret_cast<volatile uint32_t *>(base + reg); }
void wr32(uintptr_t base, uint32_t reg, uint32_t value) {
    *reinterpret_cast<volatile uint32_t *>(base + reg) = value;
    __asm__ volatile("dsb sy" ::: "memory");
}

void ioDelayUs(void *, unsigned us) {
    if (us > 50) { IODelay(us); return; }
    uint64_t frequency = 0, start = 0, now = 0;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(frequency));
    __asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(start));
    const uint64_t ticks = (frequency ? frequency : TB_XO_HZ) * us / 1000000 + 1;
    do { __asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(now)); } while (now - start < ticks);
}

struct SoftBus {
    uintptr_t sda, scl;
};

struct Session {
    Region gccLow, gccQuv, gccCore, gccVote, tlmmEast, tlmmWest, wrapper, se;
    struct geni_i2c i2c;
    SoftBus bus;
    bool soft;                             // panel bus is bit-banged on the TLMM, not GENI
    uint64_t levelHz[TB_DFS_LEVELS];
    uint32_t info[18];
    unsigned level, div;
    uint64_t actualHz;
};

// The touch pins live in two TLMM tiles: reset/irq (GPIO 4/5) in EAST, vdd and the I2C pair (GPIO 6/7/8) in WEST.
uintptr_t tlmmPin(const Session &s, unsigned gpio) {
    if (gpio <= TB_GPIO_IRQ) return s.tlmmEast.va + tb_tlmm_ctl(gpio) - tb_tlmm_ctl(TB_GPIO_RESET);
    return s.tlmmWest.va + tb_tlmm_ctl(gpio) - tb_tlmm_ctl(TB_GPIO_I2C_SDA);
}

void pinConfig(Session &s, unsigned gpio, uint32_t cfg) {
    const uintptr_t pin = tlmmPin(s, gpio);
    wr32(pin, 0, (rd32(pin, 0) & ~TB_PIN_CFG_MASK) | cfg);
}

// ---------------------------------------------------------------------------------------------------------------
// Bit-banged I2C on the two TLMM pads. The vangogh SE7 reports GENI_IF_DISABLE_RO.FIFO_IF_DISABLE (see the Novatek
// probe's step 30 / -95), i.e. the engine is strapped for GPI DMA, so the FIFO driver in GeniI2c.c has nothing to
// talk to. The pads themselves are ordinary TLMM pins: GPIO 6 = SDA, GPIO 7 = SCL (both WEST), and the board's
// external pull-ups provide the high level. Open drain is emulated with the pad's output-enable bit (TLMM +0x0 bit 9)
// while the pad level is read back from TLMM +0x4 bit 0 (pinctrl-msm msm_gpio_get). No GCC vote and no GENI state is
// involved, so this path also works before the QUP clocks are running.
// ---------------------------------------------------------------------------------------------------------------
// Quarter period. 5 us => 100 kHz, the rate the vendor adapter uses (the node has no clock-frequency property).
#define TB_SOFT_QUARTER_US 5u
#define TB_SOFT_IDLE_CFG tb_pin_cfg(TB_FUNC_GPIO, TB_DRIVE_6MA, TB_PULL_UP, 0)

static void busQuarter(const SoftBus &) { ioDelayUs(nullptr, TB_SOFT_QUARTER_US); }
static void lineLow(uintptr_t pin) {
    wr32(pin, TB_TLMM_IO_OFFSET, 0);              // load the value while the pad is still released ...
    wr32(pin, 0, TB_SOFT_IDLE_CFG | (1u << 9));   // ... then enable the output: no high glitch
}
static void lineRelease(uintptr_t pin) {
    wr32(pin, TB_TLMM_IO_OFFSET, 1);       // pre-load 1 so the release is glitch free
    wr32(pin, 0, TB_SOFT_IDLE_CFG);               // output disabled: the pull-up takes over
}
static int lineRead(uintptr_t pin) { return (int)(rd32(pin, TB_TLMM_IO_OFFSET) & 1u); }

static void busStart(const SoftBus &b) {
    lineRelease(b.sda);
    lineRelease(b.scl);
    busQuarter(b);
    lineLow(b.sda);                        // start: SDA falls while SCL is high
    busQuarter(b);
    lineLow(b.scl);
    busQuarter(b);
}
static void busStop(const SoftBus &b) {
    lineLow(b.sda);
    busQuarter(b);
    lineRelease(b.scl);
    busQuarter(b);
    lineRelease(b.sda);                    // stop: SDA rises while SCL is high
    busQuarter(b);
}
static void busRestart(const SoftBus &b) {
    lineRelease(b.sda);
    busQuarter(b);
    lineRelease(b.scl);
    busQuarter(b);
    lineLow(b.sda);
    busQuarter(b);
    lineLow(b.scl);
    busQuarter(b);
}
// Returns true when the slave acknowledged (SDA pulled low during the ninth clock).
static bool busWriteByte(const SoftBus &b, uint8_t value) {
    for (int bit = 7; bit >= 0; --bit) {
        if ((value >> bit) & 1u) lineRelease(b.sda); else lineLow(b.sda);
        busQuarter(b);
        lineRelease(b.scl);
        busQuarter(b);
        lineLow(b.scl);
        busQuarter(b);
    }
    lineRelease(b.sda);                    // ninth clock: sample the acknowledge
    busQuarter(b);
    lineRelease(b.scl);
    busQuarter(b);
    const bool ack = lineRead(b.sda) == 0;
    lineLow(b.scl);
    busQuarter(b);
    return ack;
}
static uint8_t busReadByte(const SoftBus &b, bool ack) {
    uint8_t value = 0;
    lineRelease(b.sda);                    // the slave drives the data line
    for (int bit = 7; bit >= 0; --bit) {
        busQuarter(b);
        lineRelease(b.scl);
        busQuarter(b);
        value = (uint8_t)((value << 1) | (lineRead(b.sda) ? 1u : 0u));
        lineLow(b.scl);
        busQuarter(b);
    }
    if (ack) lineLow(b.sda); else lineRelease(b.sda);
    busQuarter(b);
    lineRelease(b.scl);
    busQuarter(b);
    lineLow(b.scl);
    lineRelease(b.sda);
    busQuarter(b);
    return value;
}

// Address probe only: start, address byte, stop. The reply is the acknowledge bit.
static int busAddressAck(const SoftBus &b, uint8_t address7) {
    busStart(b);
    const bool ack = busWriteByte(b, (uint8_t)(address7 << 1));
    busStop(b);
    return ack ? 0 : -6;
}

// Register read: write the register index, repeated start, read len bytes (last one NACKed).
static int busReadRegs(const SoftBus &b, uint8_t reg, uint8_t *out, size_t len) {
    if (!len) return -22;
    busStart(b);
    if (!busWriteByte(b, (uint8_t)(TB_I2C_ADDR << 1))) { busStop(b); return -6; }
    if (!busWriteByte(b, reg)) { busStop(b); return -7; }
    busRestart(b);
    if (!busWriteByte(b, (uint8_t)((TB_I2C_ADDR << 1) | 1u))) { busStop(b); return -8; }
    for (size_t i = 0; i < len; ++i) out[i] = busReadByte(b, i + 1 < len);
    busStop(b);
    return 0;
}
static int busWriteRegs(const SoftBus &b, const uint8_t *data, size_t len) {
    if (!len) return -22;
    busStart(b);
    if (!busWriteByte(b, (uint8_t)(TB_I2C_ADDR << 1))) { busStop(b); return -6; }
    for (size_t i = 0; i < len; ++i) {
        if (!busWriteByte(b, data[i])) { busStop(b); return -9; }
    }
    busStop(b);
    return 0;
}


uint32_t ioRd(void *ctx, uint32_t reg) { return rd32(reinterpret_cast<uintptr_t>(ctx), reg); }
void ioWr(void *ctx, uint32_t reg, uint32_t value) { wr32(reinterpret_cast<uintptr_t>(ctx), reg, value); }

void fail(unsigned step, int code) {
    native_focaltech_touch_step = step;
    if (!native_focaltech_touch_error) native_focaltech_touch_error = code ? code : -1;
    printf("PICASSO_FTS fail step=%u rc=%d\n", step, code);
}
void setStage(unsigned stage) { native_focaltech_touch_stage = stage; }

bool swBranchRunning(uint32_t cbcr) { return !(cbcr & TB_CBCR_CLK_OFF) || (cbcr & 2u); }
bool clocksRunning(Session &s) {
    const uint32_t m_ahb = rd32(s.gccQuv.va, tb_gcc_offset(TB_GCC_BRANCH_M_AHB));
    const uint32_t s_ahb = rd32(s.gccQuv.va, tb_gcc_offset(TB_GCC_BRANCH_S_AHB));
    const uint32_t core = rd32(s.gccCore.va, tb_gcc_offset(TB_GCC_BRANCH_CORE));
    const uint32_t core2x = rd32(s.gccCore.va, tb_gcc_offset(TB_GCC_BRANCH_CORE_2X));
    const uint32_t s1 = rd32(s.gccQuv.va, tb_gcc_offset(TB_GCC_BRANCH_S1));
    return swBranchRunning(m_ahb) && swBranchRunning(s_ahb) && !(core & TB_CBCR_CLK_OFF) &&
           !(core2x & TB_CBCR_CLK_OFF) && !(s1 & TB_CBCR_CLK_OFF);
}

void readIdentity(Session &s) {
    s.i2c.se = (struct geni_io){reinterpret_cast<void *>(s.se.va), ioRd, ioWr, ioDelayUs};
    s.i2c.wrapper = (struct geni_io){reinterpret_cast<void *>(s.wrapper.va), ioRd, ioWr, ioDelayUs};
    if (geni_i2c_read_info(&s.i2c)) return;
    s.info[12] = s.i2c.info.qup_hw_version;
    s.info[13] = s.i2c.info.fw_revision;
    s.info[14] = s.i2c.info.if_disable;
    s.info[15] = s.i2c.info.hw_param0;
}

bool snapshot(Session &s, unsigned sub) {
    native_focaltech_touch_step = 1;
    printf("PICASSO_FTS sub=%u map gcc\n", sub);
    if (sub < 2) { printf("PICASSO_FTS sub=%u done\n", sub); return true; }
    marker(0x14);
    if (!mapRegion(TB_GCC_BASE + TB_GCC_LOW_PAGE, TB_GCC_PAGE_SIZE, s.gccLow)) { fail(1, -12); return false; }
    marker(0x15);
    if (!mapRegion(TB_GCC_BASE + TB_GCC_QUPV3_PAGE, TB_GCC_PAGE_SIZE, s.gccQuv)) { fail(1, -12); return false; }
    marker(0x16);
    if (!mapRegion(TB_GCC_BASE + TB_GCC_CORE_PAGE, TB_GCC_PAGE_SIZE, s.gccCore)) { fail(1, -12); return false; }
    marker(0x17);
    if (!mapRegion(TB_GCC_BASE + TB_GCC_VOTE_PAGE, TB_GCC_PAGE_SIZE, s.gccVote)) { fail(1, -12); return false; }
    marker(0x18);
    if (!mapRegion(TB_TLMM_BASE + TB_TLMM_EAST + tb_tlmm_ctl(TB_GPIO_RESET),
                   (TB_GPIO_IRQ - TB_GPIO_RESET + 1) * TB_GPIO_STRIDE, s.tlmmEast) ||
        !mapRegion(TB_TLMM_BASE + TB_TLMM_WEST + tb_tlmm_ctl(TB_GPIO_I2C_SDA),
                   (TB_GPIO_VDD - TB_GPIO_I2C_SDA + 1) * TB_GPIO_STRIDE, s.tlmmWest)) {
        fail(1, -12);
        return false;
    }
    marker(0x19);
    printf("PICASSO_FTS map gcc ok g=%08x q=%08x c=%08x v=%08x te=%08x tw=%08x\n",
           (unsigned)(s.gccLow.va >> 12), (unsigned)(s.gccQuv.va >> 12), (unsigned)(s.gccCore.va >> 12),
           (unsigned)(s.gccVote.va >> 12), (unsigned)(s.tlmmEast.va >> 12), (unsigned)(s.tlmmWest.va >> 12));
    if (sub < 3) { printf("PICASSO_FTS sub=%u done\n", sub); return true; }
    marker(0x1a);
    uint32_t *v = s.info;
    v[0] = rd32(s.gccVote.va, tb_gcc_offset(TB_GCC_VOTE_ENABLE));
    v[1] = rd32(s.gccQuv.va, tb_gcc_offset(TB_GCC_BRANCH_M_AHB));
    v[2] = rd32(s.gccQuv.va, tb_gcc_offset(TB_GCC_BRANCH_S_AHB));
    v[3] = rd32(s.gccCore.va, tb_gcc_offset(TB_GCC_BRANCH_CORE));
    v[4] = rd32(s.gccCore.va, tb_gcc_offset(TB_GCC_BRANCH_CORE_2X));
    v[5] = rd32(s.gccQuv.va, tb_gcc_offset(TB_GCC_BRANCH_S1));
    v[6] = rd32(s.gccLow.va, tb_gcc_offset(TB_GCC_GPLL0_MODE));
    for (unsigned l = 0; l < TB_DFS_LEVELS; ++l) {
        const uint32_t cfg = rd32(s.gccQuv.va, tb_gcc_offset(TB_GCC_DFS_S1_RCG + TB_RCG_PERF_CFG(l)));
        const uint32_t m = rd32(s.gccQuv.va, tb_gcc_offset(TB_GCC_DFS_S1_RCG + TB_RCG_PERF_M(l)));
        const uint32_t n = rd32(s.gccQuv.va, tb_gcc_offset(TB_GCC_DFS_S1_RCG + TB_RCG_PERF_N(l)));
        s.levelHz[l] = tb_dfs_level_hz(cfg, m, n);
    }
    marker(0x1b);
    printf("PICASSO_FTS read gcc ok\n");
    if (sub < 4) { printf("PICASSO_FTS sub=%u done\n", sub); return true; }
    marker(0x1c);
    v[9] = rd32(tlmmPin(s, TB_GPIO_I2C_SDA), 0);
    v[10] = rd32(tlmmPin(s, TB_GPIO_I2C_SCL), 0);
    v[11] = rd32(tlmmPin(s, TB_GPIO_VDD), 0);
    v[16] = rd32(tlmmPin(s, TB_GPIO_IRQ), TB_TLMM_INTR_CFG_OFFSET);
    printf("PICASSO_FTS read tlmm ok sda=%08x scl=%08x vdd=%08x irq=%08x\n", v[9], v[10], v[11], v[16]);
    if (sub < 5) { printf("PICASSO_FTS sub=%u done\n", sub); return true; }
    printf("PICASSO_FTS map se\n");
    marker(0x1d);
    if (!mapRegion(TB_QUP_WRAPPER_BASE, TB_QUP_WRAPPER_SIZE, s.wrapper)) { fail(1, -12); return false; }
    marker(0x1e);
    if (!mapRegion(TB_SE_BASE, TB_SE_SIZE, s.se)) { fail(1, -12); return false; }
    marker(0x1f);
    printf("PICASSO_FTS map se ok w=%08x s=%08x\n", (unsigned)(s.wrapper.va >> 12), (unsigned)(s.se.va >> 12));
    if (clocksRunning(s)) {
        marker(0x20);
        printf("PICASSO_FTS clocks running\n");
        readIdentity(s);
        marker(0x21);
        printf("PICASSO_FTS identity proto=%u fw=%08x if=%08x fifo=%u/%u w=%u hw=%08x qup=%08x\n",
               s.i2c.info.protocol, s.i2c.info.fw_revision, s.i2c.info.if_disable, s.i2c.info.tx_fifo_depth,
               s.i2c.info.rx_fifo_depth, s.i2c.info.fifo_width_bits, s.i2c.info.hw_param0,
               s.i2c.info.qup_hw_version);
    }
    marker(0x22);
    printf("PICASSO_FTS stage1 vote=%08x m_ahb=%08x s_ahb=%08x core=%08x core2x=%08x s1=%08x gpll0=%08x\n",
           v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
    return true;
}

bool enableClocks(Session &s) {
    native_focaltech_touch_step = 2;
    marker(0x31);
    // GPLL0 feeds the DFS levels: a shared vote, then wait for lock.
    wr32(s.gccVote.va, tb_gcc_offset(TB_GCC_GPLL0_ENABLE),
         rd32(s.gccVote.va, tb_gcc_offset(TB_GCC_GPLL0_ENABLE)) | 1u);
    bool locked = false;
    for (unsigned i = 0; i < 10000 && !locked; ++i) {
        locked = (rd32(s.gccLow.va, tb_gcc_offset(TB_GCC_GPLL0_MODE)) & TB_GPLL0_LOCK) != 0;
        if (!locked) IODelay(10);
    }
    marker(0x32);
    wr32(s.gccVote.va, tb_gcc_offset(TB_GCC_VOTE_ENABLE),
         rd32(s.gccVote.va, tb_gcc_offset(TB_GCC_VOTE_ENABLE)) | TB_VOTE_ALL);
    bool running = false;
    for (unsigned i = 0; i < 2000 && !running; ++i) {
        running = clocksRunning(s);
        if (!running) IODelay(10);
    }
    marker(0x33);
    s.info[0] = rd32(s.gccVote.va, tb_gcc_offset(TB_GCC_VOTE_ENABLE));
    printf("PICASSO_FTS stage2 gpll0=%08x running=%u\n", s.info[6], running ? 1u : 0u);
    if (!locked || !running) { fail(2, -110); return false; }
    marker(0x34);
    // Pins: the I2C pair runs the qup11 function; reset is driven high here (the probe pulses it in stage 3);
    // vdd is driven high in case the bootloader left the panel unpowered.
    pinConfig(s, TB_GPIO_I2C_SDA, TB_PIN_CFG_I2C);
    pinConfig(s, TB_GPIO_I2C_SCL, TB_PIN_CFG_I2C);
    pinConfig(s, TB_GPIO_RESET, TB_PIN_CFG_RESET);
    pinConfig(s, TB_GPIO_IRQ, TB_PIN_CFG_IRQ);
    pinConfig(s, TB_GPIO_VDD, TB_PIN_CFG_VDD);
    wr32(tlmmPin(s, TB_GPIO_RESET), TB_TLMM_IO_OFFSET, 1);
    wr32(tlmmPin(s, TB_GPIO_VDD), TB_TLMM_IO_OFFSET, 1);
    s.info[9] = rd32(tlmmPin(s, TB_GPIO_I2C_SDA), 0);
    s.info[10] = rd32(tlmmPin(s, TB_GPIO_I2C_SCL), 0);
    s.info[11] = rd32(tlmmPin(s, TB_GPIO_VDD), 0);
    s.info[16] = rd32(tlmmPin(s, TB_GPIO_IRQ), TB_TLMM_INTR_CFG_OFFSET);
    readIdentity(s);
    printf("PICASSO_FTS stage2 pins sda=%08x scl=%08x vdd=%08x irq=%08x se proto=%u\n",
           s.info[9], s.info[10], s.info[11], s.info[16], s.i2c.info.protocol);
    return true;
}

// FocalTech register access. Register write: one byte, bus held (the vendor adapter keeps the bus between the
// register write and the data read of one transfer). Register read: write reg, then read len bytes.
static int ftWriteReg(Session &s, uint8_t reg, int hold) {
    if (s.soft) return busWriteRegs(s.bus, &reg, 1);
    return geni_i2c_write(&s.i2c, &reg, 1, hold);
}
static int ftReadRegs(Session &s, uint8_t reg, uint8_t *out, size_t len) {
    if (s.soft) return busReadRegs(s.bus, reg, out, len);
    int r = ftWriteReg(s, reg, 1);
    if (r) return r;
    return geni_i2c_read(&s.i2c, out, len);
}

// Any plausible part number beats a hard-coded one: FT3518 answers 0x54, the FT8xxx family answers other values,
// and 0x00/0xFF only ever mean "no answer" or "bus floating".
static bool ftPlausibleId(uint8_t id) { return id != 0x00 && id != 0xFF; }

// Power, reset and identify over the bit-banged bus. Runs before GENI is touched: if the panel answers here the
// engine state is irrelevant, and if it does not the log says exactly where the transfer stopped.
static bool softProbe(Session &s) {
    native_focaltech_touch_step = 24;
    s.soft = true;                         // ftReadRegs/ftWriteReg follow this flag
    s.bus.sda = tlmmPin(s, TB_GPIO_I2C_SDA);
    s.bus.scl = tlmmPin(s, TB_GPIO_I2C_SCL);
    pinConfig(s, TB_GPIO_I2C_SDA, TB_SOFT_IDLE_CFG);
    pinConfig(s, TB_GPIO_I2C_SCL, TB_SOFT_IDLE_CFG);
    pinConfig(s, TB_GPIO_RESET, TB_PIN_CFG_RESET);
    pinConfig(s, TB_GPIO_IRQ, TB_PIN_CFG_IRQ);
    pinConfig(s, TB_GPIO_VDD, TB_PIN_CFG_VDD);
    lineRelease(s.bus.sda);
    lineRelease(s.bus.scl);
    IODelay(500);
    printf("PICASSO_FTS soft idle sda=%d scl=%d\n", lineRead(s.bus.sda), lineRead(s.bus.scl));

    wr32(tlmmPin(s, TB_GPIO_VDD), TB_TLMM_IO_OFFSET, 1);   // focaltech,vdd-gpio = <&tlmm 8>: panel 3V rail on
    IOSleep(10);
    wr32(tlmmPin(s, TB_GPIO_RESET), TB_TLMM_IO_OFFSET, 0);
    IODelay(TB_RESET_LOW_MS * 1000u);
    wr32(tlmmPin(s, TB_GPIO_RESET), TB_TLMM_IO_OFFSET, 1);
    IOSleep(TB_RESET_SETTLE_MS);

    uint8_t id[2] = {0, 0};
    int rc = -110;
    for (unsigned i = 0; i < TB_ID_POLLS; ++i) {
        rc = ftReadRegs(s, FTS_REG_CHIP_ID_A, id, 2);
        if (!rc && ftPlausibleId(id[0])) break;
        IOSleep(TB_ID_POLL_GAP_MS);
    }
    native_focaltech_touch_chip_id[0] = id[0];
    native_focaltech_touch_chip_id[1] = id[1];
    printf("PICASSO_FTS soft id=%02x %02x rc=%d\n", id[0], id[1], rc);
    if (!rc && ftPlausibleId(id[0])) return true;

    // Nothing at 0x38: sweep the bus and report the first few responders, so a wrong address or a dead rail is
    // distinguishable from a protocol problem without another boot.
    unsigned found = 0;
    for (uint8_t a = 0x08; a <= 0x77 && found < 6; ++a) {
        if (a == TB_I2C_ADDR) continue;
        if (busAddressAck(s.bus, a) == 0) {
            printf("PICASSO_FTS soft ack addr=%02x\n", a);
            ++found;
        }
    }
    printf("PICASSO_FTS soft scan found=%u\n", found);
    s.soft = false;                        // hand the panel bus back to the GENI path
    return false;
}


static int ftResetAndIdentify(Session &s) {
    // Vendor fts_reset_proc(75): reset low 5 ms, high, wait, then poll the chip id 10 times at 15 ms.
    wr32(tlmmPin(s, TB_GPIO_RESET), TB_TLMM_IO_OFFSET, 0);
    IODelay(TB_RESET_LOW_MS * 1000u);
    wr32(tlmmPin(s, TB_GPIO_RESET), TB_TLMM_IO_OFFSET, 1);
    IOSleep(TB_RESET_SETTLE_MS);
    uint8_t id[2] = {0, 0};
    int rc = -110;
    for (unsigned i = 0; i < TB_ID_POLLS; ++i) {
        rc = ftReadRegs(s, FTS_REG_CHIP_ID_A, id, 2);
        if (!rc && id[0] == 0x54) break;
        IOSleep(TB_ID_POLL_GAP_MS);
    }
    native_focaltech_touch_chip_id[0] = id[0];
    native_focaltech_touch_chip_id[1] = id[1];
    printf("PICASSO_FTS chip id=%02x %02x rc=%d\n", id[0], id[1], rc);
    if (rc) return rc;
    return ftPlausibleId(id[0]) ? 0 : GENI_ERR_PROTO;
}

bool openBus(Session &s, unsigned hz) {
    native_focaltech_touch_step = 3;
    marker(0x41);
    s.i2c.poll_limit = 400000;
    int r = geni_i2c_read_info(&s.i2c);
    if (!r) {
        printf("PICASSO_FTS geni proto=%u if=%08x fifo=%u/%u w=%u hw=%08x\n", s.i2c.info.protocol,
               s.i2c.info.if_disable, s.i2c.info.tx_fifo_depth, s.i2c.info.rx_fifo_depth,
               s.i2c.info.fifo_width_bits, s.i2c.info.hw_param0);
    }
    r = geni_i2c_init(&s.i2c);
    if (r) { fail(30, r); return false; }
    marker(0x42);
    r = geni_i2c_set_clock(&s.i2c, hz);
    if (r) { fail(31, r); return false; }
    s.actualHz = s.i2c.cur_speed_hz;
    marker(0x43);
    // Pulse reset and identify before reporting the engine state, so a dead panel does not hide behind the clocks.
    const int id_rc = ftResetAndIdentify(s);
    printf("PICASSO_FTS stage3 hz=%u proto=%u fifo=%u/%u w=%u id_rc=%d\n",
           (unsigned)s.actualHz, s.i2c.info.protocol, s.i2c.info.tx_fifo_depth, s.i2c.info.rx_fifo_depth,
           s.i2c.info.fifo_width_bits, id_rc);
    marker(0x44);
    if (id_rc) { fail(33, id_rc); return false; }
    return true;
}

static volatile uint32_t gIrqEvents;
static volatile uint32_t *gIrqPin;
static void touchIrq(void *) {
    if (!gIrqPin) return;
    if (!(gIrqPin[TB_TLMM_INTR_STATUS_OFFSET / 4] & 1)) {
        native_focaltech_touch_irq_stats[2] = native_focaltech_touch_irq_stats[2] + 1;
        return;
    }
    gIrqPin[TB_TLMM_INTR_STATUS_OFFSET / 4] = 0;
    native_focaltech_touch_irq_stats[1] = native_focaltech_touch_irq_stats[1] + 1;
    __atomic_fetch_add(&gIrqEvents, 1, __ATOMIC_RELEASE);
    thread_wakeup(reinterpret_cast<event_t>(const_cast<uint32_t *>(&gIrqEvents)));
}
extern "C" int armcpu_spi_attach(uint32_t, bool, void (*)(void *), void *, uint32_t *, uint64_t *)
    __attribute__((weak_import));
bool armTouchIrq(Session &s) {
    volatile uint32_t *pin = reinterpret_cast<volatile uint32_t *>(tlmmPin(s, TB_GPIO_IRQ));
    gIrqPin = pin;
    pin[TB_TLMM_INTR_CFG_OFFSET / 4] = TB_TLMM_INTR_CFG & ~TB_TLMM_INTR_ENABLE;
    pin[TB_TLMM_INTR_STATUS_OFFSET / 4] = 0;
    __asm__ volatile("dsb sy" ::: "memory");
    if (!armcpu_spi_attach) { native_focaltech_touch_irq_stats[0] = 2; return false; }
    const int rc = armcpu_spi_attach(TB_TLMM_SUMMARY_INTID, false, touchIrq, nullptr, nullptr, nullptr);
    if (rc) { native_focaltech_touch_irq_stats[0] = 2; return false; }
    pin[TB_TLMM_INTR_CFG_OFFSET / 4] = TB_TLMM_INTR_CFG;   // enable last
    native_focaltech_touch_irq_stats[0] = 1;
    return true;
}

bool publish(const int *xs, const int *ys, const int *ids, int n) {
    static bool physicalActive;
    if (!n && !physicalActive) return true;   // idle polls must not erase a synthetic startup swipe
    bool published = false;
    if (n > 0) picasso_hid_physical_contact();
    struct nvt_touch_publication pub;
    pub.seq = &native_touch_seq;
    pub.count = &native_touch_count;
    pub.xs = native_touch_xs;
    pub.ys = native_touch_ys;
    pub.ids = native_touch_ids;
    pub.slots = 12;
    pub.sink = &notifyThunk;
    pub.sink_context = &published;
    pub.writer_lock = &native_touch_writer_lock;
    nvt_touch_publish(&pub, xs, ys, ids, static_cast<unsigned>(n));
    if (published) physicalActive = n > 0;
    return published;
}

#define FT_PNT_BUF 63u                     // 3 header bytes + 10 * 6 bytes per contact

// Decode a FocalTech report (vendor fts_read_touchdata) into down contacts in the publication convention
// (0.1 panel pixels). Layout: [0] mode, [1] ?, [2] point count; then 6-byte records starting at 3:
// [3] event flag (top 2 bits) | x high nibble, [4] x low, [5] id (top 4 bits) | y high nibble, [6] y low.
// Event flags: 0 down, 1 up, 2 contact.
int parseReport(const uint8_t *buf, int *xs, int *ys, int *ids) {
    const unsigned num = buf[2] & 0x0Fu;
    if (num == 0x0F && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF && buf[4] == 0xFF &&
        buf[5] == 0xFF && buf[6] == 0xFF) {
        return -2;                          // all-0xFF: the firmware wants a recovery reset
    }
    int n = 0;
    for (unsigned i = 0; i < TB_MAX_CONTACTS; ++i) {
        const unsigned base = 6u * i;
        const unsigned id = buf[FTS_ID_OFF + base] >> 4;
        if (id >= TB_MAX_CONTACTS) break;
        const unsigned flag = buf[FTS_X_OFF + base] >> 6;
        if (flag != 0 && flag != 2) continue;   // up
        const int x = ((buf[FTS_X_OFF + base] & 0x0F) << 8) | buf[FTS_X_OFF + base + 1];
        const int y = ((buf[FTS_Y_OFF + base] & 0x0F) << 8) | buf[FTS_Y_OFF + base + 1];
        if (x > TB_PANEL_WIDTH_PX || y > TB_PANEL_HEIGHT_PX) continue;
        const struct tb_hid_point point = tb_hid_from_panel(x * 10, y * 10);
        xs[n] = point.x;
        ys[n] = point.y;
        ids[n] = (int)id;
        ++n;
    }
    return n;
}

// Read one frame of contacts. Returns the down-contact count, or < 0 on error (-2 = recovery needed).
int sampleContacts(Session &s, int *xs, int *ys, int *ids) {
    static unsigned dumps, errors;
    uint8_t probe[16];
    int r = ftReadRegs(s, 0x00, probe, sizeof probe);
    if (r) {
        if (errors < 6) { printf("PICASSO_FTS frame err=%d\n", r); ++errors; }
        return -1;
    }
    if (dumps < 3) {
        printf("PICASSO_FTS frame hdr %02x %02x %02x %02x %02x %02x %02x %02x | %02x %02x %02x %02x %02x %02x %02x %02x\n",
               probe[0], probe[1], probe[2], probe[3], probe[4], probe[5], probe[6], probe[7], probe[8], probe[9],
               probe[10], probe[11], probe[12], probe[13], probe[14], probe[15]);
        ++dumps;
    }
    if (probe[15] == 0xFF) {
        // No point data in the short header: the vendor reads the interrupt ack register and moves on.
        uint8_t ack = 0;
        (void)ftReadRegs(s, 0x3E, &ack, 1);
        return 0;
    }
    uint8_t buf[FT_PNT_BUF];
    r = ftReadRegs(s, 0x00, buf, FT_PNT_BUF);
    if (r) return -1;
    const int n = parseReport(buf, xs, ys, ids);
    if (n >= 0) {
        uint8_t ack = 0;
        (void)ftReadRegs(s, 0x3E, &ack, 1);
    }
    return n;
}

void frameLoop(Session &s) {
    native_focaltech_touch_step = 5;
    unsigned pollOnly = 0;
    PE_parse_boot_argn("sheng-touch-poll", &pollOnly, sizeof pollOnly);
    // The bit-banged path polls: the vangogh DT wires the panel interrupt to a line this driver has no routing for
    // (focaltech,irq-gpio = gpio5 while the node's interrupts property names gpio39), and a 10 ms poll is well
    // inside the panel's report interval anyway.
    const bool irqArmed = (pollOnly || s.soft) ? false : armTouchIrq(s);
    printf("PICASSO_FTS stage4 loop irq=%u\n", irqArmed ? 1u : 0u);
    uint32_t seen = gIrqEvents;
    unsigned printed = 0;
    unsigned busErrors = 0;
    int lastMask = 0;
    int xs[TB_MAX_CONTACTS], ys[TB_MAX_CONTACTS], ids[TB_MAX_CONTACTS];
    for (;;) {
        if (irqArmed) {
            // Interrupt driven, with a timed fallback so a polarity mismatch cannot park the thread forever.
            uint64_t deadline = 0;
            nanoseconds_to_deadline(20000000ULL, &deadline);
            assert_wait_deadline(reinterpret_cast<event_t>(const_cast<uint32_t *>(&gIrqEvents)), THREAD_UNINT, deadline);
            if (gIrqEvents != seen) clear_wait(current_thread(), THREAD_AWAKENED);
            else thread_block(THREAD_CONTINUE_NULL);
            seen = gIrqEvents;
        }
        int n = sampleContacts(s, xs, ys, ids);
        if (n == -2) {
            // Recovery reset, vendor fts_tp_state_recovery: pulse reset and wait for the id again.
            if (printed < 4) { printf("PICASSO_FTS recovery reset\n"); ++printed; }
            (void)ftResetAndIdentify(s);
            n = 0;
        } else if (n < 0) {
            // Tolerate transient bus errors: a single glitch must not kill touch permanently.
            if (++busErrors <= 8) { if (!irqArmed) IOSleep(10); continue; }
            fail(50, n);
            return;
        }
        busErrors = 0;
        ++native_focaltech_touch_valid;
        int mask = 0;
        for (int i = 0; i < n; ++i) mask |= 1 << ids[i];
        if (mask != lastMask || n > 0) {
            // Publish every frame that has contacts, and the empty frame that lifts the last set.
            if (n > 0 || lastMask) {
                if (publish(xs, ys, ids, n)) {
                    ++native_focaltech_touch_frames;
                    native_focaltech_touch_contacts = (uint32_t)n;
                    if (n > 0) {
                        native_focaltech_touch_last_x = xs[0];
                        native_focaltech_touch_last_y = ys[0];
                    }
                    if (printed < 8) {
                        printf("PICASSO_FTS report contacts=%d id=%d x=%d y=%d\n", n, n > 0 ? ids[0] : -1,
                               n > 0 ? xs[0] : -1, n > 0 ? ys[0] : -1);
                        ++printed;
                    }
                }
            }
            lastMask = mask;
        }
        if (!irqArmed) IOSleep(pollOnly ? 4 : 10);
    }
}

void runStage4(Session &s) {
    setStage(4);
    frameLoop(s);
}
}  // namespace

namespace {
void probeBody(unsigned stage, const char *where) {
    marker(0x11);
    Session *s = static_cast<Session *>(IOMalloc(sizeof(Session)));
    if (!s) { fail(0, -12); return; }
    bzero(s, sizeof(*s));
    marker(0x12);
    unsigned sub = 5;
    PE_parse_boot_argn("sheng-touch-sub", &sub, sizeof sub);
    printf("PICASSO_FTS %s start stage=%u sub=%u\n", where, stage, sub);
    unsigned hz = TB_I2C_DEFAULT_HZ;
    PE_parse_boot_argn("sheng-touch-hz", &hz, sizeof hz);
    if (hz > TB_I2C_MAX_HZ) hz = TB_I2C_MAX_HZ;
    do {
        if (!snapshot(*s, sub)) break;
        setStage(1);
        if (stage < 2) break;
        // The TLMM path first: it needs neither the QUP clocks nor a FIFO-capable engine, and it is the only path
        // that can work when GENI_IF_DISABLE_RO reports the FIFO interface as strapped off.
        if (softProbe(*s)) {
            native_focaltech_touch_step = 25;
            printf("PICASSO_FTS stage3 soft hz=%u\n", 1000000u / (4u * TB_SOFT_QUARTER_US));
            setStage(3);
            if (stage < 4) break;
            runStage4(*s);
            break;
        }
        if (!enableClocks(*s)) break;
        setStage(2);
        if (stage < 3) break;
        if (!openBus(*s, hz)) break;
        setStage(3);
        if (stage < 4) break;
        runStage4(*s);
    } while (0);
    printf("PICASSO_FTS %s end stage=%u step=%u error=%d\n", where, native_focaltech_touch_stage,
           native_focaltech_touch_step, native_focaltech_touch_error);
}

void touchThread(void *, wait_result_t) {
    printf("PICASSO_FTS thread entry\n");
    thread_set_thread_name(current_thread(), "picasso-fts");
    probeBody(native_focaltech_touch_stage_req, "thread");
}
}  // namespace

// Callable from the boot monitor. `sheng-touch=<n>` selects the last stage to run; without it the probe does nothing.
extern "C" void native_novatek_touch_probe(void) {
    unsigned stage = 0;
    if (!PE_parse_boot_argn("sheng-touch", &stage, sizeof stage) || stage < 1) return;
    if (native_focaltech_touch_stage_req) return;
    if (stage > 4) stage = 4;
    marker(0x01);
    native_focaltech_touch_stage_req = stage;
    unsigned sync = 0;
    PE_parse_boot_argn("sheng-touch-sync", &sync, sizeof sync);
    if (sync) {
        if (stage > 1) stage = 1;
        probeBody(stage, "sync");
        return;
    }
    thread_t thread = nullptr;
    marker(0x02);
    if (kernel_thread_start(touchThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else native_focaltech_touch_error = -12;
    marker(0x03);
}

// Called by the boot monitor early in the pass; reports and clears the marker left by the previous boot.
extern "C" void native_novatek_touch_boot_report(unsigned now) {
    static unsigned reported;
    if (reported || now < 30) return;
    reported = 1;
    if (!markerMap()) { printf("PICASSO_FTS imem map failed\n"); return; }
    volatile uint32_t *slot = reinterpret_cast<volatile uint32_t *>(gMarkerVa + TB_IMEM_MARKER);
    const uint32_t previous = *slot;
    *slot = 0x4654fffeu;
    __asm__ volatile("dsb sy" ::: "memory");
    const uint32_t readback = *slot;
    *slot = 0;
    __asm__ volatile("dsb sy" ::: "memory");
    printf("PICASSO_FTS imem prev=%08x rb=%08x now=%u\n", previous, readback, now);
}
