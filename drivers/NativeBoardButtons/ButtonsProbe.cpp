// SPDX-License-Identifier: BSD-2-Clause-Patent
// vangogh's hardware buttons: the side (power) key and the two volume keys.
//
// None of the three is on a SoC pin. They hang off the PMICs, which the SoC reaches over SPMI through the PMIC
// arbiter at 0xc440000 (dt: qcom,spmi@c440000, reg-names core/chnls/obsrvr/intr/cnfg). The vangogh device tree
// names them exactly once:
//
//   spmi@c440000/qcom,pm6350@0/qcom,power-on@800     qcom,pon_1  linux,code=116  (KEY_POWER)
//                                                    qcom,pon_2  linux,code=114  (KEY_VOLUMEDOWN)
//   spmi@c440000/qcom,pm7250b@2/qcom,power-on@800    the same block on the second PMIC
//
// Volume up has no node: on this board it is an ordinary PMIC GPIO configured as a pulled-up input.
//
// THIS DRIVER WRITES NOTHING TO A PMIC. The only write is the arbiter's observer command word, which asks the
// arbiter for a read; the PMICs make the phone's supplies and nothing here may disturb them. The register layout
// and the access sequence are the ones proven on hardware by oosexgang/hakios-xnu-kernel,
// drivers/input/hakios.buttons.c and drivers/power/hakios.pmic.c (arbiter v5.2, 512 channels), with the base
// addresses taken from this board's own device tree rather than from that phone's.
//
// Mapping a bit to a key without a person pressing keys at boot: the driver samples the inputs for two seconds at
// start and takes what it sees there as "released", so it never assumes whether this PMIC reports a press as a set
// bit or as a cleared bit. A level that departs from that baseline for two consecutive samples is a press.
// `sheng-buttons=<n>`: 1 inventory only, 2 (the default) inventory and the key loop.
#include <IOKit/IOLib.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <libkern/c++/OSSharedPtr.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>

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

inline uint32_t rd(uintptr_t base, uint32_t offset) { return *reinterpret_cast<volatile uint32_t *>(base + offset); }
inline void wr(uintptr_t base, uint32_t offset, uint32_t value) {
    *reinterpret_cast<volatile uint32_t *>(base + offset) = value;
}

// PMIC arbiter, version 5 (dt reg-names core/obsrvr, from vangogh-sm7250.dtsi).
constexpr uint64_t kArbCorePhys = 0x0c440000ull;
constexpr uint64_t kArbCoreSize = 0x1100ull;
constexpr uint64_t kArbObserverPhys = 0x0e600000ull;
constexpr uint64_t kArbObserverSize = 0x100000ull;
constexpr uint32_t kArbVersion = 0x000;
constexpr uint32_t kArbFeatures = 0x004;
constexpr uint32_t kArbChannelTable = 0x900;          // one 32-bit entry per channel, [19:8] = slave:peripheral
constexpr uint32_t kArbMaxChannels = 512;
constexpr uint32_t kObsCommand = 0x00;
constexpr uint32_t kObsStatus = 0x08;
constexpr uint32_t kObsData0 = 0x18;
constexpr uint32_t kObsData1 = 0x1c;
constexpr uint32_t kStatusDone = 1u << 0;
constexpr uint32_t kStatusFailure = 1u << 1;
constexpr uint32_t kStatusDenied = 1u << 2;
constexpr uint32_t kStatusDropped = 1u << 3;
constexpr uint32_t kCommandRead = 1u << 27;
constexpr uint32_t kChannelStride = 0x80;

// Peripheral registers (every PMIC block is 256 bytes).
constexpr uint8_t kRegPeripheralType = 0x04;
constexpr uint8_t kRegPeripheralSubtype = 0x05;
constexpr uint8_t kRegRtStatus = 0x10;                // PON live input state: KPDPWR and RESIN, bit layout below
// An SPMI GPIO is a different block and its status is NOT at 0x10: Linux pinctrl-spmi-gpio.c has
// PMIC_GPIO_REG_STATUS = 0x08 (bit 0 the pin level). Reading 0x10 here returned the block's interrupt
// registers, so the volume key was watched through a register that never changes with the pin.
constexpr uint8_t kRegGpioStatus = 0x08;
// The pull does not live in the mode register. Linux pinctrl-spmi-gpio.c (and the hakios PMIC driver, which reads
// the same eight bytes) puts mode at 0x40, the analogue input control at 0x42 with the pull in bits [2:0] (0-3 up,
// 4 down, 5 none) and the enable in bit 7 of 0x46. The earlier filter looked for the pull in bits [5:3] of 0x40,
// which the hardware does not use, so a pulled-up key GPIO could be skipped as "not a key input".
constexpr uint8_t kRegGpioMode = 0x40;                // [1:0] mode, 0 = input
constexpr uint8_t kRegGpioPull = 0x42;                // [2:0] pull, < 4 = up
constexpr uint8_t kRegGpioEnable = 0x46;              // [7] enabled

// Peripheral types (Linux qcom-spmi-pmic, qpnp-power-on).
constexpr uint8_t kTypePon = 0x01;
constexpr uint8_t kTypeGpio = 0x10;
constexpr uint8_t kGpioSubtypeFirst = 0x10;
constexpr uint8_t kGpioSubtypeLast = 0x13;

// HID report 2 bits (NativeHidInput.cpp: bit 1 power, bit 2 volume up, bit 3 volume down).
constexpr uint32_t kHidPower = 1u << 1;
constexpr uint32_t kHidVolumeUp = 1u << 2;
constexpr uint32_t kHidVolumeDown = 1u << 3;

constexpr unsigned kPonBlocks = 4;                    // PM6350@0, PM7250B@2, PM6150L@4 and one spare
constexpr unsigned kGpioBlocks = 48;
constexpr unsigned kPollMs = 16;
constexpr unsigned kSettle = 6;                       // equal samples in a row before a level is believed

struct Input {
    uint32_t channel;
    uint16_t ppid;
    uint8_t  baseline;      // what the input reads with nothing pressed
    uint8_t  last;
    unsigned steady;
    bool     live;
};

struct State {
    Input pon[kPonBlocks];
    unsigned ponCount;
    Input gpio[kGpioBlocks];
    unsigned gpioCount;
    uint32_t published;
    uint32_t presses[3];
};

Region gCore;
Region gObserver;
State g;
bool gDead;
unsigned gFailures;
unsigned gPolls;

typedef void (*ButtonSink)(void *, uint32_t);
ButtonSink gSink;
void *gSinkContext;

// Read one byte from the peripheral behind `channel`. nullptr, or what the arbiter said went wrong.
const char *spmiRead(uint32_t channel, uint8_t reg, uint8_t *bytes, uint32_t count) {
    const uintptr_t window = gObserver.va + kChannelStride * channel;
    uint64_t timeout = 0, start = mach_absolute_time();
    uint32_t status = 0;
    nanoseconds_to_absolutetime(2000000ull, &timeout);   // 2 ms, a transfer takes tens of microseconds

    wr(window, kObsCommand, kCommandRead | (static_cast<uint32_t>(reg) << 4) | (count - 1));
    do {
        status = rd(window, kObsStatus);
    } while (!(status & kStatusDone) && (mach_absolute_time() - start) < timeout);

    const uint32_t data0 = rd(window, kObsData0);
    const uint32_t data1 = count > 4 ? rd(window, kObsData1) : 0;

    if (!(status & kStatusDone)) { ++gFailures; return "no answer"; }
    if (status & kStatusDenied) { ++gFailures; return "denied"; }
    if (status & kStatusFailure) { ++gFailures; return "failed"; }
    if (status & kStatusDropped) { ++gFailures; return "dropped"; }
    for (uint32_t i = 0; i < count; ++i)
        bytes[i] = static_cast<uint8_t>((i < 4 ? data0 : data1) >> (8 * (i & 3)));
    return nullptr;
}

bool readByte(uint32_t channel, uint8_t reg, uint8_t *value) {
    return spmiRead(channel, reg, value, 1) == nullptr;
}

// Which keys are down right now, as HID report 2 bits.
uint32_t maskFromInputs();

void publish() {
    const uint32_t mask = maskFromInputs();
    if (mask == g.published) return;
    g.published = mask;
    if (gSink) gSink(gSinkContext, mask);
}

// Walk the arbiter's channel table and keep the blocks this driver needs.
void inventory() {
    const uint32_t version = rd(gCore.va, kArbVersion);
    const uint32_t features = rd(gCore.va, kArbFeatures);
    uint32_t channels = features & 0x7ff;
    // Vangogh answers 50010000 here: the major is packed BCD, so the byte [31:24] is 0x50 for 5.0, not 0x05.
    // Older revisions put the plain number in the same byte, and both forms appear in Qualcomm's own dumps.
    const uint32_t majorByte = (version >> 24) & 0xff;
    const uint32_t major = (majorByte & 0xf0) == 0x50 ? (majorByte >> 4) : majorByte;
    printf("PICASSO_BTN arbiter version=%08x major=%u features=%08x channels=%u\n",
           version, major, features, channels);
    if (major != 5) {
        // Version 7 moved the observer window and the command encoding. Guessing there could turn a read into a
        // write of a PMIC register, so this driver stops instead.
        printf("PICASSO_BTN arbiter is not version 5: nothing read\n");
        gDead = true;
        return;
    }
    if (channels == 0 || channels > kArbMaxChannels) channels = kArbMaxChannels;

    uint16_t seen[256];
    unsigned seenCount = 0;
    for (uint32_t n = 0; n < channels; ++n) {
        const uint32_t entry = rd(gCore.va, kArbChannelTable + 4 * n);
        if (entry == 0) continue;
        const uint16_t ppid = static_cast<uint16_t>((entry >> 8) & 0xfff);
        bool again = false;
        for (unsigned i = 0; i < seenCount; ++i) if (seen[i] == ppid) { again = true; break; }
        if (again) continue;
        if (seenCount < sizeof seen / sizeof seen[0]) seen[seenCount++] = ppid;

        uint8_t type = 0, subtype = 0;
        if (!readByte(n, kRegPeripheralType, &type)) continue;
        if (!readByte(n, kRegPeripheralSubtype, &subtype)) continue;

        if (type == kTypePon && g.ponCount < kPonBlocks) {
            Input &in = g.pon[g.ponCount++];
            in = Input{};
            in.channel = n;
            in.ppid = ppid;
            printf("PICASSO_BTN pon slave=%u peripheral=%02x channel=%u subtype=%02x\n",
                   ppid >> 8, ppid & 0xff, n, subtype);
        } else if (type == kTypeGpio && subtype >= kGpioSubtypeFirst && subtype <= kGpioSubtypeLast &&
                   g.gpioCount < kGpioBlocks) {
            uint8_t mode = 0xff, pull = 0xff, enable = 0;
            if (!readByte(n, kRegGpioMode, &mode)) continue;
            if (!readByte(n, kRegGpioPull, &pull)) continue;
            const uint8_t pullBits = pull & 0x07;
            // A block whose 0x46 does not read back as an enable bitmap (0x00 or 0xff) is accepted rather than
            // dropped, because a wrong enable convention must not hide the key.
            (void)readByte(n, kRegGpioEnable, &enable);
            if ((mode & 0x03) != 0) continue;
            if (pullBits > 3) continue;
            if (enable != 0x00 && enable != 0xff && (enable & 0x80) == 0) continue;
            Input &in = g.gpio[g.gpioCount++];
            in = Input{};
            in.channel = n;
            in.ppid = ppid;
            printf("PICASSO_BTN gpio slave=%u peripheral=%02x channel=%u subtype=%02x mode=%02x pull=%02x enable=%02x\n",
                   ppid >> 8, ppid & 0xff, n, subtype, mode, pull, enable);
        }
    }
    printf("PICASSO_BTN inventory %u blocks, %u pon, %u key inputs\n",
           seenCount, g.ponCount, g.gpioCount);
}

// Take the released state from the hardware as it is now. Two seconds of samples decide it, so a key held while
// the kernel comes up cannot become the baseline.
void learnBaseline() {
    uint8_t value = 0;
    for (unsigned round = 0; round < 5; ++round) {
        bool settled = true;
        for (unsigned i = 0; i < g.ponCount; ++i) {
            Input &in = g.pon[i];
            if (!readByte(in.channel, kRegRtStatus, &value)) { settled = false; continue; }
            if (!in.live) { in.baseline = value; in.live = true; }
            else if (value != in.baseline) settled = false;
            in.last = value;
        }
        for (unsigned i = 0; i < g.gpioCount; ++i) {
            Input &in = g.gpio[i];
            if (!readByte(in.channel, kRegGpioStatus, &value)) { settled = false; continue; }
            if (!in.live) { in.baseline = value; in.live = true; }
            else if (value != in.baseline) settled = false;
            in.last = value;
        }
        if (settled && round >= 1) break;
        IOSleep(500);
    }
    for (unsigned i = 0; i < g.ponCount; ++i)
        printf("PICASSO_BTN pon released slave=%u peripheral=%02x byte=%02x\n",
               g.pon[i].ppid >> 8, g.pon[i].ppid & 0xff, g.pon[i].baseline);
    for (unsigned i = 0; i < g.gpioCount; ++i)
        printf("PICASSO_BTN gpio released slave=%u peripheral=%02x byte=%02x\n",
               g.gpio[i].ppid >> 8, g.gpio[i].ppid & 0xff, g.gpio[i].baseline);
}

bool gPowerDown;
bool gVolumeDownDown;
bool gVolumeUpDown;
unsigned gVolumeUpChannel;  // kept across the release so the log names the key that let go

// A level that differs from the released byte for two samples is a press, whichever way the PMIC encodes it.
void sample(Input &in, uint8_t value) {
    if (!in.live) { in.live = true; in.baseline = value; in.last = value; return; }
    if (value == in.last) { if (in.steady < kSettle) ++in.steady; }
    else { in.steady = 0; in.last = value; }
}

uint32_t maskFromInputs() {
    uint32_t mask = 0;
    if (gPowerDown) mask |= kHidPower;
    if (gVolumeDownDown) mask |= kHidVolumeDown;
    if (gVolumeUpDown) mask |= kHidVolumeUp;
    return mask;
}

void poll() {
    uint8_t value = 0;

    // The power and volume-down keys live on one PON block, but all four PMICs carry one. A bit that departs from
    // its own block's released byte counts, and the result is the union over the blocks: otherwise a PON with no
    // key wired to it would keep clearing the flags the real one sets.
    uint8_t changed = 0;
    uint16_t changedPpid = 0;
    uint8_t changedStatus = 0, changedBaseline = 0;
    for (unsigned i = 0; i < g.ponCount; ++i) {
        Input &in = g.pon[i];
        if (!readByte(in.channel, kRegRtStatus, &value)) continue;
        sample(in, value);
        if (in.steady < kSettle) continue;
        const uint8_t diff = static_cast<uint8_t>(value ^ in.baseline);
        if (diff != 0) {
            changed |= diff;
            changedPpid = in.ppid;
            changedStatus = value;
            changedBaseline = in.baseline;
        }
    }
    // PON real-time status carries one bit per key, set while the key is held. Which bit is which is not portable
    // across the trees that document it: mainline qpnp-power-on uses bit 0 KPDPWR_N / bit 1 RESIN_N, while the
    // hakios PMIC driver reads the same register as 0x80 KPDPWR / 0x40 RESIN. Both layouts are accepted rather
    // than guessed, because a wrong pick here is exactly the "the key does nothing" symptom this driver started
    // with; the raw status, the released byte and the difference are printed so the log names the layout in use.
    const bool power = (changed & 0x01) != 0 || (changed & 0x80) != 0;
    const bool down = (changed & 0x02) != 0 || (changed & 0x40) != 0;
    if (power != gPowerDown) {
        gPowerDown = power;
        ++g.presses[0];
        printf("PICASSO_BTN power %s slave=%u peripheral=%02x status=%02x released=%02x diff=%02x bit=%s\n",
               power ? "PRESSED" : "released", changedPpid >> 8, changedPpid & 0xff, changedStatus, changedBaseline,
               static_cast<unsigned>(changedStatus ^ changedBaseline),
               (changedStatus ^ changedBaseline) & 0x01 ? "0" : "7");
    }
    if (down != gVolumeDownDown) {
        gVolumeDownDown = down;
        ++g.presses[1];
        printf("PICASSO_BTN volume-down %s slave=%u peripheral=%02x status=%02x released=%02x diff=%02x bit=%s\n",
               down ? "PRESSED" : "released", changedPpid >> 8, changedPpid & 0xff, changedStatus, changedBaseline,
               static_cast<unsigned>(changedStatus ^ changedBaseline),
               (changedStatus ^ changedBaseline) & 0x02 ? "1" : "6");
    }
    // Bits above those two pairs: reported once per transition so a key wired to an unexpected pin is named
    // instead of silently ignored. The inventory line says how many key inputs each PMIC offers.
    if ((changed & 0x3cu) != 0)
        printf("PICASSO_BTN pon bit outside the key map status=%02x released=%02x diff=%02x\n", changedStatus,
               changedBaseline, static_cast<unsigned>(changedStatus ^ changedBaseline));

    // Volume up: the only key on these PMICs that is a plain pulled-up GPIO. It reads low when pressed. Only the
    // level bit may differ from the released byte: boot 12 showed one block whose status flipped 0x81 -> 0x80
    // several times a second while nothing was pressed, and any wider difference is a decode artefact, not a key.
    bool up = false;
    unsigned upChannel = 0;
    for (unsigned i = 0; i < g.gpioCount; ++i) {
        Input &in = g.gpio[i];
        if (!readByte(in.channel, kRegGpioStatus, &value)) continue;
        sample(in, value);
        const uint8_t diff = static_cast<uint8_t>(value ^ in.baseline);
        if (in.steady >= kSettle && (diff & 0xfe) == 0 && (value & 0x01) == 0 && (in.baseline & 0x01) != 0) {
            up = true;
            upChannel = in.channel;
            gVolumeUpChannel = in.channel;
        }
    }
    if (up != gVolumeUpDown) {
        gVolumeUpDown = up;
        ++g.presses[2];
        printf("PICASSO_BTN volume-up %s channel=%u\n", up ? "PRESSED" : "released",
               up ? upChannel : gVolumeUpChannel);
    }
    publish();
}

void buttonsThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "PICASSO-BTN");
    inventory();
    if (gDead) {
        printf("PICASSO_BTN stopped: the arbiter answered nothing usable\n");
        return;
    }
    learnBaseline();
    printf("PICASSO_BTN watching every %u ms pon=%u gpio=%u\n", kPollMs, g.ponCount, g.gpioCount);
    for (;;) {
        poll();
        if ((++gPolls % 3750) == 0)  // about once a minute
            printf("PICASSO_BTN alive polls=%u failures=%u presses=%u/%u/%u\n",
                   gPolls, gFailures, g.presses[0], g.presses[1], g.presses[2]);
        if (gFailures > 4000) {
            printf("PICASSO_BTN too many failed reads (%u): stopping\n", gFailures);
            return;
        }
        IOSleep(kPollMs);
    }
}

}  // namespace

extern "C" {

// The HID device (drivers/NativeHidInput) registers its report writer here. It is called from the key thread with
// the report 2 mask of the keys that are down.
void native_buttons_set_sink(void (*sink)(void *, uint32_t), void *context) {
    gSink = sink;
    gSinkContext = context;
    printf("PICASSO_BTN sink %s\n", sink ? "attached" : "detached");
}

// Called once from the touch probe's entry point, which the boot monitor reaches after the kernel is up. Returns
// immediately: SPMI reads wait on hardware, so they run on their own thread.
void native_board_buttons_start(void) {
    static bool started;
    if (started) return;
    uint32_t stage = 0;
    if (!PE_parse_boot_argn("sheng-buttons", &stage, sizeof stage) || stage < 1) {
        printf("PICASSO_BTN idle: no sheng-buttons argument\n");
        return;
    }
    if (!mapRegion(kArbCorePhys, kArbCoreSize, gCore) ||
        !mapRegion(kArbObserverPhys, kArbObserverSize, gObserver)) {
        printf("PICASSO_BTN cannot map the PMIC arbiter\n");
        return;
    }
    started = true;
    if (stage < 2) {
        inventory();
        return;
    }
    thread_t thread = nullptr;
    if (kernel_thread_start(buttonsThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else {
        started = false;
        printf("PICASSO_BTN thread failed\n");
    }
}

}  // extern "C"
