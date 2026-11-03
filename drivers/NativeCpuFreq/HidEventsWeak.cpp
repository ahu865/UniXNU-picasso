// Weak fallback for the optional HID-boost counter.
//
// CpuFreqProbe.cpp references native_hid_input_events as a weak import, but the
// only definition in the tree lives in
// drivers/NativeMobileDisplay/boot-progress/PicassoBootProgressStubs.c, which is
// not built into any kernel archive. Without a definition in the final image the
// kext link fails with:
//
//   Undefined symbols for architecture arm64:
//     "_native_hid_input_events", referenced from:
//         governorThread(...) in libPicassoCpuFreq.a(CpuFreqProbe.o)
//
// Defining it weak here keeps the governor's "is the HID producer present?"
// check meaningful: the address is non-null, the counters stay zero, and the
// touch-boost path simply never triggers until a real HID driver publishes.
#include <stdint.h>

extern "C" {
__attribute__((weak, visibility("default")))
volatile uint32_t native_hid_input_events[8];
}
