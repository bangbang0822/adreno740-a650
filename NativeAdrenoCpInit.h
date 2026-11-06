/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_CPINIT_H
#define NATIVE_ADRENO_CPINIT_H
#include "NativeAdrenoRing.h"
struct NativeAdrenoCpInitConfig {
    uint64_t sqe_iova;             /* CP_SQE_INSTR_BASE, 4 KiB aligned, nonzero */
    uint64_t pwrup_reglist_iova;   /* Power-up register list, 4-byte aligned; 0 on an A6xx core, whose a6xx_cp_init reads none */
    /* Nonzero when the caller already wrote CP_SQE_INSTR_BASE before CP_SQE_CNTL, as a6xx hw_init does: the base is
     * then read back and compared instead of rewritten after the SQE started. */
    uint32_t sqe_base_programmed;
    /* 0 or 7: the pinned a7xx_cp_init sequence, which takes a power-up register list. 6: a6xx_cp_init(), one CP_ME_INIT of
     * eight words with no CP_THREAD_CONTROL and no list. */
    uint32_t generation;
};
/* One-shot A7XX command-processor start sequence, not a GPU power or reset
 * driver. The caller must have completed GMU boot, powered GX/core clocks,
 * released the CP reset, loaded and hash-verified the SQE microcode into the
 * real IOVA given here, and hold exclusive MMIO. This programs
 * CP_SQE_INSTR_BASE, then submits the pinned a7xx_cp_init sequence directly
 * through the ring: CP_THREAD_CONTROL with concurrent binning disabled and
 * CP_ME_INIT with the Linux mask, hardware-context count, operation mode and
 * power-up register list. Completion is the ring timestamp fence, so a hung
 * CP is reported as a timeout instead of a fabricated success. A second call
 * cannot re-initialize the same object. */
class NativeAdrenoCpInit final {
    bool done_=false,poisoned_=false;
    IOReturn poison();
public:
    NativeAdrenoCpInit()=default;
    ~NativeAdrenoCpInit()=default;
    NativeAdrenoCpInit(const NativeAdrenoCpInit &)=delete;
    NativeAdrenoCpInit &operator=(const NativeAdrenoCpInit &)=delete;
    /* fenced false: the words go without a timestamp fence (*fence 0) and the caller waits with NativeAdrenoRing::idle, as a7xx_cp_init
     * does with a6xx_idle; the restart after a hang uses it so the replayed submissions keep their fence numbers. */
    IOReturn run(const NativeAdrenoRingIo *,NativeAdrenoRing *,
        const NativeAdrenoCpInitConfig *,uint32_t *fence,bool fenced=true);
    bool completed() const {return done_;}
    /* A GPU restart puts the CP back at reset, so CP_ME_INIT must be sent again: a6xx_recover() runs
     * a6xx_hw_init() and a6xx_cp_init() exactly as the first bring-up did. done_ was one-shot, which
     * made every restart fail at CP_ME_INIT with kIOReturnNotReady ([gpu restart step] 3). */
    void reopen() {done_=false;poisoned_=false;}
    bool poisoned() const {return poisoned_;}
    static uint32_t meInitMask();
};
#endif
