/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoSubmit.h"
#include "AdrenoPM4.h"
#include <IOKit/IOLib.h>

/* CP_INDIRECT_BUFFER is opcode 0x3f with a 3-word payload: 64-bit GPU address
 * and IB_SIZE in dwords (bits 0..19), exactly as the pinned adreno_pm4.xml
 * domain states and Linux msm_gem_submit.c converts the userspace byte size. */
enum : uint32_t {SubmitIndirectBuffer=0x3f,SubmitIbWords=3,
    SubmitMaxBuffers=(uint32_t)NativeAdrenoSubmitMaxBuffers,SubmitMaxIbBytes=0xfffffu*4u,
    SubmitAuditBytes=256u*1024u};   /* IB bytes the write-combine audit compares per submission */
extern "C" {volatile uint32_t native_adreno_coherence[8];}
/* IBs per submission: [0] submissions, [1] IBs, [2] the most in one, [3] submissions with more than 16 (one frame was several before). */
extern "C" {volatile uint32_t native_adreno_submit_ibs[4];}
/* Where a submission's time goes, in mach_absolute_time ticks summed over the successful ones: [0] the buffers' validation,
 * [1] the sweep of completed jobs and the job references, [2] the write-combine re-assertion and audit, [3] the BO publication
 * (cache maintenance), [4] the coherence sample after it, [5] the IB record and the ring write; [6] the count. The board log
 * divides each by the count, so a submission that costs the render thread milliseconds shows which part does. */
extern "C" {volatile uint64_t native_adreno_submit_phase[7];}
extern "C" uint64_t mach_absolute_time(void);
/* The submission (1-based count) at which the write-combine audit found a dirty IB line or failed; 0 = never. */
extern "C" {volatile uint32_t native_adreno_wc_distrusted;}
/* The write-combine audit before each submission while trusted: [0] submissions audited, [1] IB lines compared, [2] dirty lines,
 * [3] stale clean lines, [4] the last audit failure (any failure also ends the trust), [5] pages found back at the default attribute
 * and write-combined again. */
extern "C" {volatile uint32_t native_adreno_wc_audit[6];}
/* Fence waits: [0] calls, [1] timeouts, [2] the fence of the last timed-out wait, [3] the fence value the GPU had written then, [4] the newest fence handed out,
 * [5] the newest completed, [6] waits for a fence no job holds (observeFence path), [7] the last fence refused as never submitted. */
extern "C" {volatile uint32_t native_adreno_fence_wait[8];}
/* The CP's state once a fence stopped advancing (NativeAdrenoRing::stallState): [0] the fence, [1] the timed-out waits of it so far, then
 * the ring's words. Taken at the 10th and again at the 1000th timed-out wait of the fence that is the next to complete. native_adreno_stall_mem
 * holds, from the same moment, the SMMU context's FSR, FAR (lo, hi) and FSYNR0, then for the BR IB1, the BV IB1 and the BV IB2 17 words: the
 * dword index in the IB where the 16 words that follow start (the IB1s are found from the ring packet before each read pointer, so the index
 * counts from the IB's start; for the BV IB2 only the fetch address is known and the index is 0xffff0000 + words before it), then the words.
 * The last 4: the map handle that was next when the stalled job was submitted, then the handles of the mappings under the BR IB1, the BV IB1
 * and the BV IB2 (0: unmapped). A handle at or above the first was mapped after the submission: its memory was freed and reused under the GPU. */
extern "C" {volatile uint32_t native_adreno_stall[2+NativeAdrenoRingStallWords];volatile uint32_t native_adreno_stall_mem[4+3*17+4];}
/* 16 words of the IB at start, from 12 dwords before the dword the thread has reached (size minus what remains). */
static void peekIb(const NativeAdrenoVm *vm,uint64_t start,uint32_t size,uint32_t remaining,volatile uint32_t *slot)
{
    uint32_t words[16]={};
    const uint32_t reached=remaining<=size?size-remaining:0,first=reached>12?reached-12:0;
    if(vm->peekWordsUncached(start+4ull*first,words,16))for(unsigned i=0;i<16;++i)words[i]=0xdeadbeef;
    slot[0]=first;
    for(unsigned i=0;i<16;++i)slot[1+i]=words[i];
}
/* 16 words around a fetch address whose IB start is unknown: from 12 words before it, or nearer when that crosses into no buffer. */
static void peekFetch(const NativeAdrenoVm *vm,uint64_t fetch,volatile uint32_t *slot)
{
    uint32_t words[16]={},before=12;
    while(before && vm->peekWordsUncached(fetch-4*before,words,16))before-=4;
    if(!before && vm->peekWordsUncached(fetch,words,16))for(unsigned i=0;i<16;++i)words[i]=0xdeadbeef;
    slot[0]=0xffff0000u+before;
    for(unsigned i=0;i<16;++i)slot[1+i]=words[i];
}
static uint32_t stall_fence,stall_timeouts,stall_stage;   /* stage: 0 none yet, 1 write pointer given again, 2 translations dropped, 3 bus drained */
static uint64_t stall_began_us,stall_last_us;   /* the fence's first and latest timed-out wait */
/* How long fences stayed pending from their first timed-out wait to their last: [i] under 1 << i ms for i < 7, [7] 64 ms or more. */
extern "C" {volatile uint32_t native_adreno_fence_pending[8];}
/* What the stall handler tried: [0] attempts, [1] the fence of the last attempt, [2] attempts after which that fence completed, [3] the
 * timed-out waits that fence had seen when it did. */
/* [4] bus drains, [5] client halt acknowledge, [6] arbiter halt acknowledge, [7] the fence of the last drain, [8] drains after which that fence completed */
extern "C" {volatile uint32_t native_adreno_unstick[15];}
/* The IBs of the last submissions: [slot] fence, iova low/high, bytes, then the first 28 dwords of the first IB (read as the kernel maps it). */
/* [0] fence, [1..2] the submission's iova, [3] its bytes, [4..5] the MAPPING's iova the handle resolves to (the
 * base the buffer sits in, so a nonzero [1] minus [4] says the IB starts inside its mapping), [6..33] the head. */
/* The head is read twice: [6..101] through peekWords (the board's dramRead32 hook, a fresh uncached
 * mapping per page) and [102..197] through peekWordsUncached (the kernel CPU view with cache
 * maintenance first). The two branches in NativeAdrenoVm are different code paths, and a client IB
 * that reads as zeros through one and real packets through the other says the reader is wrong, not
 * the GPU. [198] the two statuses, [199] how many words differ. */
extern "C" {volatile uint32_t native_adreno_ib_log[4][6+96+96+2+16+80];volatile uint32_t native_adreno_ib_log_count;}
extern "C" volatile uint32_t native_adreno_table_switches[8];
/* CP_WAIT_FOR_IDLE packets the board removed from client IBs before the CP saw them. */
extern "C" volatile uint32_t native_adreno_wfi_dropped=0;   /* a defining initialiser: bare extern "C" is only a declaration */
/* Why the drop found nothing: [0] buffers looked at, [1] no mapping for the iova, [2] mapping exists but no
 * CPU view, [3] buffers scanned, [4] the IOReturn the last refusal returned. */
extern "C" volatile uint32_t native_adreno_wfi_try[5]={};
extern "C" boolean_t PE_parse_boot_argn(const char *,void *,int);
/* Where the last recover() stopped: [0] the step (1 fence read, 2 restart hook, 3 table write, 4 nop encode, 5 replay vm, 6 replay submit), [1] the IOReturn or value there, [2] fence, [3] last submitted. */
extern "C" {volatile uint32_t native_adreno_recover_trace[4];}
/* The jobs' timeline (the newest native_adreno_tl_cap retired, allocated at attach): [0] fence | indirect buffers << 32 | ring status << 58, [1] submit
 * time (us), [2] the CP's always-on counter before the job's buffers, [3] after them (19.2 MHz ticks; 0 when not stamped), [4] GPU level at submit |
 * the submitting pid << 32, [5] when the driver saw the job retired (us, the submission clock). native_adreno_tl_count is the total. */
extern "C" {volatile uint64_t (*native_adreno_tl)[6]=nullptr;volatile uint32_t native_adreno_tl_cap=0;volatile uint32_t native_adreno_tl_count;}
/* The stall handler's actions (the newest 32): [0] time (us), [1] fence, [2] event (1 write pointer given again and translations dropped, 3 bus drain,
 * 4 snapshot, 0 the stall ended) | timed-out waits << 8, [3] time the fence had been pending (us), [4] read and write pointer before | after << 32 for
 * events 1 and 3, [5] client queue, [6] the job's first buffer handle | its buffer count << 48, [7] the sequence number. */
extern "C" {volatile uint64_t native_adreno_stall_log[32][8];volatile uint32_t native_adreno_stall_log_count;}
extern "C" volatile uint32_t native_gpu_dcvs[15] __attribute__((weak_import));
IOReturn NativeAdrenoSubmit::poison()
{
    poisoned_=true;return kIOReturnIOError;
}
NativeAdrenoSubmit::Job *NativeAdrenoSubmit::find(uint32_t fence)
{
    if(fence)for(auto &job:jobs_)if(job.active && job.fence==fence)return &job;
    return nullptr;
}
NativeAdrenoSubmit::Job *NativeAdrenoSubmit::free()
{
    for(auto &job:jobs_)if(!job.active)return &job;
    Job **items=IONewData(Job *,jobs_.count+1);
    Job *job=items?new Job:nullptr;
    if(!job) {if(items)IODeleteData(items,Job *,jobs_.count+1);return nullptr;}
    for(size_t i=0;i<jobs_.count;++i)items[i]=jobs_.items[i];
    items[jobs_.count]=job;
    if(jobs_.items)IODeleteData(jobs_.items,Job *,jobs_.count);
    jobs_.items=items;++jobs_.count;
    return job;
}
IOReturn NativeAdrenoSubmit::attach(NativeAdrenoVm *vm,NativeAdrenoRing *ring)
{
    if(attached_ || poisoned_ || vm_ || ring_)return kIOReturnBusy;
    if(!vm || !ring || vm->poisoned() || ring->poisoned())return kIOReturnBadArgument;
    vm_=vm;ring_=ring;attached_=true;stats_={};table_vm_=vm;
    if(!native_adreno_tl) {
        const uint32_t cap=16384;
        volatile uint64_t (*memory)[6]=(volatile uint64_t (*)[6])IOMallocZero(sizeof(uint64_t)*6*cap);
        if(memory) {native_adreno_tl=memory;native_adreno_tl_cap=cap;}
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::setTableSwitching(unsigned mode,IOReturn (*write)(void *,uint64_t),void *cookie)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    if(mode>2 || (mode && !write) || outstanding())return kIOReturnBadArgument;
    table_mode_=mode;write_table_=write;table_cookie_=cookie;table_vm_=vm_;
    if(mode==2)ring_->tableInstalled(vm_->ttbr0());
    return kIOReturnSuccess;
}
/* Makes vm the table the GPU runs for the next submission. Mode 1: once every earlier job has finished, the CPU writes TTBR0 (the bank
 * caches translations per ASID, so the switch needs no TLB operation). Mode 2: the ring's next prologue carries the switch, which the CP
 * runs after every earlier submission. */
IOReturn NativeAdrenoSubmit::useVm(NativeAdrenoVm *vm,const NativeAdrenoRingClock *clock)
{
    if(!table_mode_ || vm==table_vm_)return kIOReturnSuccess;
    if(table_mode_==2) {ring_->useTable(vm->ttbr0());table_vm_=vm;return kIOReturnSuccess;}
    if(!clock)return kIOReturnBadArgument;
    IOReturn status=waitAll(clock,2000000);
    if(status)return status;
    status=write_table_(table_cookie_,vm->ttbr0());
    if(status)return poison();
    native_adreno_table_switches[1]=native_adreno_table_switches[1]+1;
    table_vm_=vm;return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::forgetVm(NativeAdrenoVm *vm,const NativeAdrenoRingClock *clock,uint32_t timeout_us)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    if(!vm || vm==vm_)return kIOReturnBadArgument;
    IOReturn status=waitAll(clock,timeout_us);
    if(status)return status;
    if(table_vm_!=vm && !(table_mode_==2 && ring_->table()==vm->ttbr0()))return kIOReturnSuccess;
    /* The GPU is idle and every queued switch has run: the CPU puts the default table back. */
    status=write_table_?write_table_(table_cookie_,vm_->ttbr0()):kIOReturnNotReady;
    if(status)return poison();
    native_adreno_table_switches[1]=native_adreno_table_switches[1]+1;
    if(table_mode_==2)ring_->tableInstalled(vm_->ttbr0());
    table_vm_=vm_;return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::release(Job &job)
{
    IOReturn status=kIOReturnSuccess;
    NativeAdrenoVm *vm=job.vm?job.vm:vm_;
    for(size_t i=0;i<job.count;++i) {
        IOReturn retired=vm->retireJobAfterCompletion(job.handles[i]);
        if(retired)status=retired;else ++stats_.references_retired;
    }
    if(status)return poison();
    if(vm->acquireImported())return poison();
    {
        uint64_t start=0,end=0;
        const uint32_t stamped=ring_?ring_->timeline(job.fence,&start,&end):0;
        if(native_adreno_tl_cap) {
            const uint32_t n=native_adreno_tl_count;volatile uint64_t *slot=native_adreno_tl[n&(native_adreno_tl_cap-1)];
            slot[0]=(uint64_t)job.fence|(uint64_t)job.count<<32|(uint64_t)stamped<<58;slot[1]=job.submit_us;
            slot[2]=start;slot[3]=end;slot[4]=(uint64_t)job.level|(uint64_t)(uint32_t)job.owner<<32;
            slot[5]=clock_now_?clock_now_(clock_cookie_):0;
            native_adreno_tl_count=n+1;
        }
    }
    job={};return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::detach()
{
    if(poisoned_ || outstanding())return kIOReturnBusy;
    if(!attached_)return kIOReturnNotReady;
    attached_=false;vm_=nullptr;ring_=nullptr;return kIOReturnSuccess;
}
bool NativeAdrenoSubmit::queueOutstanding(uint64_t queue) const
{
    return rangeOutstanding(queue,1);
}
bool NativeAdrenoSubmit::rangeOutstanding(uint64_t first,uint64_t count) const
{
    for(const auto &job:jobs_)if(job.active && job.queue-first<count)return true;
    return false;
}
size_t NativeAdrenoSubmit::outstanding() const
{
    size_t count=0;for(const auto &job:jobs_)if(job.active)++count;return count;
}
IOReturn NativeAdrenoSubmit::submit(const NativeAdrenoSubmitBuffer *buffers,size_t count,
    uint64_t queue,uint32_t *fence)
{
    return submit(nullptr,buffers,count,queue,fence,nullptr);
}
IOReturn NativeAdrenoSubmit::submit(NativeAdrenoVm *vm,const NativeAdrenoSubmitBuffer *buffers,size_t count,
    uint64_t queue,uint32_t *fence,const NativeAdrenoRingClock *clock)
{
    if(!fence)return kIOReturnBadArgument;*fence=0;
    if(!vm)vm=vm_;
    NativeAdrenoVm *const vm_=vm;   /* this submission's VM: every vm_ below is the job's own */
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    /* msm_ioctl_gem_submit refuses a VM a hang made unusable. */
    if(vm->unusable())return kIOReturnNotPermitted;
    if(!buffers || !count || count>SubmitMaxBuffers)return kIOReturnBadArgument;
    native_adreno_submit_ibs[0]=native_adreno_submit_ibs[0]+1;native_adreno_submit_ibs[1]=native_adreno_submit_ibs[1]+(uint32_t)count;
    if(count>native_adreno_submit_ibs[2])native_adreno_submit_ibs[2]=(uint32_t)count;
    if(count>16)native_adreno_submit_ibs[3]=native_adreno_submit_ibs[3]+1;
    /* Phase 0 opens at the first buffer's validation and phase 1 at the sweep below; each later mark closes the section before it. */
    uint64_t phase[7];phase[0]=mach_absolute_time();
    /* Every buffer is validated against the live VM before any reference. */
    uint64_t *handles=scratch_handles_;size_t handle_count=0;
    uint32_t *words=scratch_words_;size_t word_count=0;
    static_assert(sizeof(scratch_words_)/sizeof(scratch_words_[0])>=SubmitMaxBuffers*(1+SubmitIbWords),"ring words");
    for(size_t i=0;i<count;++i) {
        const NativeAdrenoSubmitBuffer &buffer=buffers[i];
        if(!buffer.handle || !buffer.bytes || (buffer.bytes&3) || buffer.bytes>SubmitMaxIbBytes)
            return kIOReturnBadArgument;
        uint64_t iova=0;size_t bytes=0;unsigned permissions=0;
        IOReturn status=vm_->describe(buffer.handle,&iova,&bytes,&permissions);
        if(status)return status;
        if(!(permissions&ArmSmmu4KRead) || buffer.iova<iova ||
            buffer.iova-iova>bytes || buffer.bytes>bytes-(buffer.iova-iova))return kIOReturnBadArgument;
        uint32_t header=0;
        if(AdrenoPM4Type7(SubmitIndirectBuffer,SubmitIbWords,&header)!=AdrenoPM4Success)
            return poison();
        words[word_count++]=header;
        words[word_count++]=(uint32_t)buffer.iova;
        words[word_count++]=(uint32_t)(buffer.iova>>32);
        words[word_count++]=(uint32_t)(buffer.bytes/4);
        bool known=false;
        for(size_t j=0;j<handle_count;++j)if(handles[j]==buffer.handle) {known=true;break;}
        if(!known)handles[handle_count++]=buffer.handle;
    }
    phase[1]=mach_absolute_time();
    IOReturn swept=retireCompleted();
    if(swept)return swept;
    Job *job=free();
    if(!job)return kIOReturnNoMemory;
    for(size_t i=0;i<handle_count;++i) {
        IOReturn status=vm_->holdJob(handles[i]);
        if(status) {
            while(i) {vm_->retireJobAfterCompletion(handles[--i]);++stats_.references_retired;}
            return status;
        }
        ++stats_.references_held;
    }
    phase[2]=mach_absolute_time();
    uint32_t value=0;
    /* int-070: the first IB word as the kernel's cacheable alias and DRAM see it before any maintenance, then DRAM after the CPU
     * caches over every mapped BO are cleaned and invalidated. The board shows the last submission's record on a stall. */
    uint32_t cached=~0u,before=~0u,ignored=0,after=~0u;
    /* Write-combined stores this CPU made before the call (the client's command words) complete before anything below. */
    __asm__ volatile("dsb sy":::"memory");
    /* int-098/099: with write-combined user BOs and no maintenance the CP stopped at the end of the first IB, as in int-065..069 when an
     * IB sat in a CPU cache. While write-combined BOs are trusted, every line of this submission's IBs is compared as a CPU cache and
     * DRAM hold it (bounded). A dirty line means a client view still caches its BOs: maintain every BO from now on, this submission
     * included (the audit already wrote the IB lines back; the rest of the BOs may hold more). */
    if(vm_->writeCombinedTrusted()) {
        /* int-100: the dirty IB page was back at XNU's default (cacheable) attribute. Write-combine every such page again first; the
         * pmap writes its lines back on the way, so the audit below should then find none. */
        uint32_t reset=0;
        IOReturn audited=vm_->reassertWriteCombined(&reset);
        native_adreno_wc_audit[5]=native_adreno_wc_audit[5]+reset;
        uint32_t lines=0,dirty=0,stale=0;size_t budget=SubmitAuditBytes;
        /* The re-assertion above is the correction and runs every time; the audit only detects. It found no dirty line in int-141's
         * 133798 nor in i566's 2765132 (1358 audits of every eighth submission, about 2 ms each under the publish lock, so every eighth
         * frame of every client waited), so only the first 16 submissions are compared now. */
        ++audit_tick_;
        const bool audit_now=audit_tick_<=16;
        if(!audit_now)budget=0;
        for(size_t i=0;i<count && budget && !audited;++i) {   /* a failed re-assertion skips the audit and ends the trust */
            const size_t bytes=buffers[i].bytes<budget?(size_t)buffers[i].bytes:budget;
            audited=vm_->auditRange(buffers[i].iova,bytes,&lines,&dirty,&stale);budget-=bytes;
        }
        if(audit_now && audited!=kIOReturnUnsupported) {   /* no board hook (host): nothing to compare */
            native_adreno_wc_audit[0]=native_adreno_wc_audit[0]+1;
            native_adreno_wc_audit[1]=native_adreno_wc_audit[1]+lines;
            native_adreno_wc_audit[2]=native_adreno_wc_audit[2]+dirty;
            native_adreno_wc_audit[3]=native_adreno_wc_audit[3]+stale;
            if(audited)native_adreno_wc_audit[4]=audited;
        }
        if(dirty || (audited && audited!=kIOReturnUnsupported)) {
            vm_->distrustWriteCombined();
            native_adreno_wc_distrusted=native_adreno_coherence[0]+1;
        }
    }
    /* The coherence sample reads the first IB's word through a fresh uncached mapping before and after the publication: every 64th
     * submission only (two such mappings per submission were CPU time on every frame). */
    /* The board's CP stops on CP_WAIT_FOR_IDLE (see NativeAdrenoVm::dropWaitForIdle): take it out of the
     * IB the CP is about to read. Counted so the log can say how often it happens. */
    /* Off by default: the scrub writes a client BO's pages through a fresh write-combined mapping before
     * every submission, and from fix41 on the self test stopped completing at its first submission
     * (fix40 read rounds 4 4, fix41 read -1 -1). One round with it off says whether that write is the cause.
     * pipa-gpu-wfi=1 turns it back on. */
    unsigned scrub = 0;
    PE_parse_boot_argn("pipa-gpu-wfi", &scrub, sizeof scrub);
    for(size_t i=0;i<count && scrub;++i) {
        uint32_t patched=0;
        native_adreno_wfi_try[0]=native_adreno_wfi_try[0]+1;
        const IOReturn dropped=vm->dropWaitForIdle(buffers[i].iova,buffers[i].bytes,&patched);   /* the job s VM, not the kernel s */
        native_adreno_wfi_try[3]=native_adreno_wfi_try[3]+1;
        if(dropped==kIOReturnSuccess) native_adreno_wfi_dropped=native_adreno_wfi_dropped+patched;
        else if(dropped==kIOReturnBadArgument) native_adreno_wfi_try[1]=native_adreno_wfi_try[1]+1;   /* no mapping for the iova */
        else {native_adreno_wfi_try[2]=native_adreno_wfi_try[2]+1;native_adreno_wfi_try[4]=uint32_t(dropped);}   /* no spine: uncached view or leaf */
    }
    phase[3]=mach_absolute_time();
    static uint32_t coherence_turn;
    const bool sample=!(coherence_turn++&63);
    if(sample)(void)vm_->sampleWord(buffers[0].iova,&cached,&before);
    IOReturn status=vm_->publishMapped();
    if(!status && sample) {
        (void)vm_->sampleWord(buffers[0].iova,&ignored,&after);
        native_adreno_coherence[0]=native_adreno_coherence[0]+1;
        native_adreno_coherence[1]=cached;native_adreno_coherence[2]=before;native_adreno_coherence[3]=after;
        if(cached!=before || before!=after) {
            if(!native_adreno_coherence[4]) {
                native_adreno_coherence[5]=cached;native_adreno_coherence[6]=before;native_adreno_coherence[7]=after;
            }
            native_adreno_coherence[4]=native_adreno_coherence[4]+1;
        }
    }
    phase[4]=mach_absolute_time();
    /* No TLB invalidation before a submission: a map fills only invalid leaves (no TLB holds an invalid descriptor), an unmap drops
     * its own pages before it returns, and the tables above the leaves never change. The TLBIALL here emptied the GPU's TLB on every
     * submission (10944 in i568), so each frame began with page walks. A stalled fence still invalidates (below). */
    if(status) {
        for(size_t i=0;i<handle_count;++i) {vm_->retireJobAfterCompletion(handles[i]);++stats_.references_retired;}
        return status;
    }
    {
        const uint32_t slot=native_adreno_ib_log_count&3;uint32_t head[96],view[96];
        const IOReturn hookStatus=vm_->peekWords(buffers[0].iova,head,96);
        const IOReturn viewStatus=vm_->peekWordsUncached(buffers[0].iova,view,96);
        native_adreno_ib_log[slot][0]=ring_->lastSubmitted()+1;native_adreno_ib_log[slot][1]=(uint32_t)buffers[0].iova;
        native_adreno_ib_log[slot][2]=(uint32_t)(buffers[0].iova>>32);native_adreno_ib_log[slot][3]=(uint32_t)buffers[0].bytes;
        {   /* The mapping the first buffer's handle resolves to: its base, so [1..2] - [4..5] is the buffer's offset inside it. */
            uint64_t base=0;size_t mapped=0;unsigned permissions=0;
            (void)vm_->describe(buffers[0].handle,&base,&mapped,&permissions);
            native_adreno_ib_log[slot][4]=(uint32_t)base;native_adreno_ib_log[slot][5]=(uint32_t)(base>>32);
        }
        for(unsigned i=0;i<96;++i)native_adreno_ib_log[slot][6+i]=head[i];
        for(unsigned i=0;i<96;++i)native_adreno_ib_log[slot][102+i]=view[i];
        /* The tail: the head is 96 of up to 184 dwords, and the packet that stops the CP is past it.
         * The last 16 dwords of the buffer, read the same way, say whether the fence write is even
         * present at the end of the IB. */
        {uint32_t tail[16]={};const uint32_t bytes=buffers[0].bytes;
         if(bytes>64) (void)vm_->peekWordsUncached(buffers[0].iova+bytes-64,tail,16);
         for(unsigned i=0;i<16;++i)native_adreno_ib_log[slot][200+i]=tail[i];}
        /* The middle: the head is 96 of 184 dwords, so words 96..183 were never seen, and a BO that
         * reads as real packets in its first 96 and zeros after 96 says the client's own write stopped
         * there rather than that the CP failed. */
        {uint32_t mid[80]={};const uint32_t bytes=buffers[0].bytes;
         if(bytes>384)(void)vm_->peekWordsUncached(buffers[0].iova+384,mid,80);
         for(unsigned i=0;i<80;++i)native_adreno_ib_log[slot][216+i]=mid[i];}
        native_adreno_ib_log[slot][198]=uint32_t(hookStatus)|(uint32_t(viewStatus)<<16);
        {unsigned differ=0;for(unsigned i=0;i<96;++i)if(head[i]!=view[i])++differ;native_adreno_ib_log[slot][199]=differ;}
        native_adreno_ib_log_count=native_adreno_ib_log_count+1;
    }
    phase[5]=mach_absolute_time();
    status=useVm(vm,clock);
    if(status) {
        for(size_t i=0;i<handle_count;++i) {vm_->retireJobAfterCompletion(handles[i]);++stats_.references_retired;}
        return status;
    }
    status=ring_->submitFramed(words,word_count,&value);
    if(status) {
        /* A poisoned ring may already have consumed the WPTR write, so the
         * referenced buffers cannot be retired here. */
        if(ring_->poisoned()) {
            *job=Job{};++stats_.poisoned_jobs;
            for(size_t i=0;i<handle_count;++i)job->handles[i]=handles[i];
            job->count=handle_count;job->queue=queue;job->active=true;job->vm_next=vm_->nextHandle();job->vm=vm;
            return poison();
        }
        while(handle_count) {vm_->retireJobAfterCompletion(handles[--handle_count]);++stats_.references_retired;}
        /* No room in the ring yet: the caller waits for an earlier job and submits again. */
        return status==kIOReturnNoResources?kIOReturnBusy:status;
    }
    for(size_t i=0;i<handle_count;++i)job->handles[i]=handles[i];
    job->count=handle_count;job->fence=value;job->queue=queue;job->active=true;job->vm_next=vm_->nextHandle();job->vm=vm;
    job->context=ring_->context();job->word_count=word_count;
    job->submit_us=clock && clock->now_us?clock->now_us(clock->cookie):0;
    if(clock && clock->now_us) {clock_now_=clock->now_us;clock_cookie_=clock->cookie;}
    job->level=&native_gpu_dcvs?native_gpu_dcvs[1]:0;
    job->owner=next_owner_;next_owner_=0;
    for(size_t i=0;i<word_count;++i)job->words[i]=words[i];
    ++stats_.submissions;
    phase[6]=mach_absolute_time();
    for(unsigned i=0;i<6;++i)native_adreno_submit_phase[i]=native_adreno_submit_phase[i]+(phase[i+1]-phase[i]);
    native_adreno_submit_phase[6]=native_adreno_submit_phase[6]+1;
    if(outstanding()>stats_.peak_jobs)stats_.peak_jobs=outstanding();
    *fence=value;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::observeFence(uint32_t fence,const NativeAdrenoRingClock *clock,
    uint32_t timeout_us)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    if(!fence)return kIOReturnBadArgument;
    const NativeAdrenoRingFence result=ring_->observeFence(fence,clock,timeout_us,nullptr);
    if(result==NativeAdrenoRingFencePoisoned)return poison();
    if(result==NativeAdrenoRingFenceArgument)return kIOReturnBadArgument;
    if(result==NativeAdrenoRingFenceTimeout)return kIOReturnTimeout;
    return kIOReturnSuccess;
}
/* A stage runs once per stalled fence: when the stage before it has run and the fence has been pending long enough. */
static bool stage_due(uint32_t stage,uint32_t before,uint64_t pending,uint64_t after_us)
{
    return stage==before && pending>=after_us;
}
/* The CP snapshot is taken at 50 ms and at 1 s of one stall. */
static uint32_t stall_snapshots;
static bool snapshot_due(uint32_t *taken,uint64_t pending)
{
    const uint32_t want=pending>=1000000?2:pending>=50000?1:0;
    if(want<=*taken)return false;
    *taken=want;return true;
}
void NativeAdrenoSubmit::noteStall(uint32_t observed,const NativeAdrenoRingClock *clock)
{
    const uint32_t next=observed+1;   /* the fence the CP is working on */
    const uint64_t now=clock && clock->now_us?clock->now_us(clock->cookie):0;
    auto log=[&](uint32_t fence,uint32_t event,uint64_t pending,uint32_t before_r,uint32_t before_w) {
        const uint32_t n=native_adreno_stall_log_count;volatile uint64_t *slot=native_adreno_stall_log[n&31];
        const Job *owner=find(fence);
        slot[0]=now;slot[1]=fence;slot[2]=(uint64_t)event|(uint64_t)stall_timeouts<<8;slot[3]=pending;
        slot[4]=(uint64_t)before_r|(uint64_t)before_w<<16|(uint64_t)ring_->readPointer()<<32|(uint64_t)(ring_->writePointer()&0xffff)<<48;
        slot[5]=owner?owner->queue:0;slot[6]=owner?(owner->handles[0]&0xffffffffffffull)|(uint64_t)owner->count<<48:0;slot[7]=n;
        native_adreno_stall_log_count=n+1;
    };
    if(next!=stall_fence) {
        if(stall_timeouts && now) {
            const uint64_t ms=(stall_last_us-stall_began_us)/1000;
            uint32_t bucket=0;while(bucket<7 && ms>=(1ull<<bucket))++bucket;
            native_adreno_fence_pending[bucket]=native_adreno_fence_pending[bucket]+1;
            if(stall_last_us-stall_began_us>=1000)log(stall_fence,0,stall_last_us-stall_began_us,0,0);   /* shorter ends are ordinary jobs */
        }
        stall_began_us=now;stall_snapshots=0;
        /* The CP moved on: what the stall that just ended took to clear. [9] first-stage recoveries, [10] second, [11] third, [12] timeouts of the longest. */
        if(stall_stage && stall_timeouts) {
            native_adreno_unstick[8+stall_stage]=native_adreno_unstick[8+stall_stage]+1;
            if(stall_timeouts>native_adreno_unstick[12])native_adreno_unstick[12]=stall_timeouts;
        }
        stall_fence=next;stall_timeouts=0;stall_stage=0;
    }
    stall_last_us=now;
    const uint64_t pending=now-stall_began_us;
    /* A fence that stops while the CP sits idle with work in the ring (int-317 hung with RBBM idle, RPTR short of WPTR, both threads at the same
     * cache clean) is nearly always a write pointer the CP never took: giving it again after a few waits ends it in a couple of milliseconds
     * (i343: 206 of 213 stalls ended at the second stage, 60 to 85 ms each, most of it the 100 waits before the write). After 10 waits now:
     * i469 stalled 143 times in about 200 s, each a visible hitch. The same value written again changes nothing when the CP is busy. */
    /* After 10 waits the translations are dropped and the write pointer is given again. i494/i495 measured what ends these: a job lasts about
     * 5 ms against 1 ms waits, so most "stalls" are ordinary jobs still running; the write pointer is never dropped (fenced writes saw no
     * drop) and invalidating at map time changed nothing. i553: the handler ran on 6712 of 6911 jobs. It now waits for 50 ms without
     * progress, measured in time (a timed-out wait is one poll, 100 us or less), and so does the snapshot below. */
    if(now && stage_due(stall_stage,0,pending,50000)) {
        native_adreno_unstick[0]=native_adreno_unstick[0]+1;native_adreno_unstick[1]=stall_fence;stall_stage=1;
        const uint32_t r0=ring_->readPointer(),w0=ring_->writePointer()&0xffff;
        (void)vm_->invalidateTlb();(void)ring_->kick();
        log(stall_fence,1,pending,r0&0xffff,w0);
    }
    /* Still stopped well after that: the way a6xx drains the bus before a reset, which lets a write that was never acknowledged end. */
    if(now && stage_due(stall_stage,1,pending,200000)) {
        uint32_t ack[2]={};
        native_adreno_unstick[4]=native_adreno_unstick[4]+1;native_adreno_unstick[7]=stall_fence;stall_stage=3;
        const uint32_t r0=ring_->readPointer(),w0=ring_->writePointer()&0xffff;
        (void)ring_->drainBus(ack);
        native_adreno_unstick[5]=ack[0];native_adreno_unstick[6]=ack[1];
        log(stall_fence,3,pending,r0&0xffff,w0);
    }
    ++stall_timeouts;
    if(!now || !snapshot_due(&stall_snapshots,pending))return;
    uint32_t words[NativeAdrenoRingStallWords]={},fault[4]={};
    if(ring_->stallState(words))return;
    log(next,4,pending,words[8]&0xffff,words[9]&0xffff);
    (void)vm_->faultState(fault);
    for(uint32_t i=0;i<4;++i)native_adreno_stall_mem[i]=fault[i];
    uint64_t ib=0;uint32_t size=0;
    /* The IBs are the stalled job's: read them through its address space (the kernel's maps none of a client's BOs). */
    const Job *job=find(next);
    const NativeAdrenoVm *space=job && job->vm?job->vm:vm_;
    if(!ring_->ibBefore(words[8],&ib,&size))peekIb(space,ib,size,words[12],&native_adreno_stall_mem[4]);
    else peekFetch(space,(uint64_t)words[11]<<32|words[10],&native_adreno_stall_mem[4]);
    if(!ring_->ibBefore(words[16],&ib,&size))peekIb(space,ib,size,words[18],&native_adreno_stall_mem[4+17]);
    else peekFetch(space,(uint64_t)words[11]<<32|words[17],&native_adreno_stall_mem[4+17]);   /* BV IB1 base: the low word only */
    peekFetch(space,(uint64_t)words[21]<<32|words[20],&native_adreno_stall_mem[4+34]);
    {
        uint64_t brIb=(uint64_t)words[11]<<32|words[10],bvIb=(uint64_t)words[11]<<32|words[17];
        if(!ring_->ibBefore(words[8],&ib,&size))brIb=ib;
        if(!ring_->ibBefore(words[16],&ib,&size))bvIb=ib;
        native_adreno_stall_mem[55]=job?(uint32_t)job->vm_next:0;
        native_adreno_stall_mem[56]=(uint32_t)space->handleAt(brIb);
        native_adreno_stall_mem[57]=(uint32_t)space->handleAt(bvIb);
        native_adreno_stall_mem[58]=(uint32_t)space->handleAt((uint64_t)words[21]<<32|words[20]);
    }
    for(uint32_t i=0;i<NativeAdrenoRingStallWords;++i)native_adreno_stall[2+i]=words[i];
    native_adreno_stall[1]=stall_timeouts;
    native_adreno_stall[0]=next;
}
IOReturn NativeAdrenoSubmit::wait(uint32_t fence,const NativeAdrenoRingClock *clock,
    uint32_t timeout_us)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    Job *job=find(fence);
    uint32_t observed=0;
    native_adreno_fence_wait[0]=native_adreno_fence_wait[0]+1;native_adreno_unstick[13]=timeout_us;
    if(!job) {
        native_adreno_fence_wait[6]=native_adreno_fence_wait[6]+1;
        const NativeAdrenoRingFence passed=ring_->observeFence(fence,clock,timeout_us,&observed);
        if(passed==NativeAdrenoRingFencePoisoned)return poison();
        if(passed==NativeAdrenoRingFenceArgument) {native_adreno_fence_wait[7]=fence;return kIOReturnBadArgument;}
        if(passed==NativeAdrenoRingFenceTimeout) {
            ++stats_.timeouts;
            native_adreno_fence_wait[1]=native_adreno_fence_wait[1]+1;native_adreno_fence_wait[2]=fence;native_adreno_fence_wait[3]=observed;
            native_adreno_fence_wait[4]=ring_->lastSubmitted();native_adreno_fence_wait[5]=ring_->completedFence();
            noteStall(observed,clock);
            return kIOReturnTimeout;
        }
        return kIOReturnSuccess;
    }
    const NativeAdrenoRingFence result=ring_->wait(fence,clock,timeout_us,&observed);
    if(result==NativeAdrenoRingFencePoisoned)return poison();
    if(result==NativeAdrenoRingFenceArgument) {native_adreno_fence_wait[7]=fence;return kIOReturnBadArgument;}
    if(result==NativeAdrenoRingFenceTimeout) {
        ++stats_.timeouts;
        native_adreno_fence_wait[1]=native_adreno_fence_wait[1]+1;native_adreno_fence_wait[2]=fence;native_adreno_fence_wait[3]=observed;
        native_adreno_fence_wait[4]=ring_->lastSubmitted();native_adreno_fence_wait[5]=ring_->completedFence();
        noteStall(observed,clock);
        return kIOReturnTimeout;
    }
    IOReturn status=release(*job);
    if(status)return status;
    ++stats_.completed;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::waitAll(const NativeAdrenoRingClock *clock,uint32_t timeout_us)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    while(outstanding()) {
        Job *oldest=nullptr;
        for(auto &job:jobs_)if(job.active && (!oldest || job.fence-oldest->fence>0x80000000u))oldest=&job;
        const IOReturn status=wait(oldest->fence,clock,timeout_us);
        if(status)return status;
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::waitRange(uint64_t first,uint64_t count,const NativeAdrenoRingClock *clock,uint32_t timeout_us)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    while(rangeOutstanding(first,count)) {
        Job *oldest=nullptr;
        for(auto &job:jobs_)if(job.active && job.queue-first<count && (!oldest || job.fence-oldest->fence>0x80000000u))oldest=&job;
        const IOReturn status=wait(oldest->fence,clock,timeout_us);
        if(status)return status;
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::retireCompleted()
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    for(auto &job:jobs_) {
        if(!job.active)continue;
        const NativeAdrenoRingFence state=ring_->observeFence(job.fence,nullptr,0,nullptr);
        if(state==NativeAdrenoRingFencePoisoned)return poison();
        if(state!=NativeAdrenoRingFenceComplete)continue;
        const IOReturn status=release(job);
        if(status)return status;
        ++stats_.completed;
    }
    return kIOReturnSuccess;
}
bool NativeAdrenoSubmit::madeProgress()
{
    if(progress_retries_>=3)return false;
    uint32_t state[6]={};
    if(ring_->progressState(state))return false;
    bool progress=false;
    for(unsigned i=0;i<6;++i)if(state[i]!=last_cp_[i])progress=true;
    for(unsigned i=0;i<6;++i)last_cp_[i]=state[i];
    if(!progress)return false;
    ++progress_retries_;
    return true;
}
IOReturn NativeAdrenoSubmit::hangCheck(bool *hung)
{
    if(!hung)return kIOReturnBadArgument;*hung=false;
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    /* Nothing outstanding means nothing can hang: after a restart the ring's seqno equals the fence it
     * completed, and there is no job to be stuck on. Without this the check compares the same two numbers
     * every 500 ms and reports the same hang forever (fix49 read 338 restarts, fix50 still 171). */
    if(!ring_->outstanding()) {hang_fence_=0;progress_retries_=0;return kIOReturnSuccess;}
    const uint32_t last=ring_->lastSubmitted();
    if(!last)return kIOReturnSuccess;
    uint32_t fence=0;
    if(ring_->observeFence(last,nullptr,0,&fence)==NativeAdrenoRingFencePoisoned)return poison();
    if(fence!=hang_fence_) {hang_fence_=fence;progress_retries_=0;return kIOReturnSuccess;}
    if(fence>=last || madeProgress())return kIOReturnSuccess;
    progress_retries_=0;++stats_.hangs;*hung=true;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::recover(NativeAdrenoRestart restart,void *cookie,const NativeAdrenoRingClock *clock,
    NativeAdrenoRecovery *out)
{
    if(!out)return kIOReturnBadArgument;*out={};
    if(!attached_)return kIOReturnNotReady;
    if(!restart)return kIOReturnUnsupported;
    const uint32_t last=ring_->lastSubmitted();
    /* Each replayed job is submitted again, which advances the ring's seqno: the loop's bound has to be the
     * number captured here, not a value re-read after the replays. */
    const uint32_t replayLast=last;
    uint32_t fence=0;
    {const IOReturn fr=ring_->fenceValue(&fence);
    native_adreno_recover_trace[2]=fence;native_adreno_recover_trace[3]=last;
    if(fr || fence>last) {native_adreno_recover_trace[0]=1;native_adreno_recover_trace[1]=uint32_t(fr);return poison();}}
    /* What already completed retires as usual. */
    for(auto &job:jobs_)if(job.active && job.fence<=fence) {
        IOReturn status=release(job);if(status)return status;
        ++stats_.completed;
    }
    const uint32_t hung=fence<last?fence+1:fence;
    Job *guilty=fence<last?find(hung):nullptr;
    out->fence=hung;
    if(guilty) {
        NativeAdrenoVm *vm=guilty->vm?guilty->vm:vm_;
        vm->noteHang(vm!=vm_);out->guilty=vm;
    }
    IOReturn status=restart(cookie,ring_,hung);
    out->status=status;
    if(status) {native_adreno_recover_trace[0]=2;native_adreno_recover_trace[1]=uint32_t(status);return poison();}
    /* After the restart the GPU is idle with nothing outstanding, so there is no fence to be stuck on:
     * hang_fence_ must start over, or the very next check compares against the hung fence again and reports
     * the same hang forever (fix49 read hangs=338 restarts=338 in 176 s, one every 500 ms). */
    poisoned_=false;hang_fence_=0;progress_retries_=0;
    if(guilty) {status=release(*guilty);if(status)return status;}
    /* The GPU is idle after the restart's CP init: the CPU installs the kernel table, and each replayed job switches to its own. */
    if(table_mode_) {
        if(!write_table_) {native_adreno_recover_trace[0]=3;native_adreno_recover_trace[1]=0;return poison();}
        status=write_table_(table_cookie_,vm_->ttbr0());
        if(status) {native_adreno_recover_trace[0]=3;native_adreno_recover_trace[1]=uint32_t(status);return poison();}
        if(table_mode_==2)ring_->tableInstalled(vm_->ttbr0());
        table_vm_=vm_;
    }
    uint32_t nop=0;
    if(AdrenoPM4Type7(0x10,0,&nop)!=AdrenoPM4Success) {native_adreno_recover_trace[0]=4;return poison();}   /* CP_NOP */
    /* `hung` is the fence the GPU never wrote, so that job is the one being dropped; the replay starts at
     * the next one. Each replayed job must come back as its own fence: restart() set seqno_ to `completed`,
     * so submitFramed hands out completed+1, completed+2, ... and a job whose fence is further ahead than
     * that position cannot match. Fix47 read step=6 value=8 with hung=5: the loop asked for fence 6 and
     * got 8, i.e. the ring had already issued two more. */
    /* The ring's seqno carries on from where it was (fence 7 was already handed out before the hang),
     * so a replayed submission's fence is seqno_+1 whatever the job's original number was. Match the job
     * by its own fence and accept the ring's value: the two agree only when no submission happened between
     * the hung job and the restart, and that is not the case here (fix47/48 both read value=8 for next=6). */
    /* A job whose IB the GPU already stopped on is not replayed: it is what hung the ring, and replaying it
     * hangs the ring again. fix51 read hangs=169 restarts=169 with the fence climbing 173..175 in 176 s --
     * each restart replayed the same bad submission and the next check found the same hang. */
    for(uint32_t next=hung+1;next && next<=replayLast;++next) {
        Job *job=find(next);
        NativeAdrenoVm *vm=job && job->vm?job->vm:vm_;
        const bool run=job && !vm->unusable() && job->word_count;
        status=useVm(vm,clock);
        if(status) {native_adreno_recover_trace[0]=5;native_adreno_recover_trace[1]=uint32_t(status);return poison();}
        ring_->useContext(job?job->context:0);
        uint32_t value=0;
        status=ring_->submitFramed(run?job->words:&nop,run?job->word_count:1,&value);
        if(status) {native_adreno_recover_trace[0]=6;native_adreno_recover_trace[1]=uint32_t(status);return poison();}
        /* The replayed job is now watched under the fence the ring just issued, not under its old number. */
        if(job && job->active) job->fence=value;
        ++out->replayed;
    }
    ++stats_.restarts;
    return kIOReturnSuccess;
}
