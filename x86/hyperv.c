#include "hyperv.h"
#include "asm/io.h"
#include "smp.h"
#include "alloc_page.h"

enum {
    HV_TEST_DEV_SINT_ROUTE_CREATE = 1,
    HV_TEST_DEV_SINT_ROUTE_DESTROY,
    HV_TEST_DEV_SINT_ROUTE_SET_SINT,
    HV_TEST_DEV_MSG_CONN_CREATE,
    HV_TEST_DEV_MSG_CONN_DESTROY,
    HV_TEST_DEV_EVT_CONN_CREATE,
    HV_TEST_DEV_EVT_CONN_DESTROY,
};

static void synic_ctl(u32 ctl, u32 vcpu_id, u32 sint, u32 conn_id)
{
    outl((conn_id << 24) | (ctl << 16) | (vcpu_id << 8) | sint, 0x3000);
}

static void sint_enable(u8 sint, u8 vec, bool auto_eoi)
{
    wrmsr(HV_X64_MSR_SINT0 + sint,
          (u64)vec | (auto_eoi ? HV_SYNIC_SINT_AUTO_EOI : 0));
}

static void sint_disable(u8 sint)
{
    wrmsr(HV_X64_MSR_SINT0 + sint, 0xff | HV_SYNIC_SINT_MASKED);
}

void synic_sint_create(u8 sint, u8 vec, bool auto_eoi)
{
    synic_ctl(HV_TEST_DEV_SINT_ROUTE_CREATE, smp_id(), sint, 0);
    sint_enable(sint, vec, auto_eoi);
}

void synic_sint_set(u8 vcpu, u8 sint)
{
    synic_ctl(HV_TEST_DEV_SINT_ROUTE_SET_SINT, vcpu, sint, 0);
}

void synic_sint_destroy(u8 sint)
{
    sint_disable(sint);
    synic_ctl(HV_TEST_DEV_SINT_ROUTE_DESTROY, smp_id(), sint, 0);
}

void msg_conn_create(u8 sint, u8 vec, u8 conn_id)
{
    synic_ctl(HV_TEST_DEV_MSG_CONN_CREATE, smp_id(), sint, conn_id);
    sint_enable(sint, vec, true);
}

void msg_conn_destroy(u8 sint, u8 conn_id)
{
    sint_disable(sint);
    synic_ctl(HV_TEST_DEV_MSG_CONN_DESTROY, 0, 0, conn_id);
}

void evt_conn_create(u8 sint, u8 vec, u8 conn_id)
{
    synic_ctl(HV_TEST_DEV_EVT_CONN_CREATE, smp_id(), sint, conn_id);
    sint_enable(sint, vec, true);
}

void evt_conn_destroy(u8 sint, u8 conn_id)
{
    sint_disable(sint);
    synic_ctl(HV_TEST_DEV_EVT_CONN_DESTROY, 0, 0, conn_id);
}

static bool g_fast_xmm_calls_enabled;

void *hyperv_setup_hypercall(enum hyperv_hypercall_flags flags)
{
    u64 guestid = (0x8f00ull << 48);

    void *hypercall_page = alloc_page();
    if (!hypercall_page)
        report_abort("failed to allocate hypercall page");

    wrmsr(HV_X64_MSR_GUEST_OS_ID, guestid);

    wrmsr(HV_X64_MSR_HYPERCALL,
          (u64)virt_to_phys(hypercall_page) | HV_X64_MSR_HYPERCALL_ENABLE);

    g_fast_xmm_calls_enabled = flags & HYPERV_ENABLE_FAST_XMM_CALLS;
    return hypercall_page;
}

void hyperv_teardown_hypercall(void* hypercall_page)
{
    wrmsr(HV_X64_MSR_HYPERCALL, 0);
    wrmsr(HV_X64_MSR_GUEST_OS_ID, 0);
    free_page(hypercall_page);
}

/* Call hyperv hypercall */
void hyperv_hypercall(void *hypercall_page, struct hyperv_hypercall_thunk *hc)
{
    uint64_t ctl = (uint64_t)hc->code |
                   (uint64_t)hc->fast << 16 |
                   (uint64_t)hc->var_cnt << 17 |
                   (uint64_t)hc->rep_cnt << 32 |
                   (uint64_t)hc->rep_idx << 48;

    uint64_t result;

    /*
     * Note about XMM usage:
     *
     * Overall kvm unit tests build with -mno-sse -mno-sse2 for both 32 and 64 bit archs,
     * so compiler is not generating code to use those registers, so we are safe to clobber them below as long
     * as we're the only thing that uses them in this test.
     */

    if (hc->fast && g_fast_xmm_calls_enabled) {
        asm volatile ("movdqu %0, %%xmm0" ::"m"(hc->xmm[0]));
        asm volatile ("movdqu %0, %%xmm1" ::"m"(hc->xmm[1]));
        asm volatile ("movdqu %0, %%xmm2" ::"m"(hc->xmm[2]));
        asm volatile ("movdqu %0, %%xmm3" ::"m"(hc->xmm[3]));
        asm volatile ("movdqu %0, %%xmm4" ::"m"(hc->xmm[4]));
        asm volatile ("movdqu %0, %%xmm5" ::"m"(hc->xmm[5]));
    }

#ifdef __x86_64__
    register uint64_t r8 __asm__("r8") = hc->arg2; /* No gcc constraint for r8, so local register spec */
    asm volatile (
            "call   *%[hcall_page] \n"
            :"=a"(result)
            :"c"(ctl), "d"(hc->arg1), "r"(r8), [hcall_page] "m" (hypercall_page)
            :"memory");
#else
    asm volatile (
            "call   *%[hcall_page] \n"
            :"=A"(result)
            :"A"(ctl),
            "c"((uint32_t)(hc->arg1 & 0xFFFFFFFF)), "b"((uint32_t)(hc->arg1 >> 32)),
            "S"((uint32_t)(hc->arg2 & 0xFFFFFFFF)), "D"((uint32_t)(hc->arg2 >> 32)),
            [hcall_page] "m" (hypercall_page)
            :"memory");
#endif

    if (hc->fast && g_fast_xmm_calls_enabled) {
        asm volatile ("movdqu %%xmm0, %0" :"=m"(hc->xmm[0])::"memory");
        asm volatile ("movdqu %%xmm1, %0" :"=m"(hc->xmm[1])::"memory");
        asm volatile ("movdqu %%xmm2, %0" :"=m"(hc->xmm[2])::"memory");
        asm volatile ("movdqu %%xmm3, %0" :"=m"(hc->xmm[3])::"memory");
        asm volatile ("movdqu %%xmm4, %0" :"=m"(hc->xmm[4])::"memory");
        asm volatile ("movdqu %%xmm5, %0" :"=m"(hc->xmm[5])::"memory");
    }

    hc->result = result & 0xFFFFFFFF;
    hc->rep_idx += (result >> 32) & 0xFFF;
}
