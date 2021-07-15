#include "libcflat.h"
#include "vm.h"
#include "smp.h"
#include "alloc_page.h"
#include "hyperv.h"

#define MAX_CPUS 4

/* Per-VTL/per-VCPU context */
struct hv_vtl_vcpu_ctx {
	struct hv_vp_assist_page *vp_assist_page;
};

struct hv_vcpu {
	uint32_t apicid;
	uint32_t vpid;

	/* Hypercall input/argument staging buffers */
	void *input_page;
	void *output_page;

	/* Currently active VTL on this vcpu */
	int active_vtl;

	/* Per-VTL data */
	struct hv_vtl_vcpu_ctx vtl[HV_NUM_VTLS];
};

/* Per-VTL partition-wide context */
struct hv_vtl_partition_ctx
{
	void *hypercall_page;
};

static struct hv_vcpu g_vcpus[MAX_CPUS];
static struct hv_vtl_partition_ctx g_vtl_partition_ctx[HV_NUM_VTLS];

static inline struct hv_vcpu* vp_self(void)
{
	return &g_vcpus[smp_id()];
}

static inline u8 get_active_vtl(void)
{
	return READ_ONCE(vp_self()->active_vtl);
}

static inline void set_active_vtl(int vtl)
{
	WRITE_ONCE(vp_self()->active_vtl, vtl);
}

static inline struct hv_vtl_vcpu_ctx* vtl_vcpu_ctx(void)
{
	return &vp_self()->vtl[get_active_vtl()];
}

static inline struct hv_vtl_partition_ctx *vtl_partition_ctx(void)
{
	return &g_vtl_partition_ctx[get_active_vtl()];
}

static inline void* push_input_data(void* pinput, const void* src, size_t nbytes)
{
	/* We should not overflow the page boundary */
	if ((uintptr_t)pinput + nbytes > (uintptr_t)vp_self()->input_page + PAGE_SIZE) {
		report_abort("Input page overflow at %p, %zu bytes", pinput, nbytes);
	}

	memcpy(pinput, src, nbytes);
	return pinput + nbytes;
}

static inline void do_hypercall(struct hyperv_hypercall_thunk *hc)
{
	hyperv_hypercall(vtl_partition_ctx()->hypercall_page, hc);
}

static void init_vcpu(void)
{
	uint32_t apicid = smp_id();
	if (apicid >= MAX_CPUS)
		report_abort("APIC ID %u too large", apicid);

	struct hv_vcpu *vcpu = vp_self();
	vcpu->apicid = apicid;
	vcpu->vpid = rdmsr(HV_X64_MSR_VP_INDEX);
	vcpu->input_page = alloc_page();
	vcpu->output_page = alloc_page();
	if (!vcpu->input_page || !vcpu->output_page)
		report_abort("Failed to allocate input/output pages");
	vcpu->active_vtl = 0;

	/* Enable OSFXSR for XMM hypercall arguments */
	write_cr4(read_cr4() | X86_CR4_OSFXSR);
}

/* Init per-VTL vcpu context for its active VTL */
static void init_vtl_vcpu_ctx(void)
{
	vtl_vcpu_ctx()->vp_assist_page = alloc_page();
	if (!vtl_vcpu_ctx()->vp_assist_page)
		report_abort("Could not alloc vp assist page");

	wrmsr(HV_X64_MSR_VP_ASSIST_PAGE, virt_to_phys(vtl_vcpu_ctx()->vp_assist_page) | 0x1);
}

static inline void init_vtl_partition_ctx(void)
{
	vtl_partition_ctx()->hypercall_page = hyperv_setup_hypercall(HYPERV_ENABLE_FAST_XMM_CALLS);
	if (!vtl_partition_ctx()->hypercall_page)
		report_abort("Could not setup hypercall page");
}

int main(int ac, char **av)
{
	setup_vm();

#if !defined(__x86_64__)
	report_skip("VSM tests only supported for x86-64");
	goto summary;
#endif

	/* We need SMP to do any meaningful VSM testing */
	if (cpu_count() < 2)
		report_abort("VSM tests need an SMP VM");

	if (!hv_vsm_supported()) {
		report_skip("VSM extentions not present");
		goto summary;
	}

	if (!hv_xmm_hypercall_input_supported() || !hv_xmm_hypercall_output_supported()) {
		report_skip("Hypercall XMM input/output not present");
		goto summary;
	}

	/* Setup VTL0 first */
	on_cpus(lambda(void, (void *arg) {
		init_vcpu();
		init_vtl_vcpu_ctx();
	}), NULL);
	init_vtl_partition_ctx();

summary:
	return report_summary();
}
