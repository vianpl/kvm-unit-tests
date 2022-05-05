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

/* HvSetVpRegister hypercall */
static uint64_t hv_set_vp_registers(uint32_t vp_index,
                                    union hv_input_vtl target_vtl,
                                    uint32_t* names,
                                    struct hv_vp_register_val* vals,
                                    size_t nregs,
                                    bool fastcall)
{
	struct hv_vcpu *vcpu = vp_self();

	/* 12 bit field for rep_cnt */
	if (nregs > 0xFFF) {
		report_abort("Cannot fit %zu registers into rep_cnt field", nregs);
	}

	struct hv_get_set_vp_registers args;
	args.partition_id = HV_PARTITION_ID_SELF;
	args.vp_index = vp_index;
	args.input_vtl = target_vtl;

	struct hyperv_hypercall_thunk hc = {0};
	hc.fast = fastcall;
	hc.code = HVCALL_SET_VP_REGISTERS;
	hc.rep_cnt = nregs;
	hc.rep_idx = 0;

	if (fastcall) {
		if (nregs > 4) {
			report_abort("Cannot fit %zu registers in XMM call:", nregs);
		}

		hc.arg1 = ((uint64_t*)&args)[0];
		hc.arg2 = ((uint64_t*)&args)[1];

		/* 4 register names per xmm reg */
		uint32_t* dst32 = (uint32_t*)(&hc.xmm[0]);
		for (size_t i = 0; i < nregs; ++i) {
			dst32[i] = names[i];
		}

		/* 1 register value per xmm reg, starting from the next register after whatever we used for names */
		sse_reg* reg = &hc.xmm[ALIGN_UP(nregs, 4) / 4];
		for (size_t i = 0; i < nregs; ++i) {
			reg->q[0] = vals[i].low;
			reg->q[1] = vals[i].high;
			++reg;
		}
	} else {
		hc.arg1 = virt_to_phys(vcpu->input_page);

		void *pinput = vcpu->input_page;
		pinput = push_input_data(pinput, &args, sizeof(args));
		pinput = push_input_data(pinput, names, sizeof(*names) * nregs);
		pinput = push_input_data(align_ptr_up(pinput, sizeof(vals)), vals, sizeof(*vals) * nregs);
	}

	while (hc.rep_idx < hc.rep_cnt) {
		do_hypercall(&hc);
		if (hc.result != HV_STATUS_SUCCESS) {
			return hc.result;
		}
	}

	return HV_STATUS_SUCCESS;
}

static inline void set_vp_register(uint32_t name, struct hv_vp_register_val val)
{
	uint64_t res = hv_set_vp_registers(HV_VP_INDEX_SELF, HV_NO_INPUT_VTL, &name, &val, 1, true);
	assert_msg(HV_STATUS_SUCCESS == res, "Failed to write VP register 0x%x: %lx\n", name, res);
}

static inline void set_vp_register64(uint32_t name, uint64_t val)
{
	set_vp_register(name, (struct hv_vp_register_val){ .low = val, .high = 0 });
}

/* HvGetVpRegister hypercall */
static uint64_t hv_get_vp_registers(uint32_t vp_index,
                                    union hv_input_vtl target_vtl,
                                    uint32_t* names,
                                    struct hv_vp_register_val* vals,
                                    size_t nregs,
                                    bool fastcall)
{
	struct hv_vcpu *vcpu = vp_self();

	/* 12 bit field for rep_cnt */
	if (nregs > 0xFFF) {
		report_abort("Cannot fit %zu registers into rep_cnt field", nregs);
	}

	struct hv_get_set_vp_registers args;
	args.partition_id = HV_PARTITION_ID_SELF;
	args.vp_index = vp_index;
	args.input_vtl = target_vtl;

	struct hyperv_hypercall_thunk hc = {0};
	hc.fast = fastcall;
	hc.code = HVCALL_GET_VP_REGISTERS;
	hc.rep_cnt = nregs;
	hc.rep_idx = 0;

	if (fastcall) {
		if (nregs > 4) {
			report_abort("Cannot fit %zu registers in XMM call:", nregs);
		}

		hc.arg1 = ((uint64_t*)&args)[0];
		hc.arg2 = ((uint64_t*)&args)[1];

		/* 4 register names per xmm reg */
		uint32_t* dst32 = (uint32_t*)(&hc.xmm[0]);
		for (size_t i = 0; i < nregs; ++i) {
			dst32[i] = names[i];
		}
	} else {
		hc.arg1 = virt_to_phys(vcpu->input_page);
		hc.arg2 = virt_to_phys(vcpu->output_page);

		void *pinput = vcpu->input_page;
		pinput = push_input_data(pinput, &args, sizeof(args));
		pinput = push_input_data(pinput, names, sizeof(*names) * nregs);
	}

	while (hc.rep_idx < hc.rep_cnt) {
		do_hypercall(&hc);
		if (hc.result != HV_STATUS_SUCCESS) {
			return hc.result;
		}
	}

	if (fastcall) {
		/* 1 register value per xmm reg, starting from the next register after whatever we used for names */
		sse_reg* reg = &hc.xmm[ALIGN_UP(nregs, 4) / 4];
		memcpy(vals, reg, nregs * sizeof(*vals));
	} else {
		memcpy(vals, vcpu->output_page, nregs * sizeof(*vals));
	}

	return HV_STATUS_SUCCESS;
}

static inline struct hv_vp_register_val get_vp_register(uint32_t name)
{
	struct hv_vp_register_val val;
	uint64_t res = hv_get_vp_registers(HV_VP_INDEX_SELF, HV_NO_INPUT_VTL, &name, &val, 1, true);
	assert_msg(HV_STATUS_SUCCESS == res, "Failed to read VP register 0x%x: %lx\n", name, res);
	return val;
}

static inline uint64_t get_vp_register64(uint32_t name)
{
	return get_vp_register(name).low;
}

/* HvEnablePartitionVtl hypercall */
static uint64_t hv_enable_partition_vtl(uint64_t partition_id,
					uint8_t target_vtl,
					union hv_enable_partition_vtl_flags flags)
{
	struct hv_enable_partition_vtl args = {0};
	args.target_partition_id = partition_id;
	args.target_vtl = target_vtl;
	args.flags = flags;

	struct hyperv_hypercall_thunk hc = {0};
	hc.fast = true;
	hc.code = HVCALL_ENABLE_PARTITION_VTL;
	hc.arg1 = ((uint64_t*)&args)[0];
	hc.arg2 = ((uint64_t*)&args)[1];

	do_hypercall(&hc);
	return hc.result;
}

/* HvEnableVpVtl hypercall */
static uint64_t hv_enable_vp_vtl(uint64_t partition_id,
				 uint32_t vp_index,
				 uint8_t target_vtl,
				 const struct hv_initial_vp_context *ctx)
{
	struct hv_vcpu *vcpu = vp_self();

	struct hv_enable_vp_vtl args;
	args.target_partition_id = partition_id;
	args.vp_index = vp_index;
	args.target_vtl = target_vtl;
	memcpy(&args.vp_vtl_context, ctx, sizeof(*ctx));

	struct hyperv_hypercall_thunk hc = {0};
	hc.code = HVCALL_ENABLE_VP_VTL;
	hc.arg1 = virt_to_phys(vcpu->input_page);
	push_input_data(vcpu->input_page, &args, sizeof(args));

	do_hypercall(&hc);
	return hc.result;
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

static void test_get_set_vp_registers_negative(bool fast)
{
	uint32_t names[3] = { HV_X64_REGISTER_CR0, HV_X64_REGISTER_CR3, HV_X64_REGISTER_CR4 };
	struct hv_vp_register_val vals[3] = {0};

	/* Bad VP index */
	report(
		hv_get_vp_registers(0xacacacac, HV_INPUT_VTL(0), names, vals, 3, fast) == HV_STATUS_INVALID_VP_INDEX &&
		hv_set_vp_registers(0xacacacac, HV_INPUT_VTL(0), names, vals, 3, fast) == HV_STATUS_INVALID_VP_INDEX,
		"GetSetRegisters: Bad VP Index"
	);

	/* Bad VTL index */
	report(
		hv_get_vp_registers(HV_VP_INDEX_SELF, HV_INPUT_VTL(HV_INVALID_VTL), names, vals, 3, fast) != HV_STATUS_SUCCESS &&
		hv_set_vp_registers(HV_VP_INDEX_SELF, HV_INPUT_VTL(HV_INVALID_VTL), names, vals, 3, fast) != HV_STATUS_SUCCESS,
		"GetSetRegisters: Bad VTL index"
	);

	/* We should not be allowed to access VTL1 registers while in VTL0 */
	report(
		hv_get_vp_registers(HV_VP_INDEX_SELF, HV_INPUT_VTL(1), names, vals, 3, fast) == HV_STATUS_ACCESS_DENIED &&
		hv_set_vp_registers(HV_VP_INDEX_SELF, HV_INPUT_VTL(1), names, vals, 3, fast) == HV_STATUS_ACCESS_DENIED,
		"GetSetRegisters: Unpriviledged VTL"
	);

	/* Bad register id */
	names[0] = 0xacacacac;
	report(
		hv_get_vp_registers(HV_VP_INDEX_SELF, HV_INPUT_VTL(0), names, vals, 3, fast) == HV_STATUS_INVALID_PARAMETER &&
		hv_set_vp_registers(HV_VP_INDEX_SELF, HV_INPUT_VTL(0), names, vals, 3, fast) == HV_STATUS_INVALID_PARAMETER,
		"GetSetRegisters: Bad register id"
	);

	/* Zero registers */
	report(
		hv_get_vp_registers(HV_VP_INDEX_SELF, HV_INPUT_VTL(0), names, vals, 0, fast) == HV_STATUS_SUCCESS &&
		hv_set_vp_registers(HV_VP_INDEX_SELF, HV_INPUT_VTL(0), names, vals, 0, fast) == HV_STATUS_SUCCESS,
		"GetSetRegisters: Zero register count"
	);

	if (fast) {

		/*
		 * Try to push more registers than XMM calls can handle.
		 * Each XMM input reg fits 4 register names at most and 1 register value to output.
		 * So given 6 registers total, specifying 5 and more names will not fit.
		 *
		 * We will call hypercalls directly here since we need to mess with thunk structure
		 */

		struct hyperv_hypercall_thunk hc = {0};
		hc.fast = true;
		hc.arg1 = HV_PARTITION_ID_SELF;
		hc.arg2 = HV_VP_INDEX_SELF;
		hc.rep_cnt = 5;
		hc.rep_idx = 0;

		hc.code = HVCALL_GET_VP_REGISTERS;
		do_hypercall(&hc);
		report(hc.result != HV_STATUS_SUCCESS, "GetRegisters: XMM count overflow");

		hc.code = HVCALL_SET_VP_REGISTERS;
		do_hypercall(&hc);
		report(hc.result != HV_STATUS_SUCCESS, "SetRegisters: XMM count overflow");
	}
}

/*
 * Negative tests for HvEnablePartitionVtl hypercall
 */
static void test_vsm_enable_partition_vtl_negative(void)
{
	union hv_register_vsm_capabilities vsm_capabilities;
	vsm_capabilities.as_u64 = get_vp_register64(HV_REGISTER_VSM_CAPABILITIES);

	union hv_register_vsm_partition_status vsm_status;
	vsm_status.as_u64 = get_vp_register64(HV_REGISTER_VSM_PARTITION_STATUS);

	union hv_enable_partition_vtl_flags flags = {0};

	/* Bad partition id */
	report(
		hv_enable_partition_vtl(42, 0, flags) == HV_STATUS_INVALID_PARTITION_ID,
		"EnablePartitionVtl: Bad partition ID"
        );

	/* VTL0 already enabled */
	report(
		hv_enable_partition_vtl(HV_PARTITION_ID_SELF, 0, flags) == HV_STATUS_INVALID_PARAMETER,
		"EnablePartitionVtl: VTL0 already enabled"
	);

	/* Cannot enable VTL past maximum supported */
	report(
		hv_enable_partition_vtl(
		HV_PARTITION_ID_SELF, vsm_status.maximum_vtl + 1, flags) == HV_STATUS_INVALID_PARAMETER,
		"EnablePartitionVtl: Maximum VTL"
	);

	/* MBEC cannot be enabled (for VTL1 in this case) if not supported */
	if ((vsm_capabilities.mbec_vtl_mask & (1u << 1)) == 0) {
		union hv_enable_partition_vtl_flags flags = {0};
		flags.enable_mbec = 1;
		report(
			hv_enable_partition_vtl(HV_PARTITION_ID_SELF, 1, flags) == HV_STATUS_INVALID_PARAMETER,
			"EnablePartitionVtl: MBEC not allowed"
		);
	}
}

/*
 * Negative tests for HvEnableVpVtl hypercall
 */
static void test_vsm_enable_vp_vtl_negative(void)
{
	struct hv_initial_vp_context ctx = {0};

	/* Bad partition id */
	report(
		hv_enable_vp_vtl(42, HV_VP_INDEX_SELF, 1, &ctx) == HV_STATUS_INVALID_PARTITION_ID,
		"EnableVpVtl: Bad partition ID"
	);

	/* Bad vp index */
	report(
		hv_enable_vp_vtl(HV_PARTITION_ID_SELF, (u32)-1, 1, &ctx) == HV_STATUS_INVALID_VP_INDEX,
		"EnableVpVtl: Bad VP Index"
	);

	/* VTL0 already enabled */
	report(
		hv_enable_vp_vtl(HV_PARTITION_ID_SELF, HV_VP_INDEX_SELF, 0, &ctx) == HV_STATUS_INVALID_PARAMETER,
		"EnableVpVtl: VTL0 already enabled"
	);
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

	test_get_set_vp_registers_negative(true);
	test_get_set_vp_registers_negative(false);

	test_vsm_enable_partition_vtl_negative();
	test_vsm_enable_vp_vtl_negative();

summary:
	return report_summary();
}
