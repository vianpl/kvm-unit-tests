#include "libcflat.h"
#include "vm.h"
#include "smp.h"
#include "apic.h"
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
	bool is_bsp;

	/* Hypercall input/argument staging buffers */
	void *input_page;
	void *output_page;

	/* Currently active VTL on this vcpu */
	int active_vtl;

	/* Per-VTL data */
	struct hv_vtl_vcpu_ctx vtl[HV_NUM_VTLS];

	/* Function pointer set by VTL0 to call in VTL1 */
	void (*vtl_thunk_fptr) (void);

	/* Base addresses for GS and FS segments on this cpu */
	uint64_t gs_base;
	uint64_t fs_base;
};

/* Per-VTL partition-wide context */
struct hv_vtl_partition_ctx
{
	void *hypercall_page;
	void *vtl_call_va;
	void *vtl_return_va;
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

static bool read_vtl_control(struct hv_vp_vtl_control *vtl_control)
{
	if (!(rdmsr(HV_X64_MSR_VP_ASSIST_PAGE) & 0x1))
		return false;

	struct hv_vp_assist_page *vp_assist = vtl_vcpu_ctx()->vp_assist_page;
	memcpy(vtl_control, &vp_assist->vtl_control, sizeof(*vtl_control));
	return true;
}

static bool write_vtl_control(struct hv_vp_vtl_control *vtl_control)
{
	if (!(rdmsr(HV_X64_MSR_VP_ASSIST_PAGE) & 0x1))
		return false;

	struct hv_vp_assist_page *vp_assist = vtl_vcpu_ctx()->vp_assist_page;
	memcpy(&vp_assist->vtl_control, vtl_control, sizeof(*vtl_control));
	return true;
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

__attribute__((used)) static void vtl_inc(void)
{
	set_active_vtl(get_active_vtl() + 1);
}

__attribute__((used)) static void vtl_dec(void)
{
	set_active_vtl(get_active_vtl() - 1);
}

/* VA to jump to when performing a VTL call from current VTL */
__attribute__((used)) static void* vtl_call_va(void)
{
	return vtl_partition_ctx()->vtl_call_va;
}

/* VA to jump to when performing a VTL return from current VTL */
__attribute__((used)) static void* vtl_return_va(void)
{
	return vtl_partition_ctx()->vtl_return_va;
}

/*
 * VTL1 call trampoline.
 *
 * VTL0 does not save any of its own state, everything is handled by VTL1.
 * This is because we should always expect VTL interrupt entries anyway,
 * which will no go through normal VTL call gates.
 */
void __vtl_call_trampoline(void);
asm (
"__vtl_call_trampoline: \n"

	/* RCX must be 0 by spec */
	"xor    %rcx, %rcx \n"
	"call	vtl_call_va \n"
	"jmp    %rax \n"
);

/*
 * VTL1 entry
 *
 * On the very first VTL1 call we enter at the beginning.
 * When we later do a VTL return, hypervisor saves next instruction RIP for re-entry,
 * so on subsequent VTL calls we end up where we left off: right after previous VTL return.
 */
void __vtl1_entry(void);
asm (
"__vtl1_entry: \n"

	/* Save VTL0 SysV ABI scratch space registers.
	 * We don't have to do this for normal VTL calls, only for VTL interrupts,
	 * but since we don't want to figure out entry type here, we will just go for the
	 * worst case and keep it simple(r). */
	"push   %rdi \n"
	"push   %rsi \n"
	"push   %rdx \n"
	"push   %r8 \n"
	"push   %r9 \n"
	"push   %r10 \n"
	"push   %r11 \n"
	"push   %rax \n"
	"push   %rcx \n"

	/* Increment VTL number on entry */
	"call	vtl_inc \n"

	/* Entry handler is reponsible to manage VTL0's rax and rcx values
	 * by potentially passing them over to be restored by the hypervisor. */
	"pop	%rsi \n"
	"pop	%rdi \n"
	"call   vtl1_entry \n"

	/* Save the fast return flag */
	"push	%rax \n"

	/* Save VTL return VA */
	"call	vtl_return_va \n"
	"push	%rax \n"

	/* Decrement VTL number on return */
	"call	vtl_dec \n"

	/* Load fast return VA and fast return flag */
	"pop	%rax \n"
	"pop	%rcx \n"

	/* Restore previous scratch VTL state */
	"pop    %r11 \n"
	"pop    %r10 \n"
	"pop    %r9 \n"
	"pop    %r8 \n"
	"pop    %rdx \n"
	"pop    %rsi \n"
	"pop    %rdi \n"

	/* Execute VTL return */
	"call   %rax\n"

	/* VTL return will restore context past its vmcall, so we will end up here
	 * upon subsequent VTL re-entries and not at the beginning. */
	"jmp    __vtl1_entry \n"
);

/* Return true if fast return is allowed */
__attribute__((used)) static bool vtl1_entry(u64 rax, u64 rcx)
{
	struct hv_vcpu *vcpu = vp_self();
	struct hv_vp_vtl_control vtl_control;
	bool is_fast_return;

	assert(get_active_vtl() == 1);

	/* We are ready to enable interrupts now */
	sti();

	/* Read vtl control to check call reason (if VP_ASSIST page is already mapped) */
	if (read_vtl_control(&vtl_control)) {
		switch (vtl_control.entry_reason) {
		case HV_VTL_ENTRY_VTL_CALL:
			vcpu->vtl_thunk_fptr();

			/* Fast return is okay for VTL calls, because those look like a function call
			 * and RAX, RCX are scratch registers in SysV ABI, caller should've kept them safe. */
			is_fast_return = true;
			break;
		case HV_VTL_ENTRY_INTERRUPT:
			/* For VTL interrupts we need to ask hypervisor to restore guest rax/rcx */
			vtl_control.vtl_return_x64_rax = rax;
			vtl_control.vtl_return_x64_rcx = rcx;
			if (!write_vtl_control(&vtl_control))
				report_abort("Could not write VTL control when exiting from VTL interrupt");
			is_fast_return = false;
			break;
		default:
			report_abort("Unexpected VTL entry reason %u\n", vtl_control.entry_reason);
		};
	} else {
		/* Assume VTL call without a VP assist page */
		vcpu->vtl_thunk_fptr();
		is_fast_return = true;
	}

	cli();
	return is_fast_return;
}

/*
 * Perform a call to VTL1.
 * Takes fptr and executes it in VTL1 context in current VCPU.
 */
static inline void run_in_vtl1(void (*fptr)(void))
{
	vp_self()->vtl_thunk_fptr = fptr;
	__vtl_call_trampoline();
}

static void *make_vsm_page_table(void)
{
	/* Make a 4GiB identity mapped page table loosely based on what VTL0 has */
	uint64_t *l2 = alloc_pages(2); /* 4 pages */
	if (!l2)
		report_abort("Couldn't allocate page table pages for VTL1");

	for (uint64_t i = 0; i < 512 * 4; ++i)
		l2[i] = 0x1e7ull | (i << 21);

	uint64_t *l3 = alloc_page();
	if (!l3)
		report_abort("Couldn't allocate page table pages for VTL1");

	for (uint64_t i = 0; i < 4; ++i)
		l3[i] = (virt_to_phys(l2) + 4096 * i) | 0x7;

	uint64_t *l4 = alloc_page();
	if (!l4)
		report_abort("Couldn't allocate page table pages for VTL1");

	*l4 = virt_to_phys(l3) | 0x7;
	return l4;
}

/*
 * Setup (initial) VTL1 context which is a fully initialized 64-bit mode
 */
static u64 enable_vp_vtl(void *cr3, struct hv_vcpu* target_vcpu)
{
	struct hv_initial_vp_context ctx = {0};

	void *stack_page = alloc_page();
	if (!stack_page)
		report_abort("Failed to alloc VTL1 stack");

	ctx.rsp = (uintptr_t)stack_page + PAGE_SIZE;
	ctx.rip = (uintptr_t)__vtl1_entry;
	ctx.rflags = 0x2; /* Bit 1 is always set, interrupts disabled */

	struct descriptor_table_ptr dtbl;
	sgdt(&dtbl);
	ctx.gdtr.base = dtbl.base;
	ctx.gdtr.limit = dtbl.limit;
	sidt(&dtbl);
	ctx.idtr.base = dtbl.base;
	ctx.idtr.limit = dtbl.limit;

	ctx.cs.base = 0;
	ctx.cs.limit = 0xFFFFFFFF;
	ctx.cs.selector = read_cs();
	ctx.cs.type = 0b1011;
	ctx.cs.s = 1;
	ctx.cs.p = 1;
	ctx.cs.g = 1;
	ctx.cs.l = 1;

	ctx.ds.base = 0;
	ctx.ds.limit = 0xFFFFFFFF;
	ctx.ds.selector = read_ds();
	ctx.ds.type = 0b0011;
	ctx.ds.s = 1;
	ctx.ds.p = 1;
	ctx.ds.g = 1;
	ctx.ds.db = 1;
	ctx.es = ctx.ss = ctx.ds;

	/* GS_BASE is used in kvm-unit-tests to address per-cpu data */
	ctx.gs.base = target_vcpu->gs_base;
	ctx.fs.base = target_vcpu->fs_base;

	/* TR and LDTR are not used but we still need them present for vmentry */
	ctx.tr.type = 0b1011;
	ctx.tr.p = 1;
	ctx.ldtr.type = 0b0010;
	ctx.ldtr.p = 1;

	ctx.efer = EFER_LMA | EFER_LME;
	ctx.cr0 = X86_CR0_PG | X86_CR0_WP | X86_CR0_PE;
	ctx.cr4 = X86_CR4_PAE | X86_CR4_OSFXSR;
	ctx.cr3 = (uintptr_t)cr3;

	return hv_enable_vp_vtl(HV_PARTITION_ID_SELF, target_vcpu->vpid, 1, &ctx);
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
	vcpu->gs_base = rdmsr(MSR_GS_BASE);
	vcpu->fs_base = rdmsr(MSR_FS_BASE);
	vcpu->is_bsp = (rdmsr(MSR_IA32_APICBASE) & APIC_BSP) != 0;

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

	union hv_register_vsm_code_page_offsets vtl_offsets;
	vtl_offsets.as_u64 = get_vp_register64(HV_REGISTER_VSM_CODE_PAGE_OFFSETS);
	vtl_partition_ctx()->vtl_call_va = vtl_partition_ctx()->hypercall_page + vtl_offsets.vtl_call_offset;
	vtl_partition_ctx()->vtl_return_va = vtl_partition_ctx()->hypercall_page + vtl_offsets.vtl_return_offset;
}

static inline void init_vtl_x2apic(void)
{
	/* We expect VTL1 apics to be enabled, but in xapic mode and software-disabled. */
	u64 apic_base = rdmsr(MSR_IA32_APICBASE);
	if (!((apic_base & APIC_EN) && this_cpu_has(X86_FEATURE_APIC)))
		report_abort("CPU%u: VTL1 apic should be hw-enabled", smp_id());

	enable_x2apic();

	/* BSP should still be BSP and APs should still not */
	report(vp_self()->is_bsp == ((apic_base & APIC_BSP) != 0),
		"CPU%u: VTL1 apic base BSP bit is correct", smp_id());

	/* Check that apic id still matches VTL0 and cpuid value */
	u32 apicid = apic_read(APIC_ID);
	report(apicid == vp_self()->apicid && apicid == cpuid(0xb).d,
		"CPU%u: APIC ID matches VTL0", smp_id());

	/* Check that version is sane */
	u8 version = (u8)apic_read(APIC_LVR);
	report(version >= 0x10 && version <= 0x15,
		"CPU%u: VTL1 apic base BSP bit is correct", smp_id());
}

static void init_vsm(void)
{
	/* Enable VTL1 on partition */
	union hv_enable_partition_vtl_flags flags = {0};
	uint64_t res = hv_enable_partition_vtl(HV_PARTITION_ID_SELF, 1, flags);
	if (res != HV_STATUS_SUCCESS)
		report_abort("Failed to enable VTL1 for partition: 0x%lx\n", res);

	/* Make sure we see VTL1 enabled partition-wide */
	union hv_register_vsm_partition_status vsm_status;
	vsm_status.as_u64 = get_vp_register64(HV_REGISTER_VSM_PARTITION_STATUS);
	report(vsm_status.enabled_vtl_set == 0b11, "VTL1 enabled partition-wide");

	/* Populate VTL1 page table */
	void *cr3 = make_vsm_page_table();
	if (!cr3)
		report_abort("Failed to create VTL1 page table");

	/* Enable VTL1 on current cpu */
	res = enable_vp_vtl(cr3, vp_self());
	if (res != HV_STATUS_SUCCESS)
		report_abort("Failed to enable VTL1 for VP %u: 0x%lx\n", smp_id(), res);

	union hv_register_vsm_vp_status vsm_vp_status;
	vsm_vp_status.as_u64 = get_vp_register64(HV_REGISTER_VSM_VP_STATUS);
	report(vsm_vp_status.enabled_vtl_set == 0b11, "VTL1 enabled on cpu%d", smp_id());

	/* Since we now have 1 VP with VTL1 enabled, only it can enabled VTL1 for the rest of VPs (per TLFS spec).
	 * Check that this restriction holds by observing that other VPs can't enable VTL1 */
	on_cpus(lambda(void, (void* unused) {
		res = enable_vp_vtl(cr3, vp_self());
		report(res != HV_STATUS_SUCCESS, "Expected to not be able to enable VTL1 on VP%d: %lu", smp_id(), res);
	}), NULL);

	/* Use this cpu to init partition-wide VTL1 context */
	run_in_vtl1(init_vtl_partition_ctx);

	/* Enter VTL1 on this CPU and enable VTL1 for real on other CPUs */
	run_in_vtl1(lambda(void, (void) {
		for (size_t i = 0; i < cpu_count(); i++) {
			struct hv_vcpu* vcpu = &g_vcpus[i];
			if (vcpu->apicid == smp_id())
				continue;

			uint64_t res = enable_vp_vtl(cr3, vcpu);
			if (res != HV_STATUS_SUCCESS)
				report_abort("Failed to enable VTL1 for cpu%u: 0x%lx\n", vcpu->apicid, res);
		}
	}));

	/* Enter VTL1 on all cpus and init per-cpu VTL1 context */
	on_cpus(lambda(void, (void* unused) {
		/* Verify VTL1 is enabled on this cpu */
		union hv_register_vsm_vp_status vsm_vp_status;
		vsm_vp_status.as_u64 = get_vp_register64(HV_REGISTER_VSM_VP_STATUS);
		report(vsm_vp_status.enabled_vtl_set == 0b11, "VTL1 enabled on cpu%d", smp_id());

		/* Enter VTL1 to finish init */
		run_in_vtl1(lambda(void, (void) {
			init_vtl_vcpu_ctx();
			init_vtl_x2apic();
		}));
	}), NULL);
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

/*
 * Checks that initial hyper-v VSM config is as expected
 */
static void test_vsm_initial_status(void)
{
	union hv_register_vsm_capabilities vsm_capabilities;
	vsm_capabilities.as_u64 = get_vp_register64(HV_REGISTER_VSM_CAPABILITIES);

	union hv_register_vsm_partition_status vsm_status;
	vsm_status.as_u64 = get_vp_register64(HV_REGISTER_VSM_PARTITION_STATUS);
	report(
		/* Only VTL0 is enabled */
		(vsm_status.enabled_vtl_set & (1u << 0)) &&
		(vsm_status.mbec_enabled_vtl_set & ~vsm_capabilities.mbec_vtl_mask) == 0,
		"VSM partition config"
	);

	on_cpus(lambda(void, (void* unused) {
		union hv_register_vsm_vp_status vsm_vp_status;
		vsm_vp_status.as_u64 = get_vp_register64(HV_REGISTER_VSM_VP_STATUS);
		report(
			/* Only VTL0 is enabled */
			(vsm_vp_status.enabled_vtl_set & (1u << 0)) &&
			(vsm_vp_status.active_vtl == 0),
			"VSM VP config on vcpu %d", smp_id()
		);
	}), NULL);
}

int main(int ac, char **av)
{
	/*
	 * Warning on using vmalloc and passing pointers between VTLs.
	 *
	 * vmalloc modifies page tables, VTLs has their own page tables,
	 * thus any changes made by vmalloc in VTL0 will not be carried over to VTL1.
	 * As a result attempts to pass malloc/calloc-ed pointers between VTLs will result in a #PF.
	 */
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

	test_vsm_initial_status();

	init_vsm();

summary:
	return report_summary();
}
