#ifndef X86_HYPERV_H
#define X86_HYPERV_H

#include "libcflat.h"
#include "processor.h"

#define HYPERV_CPUID_FEATURES                   0x40000003

#define HV_X64_MSR_TIME_REF_COUNT_AVAILABLE     (1 << 1)
#define HV_X64_MSR_SYNIC_AVAILABLE              (1 << 2)
#define HV_X64_MSR_SYNTIMER_AVAILABLE           (1 << 3)

/* HYPERV_CPUID_FEATURES.EBX */
#define HV_X64_ACCESS_VSM                       (1 << 16)
#define HV_X64_ACCESS_VP_REGISTERS              (1 << 17)

/* HYPERV_CPUID_FEATURES.EDX */
#define HV_X64_HYPERCALL_XMM_INPUT_AVAILABLE    (1 << 4)
#define HV_X64_HYPERCALL_XMM_OUTPUT_AVAILABLE   (1 << 15)

#define HV_X64_MSR_GUEST_OS_ID                  0x40000000
#define HV_X64_MSR_HYPERCALL                    0x40000001
#define HV_X64_MSR_VP_INDEX                     0x40000002

#define HV_X64_MSR_TIME_REF_COUNT               0x40000020
#define HV_X64_MSR_REFERENCE_TSC                0x40000021
#define HV_X64_MSR_VP_ASSIST_PAGE               0x40000073

enum hv_vtl_entry_reason {
        HV_VTL_ENTRY_RESERVED = 0,
        HV_VTL_ENTRY_VTL_CALL = 1,
        HV_VTL_ENTRY_INTERRUPT = 2,
};

struct hv_vp_vtl_control {
        uint32_t entry_reason;

        union {
                uint8_t as_u8;
                struct {
                        uint8_t vina_asserted:1;
                        uint8_t reserved0:7;
                };
        };

        uint8_t reserved1[3];

        union {
                struct {
                        uint64_t vtl_return_x64_rax;
                        uint64_t vtl_return_x64_rcx;
                };

                struct {
                        uint32_t vtl_return_x86_eax;
                        uint32_t vtl_return_x86_ecx;
                        uint32_t vtl_return_x86_edx;
                        uint32_t reserved2;
                };
        };
} __attribute__((packed));

/* Define virtual processor assist page structure. */
struct hv_vp_assist_page {
        uint32_t apic_assist;
        uint32_t reserved1;
        struct hv_vp_vtl_control vtl_control;
        /* There is more data below which we don't care about yet */
} __attribute__((packed));

/* Define synthetic interrupt controller model specific registers. */
#define HV_X64_MSR_SCONTROL                     0x40000080
#define HV_X64_MSR_SVERSION                     0x40000081
#define HV_X64_MSR_SIEFP                        0x40000082
#define HV_X64_MSR_SIMP                         0x40000083
#define HV_X64_MSR_EOM                          0x40000084
#define HV_X64_MSR_SINT0                        0x40000090
#define HV_X64_MSR_SINT1                        0x40000091
#define HV_X64_MSR_SINT2                        0x40000092
#define HV_X64_MSR_SINT3                        0x40000093
#define HV_X64_MSR_SINT4                        0x40000094
#define HV_X64_MSR_SINT5                        0x40000095
#define HV_X64_MSR_SINT6                        0x40000096
#define HV_X64_MSR_SINT7                        0x40000097
#define HV_X64_MSR_SINT8                        0x40000098
#define HV_X64_MSR_SINT9                        0x40000099
#define HV_X64_MSR_SINT10                       0x4000009A
#define HV_X64_MSR_SINT11                       0x4000009B
#define HV_X64_MSR_SINT12                       0x4000009C
#define HV_X64_MSR_SINT13                       0x4000009D
#define HV_X64_MSR_SINT14                       0x4000009E
#define HV_X64_MSR_SINT15                       0x4000009F

/*
 * Synthetic Timer MSRs. Four timers per vcpu.
 */

#define HV_X64_MSR_STIMER0_CONFIG               0x400000B0
#define HV_X64_MSR_STIMER0_COUNT                0x400000B1
#define HV_X64_MSR_STIMER1_CONFIG               0x400000B2
#define HV_X64_MSR_STIMER1_COUNT                0x400000B3
#define HV_X64_MSR_STIMER2_CONFIG               0x400000B4
#define HV_X64_MSR_STIMER2_COUNT                0x400000B5
#define HV_X64_MSR_STIMER3_CONFIG               0x400000B6
#define HV_X64_MSR_STIMER3_COUNT                0x400000B7

#define HV_SYNIC_CONTROL_ENABLE                 (1ULL << 0)
#define HV_SYNIC_SIMP_ENABLE                    (1ULL << 0)
#define HV_SYNIC_SIEFP_ENABLE                   (1ULL << 0)
#define HV_SYNIC_SINT_MASKED                    (1ULL << 16)
#define HV_SYNIC_SINT_AUTO_EOI                  (1ULL << 17)
#define HV_SYNIC_SINT_VECTOR_MASK               (0xFF)
#define HV_SYNIC_SINT_COUNT                     16

#define HV_STIMER_ENABLE                (1ULL << 0)
#define HV_STIMER_PERIODIC              (1ULL << 1)
#define HV_STIMER_LAZY                  (1ULL << 2)
#define HV_STIMER_AUTOENABLE            (1ULL << 3)
#define HV_STIMER_DIRECT_MODE           (1ULL << 12)
#define HV_STIMER_SINT(config)          (__u8)(((config) >> 16) & 0x0F)

#define HV_SYNIC_STIMER_COUNT           (4)

/* Define synthetic interrupt controller message constants. */
#define HV_MESSAGE_SIZE                 (256)
#define HV_MESSAGE_PAYLOAD_BYTE_COUNT   (240)
#define HV_MESSAGE_PAYLOAD_QWORD_COUNT  (30)

/* Define hypervisor message types. */
enum hv_message_type {
        HVMSG_NONE                      = 0x00000000,

        /* Memory access messages. */
        HVMSG_UNMAPPED_GPA              = 0x80000000,
        HVMSG_GPA_INTERCEPT             = 0x80000001,

        /* Timer notification messages. */
        HVMSG_TIMER_EXPIRED                     = 0x80000010,

        /* Error messages. */
        HVMSG_INVALID_VP_REGISTER_VALUE = 0x80000020,
        HVMSG_UNRECOVERABLE_EXCEPTION   = 0x80000021,
        HVMSG_UNSUPPORTED_FEATURE               = 0x80000022,

        /* Trace buffer complete messages. */
        HVMSG_EVENTLOG_BUFFERCOMPLETE   = 0x80000040,

        /* Platform-specific processor intercept messages. */
        HVMSG_X64_IOPORT_INTERCEPT              = 0x80010000,
        HVMSG_X64_MSR_INTERCEPT         = 0x80010001,
        HVMSG_X64_CPUID_INTERCEPT               = 0x80010002,
        HVMSG_X64_EXCEPTION_INTERCEPT   = 0x80010003,
        HVMSG_X64_APIC_EOI                      = 0x80010004,
        HVMSG_X64_LEGACY_FP_ERROR               = 0x80010005
};

/* Define synthetic interrupt controller message flags. */
union hv_message_flags {
        uint8_t asu8;
        struct {
                uint8_t msg_pending:1;
                uint8_t reserved:7;
        };
};

union hv_port_id {
        uint32_t asu32;
        struct {
                uint32_t id:24;
                uint32_t reserved:8;
        } u;
};

/* Define port type. */
enum hv_port_type {
        HVPORT_MSG      = 1,
        HVPORT_EVENT            = 2,
        HVPORT_MONITOR  = 3
};

/* Define synthetic interrupt controller message header. */
struct hv_message_header {
        uint32_t message_type;
        uint8_t payload_size;
        union hv_message_flags message_flags;
        uint8_t reserved[2];
        union {
                uint64_t sender;
                union hv_port_id port;
        };
};

/* Define timer message payload structure. */
struct hv_timer_message_payload {
        uint32_t timer_index;
        uint32_t reserved;
        uint64_t expiration_time;       /* When the timer expired */
        uint64_t delivery_time; /* When the message was delivered */
};

/* Define synthetic interrupt controller message format. */
struct hv_message {
        struct hv_message_header header;
        union {
                uint64_t payload[HV_MESSAGE_PAYLOAD_QWORD_COUNT];
        } u;
};

/* Define the synthetic interrupt message page layout. */
struct hv_message_page {
        struct hv_message sint_message[HV_SYNIC_SINT_COUNT];
};

#define HV_EVENT_FLAGS_COUNT	(256 * 8)

struct hv_event_flags {
	ulong flags[HV_EVENT_FLAGS_COUNT / (8 * sizeof(ulong))];
};

struct hv_event_flags_page {
	struct hv_event_flags slot[HV_SYNIC_SINT_COUNT];
};

#define HV_X64_MSR_HYPERCALL_ENABLE             0x1

#define HV_HYPERCALL_FAST               (1u << 16)

#define HVCALL_FLUSH_VIRTUAL_ADDRESS_SPACE      0x02
#define HVCALL_MODIFY_VTL_PROTECTION_MASK       0x0c
#define HVCALL_ENABLE_PARTITION_VTL             0x0d
#define HVCALL_ENABLE_VP_VTL                    0x0f
#define HVCALL_GET_VP_REGISTERS                 0x50
#define HVCALL_SET_VP_REGISTERS                 0x51
#define HVCALL_POST_MESSAGE                     0x5c
#define HVCALL_SIGNAL_EVENT                     0x5d

struct hv_input_post_message {
	u32 connectionid;
	u32 reserved;
	u32 message_type;
	u32 payload_size;
	u64 payload[HV_MESSAGE_PAYLOAD_QWORD_COUNT];
};

static inline bool synic_supported(void)
{
   return cpuid(HYPERV_CPUID_FEATURES).a & HV_X64_MSR_SYNIC_AVAILABLE;
}

static inline bool stimer_supported(void)
{
    return cpuid(HYPERV_CPUID_FEATURES).a & HV_X64_MSR_SYNIC_AVAILABLE;
}

static inline bool hv_time_ref_counter_supported(void)
{
    return cpuid(HYPERV_CPUID_FEATURES).a & HV_X64_MSR_TIME_REF_COUNT_AVAILABLE;
}

void synic_sint_create(u8 sint, u8 vec, bool auto_eoi);
void synic_sint_set(u8 vcpu, u8 sint);
void synic_sint_destroy(u8 sint);
void msg_conn_create(u8 sint, u8 vec, u8 conn_id);
void msg_conn_destroy(u8 sint, u8 conn_id);
void evt_conn_create(u8 sint, u8 vec, u8 conn_id);
void evt_conn_destroy(u8 sint, u8 conn_id);

struct hv_reference_tsc_page {
        uint32_t tsc_sequence;
        uint32_t res1;
        uint64_t tsc_scale;
        int64_t tsc_offset;
};

/* hypercall status code */
#define HV_STATUS_SUCCESS                       0
#define HV_STATUS_INVALID_HYPERCALL_CODE        2
#define HV_STATUS_INVALID_HYPERCALL_INPUT       3
#define HV_STATUS_INVALID_ALIGNMENT             4
#define HV_STATUS_INVALID_PARAMETER             5
#define HV_STATUS_ACCESS_DENIED                 6
#define HV_STATUS_INSUFFICIENT_MEMORY           11
#define HV_STATUS_INVALID_PARTITION_ID          13
#define HV_STATUS_INVALID_VP_INDEX              14
#define HV_STATUS_INVALID_PORT_ID               17
#define HV_STATUS_INVALID_CONNECTION_ID         18
#define HV_STATUS_INSUFFICIENT_BUFFERS          19

#define HV_PARTITION_ID_SELF                    ((u64)-1)
#define HV_VP_INDEX_SELF                        ((u32)-2)

typedef unsigned __attribute__((vector_size(16))) sse128;

typedef union {
    sse128 sse;
    uint32_t l[4];
    uint64_t q[2];
} sse_reg;

enum hyperv_hypercall_flags {
        HYPERV_HYPERCALL_DEFAULT = 0,
        HYPERV_ENABLE_FAST_XMM_CALLS = 0x01,
};

struct hyperv_hypercall_thunk
{
    /* Hypercall code */
    uint16_t code;

    /* Result on return */
    uint64_t result;

    /* Continuation info */
    uint16_t rep_idx;
    uint16_t rep_cnt;

    /* Use fast calling convention */
    bool fast;

    /* Arguments for fast calls (XMM needs to be enabled explicitly) */
    uint64_t arg1;
    uint64_t arg2;
    sse_reg xmm[6];
};

static inline bool hv_xmm_hypercall_input_supported(void)
{
        return cpuid(HYPERV_CPUID_FEATURES).d & HV_X64_HYPERCALL_XMM_INPUT_AVAILABLE;
}

static inline bool hv_xmm_hypercall_output_supported(void)
{
        return cpuid(HYPERV_CPUID_FEATURES).d & HV_X64_HYPERCALL_XMM_OUTPUT_AVAILABLE;
}

static inline bool hv_vsm_supported(void)
{
    uint32_t vsm_mask = HV_X64_ACCESS_VSM | HV_X64_ACCESS_VP_REGISTERS;
    return (cpuid(HYPERV_CPUID_FEATURES).b & vsm_mask) == vsm_mask;
}

/**
 * Setup hyperv hypercall machinery
 */
void *hyperv_setup_hypercall(enum hyperv_hypercall_flags);

/**
 * Free hyperv hypercall resources
 */
void hyperv_teardown_hypercall(void *hypercall_page);

/**
 * Execute a hyperv hypercall described by thunk instance
 */
void hyperv_hypercall(void *hypercall_page, struct hyperv_hypercall_thunk *hc);

#define HV_NUM_VTLS             2
#define HV_INVALID_VTL          ((u8) -1)
#define HV_ALL_VTLS             ((u8) 0xF)

union hv_input_vtl {
    uint8_t as_u8;
    struct {
        u8 target_vtl:4;
        u8 use_target_vtl:1;
        u8 reservedz:3;
    };
};

#define HV_INPUT_VTL(num)       ((union hv_input_vtl) { .target_vtl = (num) & 0xf, .use_target_vtl = 1 })
#define HV_NO_INPUT_VTL         ((union hv_input_vtl) { .as_u8 = 0 })

struct hv_vp_register_val {
    uint64_t low;
    uint64_t high;
};

#define HV_X64_REGISTER_RSP            0x00020004
#define HV_X64_REGISTER_RIP            0x00020010
#define HV_X64_REGISTER_RFLAGS         0x00020011
#define HV_X64_REGISTER_CR0            0x00040000
#define HV_X64_REGISTER_CR3            0x00040002
#define HV_X64_REGISTER_CR4            0x00040003
#define HV_X64_REGISTER_CR8            0x00040004
#define HV_X64_REGISTER_DR7            0x00050005
#define HV_X64_REGISTER_IDTR           0x00070000
#define HV_X64_REGISTER_GDTR           0x00070001
#define HV_X64_REGISTER_EFER           0x00080001
#define HV_X64_REGISTER_APIC_BASE      0x00080003
#define HV_X64_REGISTER_SYSENTER_CS    0x00080005
#define HV_X64_REGISTER_SYSENTER_EIP   0x00080006
#define HV_X64_REGISTER_SYSENTER_ESP   0x00080007
#define HV_X64_REGISTER_STAR           0x00080008
#define HV_X64_REGISTER_LSTAR          0x00080009
#define HV_X64_REGISTER_CSTAR          0x0008000A
#define HV_X64_REGISTER_SFMASK         0x0008000B
#define HV_X64_REGISTER_TSC_AUX        0x0008007B

struct hv_get_set_vp_registers {
        uint64_t partition_id;
        uint32_t vp_index;
        union hv_input_vtl input_vtl;
        uint8_t padding[3];
} __attribute__((packed));

#define HV_FLUSH_ALL_PROCESSORS             0x00000001

struct hv_tlb_flush {
    uint64_t address_space;
    uint32_t flags;
    uint32_t _padding;
    uint64_t processor_mask;
} __attribute__((packed));

union hv_enable_partition_vtl_flags {
        uint8_t as_u8;
        struct {
                uint8_t enable_mbec:1;
                uint8_t reserved:7;
        } __attribute__((packed));
};

struct hv_enable_partition_vtl {
        uint64_t target_partition_id;
        uint8_t target_vtl;
        union hv_enable_partition_vtl_flags flags;
        uint8_t reserved[6];
} __attribute__((packed));

struct hv_x64_segment_register {
        uint64_t base;
        uint32_t limit;
        uint16_t selector;
        union {
                struct {
                        uint16_t type:4;
                        uint16_t s:1;
                        uint16_t dpl:2;
                        uint16_t p:1;
                        uint16_t reserved:4;
                        uint16_t avail:1;
                        uint16_t l:1;
                        uint16_t db:1;
                        uint16_t g:1;
                };
                uint16_t attributes;
        };
};

struct hv_x64_table_register {
        uint16_t _pad[3];
        uint16_t limit;
        uint64_t base;
};

/* Initial arch execution context set for VTL of a specific VP by HvEnableVpVtl */
struct hv_initial_vp_context {
        uint64_t rip;
        uint64_t rsp;
        uint64_t rflags;

        struct hv_x64_segment_register cs;
        struct hv_x64_segment_register ds;
        struct hv_x64_segment_register es;
        struct hv_x64_segment_register fs;
        struct hv_x64_segment_register gs;
        struct hv_x64_segment_register ss;
        struct hv_x64_segment_register tr;
        struct hv_x64_segment_register ldtr;

        struct hv_x64_table_register idtr;
        struct hv_x64_table_register gdtr;

        uint64_t efer;
        uint64_t cr0;
        uint64_t cr3;
        uint64_t cr4;
        uint64_t msr_cr_pat;
} __attribute__((packed));

struct hv_enable_vp_vtl {
        uint64_t target_partition_id;
        uint32_t vp_index;
        uint8_t target_vtl;
        uint8_t reserved[3];
        struct hv_initial_vp_context vp_vtl_context;
} __attribute__((packed));

/* VTL per-GPA protection bits */
#define HV_VTL_PROT_READ BIT(0)
#define HV_VTL_PROT_WRITE BIT(1)
#define HV_VTL_PROT_EXEC BIT(3)

/* PROT_EXEC is split into 2 when MBEC is enabled */
#define HV_VTL_PROT_UMX BIT(2)
#define HV_VTL_PROT_KMX BIT(3)

#define HV_DEFAULT_VTL_PROT_MASK (HV_VTL_PROT_READ | HV_VTL_PROT_WRITE | HV_VTL_PROT_EXEC)

struct hv_modify_vtl_protection_mask {
        uint64_t target_partition_id;
        uint32_t map_flags;
        union hv_input_vtl input_vtl;
        uint8_t reserved[3];
} __attribute__((packed));

/* Intecept message header */
struct hv_intercept_header {
        uint32_t vp_index;
        uint8_t instruction_length;
#define HV_INTERCEPT_ACCESS_READ    0
#define HV_INTERCEPT_ACCESS_WRITE   1
#define HV_INTERCEPT_ACCESS_EXECUTE 2
        uint8_t access_type_mask;
        union {
                uint16_t as_u16;
                struct {
                        uint16_t cpl:2;
                        uint16_t cr0_pe:1;
                        uint16_t cr0_am:1;
                        uint16_t efer_lma:1;
                        uint16_t debug_active:1;
                        uint16_t interruption_pending:1;
                        uint16_t reserved:9;
                };
        } exec_state;
        struct hv_x64_segment_register cs;
        uint64_t rip;
        uint64_t rflags;
} __attribute__((packed));

union hv_x64_memory_access_info {
	uint8_t as_u8;
	struct {
		uint8_t gva_valid:1;
		uint8_t _reserved:7;
	};
};

struct hv_memory_intercept_message {
	struct hv_intercept_header header;
#define HV_X64_CACHE_TYPE_UNCACHED       0
#define HV_X64_CACHE_TYPE_WRITECOMBINING 1
#define HV_X64_CACHE_TYPE_WRITETHROUGH   4
#define HV_X64_CACHE_TYPE_WRITEPROTECTED 5
#define HV_X64_CACHE_TYPE_WRITEBACK      6
	uint32_t cache_type;
	uint8_t instruction_byte_count;
	union hv_x64_memory_access_info memory_access_info;
	uint16_t _reserved;
	uint64_t gva;
	uint64_t gpa;
	uint8_t instruction_bytes[16];
	struct hv_x64_segment_register ds;
	struct hv_x64_segment_register ss;
	uint64_t rax;
	uint64_t rcx;
	uint64_t rdx;
	uint64_t rbx;
	uint64_t rsp;
	uint64_t rbp;
	uint64_t rsi;
	uint64_t rdi;
	uint64_t r8;
	uint64_t r9;
	uint64_t r10;
	uint64_t r11;
	uint64_t r12;
	uint64_t r13;
	uint64_t r14;
	uint64_t r15;
} __attribute__((packed));

#define HV_REGISTER_VSM_CODE_PAGE_OFFSETS       0x000D0002
#define HV_REGISTER_VSM_VP_STATUS               0x000D0003
#define HV_REGISTER_VSM_PARTITION_STATUS        0x000D0004
#define HV_REGISTER_VSM_CAPABILITIES            0x000D0006
#define HV_REGISTER_VSM_PARTITION_CONFIG        0x000D0007
#define HV_REGISTER_VSM_VP_SECURE_CONFIG_VTL0   0x000D0010

/* VTL call/return hypercall page offsets register */
union hv_register_vsm_code_page_offsets {
        u64 as_u64;
        struct {
                u64 vtl_call_offset:12;
                u64 vtl_return_offset:12;
                u64 reserved:40;
        } __attribute__((packed));
};

/* Partition-wide VSM status */
union hv_register_vsm_partition_status {
        uint64_t as_u64;
        struct {
                uint64_t enabled_vtl_set:16;
                uint64_t maximum_vtl:4;
                uint64_t mbec_enabled_vtl_set:16;
                uint64_t reserved:28;
        } __attribute__((packed));
};

/* Per-VCPU VSM status */
union hv_register_vsm_vp_status {
        uint64_t as_u64;
        struct {
                uint64_t active_vtl:4;
                uint64_t active_mbec_enabled:1;
                uint64_t reserved0:11;
                uint64_t enabled_vtl_set:16;
                uint64_t reserved1:32;
        } __attribute__((packed));
};

/* Advertised partition VSM capabilities */
union hv_register_vsm_capabilities {
        uint64_t as_u64;
        struct {
                uint64_t reserved:46;
                uint64_t deny_lower_vtl_startup:1;
                uint64_t mbec_vtl_mask:16;
                uint64_t dr6_shared:1;
        } __attribute__((packed));
};


union hv_register_vsm_partition_config {
        uint64_t as_u64;
        struct {
                uint64_t enable_vtl_protection:1;
                uint64_t default_vtl_protection_mask:4;
                uint64_t zero_memory_on_reset:1;
                uint64_t deny_lower_vtl_startup:1;
                uint64_t reserved0:2;
                uint64_t intercept_vp_startup:1;
                uint64_t reserved1:54;
        } __attribute__((packed));
};

/* Partition per-VTL VSM configs to configure lower VTLs */
union hv_register_vsm_vp_secure_vtl_config {
        uint64_t as_u64;
        struct {
                uint64_t mbec_enabled:1;
                uint64_t tlb_locked:1;
                uint64_t reserved0:62;
        } __attribute__((packed));
};

#endif
