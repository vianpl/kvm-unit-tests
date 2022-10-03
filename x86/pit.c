#include "libcflat.h"
#include "processor.h"
#include "isr.h"
#include "apic.h"
#include "asm/io.h"

#define PIT_IRQ_VECTOR		8

// TODO comment
#define PIT_TIMEOUT_COUNT	1000000000U

enum pit_port {
	PIT_PORT_CHANNEL_0 = 0x40,
	PIT_PORT_CHANNEL_1 = 0x41,
	PIT_PORT_CHANNEL_2 = 0x42,
	PIT_PORT_COMMAND = 0x43
};

enum pit_mode {
	PIT_MODE_ONESHOT = 0,
	PIT_MODE_HW_ONESHOT,
	PIT_MODE_RATE_GENERATOR,
	PIT_MODE_SQUARE_WAVE_GENERATOR,
	PIT_MODE_SW_TRIGGERED_STROBE,
	PIT_MODE_HW_TRIGGERED_STROBE
};

enum pit_access {
	PIT_ACCESS_LATCH_COUNT = 0,
	PIT_ACCESS_LOBYTE,
	PIT_ACCESS_HIBYTE,
	PIT_ACCESS_LOHIBYTE
};

enum pit_channel {
	PIT_CHANNEL_0 = 0,
	PIT_CHANNEL_1,
	PIT_CHANNEL_2,
	PIT_READBACK
};

static inline u8 pit_fmt(enum pit_mode mode, enum pit_access access,
			 enum pit_channel channel)
{
	return (u8)(mode << 1 | access << 4 | channel << 6);
}

static inline void pit_outb(uint8_t value, unsigned long port)
{
    //printf("outb port: 0x%lx, value 0x%x\n", port, value);
    asm volatile("outb %b0, %w1" : : "a"(value), "Nd"((unsigned short)port));
}

static inline uint8_t pit_inb(unsigned long port)
{
    unsigned char value;
    asm volatile("inb %w1, %0" : "=a" (value) : "Nd" ((unsigned short)port));
    return value;
}

static inline void pit_cmd(enum pit_mode mode, enum pit_access access,
			   enum pit_channel channel)
{
	pit_outb(pit_fmt(mode, access, channel), PIT_PORT_COMMAND);
}

static inline void pit_write_channel(u16 val, enum pit_access access,
				     enum pit_port port)
{
	if (access == PIT_ACCESS_LOBYTE || access == PIT_ACCESS_LOHIBYTE)
		pit_outb(val & 0xff, port);

	if (access == PIT_ACCESS_HIBYTE || access == PIT_ACCESS_LOHIBYTE)
		pit_outb(val >> 8, port);
}

static inline void pit_eoi(void)
{
	outb(0x20, 0x20);
}

static inline u8 pit_get_status(enum pit_channel channel)
{
	pit_outb(0xE0 | 2 << channel, PIT_PORT_COMMAND);
	return pit_inb(PIT_PORT_CHANNEL_0 + channel);
}

static inline u16 pit_get_cycles_random(void)
{
	if (this_cpu_has(X86_FEATURE_RDRAND))
		return rdrand();

	return rdtsc();
}

static volatile unsigned int nirqs = 0;
static volatile u64 irq_tsc = 0;

static inline void pit_nirqs_snap(unsigned int *snap)
{
	*snap = nirqs;
}

static void pit_irq_handler(isr_regs_t *regs)
{
	irq_tsc = rdtsc();
	nirqs++;
	printf("nirqs %u\n", nirqs);
	pit_eoi();
}

#define pit_timeout(_condition) ({		\
		unsigned int n = PIT_TIMEOUT_COUNT;	\
							\
		while (!(_condition) && --n);		\
							\
		(_condition) && n;			\
	})

static void pit_oneshot_test(enum pit_mode mode)
{
	unsigned int snap;

	/* pit_nirqs_snap(&snap); */

	/* report(!pit_timeout(nirqs != snap), "no spurious interrupts"); */
	/* printf("nirqs %u\n", nirqs); */

	pit_nirqs_snap(&snap);
	pit_cmd(mode, PIT_ACCESS_LOHIBYTE, PIT_CHANNEL_0);
	pit_write_channel(pit_get_cycles_random(), PIT_ACCESS_LOHIBYTE, PIT_PORT_CHANNEL_0);
	irq_enable();
	report(pit_timeout(nirqs == (snap + 1)), "timer interrupt");

	/*
	 * Wait for a while and check whether we didn't get interrupted
	 * multiple times.
	 */
	report(!pit_timeout(nirqs > (snap + 1)), "single interrupt test");

	irq_disable();
}

static void pit_periodic_test(enum pit_mode mode, int ntests)
{
	unsigned int snap;

	/* report(!pit_timeout(nirqs != snap), "no spurious interrupts"); */

	pit_nirqs_snap(&snap);
	pit_cmd(mode, PIT_ACCESS_LOHIBYTE, PIT_CHANNEL_0);
	pit_write_channel(pit_get_cycles_random(), PIT_ACCESS_LOHIBYTE, PIT_PORT_CHANNEL_0);
	irq_enable();

	while (ntests--) {
		if (!pit_timeout(nirqs == (snap + 1))) {
			report_fail("no periodic interrupts received");
			return;
		}

		pit_nirqs_snap(&snap);
	}

	irq_disable();
	report_pass("PIT peridic test");
}

static void pit_init(void)
{
	handle_irq(PIT_IRQ_VECTOR, pit_irq_handler);

	/* Unmask PIT interrupt in PIC */
	outb(0xFE, 0x21);

	return;
}

int main(void)
{
	setup_vm();
	pit_init();

	report_prefix_push("PIT mode 0, oneshot");
	pit_oneshot_test(PIT_MODE_ONESHOT);
	report_prefix_pop();

	//TODO Check documentation? HW_ONESHOT shouldn't work on channel 0...
	report_prefix_push("PIT mode 1, HW oneshot");
	pit_oneshot_test(PIT_MODE_HW_ONESHOT);
	report_prefix_pop();

	report_prefix_push("PIT mode 2, rate generator");
	pit_periodic_test(PIT_MODE_RATE_GENERATOR, 100);
	report_prefix_pop();

	report_prefix_push("PIT mode 3, square wave generator");
	pit_periodic_test(PIT_MODE_SQUARE_WAVE_GENERATOR, 100);
	report_prefix_pop();

	report_prefix_push("PIT mode 4, SW triggered strobe");
	pit_oneshot_test(PIT_MODE_ONESHOT);
	report_prefix_pop();

	report_prefix_push("PIT mode 5, HW triggered strobe");
	pit_periodic_test(PIT_MODE_SQUARE_WAVE_GENERATOR, 100);
	report_prefix_pop();

	/*
	TODO clock uniformity
	t2 = pit_oneshot_test(PIT_MODE_ONESHOT, 0x2000);
	tmp = t2 / (t1 / 1000);
	report(tmp >= 1900 && tmp <= 2100, "PIT uniform count");
	*/

	return report_summary();
}
