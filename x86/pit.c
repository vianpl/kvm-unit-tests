#include "libcflat.h"
#include "processor.h"
#include "isr.h"
#include "apic.h"
#include "bitops.h"
#include "asm/io.h"

#define PIC1_DATA		0x21

#define PIT_IRQ_VECTOR		8

enum pit_ioport {
	PIT_IOPORT_CHANNEL0 = 0x40,
	PIT_IOPORT_CHANNEL1 = 0x41,
	PIT_IOPORT_CHANNEL2 = 0x42,
	PIT_IOPORT_COMMAND = 0x43
};

enum pit_mode {
	PIT_CMD_ONESHOT = 0x0,
	PIT_CMD_HW_ONESHOT = 0x1 << 1,
	PIT_CMD_RATE_GENERATOR = 0x2 << 1,
	PIT_CMD_SQUARE_WAVE_GENERATOR = 0x3 << 1,
	PIT_CMD_SW_TRIGGERED_STROBE = 0x4 << 1,
	PIT_CMD_HW_TRIGGERED_STROBE = 0x5 << 1,
	/*
	 * The mode is expected to fall within 0-5; however, 6(0b110) and
	 * 7(0b111) are also possible according to the spec. "Mode 6"
	 * corresponds to "Mode 2", whereas "Mode 7" corresponds to "Mode 3".
	 */
	PIT_CMD_RATE_GENERATOR_BIS = 0x6 << 1,
	PIT_CMD_SQUARE_WAVE_GENERATOR_BIS = 0x7 << 1
};

enum pit_access {
	PIT_CMD_LATCH_COUNT = 0x0,
	PIT_CMD_LOBYTE = 0x1 << 4,
	PIT_CMD_HIBYTE = 0x2 << 4,
	PIT_CMD_LOHIBYTE = 0x3 << 4,
};

enum pit_channel {
	PIT_CMD_CHANNEL0 = 0x0,
	PIT_CMD_CHANNEL1 = 0x1 << 6,
	PIT_CMD_CHANNEL2 = 0x2 << 6,
	PIT_CMD_READBACK = 0x3 << 6
};

static inline void pit_cmd(enum pit_mode mode, enum pit_access access,
			   enum pit_channel channel)
{
	outb(mode | access | channel, PIT_IOPORT_COMMAND);
}

static inline void pit_write_channel(u16 val, enum pit_access access,
				     enum pit_ioport port)
{
	if (access == PIT_CMD_LOBYTE || access == PIT_CMD_LOHIBYTE)
		outb(val & 0xff, port);

	if (access == PIT_CMD_HIBYTE || access == PIT_CMD_LOHIBYTE)
		outb(val >> 8, port);
}

static inline void pit_eoi(void)
{
	outb(0x20, 0x20);
}

static inline void pit_channel2_gate(u8 val)
{
	if (val)
		outb(inb(0x61) | 0x01, 0x61);
	else
		outb(inb(0x61) & ~0x01, 0x61);
}

static inline u8 pit_get_status(enum pit_channel channel)
{
	/* 0x20 -> don't latch count */
	outb(PIT_CMD_READBACK | 0x20 | 2 << (channel >> 6), PIT_IOPORT_COMMAND);
	return inb(PIT_IOPORT_CHANNEL0 + (channel >> 6));
}

static inline u16 pit_get_count(enum pit_access access, enum pit_ioport port)
{
	u16 count = 0;

	if (access == PIT_CMD_LOBYTE || access == PIT_CMD_LOHIBYTE)
		count |= inb(port);

	if (access == PIT_CMD_HIBYTE || access == PIT_CMD_LOHIBYTE)
		count |= inb(port) << 8;

	return count;
}

static inline void pit_get_status_multiple(u8 *status0, u8 *status1, u8 *status2)
{
	u8 channel_mask = 0;

	if (status0)
		channel_mask |= BIT(1);
	if (status1)
		channel_mask |= BIT(2);
	if (status2)
		channel_mask |= BIT(3);

	/* 0x20 -> don't latch count */
	outb(PIT_CMD_READBACK | 0x20 | channel_mask, PIT_IOPORT_COMMAND);

	if (status0)
		*status0 = pit_get_count(PIT_CMD_LOBYTE, PIT_IOPORT_CHANNEL0);
	if (status1)
		*status1 = pit_get_count(PIT_CMD_LOBYTE, PIT_IOPORT_CHANNEL1);
	if (status2)
		*status2 = pit_get_count(PIT_CMD_LOBYTE, PIT_IOPORT_CHANNEL2);
}

static inline u16 pit_get_count_latch(enum pit_access access, enum pit_channel channel)
{
	outb(PIT_CMD_LATCH_COUNT | channel, PIT_IOPORT_COMMAND);
	return pit_get_count(access, PIT_IOPORT_CHANNEL0 + (channel >> 6));
}

static inline u16 pit_get_count_readback(enum pit_access access, enum pit_channel channel)
{
	/* 0x10 -> don't latch status */
	outb(PIT_CMD_READBACK | 0x10 | 2 << channel, PIT_IOPORT_COMMAND);
	return pit_get_count(access, PIT_IOPORT_CHANNEL0 + (channel >> 6));
}

static inline void pit_get_count_readback_multiple(u16 *count0, u16 *count1, u16 *count2)
{
	u8 status0, status1, status2;
	u8 channel_mask = 0;

	pit_get_status_multiple(&status0, &status1, &status2);

	if (count0)
		channel_mask |= BIT(1);
	if (count1)
		channel_mask |= BIT(2);
	if (count2)
		channel_mask |= BIT(3);

	/* 0x10 -> don't latch status */
	outb(PIT_CMD_READBACK| 0x10 | channel_mask, PIT_IOPORT_COMMAND);

	if (count0)
		*count0 = pit_get_count(status0 & 0x30, PIT_IOPORT_CHANNEL0);
	if (count1)
		*count1 = pit_get_count(status1 & 0x30, PIT_IOPORT_CHANNEL1);
	if (count2)
		*count2 = pit_get_count(status2 & 0x30, PIT_IOPORT_CHANNEL2);
}

static inline u16 pit_get_cycles_random(void)
{
	if (this_cpu_has(X86_FEATURE_RDRAND))
		return rdrand();

	return rdtsc();
}

static volatile unsigned int nirqs = 0;

static void pit_irq_handler(isr_regs_t *regs)
{
	nirqs++;
	pit_eoi();
}

static inline void pit_nirqs_snap(unsigned int *snap)
{
	*snap = nirqs;
}

#define _pit_timeout(_condition, _count) ({		\
		unsigned int n = _count;		\
							\
		while (!(_condition) && --n);		\
							\
		(_condition) && n;			\
	})
#define pit_timeout(_condition)		_pit_timeout(_condition, -1U)
/* polling ioports is slow, so use a shorter timeout */
#define pit_timeout_slow(_condition)	_pit_timeout(_condition, 100000)

static void test_oneshot_irq(enum pit_mode mode)
{
	unsigned int snap;

	report_prefix_pushf("PIT IRQ, mode %u", mode >> 1);
	irq_enable();

	pit_nirqs_snap(&snap);
	pit_cmd(mode, PIT_CMD_LOHIBYTE, PIT_CMD_CHANNEL0);
	pit_write_channel(pit_get_cycles_random(), PIT_CMD_LOHIBYTE, PIT_IOPORT_CHANNEL0);
	report(pit_timeout(nirqs == (snap + 1)), "oneshot interrupt");

	pit_nirqs_snap(&snap);

	/*
	 * Reprogram channel multiple times, should restart the timer but only
	 * interrupt once
	 */
	pit_write_channel(0xffff, PIT_CMD_LOHIBYTE, PIT_IOPORT_CHANNEL0);
	pit_write_channel(pit_get_cycles_random(), PIT_CMD_LOHIBYTE, PIT_IOPORT_CHANNEL0);
	report(pit_timeout(nirqs == (snap + 1)), "subsequent oneshot interrupt");

	/*
	 * Wait for a while and check whether we didn't get interrupted
	 * multiple times.
	 */
	report(!pit_timeout(nirqs > (snap + 1)), "no spurious interrupt");

	irq_disable();
	report_prefix_pop();
}

static void test_periodic_irq(enum pit_mode mode)
{
	unsigned int snap;

	report_prefix_pushf("PIT IRQ, mode %u", mode >> 1);
	irq_enable();

	pit_nirqs_snap(&snap);
	pit_cmd(mode, PIT_CMD_LOHIBYTE, PIT_CMD_CHANNEL0);
	pit_write_channel(pit_get_cycles_random(), PIT_CMD_LOHIBYTE, PIT_IOPORT_CHANNEL0);

	for (int n = 100; --n;) {
		if (!pit_timeout(nirqs == (snap + 1))) {
			report_fail("periodic interrupts");
			goto exit;
		}
		pit_nirqs_snap(&snap);
	}

	report_pass("periodic interrupts");
exit:
	irq_disable();
	report_prefix_pop();
}

static void test_status(void)
{
	u8 status1, status2, status0;

	report_prefix_push("PIT status");

	pit_cmd(PIT_CMD_SW_TRIGGERED_STROBE, PIT_CMD_LOHIBYTE, PIT_CMD_CHANNEL0);
	pit_cmd(PIT_CMD_SQUARE_WAVE_GENERATOR, PIT_CMD_LOBYTE, PIT_CMD_CHANNEL1);
	pit_cmd(PIT_CMD_HW_ONESHOT, PIT_CMD_HIBYTE, PIT_CMD_CHANNEL2);

	status0 = pit_get_status(PIT_CMD_CHANNEL0);
	if (!((status0 & 0x3e) == (PIT_CMD_SW_TRIGGERED_STROBE | PIT_CMD_LOHIBYTE))) {
		report_fail("Channel 0 status read: status 0x%x, expected 0x%x",
			    status0, (PIT_CMD_SW_TRIGGERED_STROBE | PIT_CMD_LOHIBYTE));
		goto exit;
	}

	status1 = pit_get_status(PIT_CMD_CHANNEL1);
	if (!((status1 & 0x3e) == (PIT_CMD_SQUARE_WAVE_GENERATOR | PIT_CMD_LOBYTE))) {
		report_fail("Channel 1 status read: status 0x%x, expected 0x%x",
			    status1, (PIT_CMD_SQUARE_WAVE_GENERATOR | PIT_CMD_LOBYTE));
		goto exit;
	}

	status2 = pit_get_status(PIT_CMD_CHANNEL2);
	if (!((status2 & 0x3e) == (PIT_CMD_HW_ONESHOT | PIT_CMD_HIBYTE))) {
	       report_fail("Channel 2 status read: status 0x%x, expected 0x%x",
			   status2, (PIT_CMD_HW_ONESHOT | PIT_CMD_HIBYTE));
		goto exit;
	}

	pit_get_status_multiple(&status0, &status1, &status2);
	if (!((status0 & 0x3e) == (PIT_CMD_SW_TRIGGERED_STROBE | PIT_CMD_LOHIBYTE))) {
		report_fail("Channel 0 status read: status 0x%x, expected 0x%x",
			    status0, (PIT_CMD_SW_TRIGGERED_STROBE | PIT_CMD_LOHIBYTE));
		goto exit;
	}

	if (!((status1 & 0x3e) == (PIT_CMD_SQUARE_WAVE_GENERATOR | PIT_CMD_LOBYTE))) {
		report_fail("Channel 1 status read: status 0x%x, expected 0x%x",
			    status1, (PIT_CMD_SQUARE_WAVE_GENERATOR | PIT_CMD_LOBYTE));
		goto exit;
	}

	if (!((status2 & 0x3e) == (PIT_CMD_HW_ONESHOT | PIT_CMD_HIBYTE))) {
	       report_fail("Channel 2 status read: status 0x%x, expected 0x%x",
			   status2, (PIT_CMD_HW_ONESHOT | PIT_CMD_HIBYTE));
		goto exit;
	}

	report_pass("readback");
	report_pass("readback multiple");

exit:
	report_prefix_pop();
}

static void test_count(void)
{
	u16 cycles0 = pit_get_cycles_random();
	u16 cycles1 = pit_get_cycles_random() & 0xff00;
	u16 cycles2 = pit_get_cycles_random() & 0xff;
	u16 count0, count1, count2 = 0;
	int i;

	report_prefix_push("PIT count");

	/* Configure the channels in parallel, it shouldn't confuse the PIC. */
	pit_cmd(PIT_CMD_RATE_GENERATOR, PIT_CMD_LOHIBYTE, PIT_CMD_CHANNEL0);
	pit_cmd(PIT_CMD_SQUARE_WAVE_GENERATOR, PIT_CMD_HIBYTE, PIT_CMD_CHANNEL1);
	pit_cmd(PIT_CMD_SQUARE_WAVE_GENERATOR, PIT_CMD_LOBYTE, PIT_CMD_CHANNEL2);
	pit_write_channel(cycles0, PIT_CMD_LOHIBYTE, PIT_IOPORT_CHANNEL0);
	pit_write_channel(cycles1, PIT_CMD_HIBYTE, PIT_IOPORT_CHANNEL1);
	pit_write_channel(cycles2, PIT_CMD_LOBYTE, PIT_IOPORT_CHANNEL2);

	/*
	 * Take into account that using 0 cycles is possible, and will count
	 * '0xffff + 1', '0xff00 + 1' or '0xff + 1' ticks respectively.
	 */
	cycles0 = cycles0 ? : 0xffff;
	cycles1 = cycles1 ? : 0xff00;
	cycles2 = cycles2 ? : 0xff;

	for (i = 0; i < 10000; i++) {
		/* Latch multiple channels */
		count0 = pit_get_count_latch(PIT_CMD_LOHIBYTE, PIT_CMD_CHANNEL0);
		count1 = pit_get_count_latch(PIT_CMD_HIBYTE, PIT_CMD_CHANNEL1);
		count2 = pit_get_count_latch(PIT_CMD_LOBYTE, PIT_CMD_CHANNEL2);
		if (count0 > cycles0 || count1 > cycles1 || count2 > cycles2) {
			report_fail("unexpected count value using latch\n"
				    "\tcycles: 0x%x 0x%x 0x%x, count: 0x%x 0x%x 0x%x",
				     cycles0, cycles1, cycles2, count0, count1, count2);
			goto exit;
		}

		/* Latch channels one by one using readback */
		count0 = pit_get_count_readback(PIT_CMD_LOHIBYTE, PIT_CMD_CHANNEL0);
		count1 = pit_get_count_readback(PIT_CMD_HIBYTE, PIT_CMD_CHANNEL1);
		count2 = pit_get_count_readback(PIT_CMD_LOBYTE, PIT_CMD_CHANNEL2);
		if (count0 > cycles0 || count1 > cycles1 || count2 > cycles2) {
			report_fail("unexpected count value using single readback\n"
				    "\tcycles: 0x%x 0x%x 0x%x, count: 0x%x 0x%x 0x%x",
				     cycles0, cycles1, cycles2, count0, count1, count2);
			goto exit;
		}

		/* Latch multiple channels using readback */
		pit_get_count_readback_multiple(&count0, &count1, &count2);
		if (count0 > cycles0 || count1 > cycles1 || count2 > cycles2) {
			report_fail("unexpected count value using multiple readback\n"
				    "\tcycles: 0x%x 0x%x 0x%x, count: 0x%x 0x%x 0x%x",
				     cycles0, cycles1, cycles2, count0, count1, count2);
			goto exit;
		}

		/*
		 * Unlatched count read, only use high bits from lohibyte
		 * configs.
		 */
		count0 = pit_get_count(PIT_CMD_LOHIBYTE, PIT_IOPORT_CHANNEL0) & 0xFF00;
		count1 = pit_get_count(PIT_CMD_HIBYTE, PIT_IOPORT_CHANNEL1);
		count2 = pit_get_count(PIT_CMD_LOBYTE, PIT_IOPORT_CHANNEL2);
		if (count0 > cycles0 || count1 > cycles1 || count2 > cycles2) {
			report_fail("unexpected count value using unlatched read\n"
				    "\tcycles: 0x%x 0x%x 0x%x, count: 0x%x 0x%x 0x%x",
				     cycles0, cycles1, cycles2, count0, count1, count2);
			return;
		}
	}

	report_pass("latch read");
	report_pass("readback read");
	report_pass("multiple channel readback read");
	report_pass("unlatched read");

exit:
	report_prefix_pop();
}

static void test_oneshot_channel2(enum pit_mode mode)
{
	u16 cycles = pit_get_cycles_random();

	report_prefix_pushf("PIT channel 2, mode %u", mode >> 1);

	pit_channel2_gate(0);

	pit_cmd(mode, PIT_CMD_LOHIBYTE, PIT_CMD_CHANNEL2);

	if (mode == PIT_CMD_HW_ONESHOT)
		report(!pit_timeout_slow(!(inb(0x61) & 0x20)),
		"output bit high while configured and ungated");

	if (mode == PIT_CMD_ONESHOT)
		report(!pit_timeout_slow(inb(0x61) & 0x20),
		"output bit low while configured and ungated");

	/*
	 * Force a high value in mode 1 to be able to observe the high output
	 * bit after enabling the gate.
	 */
	if (mode == PIT_CMD_HW_ONESHOT)
		cycles |= 0x8000;

	pit_write_channel(cycles, PIT_CMD_LOHIBYTE, PIT_IOPORT_CHANNEL2);

	pit_channel2_gate(1);

	if (mode == PIT_CMD_HW_ONESHOT)
		report(!(inb(0x61) & 0x20), "output bit low while counting");

	report(pit_timeout_slow(inb(0x61) & 0x20), "output bit high when done counting");

	report_prefix_pop();
}

static void test_periodic_channel2(enum pit_mode mode)
{
	u16 cycles = pit_get_cycles_random();

	report_prefix_pushf("PIT channel 2, mode %u", mode >> 1);

	pit_channel2_gate(0);

	pit_cmd(mode, PIT_CMD_LOHIBYTE, PIT_CMD_CHANNEL2);

	report(!pit_timeout_slow(!(inb(0x61) & 0x20)),
	       "output bit high while configured and ungated");

	pit_write_channel(cycles, PIT_CMD_LOHIBYTE, PIT_IOPORT_CHANNEL2);

	pit_channel2_gate(1);

	/* Observe the output low, then high */
	report(pit_timeout_slow(!(inb(0x61) & 0x20)), "output switch while counting");
	report(pit_timeout_slow(inb(0x61) & 0x20), "output switch while counting");

	report_prefix_pop();
}

static void pit_init(void)
{
	handle_irq(PIT_IRQ_VECTOR, pit_irq_handler);

	/* Unmask timer interrupt */
	outb(0xfe, 0x21);

	/* Channel 2: set the Gate high, disable speaker */
	outb((inb(0x61) & ~0x02) | 0x01, 0x61);
}

int main(void)
{
	setup_vm();
	pit_init();

	/* Bypass mode 1 and 5, they requires controlling the input gate. */
	test_oneshot_irq(PIT_CMD_ONESHOT);
	test_periodic_irq(PIT_CMD_RATE_GENERATOR);
	test_periodic_irq(PIT_CMD_SQUARE_WAVE_GENERATOR);
	test_oneshot_irq(PIT_CMD_SW_TRIGGERED_STROBE);
	test_periodic_irq(PIT_CMD_RATE_GENERATOR_BIS);
	test_periodic_irq(PIT_CMD_SQUARE_WAVE_GENERATOR_BIS);

	test_status();
	test_count();

	/*
	 * Channel 2's gate and output value are available at bits 0 and 5 of
	 * I/O port 0x61. Mode 2, 4 and 5 aren't tested as the output is only
	 * switched for one cycle, which is too short to be consistently
	 * observed on virtualized systems.
	 */
	test_oneshot_channel2(PIT_CMD_ONESHOT);
	test_oneshot_channel2(PIT_CMD_HW_ONESHOT);
	test_periodic_channel2(PIT_CMD_SQUARE_WAVE_GENERATOR);
	test_periodic_channel2(PIT_CMD_SQUARE_WAVE_GENERATOR_BIS);

	return report_summary();
}
