/*-
 * KVM pvclock (based on tsc implementation)
 *
 * Copyright (c) 1998-2003 Takeshi HASEGAWA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_clock.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/malloc.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include <sys/timetc.h>
#include <sys/kernel.h>
#include <sys/power.h>
#include <sys/smp.h>
#include <machine/clock.h>
#include <machine/md_var.h>
#include <machine/specialreg.h>
#include <sys/timetc.h> /* timecounter */
#include <machine/cpufunc.h> /* wrmsr*/
#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h> /* vtophys */

#include "cpufreq_if.h"

uint64_t	kvmclock_freq;
int		kvmclock_is_broken;
int		kvmclock_is_invariant;

uint64_t	kvmclock_last_timestamp = 0;
uint64_t	kvmclock_log_count=100;

#define KVM_CPUID_MAGIC             0x40000000
#define KVM_CPUID_FLAGS             0x40000001
#define KVM_CPUID_FLAGS_PVCLOCK     0x00000001

//static eventhandler_tag kvmclock_levels_tag, kvmclock_pre_tag, kvmclock_post_tag;

//SYSCTL_INT(_kern_timecounter, OID_AUTO, invariant_kvmclock, CTLFLAG_RDTUN,
//    &kvmclock_is_invariant, 0, "Indicates whether the kvmclock is P-state invariant");
//TUNABLE_INT("kern.timecounter.invariant_kvmclock", &kvmclock_is_invariant);

static	unsigned kvmclock_get_timecount(struct timecounter *tc);

static struct timecounter kvmclock_timecounter = {
	kvmclock_get_timecount,	/* get_timecount */
	0,			/* no poll_pps */
	~0u,			/* counter_mask */
	0,			/* frequency */
	"kvmclock",			/* name */
//	800,			/* quality (adjusted in code) */
	1000,			/* quality (adjusted in code) */
};

#define MSR_KVM_WALL_CLOCK_NEW		0x4b564d00
#define MSR_KVM_SYSTEM_TIME_NEW		0x4b564d01
#define MSR_KVM_SYSTEM_TIME		0x12
#define MSR_KVM_WALL_CLOCK              0x11

struct kvmclock_system_time_new {
	uint32_t version;
	uint32_t pad0;
	uint64_t tsc_timestamp;
	uint64_t system_time;
	uint32_t tsc_to_system_mul;
	uint8_t tsc_shift;
	uint8_t flags;
	uint8_t pad[2];
}; // __attribute__((__packed__)); // should be 32bytes

struct kvmclock_wall_clock_new {
	uint32_t version;
	uint32_t sec;
	uint32_t nsec;
};

struct kvmclock_system_time_new *kvmclock_st;
struct kvmclock_wall_clock_new *kvmclock_wc;
int kvm_probed = 0;
int kvmclock_probed = 0;

static uint64_t kvmclock_rdtsc(void)
{
	uint64_t timestamp = kvmclock_st->tsc_timestamp;
	return (timestamp);
}

void
init_kvmclock(void)
{
        uint32_t regs[4];

	// probe
	do_cpuid(KVM_CPUID_MAGIC, regs);
	kvm_probed = (memcmp("KVMKVMKVM", &regs[1], 9) == 0);

	if (!kvm_probed) {
		return;
	}
	printf("kvm probed\n");
	do_cpuid(KVM_CPUID_FLAGS, regs);
	kvmclock_probed = (regs[0] & KVM_CPUID_FLAGS_PVCLOCK);

	if (!kvmclock_probed) {
		return;
	}

	kvmclock_st = contigmalloc(4096, M_DEVBUF, M_ZERO | M_NOWAIT, 0x00000000, 0xFFFFFFFF, 4096, 0);
	kvmclock_st->version = 0;
	wrmsr(MSR_KVM_SYSTEM_TIME_NEW, (uint64_t)vtophys(kvmclock_st) | 1);

	kvmclock_wc = contigmalloc(4096, M_DEVBUF, M_ZERO | M_NOWAIT, 0x00000000, 0xFFFFFFFF, 4096, 0);
	kvmclock_wc->version = kvmclock_wc->nsec = kvmclock_wc->sec = 0;
	//	wrmsr(MSR_KVM_WALL_CLOCK_NEW, (uint64_t)vtophys(kvmclock_wc) | 1);

#if 0
	// measure
	DELAY(1000); // 1msec
	uint64_t tsc_start, tsc_end, time_start, time_end;
	tsc_start = kvmclock_st->tsc_timestamp * 1000000;
	time_start =  kvmclock_st->system_time / 1000;	
	DELAY(1000000); // 1sec	
	tsc_end = kvmclock_st->tsc_timestamp * 1000000;
	time_end =  kvmclock_st->system_time / 1000;	
	
	kvmclock_freq = (tsc_end - tsc_start) / (time_end - time_start);
	printf("kvmclock clock: %lu Hz, kernel hz=%d\n", kvmclock_freq, hz);
#endif

	kvmclock_freq = 1000000000; // 1000000000Hz = nanosecond

	printf("kvmclock clock: %lu Hz, kernel hz=%d\n", kvmclock_freq, hz);

	/*
	 * Inform CPU accounting about our boot-time clock rate.  Once the
	 * system is finished booting, we will get the real max clock rate
	 * via kvmclock_freq_max().  This also will be updated if someone loads
	 * a cpufreq driver after boot that discovers a new max frequency.
	 */
	set_cputicker(kvmclock_rdtsc, kvmclock_freq, 1);

	/* Register to find out about changes in CPU frequency. */
//	kvmclock_pre_tag = EVENTHANDLER_REGISTER(cpufreq_pre_change,
//	    kvmclock_freq_changing, NULL, EVENTHANDLER_PRI_FIRST);
//	kvmclock_post_tag = EVENTHANDLER_REGISTER(cpufreq_post_change,
//	    kvmclock_freq_changed, NULL, EVENTHANDLER_PRI_FIRST);
//	kvmclock_levels_tag = EVENTHANDLER_REGISTER(cpufreq_levels_changed,
//	    kvmclock_levels_changed, NULL, EVENTHANDLER_PRI_ANY);
}

void
init_kvmclock_tc(void)
{
	if (kvmclock_probed) {
		kvmclock_timecounter.tc_frequency = kvmclock_freq;
		tc_init(&kvmclock_timecounter);
	}
}

static int
sysctl_machdep_kvmclock_freq(SYSCTL_HANDLER_ARGS)
{
	return (EOPNOTSUPP);
/*
	int error;
	uint64_t freq;
	if (kvmclock_timecounter.tc_frequency == 0)
		return (EOPNOTSUPP);
	freq = kvmclock_freq;
	error = sysctl_handle_quad(oidp, &freq, 0, req);
	if (error == 0 && req->newptr != NULL) {
		kvmclock_freq = freq;
		kvmclock_timecounter.tc_frequency = kvmclock_freq;
	}
	return (error);
*/
}

SYSCTL_PROC(_machdep, OID_AUTO, kvmclock_freq, CTLTYPE_QUAD | CTLFLAG_RW,
    0, sizeof(u_int), sysctl_machdep_kvmclock_freq, "QU", "");

static unsigned
kvmclock_get_timecount(struct timecounter *tc)
{
	uint32_t c;
        uint32_t ts_version = 0;

	do {
		ts_version = kvmclock_st->version;
                rmb();
		c = kvmclock_st->system_time;
                rmb();
        } 
	while ((kvmclock_st->version & 1) || (ts_version != kvmclock_st->version));

	return (c);
}
