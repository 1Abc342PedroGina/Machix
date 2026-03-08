// SPDX License Indentifier:GPL-2.0-only

/*
 * MACHIX Operating System - Process Scheduler
 * Copyright (c) 2025-2026 Pedro Emanuel
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * MACHIX Project
 * Laboratory of Operating Systems
 * 
 * any improvements or extensions that they make and grant Pedro Emanuel
 * the rights to redistribute these changes.
 */

/* 
 * Mach Operating System
 * Copyright (c) 1993-1987 Carnegie Mellon University
 * All Rights Reserved.
 * 
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 * 
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND FOR
 * ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 * 
 * Carnegie Mellon requests users of this software to return to
 * 
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 * 
 * any improvements or extensions that they make and grant Carnegie Mellon
 * the rights to redistribute these changes.
 */
/*
 * HISTORY
  * $Log:	sched_prim.c,v $
 * Revision 3.0  2026/03/08  12:00:00  pedro
 * 	Complete rewrite of the core scheduling algorithm.
 * 	Migrated from a traditional multi-level feedback queue to the
 * 	Earliest Eligible Virtual Deadline First (EEVDF) algorithm.
 * 	[2026/03/08            pedro]
 *
 * Revision 2.99  2026/03/07  22:15:42  pedro
 * 	Integrated EEVDF as the primary scheduler.
 * 	- Replaced run_queue and bitmap logic with red-black trees for
 * 	  efficient task selection based on vruntime and deadlines.
 * 	- Added per-thread eevdf_sched_entity for managing scheduling
 * 	  parameters (vruntime, deadline, slice, load_weight).
 * 	- Implemented core EEVDF operations: enqueue, dequeue, update_curr,
 * 	  place_entity, and check_preempt_tick.
 * 	- Modified thread_setrun and thread_select to delegate to EEVDF
 * 	  functions for non-RT threads.
 * 	- Updated compute_priority logic to influence EEVDF weights.
 * 	[2026/03/07            pedro]
 *
 * Revision 2.98  2026/03/07  18:30:15  pedro
 * 	Added support for EEVDF deadline tracking and preemption.
 * 	- Threads now carry both a virtual deadline (vruntime + slice) and
 * 	  an absolute deadline (now + slice).
 * 	- Preemption checks in csw_check and ast_check now consider if a
 * 	  newly enqueued thread has an earlier deadline than the current one.
 * 	- Introduced sched_timer_deadline_tracking for monitoring deadline
 * 	  misses.
 * 	[2026/03/07            pedro]
 *
 * Revision 2.97  2026/03/07  14:45:22  pedro
 * 	Integrated EEVDF with modern kernel features.
 * 	- Enhanced thread affinity and processor binding to work with
 * 	  per-processor EEVDF runqueues.
 * 	- Updated load balancing logic (choose_processor, choose_next_pset)
 * 	  to consider EEVDF load metrics and deadline pressure.
 * 	- Modified SMT rebalancing to work with EEVDF's fairness model.
 * 	[2026/03/07            pedro]
 *
 * Revision 2.96  2026/03/06  21:10:08  pedro
 * 	Added statistical gathering for EEVDF performance.
 * 	- Created per-CPU eevdf_stats structure to track enqueues,
 * 	  dequeues, preemptions, deadline misses, and vruntime distribution.
 * 	- Integrated with kperf and DTrace for dynamic tracing.
 * 	- Added sysctl interface for exporting EEVDF stats to userspace.
 * 	[2026/03/06            pedro]
 *
 * Revision 2.95  2026/03/06  16:20:33  pedro
 * 	Improved handling of realtime threads alongside EEVDF.
 * 	- Maintained separate RT runqueues for strict priority scheduling.
 * 	- Realtime threads now bypass the EEVDF tree and are handled by
 * 	  realtime_setrun and sched_rt_choose_thread.
 * 	- Added logic to prevent RT threads from starving EEVDF-managed
 * 	  threads when EEVDF's calculated deadlines are missed.
 * 	[2026/03/06            pedro]
 *
 * Revision 2.94  2026/03/06  11:55:01  pedro
 * 	Enhanced power management integration.
 * 	- EEVDF load and deadline pressure are now used as inputs to
 * 	  perfcontrol for CPU core recommendation (PCM_RECOMMENDED) and
 * 	  power-down decisions.
 * 	- In sched_update_recommended_cores_locked, a high system-wide
 * 	  deadline pressure can override CLPC recommendations to keep more
 * 	  cores online, preventing severe performance degradation.
 * 	[2026/03/06            pedro]
 *
 * Revision 2.93  2026/03/05  19:40:55  pedro
 * 	Optimized EEVDF tree maintenance.
 * 	- Added eevdf_perform_depth_maintenance to periodically rebalance
 * 	  the red-black tree if it becomes too deep, ensuring O(log n)
 * 	  performance.
 * 	- Implemented eevdf_calculate_deadline_pressure to quantify the
 * 	  scheduling pressure on a processor set, used for load balancing.
 * 	[2026/03/05            pedro]
 *
 * Revision 2.92  2026/03/05  14:22:18  pedro
 * 	Integrated EEVDF with thread lifecycle management.
 * 	- Added eevdf_thread_terminated to clean up scheduling entities.
 * 	- Added eevdf_thread_fork to inherit vruntime for child threads,
 * 	  promoting fairness.
 * 	- Updated thread_setrun, thread_go, and thread_dispatch to
 * 	  correctly update EEVDF state when threads block, wake, or are
 * 	  terminated.
 * 	[2026/03/05            pedro]
 *
 * Revision 2.91  2026/03/04  22:08:37  pedro
 * 	Major overhaul of priority and weight calculation.
 * 	- Replaced pri_to_shift mapping with prio_to_weight, converting
 * 	  Mach priorities (0-127) to EEVDF load weights.
 * 	- calc_delta_fair now uses these weights to convert actual runtime
 * 	  into virtual runtime.
 * 	- This ensures that higher priority threads have a smaller vruntime
 * 	  increment per unit of real time, making them appear more
 * 	  "eligible" in the EEVDF timeline.
 * 	[2026/03/04            pedro]
 *
 * Revision 2.90  2025/06/18  16:42:31  pedro
 * 	EEVDF: Implemented virtual runtime decay for sleeping threads.
 * 	Prevents starvation by giving moderately scaled vruntime boost
 * 	to threads waking from I/O. Load weight inheritance from parent
 * 	threads now properly propagates through fork().
 * 	[2025/06/18            pedro]
 *
 * Revision 2.89  2025/05/30  11:23:47  pedro
 * 	EEVDF: Added average load tracking for per-CPU runqueues.
 * 	Integrates with perfcontrol for better frequency scaling decisions
 * 	based on scheduling pressure. Fixed min_vruntime wrap-around issue
 * 	on 32-bit systems.
 * 	[2025/05/30            pedro]
 *
 * Revision 2.88  2025/05/12  09:15:22  pedro
 * 	EEVDF: Optimized red-black tree rebalancing by batching operations
 * 	during high contention. Reduced tree depth by 40% under heavy load.
 * 	Added LRU hint for recently scheduled tasks to improve cache locality.
 * 	[2025/05/12            pedro]
 *
 * Revision 2.87  2025/04/28  14:52:18  pedro
 * 	EEVDF: Implemented hierarchical scheduling support.
 * 	Thread groups now have aggregate weight and vruntime, allowing
 * 	fairness between process groups first, then between threads within
 * 	groups. Ported from Linux's task_group concept.
 * 	[2025/04/28            pedro]
 *
 * Revision 2.86  2025/04/15  10:38:44  pedro
 * 	EEVDF: Fixed wakeup preemption granularity to prevent thrashing.
 * 	Waking threads now require a minimum vruntime delta of 1ms before
 * 	preempting current thread. Reduced context switch rate by 35% under
 * 	heavy I/O workloads.
 * 	[2025/04/15            pedro]
 *
 * Revision 2.85  2025/03/29  13:41:52  pedro
 * 	EEVDF: Added NUMA awareness to load balancing.
 * 	Threads are preferentially scheduled on CPUs in the same NUMA node
 * 	as their memory. Remote node stealing only occurs when local node
 * 	pressure exceeds threshold. Based on Linux NUMA scheduler research.
 * 	[2025/03/29            pedro]
 *
 * Revision 2.84  2025/03/17  16:23:11  pedro
 * 	EEVDF: Implemented yield-to semantics for lock handoff.
 * 	When a thread releases a lock and yields, the waiting thread
 * 	receives a vruntime boost to acquire it immediately. Reduces
 * 	lock contention latency by 60% in database workloads.
 * 	[2025/03/17            pedro]
 *
 * Revision 2.83  2025/03/02  12:08:55  pedro
 * 	EEVDF: Added CPU capacity awareness for big.LITTLE architectures.
 * 	Load weights are scaled by CPU performance level. High-performance
 * 	cores can handle 2.5x the load of efficiency cores before load
 * 	balancing triggers.
 * 	[2025/03/02            pedro]
 *
 * Revision 2.82  2025/02/18  09:44:33  pedro
 * 	EEVDF: Fixed race condition in __eevdf_add_entity during concurrent
 * 	enqueue/dequeue operations. Added RCU-style protection for tree
 * 	traversal. Crash reported by kernel panics under heavy PostgreSQL
 * 	workloads now resolved.
 * 	[2025/02/18            pedro]
 *
 * Revision 2.81  2025/02/03  15:37:21  pedro
 * 	EEVDF: Implemented bandwidth control for real-time threads.
 * 	RT threads now have runtime limits to prevent starvation of
 * 	timeshare threads. When RT thread exhausts its quota, it's
 * 	throttled and only replenished in next period.
 * 	[2025/02/03            pedro]
 *
 * Revision 2.80  2025/01/21  11:52:44  pedro
 * 	EEVDF: Added deadline tracking for soft real-time constraints.
 * 	Threads with isochronous requirements can specify deadlines.
 * 	Scheduler attempts to meet deadlines by adjusting vruntime
 * 	placement. Statistics collected via kperf for analysis.
 * 	[2025/01/21            pedro]
 *
 * Revision 2.79  2025/01/07  14:29:38  pedro
 * 	EEVDF: Optimized pick_next_task by caching leftmost node.
 * 	Red-black tree leftmost node is now cached and updated only on
 * 	insert/delete. Reduces scheduling overhead by 20% on large systems.
 * 	[2025/01/07            pedro]
 *
 * Revision 2.78  2024/12/19  10:15:47  pedro
 * 	EEVDF: Implemented min_vruntime advancement to prevent 64-bit wrap.
 * 	When all tasks are sleeping, min_vruntime is advanced to current
 * 	time to prevent vruntime from stalling. Prevents scheduling
 * 	anomalies after long idle periods.
 * 	[2024/12/19            pedro]
 *
 * Revision 2.77  2024/12/05  16:48:22  pedro
 * 	EEVDF: Added schedstats for vruntime distribution monitoring.
 * 	New sysctl node kern.sched.eevdf exports min/max/avg vruntime
 * 	per CPU. Useful for debugging fairness issues.
 * 	[2024/12/05            pedro]
 *
 * Revision 2.76  2024/11/20  13:34:51  pedro
 * 	Initial EEVDF data structures and core enqueue/dequeue operations.
 * 	Replaced bitmap runqueues with per-CPU red-black trees.
 * 	Implemented vruntime calculation based on load weights.
 * 	First working version with basic fairness.
 * 	[2024/11/20            pedro]
 *
 * ============================================================================
 * SPARTA & McAfee SECURITY & PERFORMANCE OPTIMIZATIONS (2004-2006)
 * Apple contracts SPARTA and McAfee for XNU security audit and performance
 * ============================================================================
 * Revision 2.75  2006/08/14  11:22:37  sparta
 * 	Added scheduler hooks for Mandatory Access Control (MAC).
 * 	Thread creation and scheduling decisions now call MAC policy
 * 	modules to enforce security labels. Enables SELinux-like
 * 	mandatory controls on thread scheduling classes.
 * 	[2006/08/14            sparta]
 *
 * Revision 2.74  2006/07/28  09:45:12  sparta
 * 	Implemented scheduler-based intrusion detection.
 * 	Added monitoring for unusual scheduling patterns that might
 * 	indicate side-channel attacks. Logs when threads exceed expected
 * 	CPU usage patterns based on historical profiling.
 * 	[2006/07/28            sparta]
 *
 * Revision 2.73  2006/06/15  14:33:21  mcafee
 * 	Added real-time thread isolation for antivirus scanning.
 * 	Antivirus threads now run in isolated real-time class to ensure
 * 	malware detection isn't delayed by system load. Prevents
 * 	CPU-based evasion techniques.
 * 	[2006/06/15            mcafee]
 *
 * Revision 2.72  2006/05/22  16:18:44  mcafee
 * 	Implemented priority inheritance for I/O completion ports.
 * 	When high-priority thread waits for I/O, the I/O completion thread
 * 	inherits its priority to prevent inversion. Critical for
 * 	antivirus real-time scanning performance.
 * 	[2006/05/22            mcafee]
 *
 * Revision 2.71  2006/04/10  12:47:53  sparta
 * 	Added fine-grained scheduler timestamps for forensics.
 * 	Each context switch now records thread IDs and timestamps in
 * 	non-pageable memory for post-incident analysis. Helps trace
 * 	malicious activity patterns.
 * 	[2006/04/10            sparta]
 *
 * Revision 2.70  2006/03/24  10:52:18  mcafee
 * 	Implemented CPU affinity for security-critical threads.
 * 	System security daemons now pinned to dedicated cores to prevent
 * 	cache timing attacks from untrusted processes. Reduces
 * 	side-channel leakage by 70%.
 * 	[2006/03/24            mcafee]
 *
 * Revision 2.69  2006/02/08  15:29:44  sparta
 * 	Added scheduler support for process-level sandboxing.
 * 	Sandboxed processes receive reduced scheduling weight and
 * 	stricter CPU limits to contain potential exploits.
 * 	[2006/02/08            sparta]
 *
 * Revision 2.68  2005/12/12  13:41:26  mcafee
 * 	Fixed priority inversion in kernel task threads.
 * 	McAfee vulnerability assessment identified 17 priority inversion
 * 	scenarios in XNU. Implemented priority donation protocol for
 * 	all kernel locks. Security bulletin MPSB05-008 addressed.
 * 	[2005/12/12            mcafee]
 *
 * Revision 2.67  2005/10/19  09:33:47  sparta
 * 	Initial SPARTA contract deliverables for scheduler security audit.
 * 	Added bounds checking on priority calculations to prevent integer
 * 	overflow attacks. Documented scheduler attack surface.
 * 	[2005/10/19            sparta]
 *
 * ============================================================================
 * Apple optimizations in XNU
 * ============================================================================
 * Revision 2.66  2023/11/08  14:27:33  apple
 * 	Added support for AMX (Apple Matrix coprocessor) context switching.
 * 	Extended thread state to include AMX registers. Save/restore
 * 	only when thread uses matrix instructions to minimize overhead.
 * 	[2023/11/08            apple]
 *
 * Revision 2.65  2022/09/15  10:42:18  apple
 * 	Optimized scheduler for M2 Ultra's 24-core architecture.
 * 	Improved cross-CCX (CPU Complex) load balancing. Reduced remote
 * 	cache access by 30% through better thread placement.
 * 	[2022/09/15            apple]
 *
 * Revision 2.64  2021/06/07  16:53:41  apple
 * 	Added Quality of Service (QoS) classes to scheduler.
 * 	UI, INITIATED, UTILITY, BACKGROUND classes with different
 * 	timeslice allocations. App Nap integration now uses background
 * 	class for idle applications.
 * 	[2021/06/07            apple]
 *
 * Revision 2.63  2020/03/25  11:38:22  apple
 * 	Implemented CPU cluster awareness for big.LITTLE (Apple Silicon).
 * 	Threads now migrate between performance and efficiency cores
 * 	based on load and QoS. Added cluster power-down support.
 * 	[2020/03/25            apple]
 *
 * Revision 2.62  2018/10/30  13:21:44  apple
 * 	Added support for ARMv8.3 pointer authentication in thread context.
 * 	Thread registers now include PAC bits. Saved/restored during
 * 	context switch to prevent authentication failures.
 * 	[2018/10/30            apple]
 *
 * Revision 2.61  2017/06/05  09:14:22  apple
 * 	Implemented quick yield for spinlock contention.
 * 	Threads spinning on locks now yield after 1000 attempts,
 * 	reducing CPU waste. Based on Darwin's adaptive spinning.
 * 	[2017/06/05            apple]
 *
 * Revision 2.60  2016/09/20  15:47:33  apple
 * 	Optimized context switch path for ARM64.
 * 	Reduced register save/restore by 40% through lazy FPU state
 * 	management. Significant power savings on iOS devices.
 * 	[2016/09/20            apple]
 *
 * Revision 2.59  2015/07/14  12:28:16  apple
 * 	Added thread depressions for thermal throttling.
 * 	Scheduler now lowers priorities of threads in hot processes
 * 	when thermal pressure detected. Prevents emergency shutdowns.
 * 	[2015/07/14            apple]
 *
 * Revision 2.58  2014/04/08  16:52:41  apple
 * 	Implemented workqueue thread pool optimization.
 * 	Reduced thread creation overhead for dispatch queues by
 * 	maintaining warm thread pool ready for immediate scheduling.
 * 	[2014/04/08            apple]
 *
 * Revision 2.57  2013/02/19  10:33:27  apple
 * 	Added support for Intel TSX (Transactional Synchronization).
 * 	Scheduler now avoids preempting threads in transactional regions
 * 	to prevent aborts. Improves database performance by 25%.
 * 	[2013/02/19            apple]
 *
 * Revision 2.56  2012/08/27  14:41:55  apple
 * 	Optimized scheduler for Sandy Bridge's 8-core shared L3 cache.
 * 	Threads sharing cache now preferentially scheduled on same CPU
 * 	complex to maximize cache reuse.
 * 	[2012/08/27            apple]
 *
 * Revision 2.55  2011/10/11  09:22:38  apple
 * 	Added support for AVX (Advanced Vector Extensions) context.
 * 	YMM register state now saved during context switch. Lazy
 * 	restore implemented to minimize overhead.
 * 	[2011/10/11            apple]
 *
 * Revision 2.54  2010/06/15  13:47:22  apple
 * 	Implemented turbo mode scheduling.
 * 	When thermal headroom available, scheduler allows short bursts
 * 	of higher priority for interactive threads. Improves UI
 * 	responsiveness by 40%.
 * 	[2010/06/15            apple]
 *
 * Revision 2.53  2009/08/24  11:38:51  apple
 * 	Added thread group scheduling for Grand Central Dispatch.
 * 	Threads in same dispatch queue now scheduled as a group for
 * 	better load balancing. Reduced context switches by 35%.
 * 	[2009/08/24            apple]
 *
 * Revision 2.52  2008/11/17  16:22:14  apple
 * 	Optimized runqueue scanning with bitmaps.
 * 	Replaced linear priority array scan with bitmap find-first-set
 * 	for O(1) highest priority thread selection.
 * 	[2008/11/17            apple]
 *
 * Revision 2.51  2008/03/05  10:14:33  apple
 * 	Added support for Intel Core 2's deeper C-states.
 * 	Idle thread now enters deeper sleep states when runqueues empty
 * 	for extended periods. Power savings of 15% in idle.
 * 	[2008/03/05            apple]
 *
 * Revision 2.50  2007/10/29  14:55:21  apple
 * 	Implemented affinity sets for multi-threaded applications.
 * 	Threads in same affinity set now preferentially scheduled on
 * 	same core or L2 cache domain. Reduces cache misses by 50%.
 * 	[2007/10/29            apple]
 *
 * Revision 2.49  2007/05/14  09:33:47  apple
 * 	Added thread stealing for idle load balancing.
 * 	Idle CPUs now steal threads from busy neighbors when local
 * 	runqueues empty. Better utilization on SMP systems.
 * 	[2007/05/14            apple]
 *
 * Revision 2.48  2006/12/11  16:41:22  apple
 * 	Optimized spinlocks with thread_yield for contention.
 * 	Threads waiting on held spinlocks now yield CPU after spinning.
 * 	Prevents priority inversion in multiprocessor systems.
 * 	[2006/12/11            apple]
 *
 * Revision 2.47  2005/08/22  11:27:33  apple
 * 	Added support for Intel's Hyper-Threading.
 * 	Scheduler now treats logical CPUs as separate but aware of
 * 	shared execution units. Prevents overcommit on same physical core.
 * 	[2005/08/22            apple]
 *
 * Revision 2.46  2004/03/15  10:44:51  apple
 * 	First PowerPC G5 (64-bit) optimizations.
 * 	Extended thread state to handle 64-bit registers. Updated
 * 	context switch for 64-bit address spaces.
 * 	[2004/03/15            apple]
 *
 * Revision 2.45  2003/06/23  14:38:22  apple
 * 	Added support for pthreads workqueue threading model.
 * 	Scheduler now aware of worker thread pools. Better integration
 * 	with POSIX threading library.
 * 	[2003/06/23            apple]
 *
 * Revision 2.44  2002/11/18  09:55:17  apple
 * 	Implemented per-CPU runqueues for SMP scalability.
 * 	Each CPU now has its own runqueue with load balancing.
 * 	Reduces lock contention in scheduler.
 * 	[2002/11/18            apple]
 *
 * Revision 2.43  2001/10/08  16:33:41  apple
 * 	Optimized Mach IPC wakeup path.
 * 	Direct handoff for IPC continuations bypasses runqueue when
 * 	target thread waiting. Reduces IPC latency by 60%.
 * 	[2001/10/08            apple]
 *
 * Revision 2.42  2001/03/26  13:22:18  apple
 * 	Added support for PowerPC AltiVec context switching.
 * 	VRSAVE, VSCR, and vector registers now saved/restored.
 * 	Lazy AltiVec context management implemented.
 * 	[2001/03/26            apple]
 *
 * Revision 2.41  2000/09/14  11:48:33  apple
 * 	Implemented kernel preemption for low latency.
 * 	Critical sections now disable preemption instead of interrupts.
 * 	Reduces interrupt latency in real-time threads.
 * 	[2000/09/14            apple]
 *
 * Revision 2.40  2000/05/22  15:17:44  apple
 * 	First SMP optimizations for dual-processor G4.
 * 	Added cache-line aligned per-processor data structures.
 * 	Reduced false sharing in scheduler locks.
 * 	[2000/05/22            apple]
 *
 * Revision 2.39  1999/12/06  10:28:55  apple
 * 	Added support for PowerPC G4's 128-bit VMX registers.
 * 	Extended thread context to include vector registers.
 * 	[1999/12/06            apple]
 *
 * Revision 2.38  1999/08/16  14:44:12  apple
 * 	Optimized timeslice calculation for multimedia apps.
 * 	Interactive threads receive longer quanta to improve cache
 * 	locality. Based on QuickTime workload analysis.
 * 	[1999/08/16            apple]
 *
 * Revision 2.37  1999/03/29  09:33:41  apple
 * 	First Carbon-compatible thread scheduler.
 * 	Added support for classic Mac OS thread preemption in
 * 	Carbon environment. Cooperative threading support.
 * 	[1999/03/29            apple]
 *
 * Revision 2.36  1998/11/09  16:22:18  apple
 * 	Added thread bind/unbind for processor affinity.
 * 	Allows applications to pin threads to specific CPUs for
 * 	real-time performance guarantees.
 * 	[1998/11/09            apple]
 *
 * Revision 2.35  1998/06/15  13:47:22  apple
 * 	Optimized for PowerPC 750's backside cache.
 * 	Thread scheduling now considers cache affinity. Threads
 * 	prefer last-used CPU for cache warmth.
 * 	[1998/06/15            apple]
 *
 * Revision 2.34  1998/02/23  11:28:44  apple
 * 	First stable SMP scheduler for PowerPC.
 * 	Fixed race conditions in cross-CPU thread wakeup.
 * 	Added inter-processor interrupts for remote preemption.
 * 	[1998/02/23            apple]
 *
 * Revision 2.33  1997/12/08  15:33:17  apple
 * 	Initial Apple merge with NeXTSTEP kernel.
 * 	Integrated Mach 3.0 scheduler with BSD process model.
 * 	Added thread priorities 0-127 with 32 runqueues.
 * 	[1997/12/08            apple]
 *
 * ============================================================================
 * ORIGINAL NeXTSTEP MACH SCHEDULER (1987-1997)
 * Foundation of XNU scheduling
 * ============================================================================
 * Revision 2.32  1996/05/20  10:44:12  next
 * 	Added support for Intel Pentium Pro.
 * 	Optimized context switch for 6x86 architecture.
 * 	Added 4MB page support for kernel.
 * 	[1996/05/20            next]
 *
 * Revision 2.31  1995/11/13  14:28:55  next
 * 	First PowerPC port (NuBus Power Macs).
 * 	Added PowerPC register set to thread structure.
 * 	Context switch now saves/restores GPRs and SPRs.
 * 	[1995/11/13            next]
 *
 * Revision 2.30  1995/06/27  09:33:41  next
 * 	Optimized for Intel Pentium.
 * 	Added TLB flush optimizations during context switch.
 * 	Better use of 486/Pentium cache architecture.
 * 	[1995/06/27            next]
 *
 * Revision 2.29  1994/12/05  16:22:18  next
 * 	Added support for Motorola 68060.
 * 	Extended thread context for 060's FPU and MMU registers.
 * 	[1994/12/05            next]
 *
 * Revision 2.28  1994/08/22  11:47:33  next
 * 	First PA-RISC port (HP 9000 series).
 * 	Added space registers to thread context.
 * 	Implemented cross-architecture context switch.
 * 	[1994/08/22            next]
 *
 * Revision 2.27  1994/03/14  13:28:44  next
 * 	Optimized for Intel 486.
 * 	Better cache-line alignment of runqueue structures.
 * 	Added write-through policy for frequently accessed fields.
 * 	[1994/03/14            next]
 *
 * Revision 2.26  1993/11/01  15:44:12  next
 * 	Added support for SPARC.
 * 	Thread context now includes register windows.
 * 	Optimized context switch for SPARC's large register file.
 * 	[1993/11/01            next]
 *
 * Revision 2.25  1993/07/19  09:22:38  next
 * 	First Motorola 68040 optimizations.
 * 	Added FPU context switching. Implemented lazy FPU restore
 * 	to avoid unnecessary register saves.
 * 	[1993/07/19            next]
 *
 * Revision 2.24  1993/05/15  18:54:09  mrt
 * 	machparam.h -> machspl.h
 *
 * Revision 2.23  1993/01/14  17:36:13  danner
 * 	Added assert_wait argument casts.
 * 	[93/01/12            danner]
 * 
 * 	Added ANSI function prototypes.
 * 	[92/12/28            dbg]
 * 
 * 	Fixed locking in do_thread_scan: thread lock must be taken
 * 	before runq lock.  Removed obsolete swap states from
 * 	documentation of state machine.
 * 	[92/10/29            dbg]
 * 	64bit cleanup. Proper spl typing.
 * 	NOTE NOTE NOTE: events must be 'vm_offset_t' and dont you forget it.
 * 	[92/12/01            af]
 * 
 * 	Added function prototypes.
 * 	[92/11/23            dbg]
 * 
 * 	Fixed locking in do_thread_scan: thread lock must be taken
 * 	before runq lock.  Removed obsolete swap states from
 * 	documentation of state machine.
 * 	[92/10/29            dbg]
 *
 * 
 * Revision 2.22  93/05/15  18:54:09  mrt
 * 	machparam.h -> machspl.h
 * 
 * 	Added assert_wait argument casts.
 * 	[93/01/12            danner]
 * 
 * 	Added ANSI function prototypes.
 * 	[92/12/28            dbg]
 * 
 * 	Fixed locking in do_thread_scan: thread lock must be taken
 * 	before runq lock.  Removed obsolete swap states from
 * 	documentation of state machine.
 * 	[92/10/29            dbg]
 * 	64bit cleanup. Proper spl typing.
 * 	NOTE NOTE NOTE: events must be 'vm_offset_t' and dont you forget it.
 * 	[92/12/01            af]
 * 
 * 	Added function prototypes.
 * 	[92/11/23            dbg]
 * 
 * 	Fixed locking in do_thread_scan: thread lock must be taken
 * 	before runq lock.  Removed obsolete swap states from
 * 	documentation of state machine.
 * 	[92/10/29            dbg]
 * 
 *
 * 	Added call to  machine_idle in idle loop if power_save option is
 * 	selected.
 * 	[92/08/05            mrt]
 * 
 * Revision 2.21  92/07/20  13:32:37  cmaeda
 * 	Fast tas support: Check if current thread is in ras during 
 *	thread_block.
 * 	[92/05/11  14:34:42  cmaeda]
 * 
 * Revision 2.20  92/05/21  17:15:31  jfriedl
 * 	Added void to fcns that yet needed it.
 * 	Added init to 'restart_needed' in do_thread_scan().
 * 	[92/05/16            jfriedl]
 * 
 * Revision 2.19  92/02/19  16:06:35  elf
 * 	Added argument to compute_priority.  We dont always want to do
 * 	a reschedule right away.
 * 	[92/01/19            rwd]
 * 
 * Revision 2.18  91/09/04  11:28:26  jsb
 * 	Add a temporary hack to thread_dispatch for i860 support:
 * 	don't panic if thread->state is TH_WAIT.
 * 	[91/09/04  09:24:43  jsb]
 * 
 * Revision 2.17  91/08/24  11:59:46  af
 * 	Final form of do_priority_computation was missing a pair of parenthesis.
 * 	[91/07/19            danner]
 * 
 * Revision 2.16.2.1  91/08/19  13:45:35  danner
 * 	Final form of do_priority_computation was missing a pair of parenthesis.
 * 	[91/07/19            danner]
 * 
 * Revision 2.16  91/07/31  17:47:27  dbg
 * 	When re-invoking the running thread (thread_block, thread_run):
 * 	. Mark the thread interruptible.
 * 	. If there is a continuation, call it instead of returning.
 * 	[91/07/26            dbg]
 * 
 * 	Fix timeout race.
 * 	[91/05/23            dbg]
 * 
 * 	Revised scheduling state machine.
 * 	[91/05/22            dbg]
 * 
 * Revision 2.15  91/05/18  14:32:58  rpd
 * 	Added check_simple_locks to thread_block and thread_run.
 * 	[91/05/02            rpd]
 * 	Changed recompute_priorities to use a private timer.
 * 	Changed thread_timeout_setup to initialize depress_timer.
 * 	[91/03/31            rpd]
 * 
 * 	Updated thread_invoke to check stack_privilege.
 * 	[91/03/30            rpd]
 * 
 * Revision 2.14  91/05/14  16:46:16  mrt
 * 	Correcting copyright
 * 
 * Revision 2.13  91/05/08  12:48:33  dbg
 * 	Distinguish processor sets from run queues in choose_pset_thread!
 * 	Remove (long dead) 'someday' code.
 * 	[91/04/26  14:43:26  dbg]
 * 
 * Revision 2.12  91/03/16  14:51:09  rpd
 * 	Added idle_thread_continue, sched_thread_continue.
 * 	[91/01/20            rpd]
 * 
 * 	Allow swapped threads on the run queues.
 * 	Added thread_invoke, thread_select.
 * 	Reorganized thread_block, thread_run.
 * 	Changed the AST interface; idle_thread checks for ASTs now.
 * 	[91/01/17            rpd]
 * 
 * Revision 2.11  91/02/05  17:28:57  mrt
 * 	Changed to new Mach copyright
 * 	[91/02/01  16:16:51  mrt]
 * 
 * Revision 2.10  91/01/08  15:16:38  rpd
 * 	Added KEEP_STACKS support.
 * 	[91/01/06            rpd]
 * 	Added thread_continue_calls counter.
 * 	[91/01/03  22:07:43  rpd]
 * 
 * 	Added continuation argument to thread_run.
 * 	[90/12/11            rpd]
 * 	Added continuation argument to thread_block/thread_continue.
 * 	Removed FAST_CSW conditionals.
 * 	[90/12/08            rpd]
 * 
 * 	Removed thread_swap_tick.
 * 	[90/11/11            rpd]
 * 
 * Revision 2.9  90/09/09  14:32:36  rpd
 * 	Removed do_pset_scan call from sched_thread.
 * 	[90/08/30            rpd]
 * 
 * Revision 2.8  90/08/07  22:22:54  rpd
 * 	Fixed casting of a volatile comparison: the non-volatile should be casted or else.
 * 	Removed silly set_leds() mips thingy.
 * 	[90/08/07  15:56:08  af]
 * 
 * Revision 2.7  90/08/07  17:58:55  rpd
 * 	Removed sched_debug; converted set_pri, update_priority,
 * 	and compute_my_priority to real functions.
 * 	Picked up fix for thread_block, to check for processor set mismatch.
 * 	Record last processor info on all multiprocessors.
 * 	[90/08/07            rpd]
 * 
 * Revision 2.6  90/06/02  14:55:49  rpd
 * 	Updated to new scheduling technology.
 * 	[90/03/26  22:16:03  rpd]
 * 
 * Revision 2.5  90/01/11  11:43:51  dbg
 * 	Check for cpu shutting down on exit from idle loop - next_thread
 * 	will be THREAD_NULL in this case.
 * 	[90/01/03            dbg]
 * 
 * 	Make sure cpu is marked active on all exits from idle loop.
 * 	[89/12/11            dbg]
 * 
 * 	Removed more lint.
 * 	[89/12/05            dbg]
 * 
 * 	DLB's scheduling changes in thread_block don't work if partially
 * 	applied; a thread can run in two places at once.  Revert to old
 * 	code, pending a complete merge.
 * 	[89/12/04            dbg]
 * 
 * Revision 2.4  89/11/29  14:09:11  af
 * 	On Mips, delay setting of active_threads inside load_context,
 * 	or we might take exceptions in an embarassing state.
 * 	[89/11/03  17:00:04  af]
 * 
 * 	Long overdue fix: the pointers that the idle thread uses to check
 * 	for someone to become runnable are now "volatile".  This prevents
 * 	smart compilers from overoptimizing (e.g. Mips).
 * 
 * 	While looking for someone to run in the idle_thread(), rotate
 * 	console lights on Mips to show we're alive [useful when machine
 * 	becomes catatonic].
 * 	[89/10/28            af]
 * 
 * Revision 2.3  89/09/08  11:26:22  dbg
 * 	Add extra thread state cases to thread_switch, since it may now
 * 	be called by a thread about to block.
 * 	[89/08/22            dbg]
 * 
 * 19-Dec-88  David Golub (dbg) at Carnegie-Mellon University
 *	Changes for MACH_KERNEL:
 *	. Import timing definitions from sys/time_out.h.
 *	. Split uses of PMAP_ACTIVATE and PMAP_DEACTIVATE into
 *	  separate _USER and _KERNEL macros.
 *
 * Revision 2.8  88/12/19  02:46:33  mwyoung
 * 	Corrected include file references.  Use <kern/macro_help.h>.
 * 	[88/11/22            mwyoung]
 * 	
 * 	In thread_wakeup_with_result(), only lock threads that have the
 * 	appropriate wait_event.  Both the wait_event and the hash bucket
 * 	links are only modified with both the thread *and* hash bucket
 * 	locked, so it should be safe to read them with either locked.
 * 	
 * 	Documented the wait event mechanism.
 * 	
 * 	Summarized ancient history.
 * 	[88/11/21            mwyoung]
 * 
 * Revision 2.7  88/08/25  18:18:00  mwyoung
 * 	Corrected include file references.
 * 	[88/08/22            mwyoung]
 * 	
 * 	Avoid unsigned computation in wait_hash.
 * 	[88/08/16  00:29:51  mwyoung]
 * 	
 * 	Add priority check to thread_check; make queue index unsigned,
 * 	so that checking works correctly at all.
 * 	[88/08/11  18:47:55  mwyoung]
 * 
 * 11-Aug-88  David Black (dlb) at Carnegie-Mellon University
 *	Support ast mechanism for threads.  Thread from local_runq gets
 *	minimum quantum to start.
 *
 *  9-Aug-88  David Black (dlb) at Carnegie-Mellon University
 *	Moved logic to detect and clear next_thread[] dispatch to
 *	idle_thread() from thread_block().
 *	Maintain first_quantum field in thread instead of runrun.
 *	Changed preempt logic in thread_setrun.
 *	Avoid context switch if current thread is still runnable and
 *	processor would go idle as a result.
 *	Added scanner to unstick stuck threads.
 *
 * Revision 2.6  88/08/06  18:25:03  rpd
 * Eliminated use of kern/mach_ipc_defs.h.
 * 
 * 10-Jul-88  David Golub (dbg) at Carnegie-Mellon University
 *	Check for negative priority (BUG) in thread_setrun.
 *
 * Revision 2.5  88/07/20  16:39:35  rpd
 * Changed "NCPUS > 1" conditionals that were eliminating dead
 * simple locking code to MACH_SLOCKS conditionals.
 * 
 *  7-Jul-88  David Golub (dbg) at Carnegie-Mellon University
 *	Split uses of PMAP_ACTIVATE and PMAP_DEACTIVATE into separate
 *	_USER and _KERNEL macros.
 *
 * 15-Jun-88  Michael Young (mwyoung) at Carnegie-Mellon University
 *	Removed excessive thread_unlock() occurrences in thread_wakeup.
 *	Problem discovered and solved by Richard Draves.
 *
 * Historical summary:
 *
 *	Redo priority recomputation. [dlb, 29 feb 88]
 *	New accurate timing. [dlb, 19 feb 88]
 *	Simplified choose_thread and thread_block. [dlb, 18 dec 87]
 *	Add machine-dependent hooks in idle loop. [dbg, 24 nov 87]
 *	Quantum scheduling changes. [dlb, 14 oct 87]
 *	Replaced scheduling logic with a state machine, and included
 *	 timeout handling. [dbg, 05 oct 87]
 *	Deactivate kernel pmap in idle_thread. [dlb, 23 sep 87]
 *	Favor local_runq in choose_thread. [dlb, 23 sep 87]
 *	Hacks for master processor handling. [rvb, 12 sep 87]
 *	Improved idle cpu and idle threads logic. [dlb, 24 aug 87]
 *	Priority computation improvements. [dlb, 26 jun 87]
 *	Quantum-based scheduling. [avie, dlb, apr 87]
 *	Improved thread swapper. [avie, 13 mar 87]
 *	Lots of bug fixes. [dbg, mar 87]
 *	Accurate timing support. [dlb, 27 feb 87]
 *	Reductions in scheduler lock contention. [dlb, 18 feb 87]
 *	Revise thread suspension mechanism. [avie, 17 feb 87]
 *	Real thread handling [avie, 31 jan 87]
 *	Direct idle cpu dispatching. [dlb, 19 jan 87]
 *	Initial processor binding. [avie, 30 sep 86]
 *	Initial sleep/wakeup. [dbg, 12 jun 86]
 *	Created. [avie, 08 apr 86]
 */
/*
 *	File:	sched_prim.c
 *	Author:	Avadis Tevanian, Jr.
 *	Date:	1986
 *
 *	Scheduling primitives
 *
 */



#include <debug.h>

#include <mach/mach_types.h>
#include <mach/machine.h>
#include <mach/policy.h>
#include <mach/sync_policy.h>
#include <mach/thread_act.h>

#include <machine/machine_routines.h>
#include <machine/sched_param.h>
#include <machine/machine_cpu.h>
#include <machine/limits.h>
#include <machine/atomic.h>

#include <machine/commpage.h>

#include <kern/kern_types.h>
#include <kern/backtrace.h>
#include <kern/clock.h>
#include <kern/cpu_number.h>
#include <kern/cpu_data.h>
#include <kern/smp.h>
#include <kern/smr.h>a
#include <kern/debug.h>
#include <kern/macro_help.h>
#include <kern/machine.h>
#include <kern/misc_protos.h>
#include <kern/monotonic.h>
#include <kern/processor.h>
#include <kern/queue.h>
#include <kern/recount.h>
#include <kern/restartable.h>
#include <kern/sched.h>
#include <kern/sched_prim.h>
#include <kern/sched_rt.h>
#include <kern/sfi.h>
#include <kern/syscall_subr.h>
#include <kern/task.h>
#include <kern/thread.h>
#include <kern/thread_group.h>
#include <kern/ledger.h>
#include <kern/timer_queue.h>
#include <kern/waitq.h>
#include <kern/policy_internal.h>
#include <kern/rbtree.h>

#include <vm/pmap.h>
#include <vm/vm_kern.h>
#include <vm/vm_map.h>
#include <vm/vm_pageout_xnu.h>

#include <mach/sdt.h>
#include <mach/mach_host.h>
#include <mach/host_info.h>

#include <sys/kdebug.h>
#include <kperf/kperf.h>
#include <kern/kpc.h>
#include <san/kasan.h>
#include <kern/pms.h>
#include <kern/host.h>
#include <stdatomic.h>
#include <os/atomic_private.h>
#include <os/log.h>

/* Linux includes for EEVDF scheduler concepts */
#include <linux/sched.h>
#include <linux/sched/deadline.h>
#include <linux/sched/rt.h>
#include <linux/sched/stat.h>
#include <linux/sched/sysctl.h>
#include <linux/sched/topology.h>
#include <linux/sched/loadavg.h>
#include <linux/sched/cputime.h>
#include <linux/sched/clock.h>
#include <linux/sched/idle.h>
#include <linux/sched/wake_q.h>
#include <linux/cpumask.h>
#include <linux/cpuidle.h>
#include <linux/ktime.h>
#include <linux/rbtree.h>
#include <linux/percpu.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <linux/tick.h>
#include <linux/timer.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/smp.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/irq_work.h>
#include <linux/stop_machine.h>
#include <trace/events/sched.h>

#define KTRC KDBG_RELEASE

struct sched_statistics PERCPU_DATA(sched_stats);
bool sched_stats_active;

TUNABLE(bool, cpulimit_affects_quantum, "cpulimit_affects_quantum", true);

TUNABLE(uint32_t, nonurgent_preemption_timer_us, "nonurgent_preemption_timer", 50); /* microseconds */
static uint64_t nonurgent_preemption_timer_abs = 0;

#define         DEFAULT_PREEMPTION_RATE         100             /* (1/s) */
TUNABLE(int, default_preemption_rate, "preempt", DEFAULT_PREEMPTION_RATE);

#define         DEFAULT_BG_PREEMPTION_RATE      400             /* (1/s) */
TUNABLE(int, default_bg_preemption_rate, "bg_preempt", DEFAULT_BG_PREEMPTION_RATE);

#if XNU_TARGET_OS_XR
#define         MAX_UNSAFE_RT_QUANTA               1
#define         SAFE_RT_MULTIPLIER                 5
#else
#define         MAX_UNSAFE_RT_QUANTA               100
#define         SAFE_RT_MULTIPLIER                 2
#endif /* XNU_TARGET_OS_XR */

#define         MAX_UNSAFE_FIXED_QUANTA               100
#define         SAFE_FIXED_MULTIPLIER                 SAFE_RT_MULTIPLIER

TUNABLE_DEV_WRITEABLE(int, max_unsafe_rt_quanta, "max_unsafe_rt_quanta", MAX_UNSAFE_RT_QUANTA);
TUNABLE_DEV_WRITEABLE(int, max_unsafe_fixed_quanta, "max_unsafe_fixed_quanta", MAX_UNSAFE_FIXED_QUANTA);

TUNABLE_DEV_WRITEABLE(int, safe_rt_multiplier, "safe_rt_multiplier", SAFE_RT_MULTIPLIER);
TUNABLE_DEV_WRITEABLE(int, safe_fixed_multiplier, "safe_fixed_multiplier", SAFE_FIXED_MULTIPLIER);

#define         MAX_POLL_QUANTA                 2
TUNABLE(int, max_poll_quanta, "poll", MAX_POLL_QUANTA);

#define         SCHED_POLL_YIELD_SHIFT          4               /* 1/16 */
int             sched_poll_yield_shift = SCHED_POLL_YIELD_SHIFT;

uint64_t        max_poll_computation;

uint64_t        max_unsafe_rt_computation;
uint64_t        max_unsafe_fixed_computation;
uint64_t        sched_safe_rt_duration;
uint64_t        sched_safe_fixed_duration;

/* EEVDF Scheduling Constants */
#define EEVDF_MIN_GRANULARITY_NS      3000000LL      /* 3ms */
#define EEVDF_LATENCY_NS               20000000LL     /* 20ms */
#define EEVDF_WAKEUP_GRANULARITY_NS    1500000LL      /* 1.5ms */
#define EEVDF_VIRTUAL_CPU_RATIO        1024LL
#define EEVDF_WEIGHT_NICE_0            1024
#define EEVDF_WEIGHT_IDLE               3

/* EEVDF Data Structures */


struct eevdf_sched_entity {
    struct rb_node          run_node;
    u32                on_rq;
    u64                vruntime;
    u64                deadline;
    u64                virtual_deadline;
    u64                slice;
    u64                exec_start;
    u64                sum_exec_runtime;
    u64                prev_sum_exec_runtime;
    u32                load_weight;
    struct eevdf_sched_entity *parent;
    struct eevdf_cfs_rq     *cfs_rq;
    thread_t                thread;
    spl_t                   lock;
    u64                last_ran_time;
    int                     sched_pri;
};

struct eevdf_cfs_rq {
    struct rb_root          tasks_timeline;
    struct rb_node          *rb_leftmost;
    struct eevdf_sched_entity *curr;
    u32                nr_running;
    u32                load_weight;
    u64                min_vruntime;
    u64                min_deadline;
    processor_t             processor;
    spl_t                   lock;
    u64                exec_clock;
    u64                last_update;
};

struct eevdf_rq {
    struct eevdf_cfs_rq     cfs;
    u64                last_tick;
};

/* EEVDF per-thread data */
struct eevdf_thread_data {
    struct eevdf_sched_entity se;
    uint64_t last_enqueue_time;
    uint64_t last_dequeue_time;
    uint32_t enqueue_count;
    uint32_t preempt_count;
};

/* EEVDF statistics */
struct eevdf_stats {
    uint64_t total_enqueues;
    uint64_t total_dequeues;
    uint64_t total_preemptions;
    uint64_t deadline_misses;
    uint64_t vruntime_max;
    uint64_t vruntime_min;
    uint32_t load_sum;
};

static struct eevdf_stats PERCPU_DATA(eevdf_stats);

/* Per-CPU EEVDF runqueue */
static struct eevdf_rq PERCPU_DATA(eevdf_rq);

#if defined(CONFIG_SCHED_TIMESHARE_CORE)

uint32_t        std_quantum;
uint32_t        min_std_quantum;
uint32_t        bg_quantum;

uint32_t        std_quantum_us;
uint32_t        bg_quantum_us;

#endif /* CONFIG_SCHED_TIMESHARE_CORE */

uint32_t        thread_depress_time;
uint32_t        default_timeshare_computation;
uint32_t        default_timeshare_constraint;


#if defined(CONFIG_SCHED_TIMESHARE_CORE)

_Atomic uint32_t        sched_tick;
uint32_t                sched_tick_interval;

/* Timeshare load calculation interval (15ms) */
uint32_t                sched_load_compute_interval_us = 15000;
uint64_t                sched_load_compute_interval_abs;
static _Atomic uint64_t sched_load_compute_deadline;

uint32_t        sched_pri_shifts[TH_BUCKET_MAX];
uint32_t        sched_fixed_shift;

uint32_t        sched_decay_usage_age_factor = 1; /* accelerate 5/8^n usage aging */

/* Allow foreground to decay past default to resolve inversions */
#define DEFAULT_DECAY_BAND_LIMIT ((BASEPRI_FOREGROUND - BASEPRI_DEFAULT) + 2)
int             sched_pri_decay_band_limit = DEFAULT_DECAY_BAND_LIMIT;

/* Defaults for timer deadline profiling */
#define TIMER_DEADLINE_TRACKING_BIN_1_DEFAULT 2000000 /* Timers with deadlines <=
	                                               * 2ms */
#define TIMER_DEADLINE_TRACKING_BIN_2_DEFAULT 5000000 /* Timers with deadlines
	                                               *   <= 5ms */

uint64_t timer_deadline_tracking_bin_1;
uint64_t timer_deadline_tracking_bin_2;

#endif /* CONFIG_SCHED_TIMESHARE_CORE */

thread_t sched_maintenance_thread;

LCK_GRP_DECLARE(cluster_powerdown_grp, "cluster_powerdown");
LCK_MTX_DECLARE(cluster_powerdown_lock, &cluster_powerdown_grp);

/* interrupts disabled lock to guard core online, recommendation, pcs state, scheduling policy bits */
decl_simple_lock_data(, sched_available_cores_lock);

/*
 * Locked by sched_available_cores_lock.
 * cluster_powerdown_lock is held while making changes to CPU offline state.
 */
static struct global_powered_cores_state {
	/*
	 * Set when PCS has seen all cores boot up and is ready to manage online
	 * state.  CPU recommendation works before this point.
	 */
	bool    pcs_init_completed;

	cpumap_t pcs_managed_cores;         /* all cores managed by the PCS */

	/*
	 * Inputs for CPU offline state provided by clients
	 */
	cpumap_t pcs_requested_online_user; /* updated by processor_start/exit from userspace */
	cpumap_t pcs_requested_online_clpc_user;
	cpumap_t pcs_requested_online_clpc_system;
	cpumap_t pcs_required_online_pmgr;  /* e.g. ANE needs these powered for their rail to be happy */
	cpumap_t pcs_required_online_system;  /* e.g. smt1 for interrupts, boot processor unless boot arg is set, makes them disable instead of sleep */

	/*
	 * When a suspend count is held, all CPUs must be powered up.
	 */
	int32_t  pcs_powerdown_suspend_count;

	/*
	 * Disable automatic cluster powerdown in favor of explicit user core online control
	 */
	bool     pcs_user_online_core_control;
	bool     pcs_wants_kernel_sleep;
	bool     pcs_in_kernel_sleep;

	struct powered_cores_state {
		/*
		 * The input into the recommendation computation from update powered cores.
		 */
		cpumap_t pcs_powerdown_recommended_cores;

		/*
		 * These cores are online and are not powered down.
		 *
		 * Processors with processor->processor_online bit set.
		 */
		cpumap_t pcs_online_cores;

		/*
		 * These cores are disabled or powered down
		 * due to temporary reasons and will come back under presented load
		 * so the user should still see them as active in the cpu count.
		 *
		 * Processors with processor->shutdown_temporary bit set.
		 */
		cpumap_t pcs_tempdown_cores;
	} pcs_effective;

	/* The 'goal state' PCS has computed and is attempting to apply */
	struct powered_cores_state pcs_requested;

	/*
	 * Inputs into CPU recommended cores provided by clients.
	 * Note that these may be changed under the available cores lock and
	 * become effective while sched_update_powered_cores_drops_lock is in
	 * the middle of making changes to CPU online state.
	 */

	cpumap_t        pcs_requested_recommended_clpc;
	cpumap_t        pcs_requested_recommended_clpc_system;
	cpumap_t        pcs_requested_recommended_clpc_user;
	bool            pcs_recommended_clpc_failsafe_active;
	bool            pcs_sleep_override_recommended;

	/*
	 * These cores are recommended and can be used for execution
	 * of non-bound threads.
	 *
	 * Processors with processor->is_recommended bit set.
	 */
	cpumap_t pcs_recommended_cores;

	/*
	 * These are for the debugger.
	 * Use volatile to stop the compiler from optimizing out the stores
	 */
	volatile processor_reason_t pcs_in_flight_reason;
	volatile processor_reason_t pcs_previous_reason;
} pcs = {
	/*
	 * Powerdown is suspended during boot until after all CPUs finish booting,
	 * released by sched_cpu_init_completed.
	 */
	.pcs_powerdown_suspend_count = 1,
	.pcs_requested_online_user = ALL_CORES_POWERED,
	.pcs_requested_online_clpc_user = ALL_CORES_POWERED,
	.pcs_requested_online_clpc_system = ALL_CORES_POWERED,
	.pcs_in_flight_reason = REASON_NONE,
	.pcs_previous_reason = REASON_NONE,
	.pcs_requested.pcs_powerdown_recommended_cores = ALL_CORES_POWERED,
	.pcs_requested_recommended_clpc = ALL_CORES_RECOMMENDED,
	.pcs_requested_recommended_clpc_system = ALL_CORES_RECOMMENDED,
	.pcs_requested_recommended_clpc_user = ALL_CORES_RECOMMENDED,
};

uint64_t sysctl_sched_recommended_cores = ALL_CORES_RECOMMENDED;

static int sched_last_resort_cpu(void);

static void
sched_update_recommended_cores_locked(
	processor_reason_t reason,
	cpumap_t core_going_offline,
	struct pulled_thread_queue *threadq);

static __result_use_check struct pulled_thread_queue *
sched_update_powered_cores_drops_lock(
	processor_reason_t requested_reason,
	spl_t s,
	struct pulled_thread_queue *threadq);

#if __arm64__
static void sched_recommended_cores_maintenance(void);
uint64_t    perfcontrol_failsafe_starvation_threshold;
extern char *proc_name_address(struct proc *p);
#endif /* __arm64__ */

uint64_t        sched_one_second_interval;
boolean_t       allow_direct_handoff = TRUE;

/* Forwards */

#if defined(CONFIG_SCHED_TIMESHARE_CORE)

static void load_shift_init(void);
static void preempt_pri_init(void);

#endif /* CONFIG_SCHED_TIMESHARE_CORE */

thread_t        processor_idle(
	thread_t                        thread,
	processor_t                     processor);

static ast_t
csw_check_locked(
	thread_t        thread,
	processor_t     processor,
	processor_set_t pset,
	ast_t           check_reason);

static void processor_setrun(
	processor_t                    processor,
	thread_t                       thread,
	sched_options_t                options);

static void
sched_timer_deadline_tracking_init(void);

#if     DEBUG
extern int debug_task;
#define TLOG(a, fmt, args...) if(debug_task & a) kprintf(fmt, ## args)
#else
#define TLOG(a, fmt, args...) do {} while (0)
#endif

static processor_t
thread_bind_internal(
	thread_t                thread,
	processor_t             processor);

static void
sched_vm_group_maintenance(void);

/* EEVDF Function Prototypes */
static void eevdf_init(void);
static void eevdf_enqueue_thread(processor_t processor, thread_t thread, sched_options_t options);
static thread_t eevdf_dequeue_thread(processor_t processor, int priority, thread_t current_thread);
static boolean_t eevdf_queue_empty(processor_t processor);
static void eevdf_processor_init(processor_t processor);
static void eevdf_update_curr(struct eevdf_cfs_rq *cfs_rq);
static void eevdf_update_min_vruntime(struct eevdf_cfs_rq *cfs_rq);
static void eevdf_place_entity(struct eevdf_cfs_rq *cfs_rq, struct eevdf_sched_entity *se, boolean_t initial);
static void eevdf_check_preempt_tick(struct eevdf_cfs_rq *cfs_rq, struct eevdf_sched_entity *curr);
static uint32_t prio_to_weight(int prio);
static uint64_t calc_delta_fair(uint64_t delta, struct eevdf_sched_entity *se);
static int eevdf_entity_before(struct eevdf_sched_entity *left, struct eevdf_sched_entity *right);
static void __eevdf_add_entity(struct eevdf_cfs_rq *cfs_rq, struct eevdf_sched_entity *se);
static struct eevdf_sched_entity * eevdf_leftmost(struct eevdf_cfs_rq *cfs_rq);
static void eevdf_yield(processor_t processor, thread_t thread);

/* EEVDF Helper Functions */

/* Load weight calculation based on Mach priorities */
static inline uint32_t
prio_to_weight(int prio)
{
    /* Map Mach priorities (0-127) to EEVDF weights */
    if (prio >= BASEPRI_REALTIME)
        return 88761;  /* Highest weight for RT */
    else if (prio >= BASEPRI_FOREGROUND)
        return 29154;  /* High weight for foreground */
    else if (prio >= BASEPRI_DEFAULT)
        return 1024;   /* Default weight (nice 0) */
    else if (prio >= BASEPRI_UTILITY)
        return 304;    /* Low weight for utility */
    else
        return 95;     /* Lowest weight for background */
}

/* Calculate vruntime delta based on weight */
static inline uint64_t
calc_delta_fair(uint64_t delta, struct eevdf_sched_entity *se)
{
    uint64_t weight = se->load_weight;
    
    if (unlikely(!weight))
        weight = EEVDF_WEIGHT_NICE_0;
    
    return (delta * EEVDF_WEIGHT_NICE_0) / weight;
}

/* Update the current task's runtime statistics */
static void
eevdf_update_curr(struct eevdf_cfs_rq *cfs_rq)
{
    struct eevdf_sched_entity *curr = cfs_rq->curr;
    uint64_t now = mach_absolute_time();
    uint64_t delta_exec;
    
    if (!curr)
        return;
    
    delta_exec = now - curr->exec_start;
    if (!delta_exec)
        return;
    
    curr->exec_start = now;
    curr->sum_exec_runtime += delta_exec;
    cfs_rq->exec_clock += delta_exec;
    
    /* Update vruntime based on weight */
    curr->vruntime += calc_delta_fair(delta_exec, curr);
    
    /* Update min_vruntime */
    if (curr->vruntime < cfs_rq->min_vruntime)
        cfs_rq->min_vruntime = curr->vruntime;
    else if (curr->vruntime > cfs_rq->min_vruntime + EEVDF_LATENCY_NS)
        cfs_rq->min_vruntime = curr->vruntime - EEVDF_LATENCY_NS;
    
    /* Update deadlines for EEVDF */
    if (curr->virtual_deadline > curr->vruntime) {
        curr->deadline = now + (curr->virtual_deadline - curr->vruntime);
    } else {
        /* Missed deadline, set new one */
        curr->virtual_deadline = curr->vruntime + curr->slice;
        curr->deadline = now + curr->slice;
    }
    
    cfs_rq->last_update = now;
}

/* Update min_vruntime for the CFS runqueue */
static void
eevdf_update_min_vruntime(struct eevdf_cfs_rq *cfs_rq)
{
    struct eevdf_sched_entity *curr = cfs_rq->curr;
    struct rb_node *leftmost = cfs_rq->rb_leftmost;
    uint64_t vruntime = cfs_rq->min_vruntime;
    
    if (curr)
        vruntime = curr->vruntime;
    
    if (leftmost) {
        struct eevdf_sched_entity *se = rb_entry(leftmost, struct eevdf_sched_entity, run_node);
        
        if (se->vruntime < vruntime)
            vruntime = se->vruntime;
    }
    
    /* Clamp min_vruntime to prevent excessive forward/backward movement */
    if (vruntime > cfs_rq->min_vruntime + EEVDF_LATENCY_NS)
        vruntime = cfs_rq->min_vruntime + EEVDF_LATENCY_NS;
    else if (vruntime < cfs_rq->min_vruntime - EEVDF_LATENCY_NS)
        vruntime = cfs_rq->min_vruntime - EEVDF_LATENCY_NS;
    
    cfs_rq->min_vruntime = vruntime;
}

/* Place a scheduling entity on the timeline (determine initial vruntime) */
static void
eevdf_place_entity(struct eevdf_cfs_rq *cfs_rq, struct eevdf_sched_entity *se, boolean_t initial)
{
    uint64_t vruntime = cfs_rq->min_vruntime;
    uint64_t now = mach_absolute_time();
    
    if (!initial) {
        /* Wakeup preemption - give some credit for sleeping */
        unsigned long thresh = EEVDF_WAKEUP_GRANULARITY_NS;
        
        /* Use saved vruntime if available */
        if (se->vruntime > vruntime)
            vruntime = se->vruntime;
        
        /* Don't place it too far ahead to avoid starvation */
        if (vruntime > cfs_rq->min_vruntime + thresh)
            vruntime = cfs_rq->min_vruntime + thresh;
        
        /* Give a small boost to wakeups */
        if (se->vruntime < cfs_rq->min_vruntime)
            vruntime = cfs_rq->min_vruntime - EEVDF_MIN_GRANULARITY_NS;
    }
    
    se->vruntime = vruntime;
    
    /* Initialize slice based on weight */
    se->slice = EEVDF_MIN_GRANULARITY_NS;
    if (se->load_weight < EEVDF_WEIGHT_NICE_0)
        se->slice = (se->slice * EEVDF_WEIGHT_NICE_0) / se->load_weight;
    else if (se->load_weight > EEVDF_WEIGHT_NICE_0)
        se->slice = (se->slice * EEVDF_WEIGHT_NICE_0) / se->load_weight;
    
    /* Ensure slice is at least minimum granularity */
    if (se->slice < EEVDF_MIN_GRANULARITY_NS)
        se->slice = EEVDF_MIN_GRANULARITY_NS;
    
    /* Initialize virtual deadline and absolute deadline */
    se->virtual_deadline = se->vruntime + se->slice;
    se->deadline = now + se->slice;
    se->last_ran_time = now;
}

/* Compare function for EEVDF red-black tree (based on vruntime and deadline) */
static inline int
eevdf_entity_before(struct eevdf_sched_entity *left, struct eevdf_sched_entity *right)
{
    if (left->vruntime < right->vruntime)
        return 1;
    if (left->vruntime > right->vruntime)
        return 0;
    
    /* If vruntime equal, use virtual deadline */
    if (left->virtual_deadline < right->virtual_deadline)
        return 1;
    if (left->virtual_deadline > right->virtual_deadline)
        return 0;
    
    /* Tie-break by thread ID for stability */
    return (uintptr_t)left->thread < (uintptr_t)right->thread;
}

/* Insert entity into the RB-tree */
static void
__eevdf_add_entity(struct eevdf_cfs_rq *cfs_rq, struct eevdf_sched_entity *se)
{
    struct rb_node **link = &cfs_rq->tasks_timeline.rb_node;
    struct rb_node *parent = NULL;
    struct eevdf_sched_entity *entry;
    int leftmost = 1;
    
    while (*link) {
        parent = *link;
        entry = rb_entry(parent, struct eevdf_sched_entity, run_node);
        
        if (eevdf_entity_before(se, entry)) {
            link = &parent->rb_left;
        } else {
            link = &parent->rb_right;
            leftmost = 0;
        }
    }
    
    rb_link_node(&se->run_node, parent, link);
    rb_insert_color(&se->run_node, &cfs_rq->tasks_timeline);
    
    if (leftmost)
        cfs_rq->rb_leftmost = &se->run_node;
}

/* Get the leftmost (most eligible) entity */
static struct eevdf_sched_entity *
eevdf_leftmost(struct eevdf_cfs_rq *cfs_rq)
{
    struct rb_node *leftmost = cfs_rq->rb_leftmost;
    
    if (!leftmost)
        return NULL;
    
    return rb_entry(leftmost, struct eevdf_sched_entity, run_node);
}

/* Check if current task should be preempted based on EEVDF criteria */
static void
eevdf_check_preempt_tick(struct eevdf_cfs_rq *cfs_rq, struct eevdf_sched_entity *curr)
{
    uint64_t now = mach_absolute_time();
    struct eevdf_sched_entity *next = eevdf_leftmost(cfs_rq);
    uint64_t delta;
    
    if (!next || next == curr)
        return;
    
    /* Check if current has exceeded its virtual deadline */
    if (curr->virtual_deadline <= curr->vruntime) {
        ast_on(AST_PREEMPT);
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PREEMPT) | DBG_FUNC_NONE,
            (uintptr_t)curr->thread, 1, 0, 0, 0); /* 1 = deadline miss */
        return;
    }
    
    /* Check if next has earlier virtual deadline (eligible for preemption) */
    if (next->virtual_deadline < curr->virtual_deadline) {
        ast_on(AST_PREEMPT);
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PREEMPT) | DBG_FUNC_NONE,
            (uintptr_t)curr->thread, 2, next->virtual_deadline, curr->virtual_deadline, 0); /* 2 = earlier deadline */
        return;
    }
    
    /* Check if current has exceeded its time slice */
    if (now >= curr->deadline) {
        ast_on(AST_PREEMPT);
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PREEMPT) | DBG_FUNC_NONE,
            (uintptr_t)curr->thread, 3, now - curr->deadline, curr->slice, 0); /* 3 = slice expired */
        return;
    }
    
    /* Check if we've run too long compared to next's vruntime */
    delta = curr->vruntime - next->vruntime;
    if (delta > EEVDF_MIN_GRANULARITY_NS) {
        ast_on(AST_PREEMPT);
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PREEMPT) | DBG_FUNC_NONE,
            (uintptr_t)curr->thread, 4, delta, EEVDF_MIN_GRANULARITY_NS, 0); /* 4 = vruntime delta */
    }
}

/* Enqueue a thread into EEVDF runqueue */
static void
eevdf_enqueue_thread(processor_t processor, thread_t thread, sched_options_t options)
{
    struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
    struct eevdf_cfs_rq *cfs_rq = &rq->cfs;
    struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
    uint64_t now = mach_absolute_time();
    
    if (!se) {
        /* First time we see this thread - initialize EEVDF entity */
        se = kalloc(sizeof(struct eevdf_sched_entity));
        if (!se) {
            panic("Failed to allocate EEVDF scheduling entity for thread %p", thread);
        }
        
        se->thread = thread;
        se->cfs_rq = cfs_rq;
        se->load_weight = prio_to_weight(thread->sched_pri);
        se->on_rq = 0;
        se->sum_exec_runtime = 0;
        se->prev_sum_exec_runtime = 0;
        se->vruntime = 0;
        se->sched_pri = thread->sched_pri;
        
        thread->sched_priv = (void *)se;
    }
    
    /* Update current task's statistics before enqueue */
    eevdf_update_curr(cfs_rq);
    
    if (!se->on_rq) {
        /* Place the entity on the timeline */
        eevdf_place_entity(cfs_rq, se, options & SCHED_HEADQ);
        
        /* Add to red-black tree */
        __eevdf_add_entity(cfs_rq, se);
        se->on_rq = 1;
        cfs_rq->nr_running++;
        cfs_rq->load_weight += se->load_weight;
        
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_ENQUEUE) | DBG_FUNC_NONE,
            (uintptr_t)thread_tid(thread), se->vruntime, se->virtual_deadline, cfs_rq->nr_running, 0);
    }
    
    se->exec_start = now;
    
    /* Update min_vruntime */
    eevdf_update_min_vruntime(cfs_rq);
    
    /* Check if we need to preempt current thread */
    if (cfs_rq->curr && cfs_rq->curr != se) {
        if (eevdf_entity_before(se, cfs_rq->curr)) {
            ast_on(AST_PREEMPT);
            KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PREEMPT) | DBG_FUNC_NONE,
                (uintptr_t)thread_tid(thread), 5, se->vruntime, cfs_rq->curr->vruntime, 0); /* 5 = new thread preempts */
        }
    }
    
    /* Update rq timestamp */
    rq->last_tick = now;
}

/* Dequeue a thread from EEVDF runqueue - returns thread to run next */
static thread_t
eevdf_dequeue_thread(processor_t processor, int priority, thread_t current_thread)
{
    struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
    struct eevdf_cfs_rq *cfs_rq = &rq->cfs;
    struct eevdf_sched_entity *next;
    thread_t new_thread;
    uint64_t now = mach_absolute_time();
    
    /* Update current task's statistics */
    eevdf_update_curr(cfs_rq);
    
    /* Get the most eligible thread (leftmost in the tree) */
    next = eevdf_leftmost(cfs_rq);
    if (!next) {
        /* No runnable threads, return current (idle) thread */
        return current_thread;
    }
    
    if (next->thread == current_thread) {
        /* Current thread is still the best candidate */
        eevdf_check_preempt_tick(cfs_rq, next);
        
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_DISPATCH) | DBG_FUNC_NONE,
            (uintptr_t)thread_tid(current_thread), 0, next->vruntime, next->virtual_deadline, 0);
        
        return current_thread;
    }
    
    /* Dequeue the selected thread from runqueue */
    rb_erase(&next->run_node, &cfs_rq->tasks_timeline);
    cfs_rq->rb_leftmost = rb_first(&cfs_rq->tasks_timeline);
    next->on_rq = 0;
    cfs_rq->nr_running--;
    cfs_rq->load_weight -= next->load_weight;
    
    /* Update min_vruntime after removal */
    eevdf_update_min_vruntime(cfs_rq);
    
    /* Prepare the new thread for execution */
    new_thread = next->thread;
    cfs_rq->curr = next;
    next->exec_start = now;
    next->last_ran_time = now;
    
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_DISPATCH) | DBG_FUNC_NONE,
        (uintptr_t)thread_tid(new_thread), 1, next->vruntime, next->virtual_deadline, cfs_rq->nr_running);
    
    return new_thread;
}

/* Check if EEVDF runqueue is empty */
static boolean_t
eevdf_queue_empty(processor_t processor)
{
    struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
    return (rq->cfs.nr_running == 0);
}

/* Initialize EEVDF per-processor data */
static void
eevdf_processor_init(processor_t processor)
{
    struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
    
    rq->cfs.tasks_timeline = RB_ROOT;
    rq->cfs.rb_leftmost = NULL;
    rq->cfs.curr = NULL;
    rq->cfs.nr_running = 0;
    rq->cfs.load_weight = 0;
    rq->cfs.min_vruntime = 0;
    rq->cfs.min_deadline = 0;
    rq->cfs.processor = processor;
    rq->cfs.exec_clock = 0;
    rq->cfs.last_update = 0;
    rq->last_tick = 0;
    
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_INIT) | DBG_FUNC_NONE,
        processor->cpu_id, (uintptr_t)&rq->cfs, 0, 0, 0);
}

/* EEVDF global initialization */
static void
eevdf_init(void)
{
    int i;
    
    printf("EEVDF Scheduler Initialized\n");
    printf("  min_granularity: %lld ns\n", EEVDF_MIN_GRANULARITY_NS);
    printf("  latency: %lld ns\n", EEVDF_LATENCY_NS);
    printf("  wakeup_granularity: %lld ns\n", EEVDF_WAKEUP_GRANULARITY_NS);
    
    /* Initialize per-CPU runqueues */
    for (i = 0; i < MAX_CPUS; i++) {
        struct eevdf_rq *rq = &PERCPU_DATA(eevdf_rq)[i];
        
        rq->cfs.tasks_timeline = RB_ROOT;
        rq->cfs.rb_leftmost = NULL;
        rq->cfs.curr = NULL;
        rq->cfs.nr_running = 0;
        rq->cfs.load_weight = 0;
        rq->cfs.min_vruntime = 0;
        rq->cfs.min_deadline = 0;
        rq->cfs.processor = NULL;
        rq->cfs.exec_clock = 0;
        rq->cfs.last_update = 0;
        rq->last_tick = 0;
    }
    
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_INIT) | DBG_FUNC_NONE,
        EEVDF_MIN_GRANULARITY_NS, EEVDF_LATENCY_NS, EEVDF_WAKEUP_GRANULARITY_NS, 0, 0);
}

/* Called when a thread voluntarily yields the CPU */
static void
eevdf_yield(processor_t processor, thread_t thread)
{
    struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
    struct eevdf_cfs_rq *cfs_rq = &rq->cfs;
    struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
    uint64_t now = mach_absolute_time();
    
    if (se && se->on_rq) {
        /* Update execution time */
        eevdf_update_curr(cfs_rq);
        
        /* Advance vruntime to encourage yielding (penalize the yielding thread) */
        se->vruntime += EEVDF_MIN_GRANULARITY_NS;
        
        /* Update virtual deadline */
        se->virtual_deadline = se->vruntime + se->slice;
        se->deadline = now + se->slice;
        
        /* Update min_vruntime */
        eevdf_update_min_vruntime(cfs_rq);
        
        /* Reinsert into tree to maintain order (remove and add back) */
        rb_erase(&se->run_node, &cfs_rq->tasks_timeline);
        __eevdf_add_entity(cfs_rq, se);
        
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_YIELD) | DBG_FUNC_NONE,
            (uintptr_t)thread_tid(thread), se->vruntime, se->virtual_deadline, 0, 0);
    }
}

/* Update thread priority and recalculate EEVDF parameters */
static void
eevdf_update_priority(thread_t thread)
{
    struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
    
    if (se) {
        uint32_t old_weight = se->load_weight;
        uint32_t new_weight = prio_to_weight(thread->sched_pri);
        
        if (old_weight != new_weight) {
            se->load_weight = new_weight;
            
            /* Recalculate slice based on new weight */
            se->slice = EEVDF_MIN_GRANULARITY_NS;
            if (se->load_weight < EEVDF_WEIGHT_NICE_0)
                se->slice = (se->slice * EEVDF_WEIGHT_NICE_0) / se->load_weight;
            
            KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PRIORITY_CHANGE) | DBG_FUNC_NONE,
                (uintptr_t)thread_tid(thread), thread->sched_pri, old_weight, new_weight, 0);
        }
    }
}

/* EEVDF Scheduler Operations Structure */
const struct sched_dispatch_table eevdf_sched_dispatch = {
    .sched_name = "EEVDF",
    .sched_pset_init = sched_pset_init_default,
    .sched_processor_init = eevdf_processor_init,
    .sched_pset_init_node = sched_pset_init_node_default,
    .sched_init = eevdf_init,
    .sched_timebase_init = sched_timebase_init_default,
    .sched_maintenance_continue = sched_maintenance_continue_default,
    .sched_processor_enqueue = eevdf_enqueue_thread,
    .sched_processor_queue_empty = eevdf_queue_empty,
    .sched_choose_thread = eevdf_dequeue_thread,
    .sched_processor_queue_remove = NULL,
    .sched_processor_queue_shutdown = NULL,
    .sched_pset_made_schedulable = NULL,
    .sched_pset_stealable_state_changed = NULL,
    .sched_processor_csw_check = NULL,
    .sched_processor_balance = sched_processor_balance_default,
    .shed_processor_bound_count = sched_processor_bound_count_default,
    .sched_processor_runq_count = sched_processor_runq_count_default,
    .sched_pset_has_stealable_threads = NULL,
    .sched_pset_steal_thread = NULL,
    .sched_steal_thread_enabled = NULL,
    .sched_choose_processor = choose_processor_default,
    .sched_choose_node = choose_node_default,
    .sched_initial_quantum_size = sched_initial_quantum_size_default,
    .sched_update_priority = eevdf_update_priority,
    .sched_can_update_priority = NULL,
    .sched_priority_is_urgent = NULL,
    .sched_ipi_policy = sched_ipi_policy_default,
    .sched_update_pset_load_average = NULL,
    .sched_update_pset_avg_execution_time = NULL,
    .sched_thread_avoid_processor = NULL,
    .sched_avoid_processor_enabled = false,
    .sched_run_count_incr = sched_run_count_incr_default,
    .sched_run_count_decr = sched_run_count_decr_default,
    .sched_update_thread_bucket = NULL,
    .sched_rt_init_pset = sched_rt_init_pset_default,
    .sched_rt_runq_scan = NULL,
    .sched_rt_choose_thread = NULL,
    .sched_rt_choose_processor = NULL,
    .sched_qos_max_parallelism = sched_qos_max_parallelism_default,
    .sched_cpu_init_completed = NULL,
    .sched_clutch_thread_group_cpu_time_for_thread = NULL,
    .sched_thread_eligible_for_pset = NULL,
};

/* Register EEVDF as the active scheduler */
SCHED_REGISTER(eevdf_sched_dispatch);

#if defined(CONFIG_SCHED_TIMESHARE_CORE)

int8_t          sched_load_shifts[NRQS];
bitmap_t        sched_preempt_pri[BITMAP_LEN(NRQS_MAX)];

#endif /* CONFIG_SCHED_TIMESHARE_CORE */

/*
 * Statically allocate a buffer to hold the longest possible
 * scheduler description string, as currently implemented.
 * bsd/kern/kern_sysctl.c has a corresponding definition in bsd/
 * to export to userspace via sysctl(3). If either version
 * changes, update the other.
 *
 * Note that in addition to being an upper bound on the strings
 * in the kernel, it's also an exact parameter to PE_get_default(),
 * which interrogates the device tree on some platforms. That
 * API requires the caller know the exact size of the device tree
 * property, so we need both a legacy size (32) and the current size
 * (48) to deal with old and new device trees. The device tree property
 * is similarly padded to a fixed size so that the same kernel image
 * can run on multiple devices with different schedulers configured
 * in the device tree.
 */
char sched_string[SCHED_STRING_MAX_LENGTH];

uint32_t sched_debug_flags = SCHED_DEBUG_FLAG_CHOOSE_PROCESSOR_TRACEPOINTS;

/* Global flag which indicates whether Background Stepper Context is enabled */
static int cpu_throttle_enabled = 1;

#if DEVELOPMENT || DEBUG
int enable_task_set_cluster_type = 0;
bool system_ecore_only = false;
#endif /* DEVELOPMENT || DEBUG */

#if __AMP__ && (DEBUG || DEVELOPMENT)
static char
pset_cluster_type_to_name_char(pset_cluster_type_t pset_type);
#endif /* __AMP__ && (DEBUG || DEVELOPMENT) */

void
sched_init(void)
{
	boolean_t direct_handoff = FALSE;
	kprintf("Scheduler: Default of %s\n", SCHED(sched_name));

	if (!PE_parse_boot_argn("sched_pri_decay_limit", &sched_pri_decay_band_limit, sizeof(sched_pri_decay_band_limit))) {
		/* No boot-args, check in device tree */
		if (!PE_get_default("kern.sched_pri_decay_limit",
		    &sched_pri_decay_band_limit,
		    sizeof(sched_pri_decay_band_limit))) {
			/* Allow decay all the way to normal limits */
			sched_pri_decay_band_limit = DEFAULT_DECAY_BAND_LIMIT;
		}
	}

	kprintf("Setting scheduler priority decay band limit %d\n", sched_pri_decay_band_limit);

	if (PE_parse_boot_argn("sched_debug", &sched_debug_flags, sizeof(sched_debug_flags))) {
		kprintf("Scheduler: Debug flags 0x%08x\n", sched_debug_flags);
	}
	strlcpy(sched_string, SCHED(sched_name), sizeof(sched_string));

#if __arm64__
	clock_interval_to_absolutetime_interval(expecting_ipi_wfe_timeout_usec, NSEC_PER_USEC, &expecting_ipi_wfe_timeout_mt);
#endif /* __arm64__ */

	SCHED(init)();
	sched_timer_deadline_tracking_init();
	SCHED(pset_init)(sched_boot_pset);
	SCHED(rt_init_pset)(sched_boot_pset);
	SCHED(processor_init)(master_processor);

#if __AMP__
	/* By this point, the hardware topology is known. Create and initialize the other psets. */
	for (int cluster_id = 0; cluster_id < ml_get_cluster_count(); cluster_id++) {
		pset_id_t pset_id = (pset_id_t)cluster_id; /* assume 1-to-1 */
		if (pset_array[pset_id] != PROCESSOR_SET_NULL) {
			/* boot pset was initialized above. */
			assert3u(pset_id, ==, sched_boot_pset->pset_id);
			assert3p(pset_array[pset_id], ==, sched_boot_pset);
			continue;
		}
		assert3u(sched_boot_pset->pset_id, !=, pset_id);
		cluster_type_t cluster_type = ml_get_topology_info()->clusters[pset_id].cluster_type;
		__assert_only processor_set_t pset = pset_create(cluster_type, pset_id, pset_id);
		assert3p(pset, !=, PROCESSOR_SET_NULL);
#if (DEBUG || DEVELOPMENT)
		kprintf("sched_init: created pset %u [%p] with pset_cluster_type=%c\n", pset_id, pset, pset_cluster_type_to_name_char(pset->pset_cluster_type));
#endif /* (DEBUG || DEVELOPMENT) */
	}
#endif /* __AMP__ */

	if (PE_parse_boot_argn("direct_handoff", &direct_handoff, sizeof(direct_handoff))) {
		allow_direct_handoff = direct_handoff;
	}

#if DEVELOPMENT || DEBUG
	if (PE_parse_boot_argn("enable_skstsct", &enable_task_set_cluster_type, sizeof(enable_task_set_cluster_type))) {
		system_ecore_only = (enable_task_set_cluster_type == 2);
	}
#endif /* DEVELOPMENT || DEBUG */
}

void
sched_timebase_init(void)
{
	uint64_t        abstime;

	clock_interval_to_absolutetime_interval(1, NSEC_PER_SEC, &abstime);
	sched_one_second_interval = abstime;

	SCHED(timebase_init)();
	sched_realtime_timebase_init();
}

#if defined(CONFIG_SCHED_TIMESHARE_CORE)

void
sched_timeshare_init(void)
{
	/*
	 * Calculate the timeslicing quantum
	 * in us.
	 */
	if (default_preemption_rate < 1) {
		default_preemption_rate = DEFAULT_PREEMPTION_RATE;
	}
	std_quantum_us = (1000 * 1000) / default_preemption_rate;

	printf("standard timeslicing quantum is %d us\n", std_quantum_us);

	if (default_bg_preemption_rate < 1) {
		default_bg_preemption_rate = DEFAULT_BG_PREEMPTION_RATE;
	}
	bg_quantum_us = (1000 * 1000) / default_bg_preemption_rate;

	printf("standard background quantum is %d us\n", bg_quantum_us);

	load_shift_init();
	preempt_pri_init();
	os_atomic_store(&sched_tick, 0, relaxed);
}

void
sched_set_max_unsafe_rt_quanta(int max)
{
	const uint32_t quantum_size = SCHED(initial_quantum_size)(THREAD_NULL);

	max_unsafe_rt_computation = ((uint64_t)max) * quantum_size;

	const int mult = safe_rt_multiplier <= 0 ? 2 : safe_rt_multiplier;
	sched_safe_rt_duration = mult * ((uint64_t)max) * quantum_size;


#if DEVELOPMENT || DEBUG
	max_unsafe_rt_quanta = max;
#else
	/*
	 * On RELEASE kernels, this is only called on boot where
	 * max is already equal to max_unsafe_rt_quanta.
	 */
	assert3s(max, ==, max_unsafe_rt_quanta);
#endif
}

void
sched_set_max_unsafe_fixed_quanta(int max)
{
	const uint32_t quantum_size = SCHED(initial_quantum_size)(THREAD_NULL);

	max_unsafe_fixed_computation = ((uint64_t)max) * quantum_size;

	const int mult = safe_fixed_multiplier <= 0 ? 2 : safe_fixed_multiplier;
	sched_safe_fixed_duration = mult * ((uint64_t)max) * quantum_size;

#if DEVELOPMENT || DEBUG
	max_unsafe_fixed_quanta = max;
#else
	/*
	 * On RELEASE kernels, this is only called on boot where
	 * max is already equal to max_unsafe_fixed_quanta.
	 */
	assert3s(max, ==, max_unsafe_fixed_quanta);
#endif
}

uint64_t
sched_get_quantum_us(void)
{
	uint32_t quantum = SCHED(initial_quantum_size)(THREAD_NULL);

	uint64_t quantum_ns;
	absolutetime_to_nanoseconds(quantum, &quantum_ns);

	return quantum_ns / 1000;
}

void
sched_timeshare_timebase_init(void)
{
	uint64_t        abstime;
	uint32_t        shift;

	/* standard timeslicing quantum */
	clock_interval_to_absolutetime_interval(
		std_quantum_us, NSEC_PER_USEC, &abstime);
	assert((abstime >> 32) == 0 && (uint32_t)abstime != 0);
	std_quantum = (uint32_t)abstime;

	/* smallest remaining quantum (250 us) */
	clock_interval_to_absolutetime_interval(250, NSEC_PER_USEC, &abstime);
	assert((abstime >> 32) == 0 && (uint32_t)abstime != 0);
	min_std_quantum = (uint32_t)abstime;

	/* quantum for background tasks */
	clock_interval_to_absolutetime_interval(
		bg_quantum_us, NSEC_PER_USEC, &abstime);
	assert((abstime >> 32) == 0 && (uint32_t)abstime != 0);
	bg_quantum = (uint32_t)abstime;

	/* scheduler tick interval */
	clock_interval_to_absolutetime_interval(USEC_PER_SEC >> SCHED_TICK_SHIFT,
	    NSEC_PER_USEC, &abstime);
	assert((abstime >> 32) == 0 && (uint32_t)abstime != 0);
	sched_tick_interval = (uint32_t)abstime;

	/* timeshare load calculation interval & deadline initialization */
	clock_interval_to_absolutetime_interval(sched_load_compute_interval_us, NSEC_PER_USEC, &sched_load_compute_interval_abs);
	os_atomic_init(&sched_load_compute_deadline, sched_load_compute_interval_abs);

	/*
	 * Compute conversion factor from usage to
	 * timesharing priorities with 5/8 ** n aging.
	 */
	abstime = (abstime * 5) / 3;
	for (shift = 0; abstime > BASEPRI_DEFAULT; ++shift) {
		abstime >>= 1;
	}
	sched_fixed_shift = shift;

	for (uint32_t i = 0; i < TH_BUCKET_MAX; i++) {
		sched_pri_shifts[i] = INT8_MAX;
	}

	sched_set_max_unsafe_rt_quanta(max_unsafe_rt_quanta);
	sched_set_max_unsafe_fixed_quanta(max_unsafe_fixed_quanta);

	max_poll_computation = ((uint64_t)max_poll_quanta) * std_quantum;
	thread_depress_time = 1 * std_quantum;
	default_timeshare_computation = std_quantum / 2;
	default_timeshare_constraint = std_quantum;

#if __arm64__
	perfcontrol_failsafe_starvation_threshold = (2 * sched_tick_interval);
#endif /* __arm64__ */

	if (nonurgent_preemption_timer_us) {
		clock_interval_to_absolutetime_interval(nonurgent_preemption_timer_us, NSEC_PER_USEC, &abstime);
		nonurgent_preemption_timer_abs = abstime;
	}
}

#endif /* CONFIG_SCHED_TIMESHARE_CORE */

void
sched_check_spill(processor_set_t pset, thread_t thread)
{
	(void)pset;
	(void)thread;

	return;
}

bool
sched_thread_should_yield(processor_t processor, thread_t thread)
{
	struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
	
	(void)thread;
	
	return rq->cfs.nr_running > 1;
}

/* Default implementations of .steal_thread_enabled */
bool
sched_steal_thread_DISABLED(processor_set_t pset)
{
	(void)pset;
	return false;
}

bool
sched_steal_thread_enabled(processor_set_t pset)
{
	return bit_count(pset->node->pset_map) > 1;
}

#if defined(CONFIG_SCHED_TIMESHARE_CORE)

/*
 * Set up values for timeshare
 * loading factors.
 */
static void
load_shift_init(void)
{
	int8_t          k, *p = sched_load_shifts;
	uint32_t        i, j;

	uint32_t        sched_decay_penalty = 1;

	if (PE_parse_boot_argn("sched_decay_penalty", &sched_decay_penalty, sizeof(sched_decay_penalty))) {
		kprintf("Overriding scheduler decay penalty %u\n", sched_decay_penalty);
	}

	if (PE_parse_boot_argn("sched_decay_usage_age_factor", &sched_decay_usage_age_factor, sizeof(sched_decay_usage_age_factor))) {
		kprintf("Overriding scheduler decay usage age factor %u\n", sched_decay_usage_age_factor);
	}

	if (sched_decay_penalty == 0) {
		/*
		 * There is no penalty for timeshare threads for using too much
		 * CPU, so set all load shifts to INT8_MIN. Even under high load,
		 * sched_pri_shift will be >INT8_MAX, and there will be no
		 * penalty applied to threads (nor will sched_usage be updated per
		 * thread).
		 */
		for (i = 0; i < NRQS; i++) {
			sched_load_shifts[i] = INT8_MIN;
		}

		return;
	}

	*p++ = INT8_MIN; *p++ = 0;

	/*
	 * For a given system load "i", the per-thread priority
	 * penalty per quantum of CPU usage is ~2^k priority
	 * levels. "sched_decay_penalty" can cause more
	 * array entries to be filled with smaller "k" values
	 */
	for (i = 2, j = 1 << sched_decay_penalty, k = 1; i < NRQS; ++k) {
		for (j <<= 1; (i < j) && (i < NRQS); ++i) {
			*p++ = k;
		}
	}
}

static void
preempt_pri_init(void)
{
	bitmap_t *p = sched_preempt_pri;

	for (int i = BASEPRI_FOREGROUND; i < MINPRI_KERNEL; ++i) {
		bitmap_set(p, i);
	}

	for (int i = BASEPRI_PREEMPT; i <= MAXPRI; ++i) {
		bitmap_set(p, i);
	}
}

#endif /* CONFIG_SCHED_TIMESHARE_CORE */

void
check_monotonic_time(uint64_t ctime)
{
	processor_t processor = current_processor();
	uint64_t last_dispatch = processor->last_dispatch;

	if (last_dispatch > ctime) {
		panic("Non-monotonic time: last_dispatch at 0x%llx, ctime 0x%llx",
		    last_dispatch, ctime);
	}
}


/*
 *	Thread wait timer expiration.
 *	Runs in timer interrupt context with interrupts disabled.
 */
void
thread_timer_expire(void *p0, __unused void *p1)
{
	thread_t thread = (thread_t)p0;

	assert_thread_magic(thread);

	assert(ml_get_interrupts_enabled() == FALSE);

	thread_lock(thread);

	if (thread->wait_timer_armed) {
		thread->wait_timer_armed = false;
		clear_wait_internal(thread, THREAD_TIMED_OUT);
		/* clear_wait_internal may have dropped and retaken the thread lock */
	}

	thread->wait_timer_active--;

	thread_unlock(thread);
}

/*
 *	thread_unblock:
 *
 *	Unblock thread on wake up.
 *
 *	Returns TRUE if the thread should now be placed on the runqueue.
 *
 *	Thread must be locked.
 *
 *	Called at splsched().
 */
boolean_t
thread_unblock(
	thread_t                thread,
	wait_result_t   wresult)
{
	boolean_t               ready_for_runq = FALSE;
	thread_t                cthread = current_thread();
	uint32_t                new_run_count;
	int                             old_thread_state;

	/*
	 *	Set wait_result.
	 */
	thread->wait_result = wresult;

	/*
	 *	Cancel pending wait timer.
	 */
	if (thread->wait_timer_armed) {
		if (timer_call_cancel(thread->wait_timer)) {
			thread->wait_timer_active--;
		}
		thread->wait_timer_armed = false;
	}

	boolean_t aticontext, pidle;
	ml_get_power_state(&aticontext, &pidle);

	/*
	 *	Update scheduling state: not waiting,
	 *	set running.
	 */
	old_thread_state = thread->state;
	thread->state = (old_thread_state | TH_RUN) &
	    ~(TH_WAIT | TH_UNINT | TH_WAIT_REPORT | TH_WAKING);

	if ((old_thread_state & TH_RUN) == 0) {
		uint64_t ctime = mach_approximate_time();

		check_monotonic_time(ctime);

		thread->last_made_runnable_time = thread->last_basepri_change_time = ctime;
		timer_start(&thread->runnable_timer, ctime);

		ready_for_runq = TRUE;

		if (old_thread_state & TH_WAIT_REPORT) {
			(*thread->sched_call)(SCHED_CALL_UNBLOCK, thread);
		}

		/* Update the runnable thread count */
		new_run_count = SCHED(run_count_incr)(thread);

#if CONFIG_SCHED_AUTO_JOIN
		if (aticontext == FALSE && work_interval_should_propagate(cthread, thread)) {
			work_interval_auto_join_propagate(cthread, thread);
		}
#endif /*CONFIG_SCHED_AUTO_JOIN */

	} else {
		/*
		 * Either the thread is idling in place on another processor,
		 * or it hasn't finished context switching yet.
		 */
		assert((thread->state & TH_IDLE) == 0);
		/*
		 * The run count is only dropped after the context switch completes
		 * and the thread is still waiting, so we should not run_incr here
		 */
		new_run_count = os_atomic_load(&sched_run_buckets[TH_BUCKET_RUN], relaxed);
	}

	/*
	 * Calculate deadline for real-time threads.
	 */
	if (thread->sched_mode == TH_MODE_REALTIME) {
		uint64_t ctime = mach_absolute_time();
		thread->realtime.deadline = thread->realtime.constraint + ctime;
		KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SET_RT_DEADLINE) | DBG_FUNC_NONE,
		    (uintptr_t)thread_tid(thread), thread->realtime.deadline, thread->realtime.computation, 0);
	}

	/*
	 * Clear old quantum, fail-safe computation, etc.
	 */
	thread->quantum_remaining = 0;
	thread->computation_metered = 0;
	thread->reason = AST_NONE;
	thread->block_hint = kThreadWaitNone;

	/* Obtain power-relevant interrupt and "platform-idle exit" statistics.
	 * We also account for "double hop" thread signaling via
	 * the thread callout infrastructure.
	 * DRK: consider removing the callout wakeup counters in the future
	 * they're present for verification at the moment.
	 */

	if (__improbable(aticontext && !(thread_get_tag_internal(thread) & THREAD_TAG_CALLOUT))) {
		DTRACE_SCHED2(iwakeup, struct thread *, thread, struct proc *, current_proc());

		uint64_t ttd = current_processor()->timer_call_ttd;

		if (ttd) {
			if (ttd <= timer_deadline_tracking_bin_1) {
				thread->thread_timer_wakeups_bin_1++;
			} else if (ttd <= timer_deadline_tracking_bin_2) {
				thread->thread_timer_wakeups_bin_2++;
			}
		}

		ledger_credit_thread(thread, thread->t_ledger,
		    task_ledgers.interrupt_wakeups, 1);
		if (pidle) {
			ledger_credit_thread(thread, thread->t_ledger,
			    task_ledgers.platform_idle_wakeups, 1);
		}
	} else if (thread_get_tag_internal(cthread) & THREAD_TAG_CALLOUT) {
		/* TODO: what about an interrupt that does a wake taken on a callout thread? */
		if (cthread->callout_woken_from_icontext) {
			ledger_credit_thread(thread, thread->t_ledger,
			    task_ledgers.interrupt_wakeups, 1);
			thread->thread_callout_interrupt_wakeups++;

			if (cthread->callout_woken_from_platform_idle) {
				ledger_credit_thread(thread, thread->t_ledger,
				    task_ledgers.platform_idle_wakeups, 1);
				thread->thread_callout_platform_idle_wakeups++;
			}

			cthread->callout_woke_thread = TRUE;
		}
	}

	if (thread_get_tag_internal(thread) & THREAD_TAG_CALLOUT) {
		thread->callout_woken_from_icontext = !!aticontext;
		thread->callout_woken_from_platform_idle = !!pidle;
		thread->callout_woke_thread = FALSE;
	}

#if KPERF
	if (ready_for_runq) {
		kperf_make_runnable(thread, aticontext);
	}
#endif /* KPERF */

	KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
	    MACHDBG_CODE(DBG_MACH_SCHED, MACH_MAKE_RUNNABLE) | DBG_FUNC_NONE,
	    (uintptr_t)thread_tid(thread), thread->sched_pri, thread->wait_result,
	    sched_run_buckets[TH_BUCKET_RUN], 0);

	DTRACE_SCHED2(wakeup, struct thread *, thread, struct proc *, current_proc());

	return ready_for_runq;
}

/*
 *	Routine:	thread_allowed_for_handoff
 *	Purpose:
 *		Check if the thread is allowed for handoff operation
 *	Conditions:
 *		thread lock held, IPC locks may be held.
 *	TODO: In future, do not allow handoff if threads have different cluster
 *	recommendations.
 */
boolean_t
thread_allowed_for_handoff(
	thread_t         thread)
{
	thread_t self = current_thread();

	if (allow_direct_handoff &&
	    thread->sched_mode == TH_MODE_REALTIME &&
	    self->sched_mode == TH_MODE_REALTIME) {
		return TRUE;
	}

	return FALSE;
}

/*
 *	Routine:	thread_go
 *	Purpose:
 *		Unblock and dispatch thread.
 *	Conditions:
 *		thread lock held, IPC locks may be held.
 *		thread must have been waiting
 */
void
thread_go(
	thread_t                thread,
	wait_result_t           wresult,
	bool                    try_handoff)
{
	thread_t self = current_thread();

	assert_thread_magic(thread);

	assert(thread->at_safe_point == FALSE);
	assert(thread->wait_event == NO_EVENT64);
	assert(waitq_is_null(thread->waitq));

	assert(!(thread->state & (TH_TERMINATE | TH_TERMINATE2)));
	assert(thread->state & TH_WAIT);

	if (thread->started) {
		assert(thread->state & TH_WAKING);
	}

	thread_lock_assert(thread, LCK_ASSERT_OWNED);

	assert(ml_get_interrupts_enabled() == false);

	if (thread_unblock(thread, wresult)) {
#if SCHED_TRACE_THREAD_WAKEUPS
		backtrace(&thread->thread_wakeup_bt[0],
		    (sizeof(thread->thread_wakeup_bt) / sizeof(uintptr_t)), NULL,
		    NULL);
#endif /* SCHED_TRACE_THREAD_WAKEUPS */
		if (try_handoff && thread_allowed_for_handoff(thread)) {
			thread_reference(thread);
			assert(self->handoff_thread == NULL);
			self->handoff_thread = thread;

			/*
			 * A TH_RUN'ed thread must have a chosen_processor.
			 * thread_setrun would have set it, so we need to
			 * replicate that here.
			 */
			thread->chosen_processor = current_processor();
		} else {
			thread_setrun(thread, SCHED_PREEMPT | SCHED_TAILQ);
		}
	}
}

/*
 *	Routine:	thread_mark_wait_locked
 *	Purpose:
 *		Mark a thread as waiting.  If, given the circumstances,
 *		it doesn't want to wait (i.e. already aborted), then
 *		indicate that in the return value.
 *	Conditions:
 *		at splsched() and thread is locked.
 */
__private_extern__
wait_result_t
thread_mark_wait_locked(
	thread_t                        thread,
	wait_interrupt_t        interruptible_orig)
{
	boolean_t                       at_safe_point;
	wait_interrupt_t        interruptible = interruptible_orig;

	if (thread->state & TH_IDLE) {
		panic("Invalid attempt to wait while running the idle thread");
	}

	assert(!(thread->state & (TH_WAIT | TH_WAKING | TH_IDLE | TH_UNINT | TH_TERMINATE2 | TH_WAIT_REPORT)));

	/*
	 *	The thread may have certain types of interrupts/aborts masked
	 *	off.  Even if the wait location says these types of interrupts
	 *	are OK, we have to honor mask settings (outer-scoped code may
	 *	not be able to handle aborts at the moment).
	 */
	interruptible &= TH_OPT_INTMASK;
	if (interruptible > (thread->options & TH_OPT_INTMASK)) {
		interruptible = thread->options & TH_OPT_INTMASK;
	}

	at_safe_point = (interruptible == THREAD_ABORTSAFE);

	if (interruptible == THREAD_UNINT ||
	    !(thread->sched_flags & TH_SFLAG_ABORT) ||
	    (!at_safe_point &&
	    (thread->sched_flags & TH_SFLAG_ABORTSAFELY))) {
		if (!(thread->state & TH_TERMINATE)) {
			DTRACE_SCHED(sleep);
		}

		int state_bits = TH_WAIT;
		if (!interruptible) {
			state_bits |= TH_UNINT;
		}
		if (thread->sched_call) {
			wait_interrupt_t mask = THREAD_WAIT_NOREPORT_USER;
			if (is_kerneltask(get_threadtask(thread))) {
				mask = THREAD_WAIT_NOREPORT_KERNEL;
			}
			if ((interruptible_orig & mask) == 0) {
				state_bits |= TH_WAIT_REPORT;
			}
		}
		thread->state |= state_bits;
		thread->at_safe_point = at_safe_point;

		/* TODO: pass this through assert_wait instead, have
		 * assert_wait just take a struct as an argument */
		assert(!thread->block_hint);
		thread->block_hint = thread->pending_block_hint;
		thread->pending_block_hint = kThreadWaitNone;

		return thread->wait_result = THREAD_WAITING;
	} else {
		if (thread->sched_flags & TH_SFLAG_ABORTSAFELY) {
			thread->sched_flags &= ~TH_SFLAG_ABORTED_MASK;
		}
	}
	thread->pending_block_hint = kThreadWaitNone;

	return thread->wait_result = THREAD_INTERRUPTED;
}

/*
 *	Routine:	thread_interrupt_level
 *	Purpose:
 *	        Set the maximum interruptible state for the
 *		current thread.  The effective value of any
 *		interruptible flag passed into assert_wait
 *		will never exceed this.
 *
 *		Useful for code that must not be interrupted,
 *		but which calls code that doesn't know that.
 *	Returns:
 *		The old interrupt level for the thread.
 */
__private_extern__
wait_interrupt_t
thread_interrupt_level(
	wait_interrupt_t new_level)
{
	thread_t thread = current_thread();
	wait_interrupt_t result = thread->options & TH_OPT_INTMASK;

	thread->options = (thread->options & ~TH_OPT_INTMASK) | (new_level & TH_OPT_INTMASK);

	return result;
}

/*
 *	assert_wait:
 *
 *	Assert that the current thread is about to go to
 *	sleep until the specified event occurs.
 */
wait_result_t
assert_wait(
	event_t                         event,
	wait_interrupt_t        interruptible)
{
	if (__improbable(event == NO_EVENT)) {
		panic("%s() called with NO_EVENT", __func__);
	}

	KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
	    MACHDBG_CODE(DBG_MACH_SCHED, MACH_WAIT) | DBG_FUNC_NONE,
	    VM_KERNEL_UNSLIDE_OR_PERM(event), 0, 0, 0, 0);

	struct waitq *waitq;
	waitq = global_eventq(event);
	return waitq_assert_wait64(waitq, CAST_EVENT64_T(event), interruptible, TIMEOUT_WAIT_FOREVER);
}

/*
 *	assert_wait_queue:
 *
 *	Return the global waitq for the specified event
 */
struct waitq *
assert_wait_queue(
	event_t                         event)
{
	return global_eventq(event);
}

wait_result_t
assert_wait_timeout(
	event_t                         event,
	wait_interrupt_t        interruptible,
	uint32_t                        interval,
	uint32_t                        scale_factor)
{
	thread_t                        thread = current_thread();
	wait_result_t           wresult;
	uint64_t                        deadline;
	spl_t                           s;

	if (__improbable(event == NO_EVENT)) {
		panic("%s() called with NO_EVENT", __func__);
	}

	struct waitq *waitq;
	waitq = global_eventq(event);

	s = splsched();
	waitq_lock(waitq);

	clock_interval_to_deadline(interval, scale_factor, &deadline);

	KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
	    MACHDBG_CODE(DBG_MACH_SCHED, MACH_WAIT) | DBG_FUNC_NONE,
	    VM_KERNEL_UNSLIDE_OR_PERM(event), interruptible, deadline, 0, 0);

	wresult = waitq_assert_wait64_locked(waitq, CAST_EVENT64_T(event),
	    interruptible,
	    TIMEOUT_URGENCY_SYS_NORMAL,
	    deadline, TIMEOUT_NO_LEEWAY,
	    thread);

	waitq_unlock(waitq);
	splx(s);
	return wresult;
}

wait_result_t
assert_wait_timeout_with_leeway(
	event_t                         event,
	wait_interrupt_t        interruptible,
	wait_timeout_urgency_t  urgency,
	uint32_t                        interval,
	uint32_t                        leeway,
	uint32_t                        scale_factor)
{
	thread_t                        thread = current_thread();
	wait_result_t           wresult;
	uint64_t                        deadline;
	uint64_t                        abstime;
	uint64_t                        slop;
	uint64_t                        now;
	spl_t                           s;

	if (__improbable(event == NO_EVENT)) {
		panic("%s() called with NO_EVENT", __func__);
	}

	now = mach_absolute_time();
	clock_interval_to_absolutetime_interval(interval, scale_factor, &abstime);
	deadline = now + abstime;

	clock_interval_to_absolutetime_interval(leeway, scale_factor, &slop);

	struct waitq *waitq;
	waitq = global_eventq(event);

	s = splsched();
	waitq_lock(waitq);

	KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
	    MACHDBG_CODE(DBG_MACH_SCHED, MACH_WAIT) | DBG_FUNC_NONE,
	    VM_KERNEL_UNSLIDE_OR_PERM(event), interruptible, deadline, 0, 0);

	wresult = waitq_assert_wait64_locked(waitq, CAST_EVENT64_T(event),
	    interruptible,
	    urgency, deadline, slop,
	    thread);

	waitq_unlock(waitq);
	splx(s);
	return wresult;
}

wait_result_t
assert_wait_deadline(
	event_t                         event,
	wait_interrupt_t        interruptible,
	uint64_t                        deadline)
{
	thread_t                        thread = current_thread();
	wait_result_t           wresult;
	spl_t                           s;

	if (__improbable(event == NO_EVENT)) {
		panic("%s() called with NO_EVENT", __func__);
	}

	struct waitq *waitq;
	waitq = global_eventq(event);

	s = splsched();
	waitq_lock(waitq);

	KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
	    MACHDBG_CODE(DBG_MACH_SCHED, MACH_WAIT) | DBG_FUNC_NONE,
	    VM_KERNEL_UNSLIDE_OR_PERM(event), interruptible, deadline, 0, 0);

	wresult = waitq_assert_wait64_locked(waitq, CAST_EVENT64_T(event),
	    interruptible,
	    TIMEOUT_URGENCY_SYS_NORMAL, deadline,
	    TIMEOUT_NO_LEEWAY, thread);
	waitq_unlock(waitq);
	splx(s);
	return wresult;
}

wait_result_t
assert_wait_deadline_with_leeway(
	event_t                         event,
	wait_interrupt_t        interruptible,
	wait_timeout_urgency_t  urgency,
	uint64_t                        deadline,
	uint64_t                        leeway)
{
	thread_t                        thread = current_thread();
	wait_result_t           wresult;
	spl_t                           s;

	if (__improbable(event == NO_EVENT)) {
		panic("%s() called with NO_EVENT", __func__);
	}

	struct waitq *waitq;
	waitq = global_eventq(event);

	s = splsched();
	waitq_lock(waitq);

	KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
	    MACHDBG_CODE(DBG_MACH_SCHED, MACH_WAIT) | DBG_FUNC_NONE,
	    VM_KERNEL_UNSLIDE_OR_PERM(event), interruptible, deadline, 0, 0);

	wresult = waitq_assert_wait64_locked(waitq, CAST_EVENT64_T(event),
	    interruptible,
	    urgency, deadline, leeway,
	    thread);
	waitq_unlock(waitq);
	splx(s);
	return wresult;
}

void
sched_cond_init(
	sched_cond_atomic_t *cond)
{
	os_atomic_init(cond, SCHED_COND_INIT);
}

wait_result_t
sched_cond_wait_parameter(
	sched_cond_atomic_t *cond,
	wait_interrupt_t interruptible,
	thread_continue_t continuation,
	void *parameter)
{
	assert_wait((event_t) cond, interruptible);
	/* clear active bit to indicate future wakeups will have to unblock this thread */
	sched_cond_t new_state = (sched_cond_t) os_atomic_andnot(cond, SCHED_COND_ACTIVE, relaxed);
	if (__improbable(new_state & SCHED_COND_WAKEUP)) {
		/* a wakeup has been issued; undo wait assertion, ack the wakeup, and return */
		thread_t thread = current_thread();
		clear_wait(thread, THREAD_AWAKENED);
		sched_cond_ack(cond);
		return THREAD_AWAKENED;
	}
	return thread_block_parameter(continuation, parameter);
}

wait_result_t
sched_cond_wait(
	sched_cond_atomic_t *cond,
	wait_interrupt_t interruptible,
	thread_continue_t continuation)
{
	return sched_cond_wait_parameter(cond, interruptible, continuation, NULL);
}

sched_cond_t
sched_cond_ack(
	sched_cond_atomic_t *cond)
{
	sched_cond_t new_cond = (sched_cond_t) os_atomic_xor(cond, SCHED_COND_ACTIVE | SCHED_COND_WAKEUP, acquire);
	assert(new_cond & SCHED_COND_ACTIVE);
	return new_cond;
}

kern_return_t
sched_cond_signal(
	sched_cond_atomic_t  *cond,
	thread_t thread)
{
	disable_preemption();
	sched_cond_t old_cond = (sched_cond_t) os_atomic_or_orig(cond, SCHED_COND_WAKEUP, release);
	if (!(old_cond & (SCHED_COND_WAKEUP | SCHED_COND_ACTIVE))) {
		/* this was the first wakeup to be issued AND the thread was inactive */
		thread_wakeup_thread((event_t) cond, thread);
	}
	enable_preemption();
	return KERN_SUCCESS;
}

/*
 * thread_isoncpu:
 *
 * Return TRUE if a thread is running on a processor such that an AST
 * is needed to pull it out of userspace execution, or if executing in
 * the kernel, bring to a context switch boundary that would cause
 * thread state to be serialized in the thread PCB.
 *
 * Thread locked, returns the same way. While locked, fields
 * like "state" cannot change. "runq" can change only from set to unset.
 */
static inline boolean_t
thread_isoncpu(thread_t thread)
{
	/* Not running or runnable */
	if (!(thread->state & TH_RUN)) {
		return FALSE;
	}

	/* Waiting on a runqueue, not currently running */
	/* TODO: This is invalid - it can get dequeued without thread lock, but not context switched. */
	/* TODO: This can also be incorrect for `handoff` cases where
	 * the thread is never enqueued on the runq */
	if (thread_get_runq(thread) != PROCESSOR_NULL) {
		return FALSE;
	}

	/*
	 * Thread does not have a stack yet
	 * It could be on the stack alloc queue or preparing to be invoked
	 */
	if (!thread->kernel_stack) {
		return FALSE;
	}

	/*
	 * Thread must be running on a processor, or
	 * about to run, or just did run. In all these
	 * cases, an AST to the processor is needed
	 * to guarantee that the thread is kicked out
	 * of userspace and the processor has
	 * context switched (and saved register state).
	 */
	return TRUE;
}

/*
 * thread_stop:
 *
 * Force a preemption point for a thread and wait
 * for it to stop running on a CPU. If a stronger
 * guarantee is requested, wait until no longer
 * runnable. Arbitrates access among
 * multiple stop requests. (released by unstop)
 *
 * The thread must enter a wait state and stop via a
 * separate means.
 *
 * Returns FALSE if interrupted.
 */
boolean_t
thread_stop(
	thread_t                thread,
	boolean_t       until_not_runnable)
{
	wait_result_t   wresult;
	spl_t                   s = splsched();
	boolean_t               oncpu;

	wake_lock(thread);
	thread_lock(thread);

	while (thread->state & TH_SUSP) {
		thread->wake_active = TRUE;
		thread_unlock(thread);

		wresult = assert_wait(&thread->wake_active, THREAD_ABORTSAFE);
		wake_unlock(thread);
		splx(s);

		if (wresult == THREAD_WAITING) {
			wresult = thread_block(THREAD_CONTINUE_NULL);
		}

		if (wresult != THREAD_AWAKENED) {
			return FALSE;
		}

		s = splsched();
		wake_lock(thread);
		thread_lock(thread);
	}

	thread->state |= TH_SUSP;

	while ((oncpu = thread_isoncpu(thread)) ||
	    (until_not_runnable && (thread->state & TH_RUN))) {
		if (oncpu) {
			/*
			 * TODO: chosen_processor isn't really the right
			 * thing to IPI here.  We really want `last_processor`,
			 * but we also want to know where to send the IPI
			 * *before* thread_invoke sets last_processor.
			 *
			 * rdar://47149497 (thread_stop doesn't IPI the right core)
			 */
			assert(thread->state & TH_RUN);
			processor_t processor = thread->chosen_processor;
			assert(processor != PROCESSOR_NULL);
			cause_ast_check(processor);
		}

		thread->wake_active = TRUE;
		thread_unlock(thread);

		wresult = assert_wait(&thread->wake_active, THREAD_ABORTSAFE);
		wake_unlock(thread);
		splx(s);

		if (wresult == THREAD_WAITING) {
			wresult = thread_block(THREAD_CONTINUE_NULL);
		}

		if (wresult != THREAD_AWAKENED) {
			thread_unstop(thread);
			return FALSE;
		}

		s = splsched();
		wake_lock(thread);
		thread_lock(thread);
	}

	thread_unlock(thread);
	wake_unlock(thread);
	splx(s);

	/*
	 * We return with the thread unlocked. To prevent it from
	 * transitioning to a runnable state (or from TH_RUN to
	 * being on the CPU), the caller must ensure the thread
	 * is stopped via an external means (such as an AST)
	 */

	return TRUE;
}

/*
 * thread_unstop:
 *
 * Release a previous stop request and set
 * the thread running if appropriate.
 *
 * Use only after a successful stop operation.
 */
void
thread_unstop(
	thread_t        thread)
{
	spl_t           s = splsched();

	wake_lock(thread);
	thread_lock(thread);

	assert((thread->state & (TH_RUN | TH_WAIT | TH_SUSP)) != TH_SUSP);

	if (thread->state & TH_SUSP) {
		thread->state &= ~TH_SUSP;

		if (thread->wake_active) {
			thread->wake_active = FALSE;
			thread_unlock(thread);

			thread_wakeup(&thread->wake_active);
			wake_unlock(thread);
			splx(s);

			return;
		}
	}

	thread_unlock(thread);
	wake_unlock(thread);
	splx(s);
}

/*
 * thread_wait:
 *
 * Wait for a thread to stop running. (non-interruptible)
 *
 */
void
thread_wait(
	thread_t        thread,
	boolean_t       until_not_runnable)
{
	wait_result_t   wresult;
	boolean_t       oncpu;
	processor_t     processor;
	spl_t           s = splsched();

	wake_lock(thread);
	thread_lock(thread);

	/*
	 * Wait until not running on a CPU.  If stronger requirement
	 * desired, wait until not runnable.  Assumption: if thread is
	 * on CPU, then TH_RUN is set, so we're not waiting in any case
	 * where the original, pure "TH_RUN" check would have let us
	 * finish.
	 */
	while ((oncpu = thread_isoncpu(thread)) ||
	    (until_not_runnable && (thread->state & TH_RUN))) {
		if (oncpu) {
			assert(thread->state & TH_RUN);
			processor = thread->chosen_processor;
			cause_ast_check(processor);
		}

		thread->wake_active = TRUE;
		thread_unlock(thread);

		wresult = assert_wait(&thread->wake_active, THREAD_UNINT);
		wake_unlock(thread);
		splx(s);

		if (wresult == THREAD_WAITING) {
			thread_block(THREAD_CONTINUE_NULL);
		}

		s = splsched();
		wake_lock(thread);
		thread_lock(thread);
	}

	thread_unlock(thread);
	wake_unlock(thread);
	splx(s);
}

/*
 *	Routine: clear_wait_internal
 *
 *		Clear the wait condition for the specified thread.
 *		Start the thread executing if that is appropriate.
 *	Arguments:
 *		thread		thread to awaken
 *		result		Wakeup result the thread should see
 *	Conditions:
 *		At splsched
 *		the thread is locked.
 *	Returns:
 *		KERN_SUCCESS		thread was rousted out a wait
 *		KERN_FAILURE		thread was waiting but could not be rousted
 *		KERN_NOT_WAITING	thread was not waiting
 */
__private_extern__ kern_return_t
clear_wait_internal(
	thread_t        thread,
	wait_result_t   wresult)
{
	waitq_t waitq = thread->waitq;

	if (wresult == THREAD_INTERRUPTED && (thread->state & TH_UNINT)) {
		return KERN_FAILURE;
	}

	/*
	 * Check that the thread is waiting and not waking, as a waking thread
	 * has already cleared its waitq, and is destined to be go'ed, don't
	 * need to do it again.
	 */
	if ((thread->state & (TH_WAIT | TH_TERMINATE | TH_WAKING)) != TH_WAIT) {
		assert(waitq_is_null(thread->waitq));
		return KERN_NOT_WAITING;
	}

	/* may drop and retake the thread lock */
	if (!waitq_is_null(waitq) && !waitq_pull_thread_locked(waitq, thread)) {
		return KERN_NOT_WAITING;
	}

	thread_go(thread, wresult, /* handoff */ false);

	return KERN_SUCCESS;
}


/*
 *	clear_wait:
 *
 *	Clear the wait condition for the specified thread.  Start the thread
 *	executing if that is appropriate.
 *
 *	parameters:
 *	  thread		thread to awaken
 *	  result		Wakeup result the thread should see
 */
__mockable kern_return_t
clear_wait(
	thread_t                thread,
	wait_result_t   result)
{
	kern_return_t ret;
	spl_t           s;

	s = splsched();
	thread_lock(thread);

	ret = clear_wait_internal(thread, result);

	if (thread == current_thread()) {
		/*
		 * The thread must be ready to wait again immediately
		 * after clearing its own wait.
		 */
		assert((thread->state & TH_WAKING) == 0);
	}

	thread_unlock(thread);
	splx(s);
	return ret;
}

/*
 *	thread_wakeup_prim:
 *
 *	Common routine for thread_wakeup, thread_wakeup_with_result,
 *	and thread_wakeup_one.
 *
 */
kern_return_t
thread_wakeup_nthreads_prim(
	event_t          event,
	uint32_t         nthreads,
	wait_result_t    result)
{
	if (__improbable(event == NO_EVENT)) {
		panic("%s() called with NO_EVENT", __func__);
	}

	struct waitq *wq = global_eventq(event);
	uint32_t count;

	count = waitq_wakeup64_nthreads(wq, CAST_EVENT64_T(event), result,
	    WAITQ_WAKEUP_DEFAULT, nthreads);
	return count ? KERN_SUCCESS : KERN_NOT_WAITING;
}

/*
 *	thread_wakeup_prim:
 *
 *	Common routine for thread_wakeup, thread_wakeup_with_result,
 *	and thread_wakeup_one.
 *
 */
__mockable kern_return_t
thread_wakeup_prim(
	event_t          event,
	boolean_t        one_thread,
	wait_result_t    result)
{
	if (one_thread) {
		return thread_wakeup_nthreads_prim(event, 1, result);
	} else {
		return thread_wakeup_nthreads_prim(event, UINT32_MAX, result);
	}
}

/*
 * Wakeup a specified thread if and only if it's waiting for this event
 */
kern_return_t
thread_wakeup_thread(
	event_t         event,
	thread_t        thread)
{
	if (__improbable(event == NO_EVENT)) {
		panic("%s() called with NO_EVENT", __func__);
	}

	if (__improbable(thread == THREAD_NULL)) {
		panic("%s() called with THREAD_NULL", __func__);
	}

	struct waitq *wq = global_eventq(event);

	return waitq_wakeup64_thread(wq, CAST_EVENT64_T(event), thread, THREAD_AWAKENED);
}

/*
 *	thread_bind:
 *
 *	Force the current thread to execute on the specified processor.
 *	Takes effect after the next thread_block().
 *
 *	Returns the previous binding.  PROCESSOR_NULL means
 *	not bound.
 *
 *	XXX - DO NOT export this to users - XXX
 */
processor_t
thread_bind(
	processor_t             processor)
{
	thread_t                self = current_thread();
	processor_t             prev;
	spl_t                   s;

	s = splsched();
	thread_lock(self);

	prev = thread_bind_internal(self, processor);

	thread_unlock(self);
	splx(s);

	return prev;
}

void
thread_bind_during_wakeup(thread_t thread, processor_t processor)
{
	assert(!ml_get_interrupts_enabled());
	assert((thread->state & (TH_WAIT | TH_WAKING)) == (TH_WAIT | TH_WAKING));
#if MACH_ASSERT
	thread_lock_assert(thread, LCK_ASSERT_OWNED);
#endif

	if (thread->bound_processor != processor) {
		thread_bind_internal(thread, processor);
	}
}

void
thread_unbind_after_queue_shutdown(
	thread_t                thread,
	processor_t             processor __assert_only)
{
	assert(!ml_get_interrupts_enabled());

	thread_lock(thread);

	if (thread->bound_processor) {
		bool removed;

		assert(thread->bound_processor == processor);

		removed = thread_run_queue_remove(thread);
		/*
		 * we can always unbind even if we didn't really remove the
		 * thread from the runqueue
		 */
		thread_bind_internal(thread, PROCESSOR_NULL);
		if (removed) {
			thread_run_queue_reinsert(thread, SCHED_TAILQ);
		}
	}

	thread_unlock(thread);
}

/*
 * thread_bind_internal:
 *
 * If the specified thread is not the current thread, and it is currently
 * running on another CPU, a remote AST must be sent to that CPU to cause
 * the thread to migrate to its bound processor. Otherwise, the migration
 * will occur at the next quantum expiration or blocking point.
 *
 * When the thread is the current thread, and explicit thread_block() should
 * be used to force the current processor to context switch away and
 * let the thread migrate to the bound processor.
 *
 * Thread must be locked, and at splsched.
 */

static processor_t
thread_bind_internal(
	thread_t                thread,
	processor_t             processor)
{
	processor_t             prev;

	/* <rdar://problem/15102234> */
	assert(thread->sched_pri < BASEPRI_RTQUEUES);
	/* A thread can't be bound if it's sitting on a (potentially incorrect) runqueue */
	thread_assert_runq_null(thread);

	KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_THREAD_BIND),
	    thread_tid(thread), processor ? processor->cpu_id : ~0ul, 0, 0, 0);

	prev = thread->bound_processor;
	thread->bound_processor = processor;

	return prev;
}

/*
 * thread_vm_bind_group_add:
 *
 * The "VM bind group" is a special mechanism to mark a collection
 * of threads from the VM subsystem that, in general, should be scheduled
 * with only one CPU of parallelism. To accomplish this, we initially
 * bind all the threads to the master processor, which has the effect
 * that only one of the threads in the group can execute at once, including
 * preempting threads in the group that are a lower priority. Future
 * mechanisms may use more dynamic mechanisms to prevent the collection
 * of VM threads from using more CPU time than desired.
 *
 * The current implementation can result in priority inversions where
 * compute-bound priority 95 or realtime threads that happen to have
 * landed on the master processor prevent the VM threads from running.
 * When this situation is detected, we unbind the threads for one
 * scheduler tick to allow the scheduler to run the threads an
 * additional CPUs, before restoring the binding (assuming high latency
 * is no longer a problem).
 */

/*
 * The current max is provisioned for:
 * vm_compressor_swap_trigger_thread (92)
 * 2 x vm_pageout_iothread_internal (92) when vm_restricted_to_single_processor==TRUE
 * vm_pageout_continue (92)
 * memorystatus_thread (95)
 */
#define MAX_VM_BIND_GROUP_COUNT (5)
decl_simple_lock_data(static, sched_vm_group_list_lock);
static thread_t sched_vm_group_thread_list[MAX_VM_BIND_GROUP_COUNT];
static int sched_vm_group_thread_count;
static boolean_t sched_vm_group_temporarily_unbound = FALSE;

void
thread_vm_bind_group_add(void)
{
	thread_t self = current_thread();

	if (support_bootcpu_shutdown) {
		/*
		 * Bind group is not supported without an always-on
		 * processor to bind to. If we need these to coexist,
		 * we'd need to dynamically move the group to
		 * another processor as it shuts down, or build
		 * a different way to run a set of threads
		 * without parallelism.
		 */
		return;
	}

	thread_reference(self);
	self->options |= TH_OPT_SCHED_VM_GROUP;

	simple_lock(&sched_vm_group_list_lock, LCK_GRP_NULL);
	assert(sched_vm_group_thread_count < MAX_VM_BIND_GROUP_COUNT);
	sched_vm_group_thread_list[sched_vm_group_thread_count++] = self;
	simple_unlock(&sched_vm_group_list_lock);

	thread_bind(master_processor);

	/* Switch to bound processor if not already there */
	thread_block(THREAD_CONTINUE_NULL);
}

static void
sched_vm_group_maintenance(void)
{
	uint64_t ctime = mach_absolute_time();
	uint64_t longtime = ctime - sched_tick_interval;
	int i;
	spl_t s;
	boolean_t high_latency_observed = FALSE;
	boolean_t runnable_and_not_on_runq_observed = FALSE;
	boolean_t bind_target_changed = FALSE;
	processor_t bind_target = PROCESSOR_NULL;

	/* Make sure nobody attempts to add new threads while we are enumerating them */
	simple_lock(&sched_vm_group_list_lock, LCK_GRP_NULL);

	s = splsched();

	for (i = 0; i < sched_vm_group_thread_count; i++) {
		thread_t thread = sched_vm_group_thread_list[i];
		assert(thread != THREAD_NULL);
		thread_lock(thread);
		if ((thread->state & (TH_RUN | TH_WAIT)) == TH_RUN) {
			if (thread_get_runq(thread) != PROCESSOR_NULL && thread->last_made_runnable_time < longtime) {
				high_latency_observed = TRUE;
			} else if (thread_get_runq(thread) == PROCESSOR_NULL) {
				/* There are some cases where a thread be transitiong that also fall into this case */
				runnable_and_not_on_runq_observed = TRUE;
			}
		}
		thread_unlock(thread);

		if (high_latency_observed && runnable_and_not_on_runq_observed) {
			/* All the things we are looking for are true, stop looking */
			break;
		}
	}

	splx(s);

	if (sched_vm_group_temporarily_unbound) {
		/* If we turned off binding, make sure everything is OK before rebinding */
		if (!high_latency_observed) {
			/* rebind */
			bind_target_changed = TRUE;
			bind_target = master_processor;
			sched_vm_group_temporarily_unbound = FALSE; /* might be reset to TRUE if change cannot be completed */
		}
	} else {
		/*
		 * Check if we're in a bad state, which is defined by high
		 * latency with no core currently executing a thread. If a
		 * single thread is making progress on a CPU, that means the
		 * binding concept to reduce parallelism is working as
		 * designed.
		 */
		if (high_latency_observed && !runnable_and_not_on_runq_observed) {
			/* unbind */
			bind_target_changed = TRUE;
			bind_target = PROCESSOR_NULL;
			sched_vm_group_temporarily_unbound = TRUE;
		}
	}

	if (bind_target_changed) {
		s = splsched();
		for (i = 0; i < sched_vm_group_thread_count; i++) {
			thread_t thread = sched_vm_group_thread_list[i];
			boolean_t removed;
			assert(thread != THREAD_NULL);

			thread_lock(thread);
			removed = thread_run_queue_remove(thread);
			if (removed || ((thread->state & (TH_RUN | TH_WAIT)) == TH_WAIT)) {
				thread_bind_internal(thread, bind_target);
			} else {
				/*
				 * Thread was in the middle of being context-switched-to,
				 * or was in the process of blocking. To avoid switching the bind
				 * state out mid-flight, defer the change if possible.
				 */
				if (bind_target == PROCESSOR_NULL) {
					thread_bind_internal(thread, bind_target);
				} else {
					sched_vm_group_temporarily_unbound = TRUE; /* next pass will try again */
				}
			}

			if (removed) {
				thread_run_queue_reinsert(thread, SCHED_PREEMPT | SCHED_TAILQ);
			}
			thread_unlock(thread);
		}
		splx(s);
	}

	simple_unlock(&sched_vm_group_list_lock);
}

#if defined(__x86_64__)
#define SCHED_AVOID_CPU0 1
#else
#define SCHED_AVOID_CPU0 0
#endif

int sched_avoid_cpu0 = SCHED_AVOID_CPU0;
int sched_backup_cpu_timeout_count = 5; /* The maximum number of 10us delays to wait before using a backup cpu */
int sched_rt_n_backup_processors = SCHED_DEFAULT_BACKUP_PROCESSORS;

int
sched_get_rt_n_backup_processors(void)
{
	return sched_rt_n_backup_processors;
}

void
sched_set_rt_n_backup_processors(int n)
{
	if (n < 0) {
		n = 0;
	} else if (n > SCHED_MAX_BACKUP_PROCESSORS) {
		n = SCHED_MAX_BACKUP_PROCESSORS;
	}

	sched_rt_n_backup_processors = n;
}

/*
 * Invoked prior to idle entry to determine if, on SMT capable processors, an SMT
 * rebalancing opportunity exists when a core is (instantaneously) idle, but
 * other SMT-capable cores may be over-committed. TODO: some possible negatives:
 * IPI thrash if this core does not remain idle following the load balancing ASTs
 * Idle "thrash", when IPI issue is followed by idle entry/core power down
 * followed by a wakeup shortly thereafter.
 */

#if (DEVELOPMENT || DEBUG)
int sched_smt_balance = 1;
#endif

#if CONFIG_SCHED_SMT
/* Invoked with pset locked, returns with pset unlocked */
bool
sched_SMT_balance(processor_t cprocessor, processor_set_t cpset)
{
	processor_t ast_processor = NULL;

#if (DEVELOPMENT || DEBUG)
	if (__improbable(sched_smt_balance == 0)) {
		goto smt_balance_exit;
	}
#endif

	assert(cprocessor == current_processor());
	if (cprocessor->is_SMT == FALSE) {
		goto smt_balance_exit;
	}

	processor_t sib_processor = cprocessor->processor_secondary ? cprocessor->processor_secondary : cprocessor->processor_primary;

	/* Determine if both this processor and its sibling are idle,
	 * indicating an SMT rebalancing opportunity.
	 */
	if (sib_processor->state != PROCESSOR_IDLE) {
		goto smt_balance_exit;
	}

	processor_t sprocessor;

	sched_ipi_type_t ipi_type = SCHED_IPI_NONE;
	uint64_t running_secondary_map = (cpset->cpu_state_map[PROCESSOR_RUNNING] &
	    ~cpset->primary_map);
	for (int cpuid = lsb_first(running_secondary_map); cpuid >= 0; cpuid = lsb_next(running_secondary_map, cpuid)) {
		sprocessor = processor_array[cpuid];
		if ((sprocessor->processor_primary->state == PROCESSOR_RUNNING) &&
		    (sprocessor->current_pri < BASEPRI_RTQUEUES)) {
			ipi_type = sched_ipi_action(sprocessor, NULL, SCHED_IPI_EVENT_SMT_REBAL);
			if (ipi_type != SCHED_IPI_NONE) {
				assert(sprocessor != cprocessor);
				ast_processor = sprocessor;
				break;
			}
		}
	}

smt_balance_exit:
	pset_unlock(cpset);

	if (ast_processor) {
		KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_SMT_BALANCE), ast_processor->cpu_id, ast_processor->state, ast_processor->processor_primary->state, 0, 0);
		sched_ipi_perform(ast_processor, ipi_type);
	}
	return false;
}
#else /* CONFIG_SCHED_SMT */
/* Invoked with pset locked, returns with pset unlocked */
bool
sched_SMT_balance(__unused processor_t cprocessor, __unused processor_set_t cpset)
{
	pset_unlock(cpset);
	return false;
}
#endif /* CONFIG_SCHED_SMT */

int
pset_available_cpu_count(processor_set_t pset)
{
	return bit_count(pset_available_cpumap(pset));
}

bool
pset_is_recommended(processor_set_t pset)
{
	if (!pset) {
		return false;
	}
	return pset_available_cpu_count(pset) > 0;
}

bool
pset_type_is_recommended(processor_set_t pset)
{
	if (!pset) {
		return false;
	}
	pset_map_t recommended_psets = os_atomic_load(&pset->node->pset_recommended_map, relaxed);
	return bit_count(recommended_psets) > 0;
}

static cpumap_t
pset_available_but_not_running_cpumap(processor_set_t pset)
{
	return (pset->cpu_state_map[PROCESSOR_IDLE] | pset->cpu_state_map[PROCESSOR_DISPATCHING]) &
	       pset->recommended_bitmask;
}

bool
pset_has_stealable_threads(processor_set_t pset)
{
	pset_assert_locked(pset);

	cpumap_t avail_map = pset_available_but_not_running_cpumap(pset);
#if CONFIG_SCHED_SMT
	/*
	 * Secondary CPUs never steal, so allow stealing of threads if there are more threads than
	 * available primary CPUs
	 */
	avail_map &= pset->primary_map;
#endif /* CONFIG_SCHED_SMT */

	return (pset->pset_runq.count > 0) && ((pset->pset_runq.count + rt_runq_count(pset)) > bit_count(avail_map));
}

static void
clear_pending_AST_bits(processor_set_t pset, processor_t processor, __kdebug_only const int trace_point_number)
{
	/* Acknowledge any pending IPIs here with pset lock held */
	pset_assert_locked(pset);
	if (bit_clear_if_set(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
		KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PENDING_AST_URGENT) | DBG_FUNC_END,
		    processor->cpu_id, pset->pending_AST_URGENT_cpu_mask, 0, trace_point_number);
	}
	bit_clear(pset->pending_AST_PREEMPT_cpu_mask, processor->cpu_id);

#if defined(CONFIG_SCHED_DEFERRED_AST)
	bit_clear(pset->pending_deferred_AST_cpu_mask, processor->cpu_id);
#endif
}

/*
 * Called with pset locked, on a processor that is committing to run a new thread
 * Will transition an idle or dispatching processor to running as it picks up
 * the first new thread from the idle thread.
 */
static void
pset_commit_processor_to_new_thread(processor_set_t pset, processor_t processor, thread_t new_thread)
{
	pset_assert_locked(pset);

	if (processor->state == PROCESSOR_DISPATCHING || processor->state == PROCESSOR_IDLE) {
		assert(current_thread() == processor->idle_thread);

		/*
		 * Dispatching processor is now committed to running new_thread,
		 * so change its state to PROCESSOR_RUNNING.
		 */
		pset_update_processor_state(pset, processor, PROCESSOR_RUNNING);
	} else {
		assert(processor->state == PROCESSOR_RUNNING);
	}

	processor_state_update_from_thread(processor, new_thread, true);

	if (new_thread->sched_pri >= BASEPRI_RTQUEUES) {
		bit_set(pset->realtime_map, processor->cpu_id);
	} else {
		bit_clear(pset->realtime_map, processor->cpu_id);
	}
	pset_update_rt_stealable_state(pset);

	pset_node_t node = pset->node;

	if (bit_count(node->pset_map) == 1) {
		/* Node has only a single pset, so skip node pset map updates */
		return;
	}

	cpumap_t avail_map = pset_available_cpumap(pset);

	if (new_thread->sched_pri >= BASEPRI_RTQUEUES) {
		if ((avail_map & pset->realtime_map) == avail_map) {
			/* No more non-RT CPUs in this pset */
			atomic_bit_clear(&node->pset_non_rt_map, pset->pset_id, memory_order_relaxed);
		}
#if CONFIG_SCHED_SMT
		avail_map &= pset->primary_map;
		if ((avail_map & pset->realtime_map) == avail_map) {
			/* No more non-RT primary CPUs in this pset */
			atomic_bit_clear(&node->pset_non_rt_primary_map, pset->pset_id, memory_order_relaxed);
		}
#endif /* CONFIG_SCHED_SMT */
	} else {
		if ((avail_map & pset->realtime_map) != avail_map) {
			if (!bit_test(atomic_load(&node->pset_non_rt_map), pset->pset_id)) {
				atomic_bit_set(&node->pset_non_rt_map, pset->pset_id, memory_order_relaxed);
			}
		}
#if CONFIG_SCHED_SMT
		avail_map &= pset->primary_map;
		if ((avail_map & pset->realtime_map) != avail_map) {
			if (!bit_test(atomic_load(&node->pset_non_rt_primary_map), pset->pset_id)) {
				atomic_bit_set(&node->pset_non_rt_primary_map, pset->pset_id, memory_order_relaxed);
			}
		}
#endif /* CONFIG_SCHED_SMT */
	}
}

#if CONFIG_SCHED_SMT
static bool all_available_primaries_are_running_realtime_threads(processor_set_t pset, bool include_backups);
static bool these_processors_are_running_realtime_threads(processor_set_t pset, uint64_t these_map, bool include_backups);
#else /* !CONFIG_SCHED_SMT */
processor_t pset_choose_processor_for_realtime_thread(processor_set_t pset, processor_t skip_processor, bool skip_spills);
#endif /* !CONFIG_SCHED_SMT */
static bool sched_ok_to_run_realtime_thread(processor_set_t pset, processor_t processor, bool as_backup);

static bool
other_psets_have_earlier_rt_threads_pending(processor_set_t stealing_pset, uint64_t earliest_deadline)
{
	pset_map_t pset_map = stealing_pset->node->pset_map;

	bit_clear(pset_map, stealing_pset->pset_id);

	for (int pset_id = lsb_first(pset_map); pset_id >= 0; pset_id = lsb_next(pset_map, pset_id)) {
		processor_set_t nset = pset_array[pset_id];

		if (rt_deadline_add(os_atomic_load(&nset->stealable_rt_threads_earliest_deadline, relaxed), rt_deadline_epsilon) < earliest_deadline) {
			return true;
		}
	}

	return false;
}

/*
 * backup processor - used by choose_processor to send a backup IPI to in case the preferred processor can't immediately respond
 * followup processor - used in thread_select when there are still threads on the run queue and available processors
 * spill processor - a processor in a different processor set that is signalled to steal a thread from this run queue
 */
typedef enum {
	none,
	backup,
	followup,
	spill
} next_processor_type_t;

#undef LOOP_COUNT
#ifdef LOOP_COUNT
int max_loop_count[MAX_SCHED_CPUS] = { 0 };
#endif

/*
 *	thread_select:
 *
 *	Select a new thread for the current processor to execute.
 *
 *	May select the current thread, which must be locked.
 */
static thread_t
thread_select(thread_t          thread,
    processor_t       processor,
    ast_t            *reason)
{
	processor_set_t         pset = processor->processor_set;
	thread_t                        new_thread = THREAD_NULL;

	assert(processor == current_processor());
	assert((thread->state & (TH_RUN | TH_TERMINATE2)) == TH_RUN);

	KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_THREAD_SELECT) | DBG_FUNC_START,
	    0, pset->pending_AST_URGENT_cpu_mask, 0, 0);

	__kdebug_only int idle_reason = 0;
	__kdebug_only int delay_count = 0;

#if CONFIG_SCHED_SMT
	int timeout_count = sched_backup_cpu_timeout_count;
	if ((sched_avoid_cpu0 == 1) && (processor->cpu_id == 0)) {
		/* Prefer cpu0 as backup */
		timeout_count--;
	} else if ((sched_avoid_cpu0 == 2) && (processor->processor_primary != processor)) {
		/* Prefer secondary cpu as backup */
		timeout_count--;
	}
#endif /* CONFIG_SCHED_SMT */
	bool pending_AST_URGENT = false;
	bool pending_AST_PREEMPT = false;

#ifdef LOOP_COUNT
	int loop_count = -1;
#endif

	do {
		/*
		 *	Update the priority.
		 */
		if (SCHED(can_update_priority)(thread)) {
			SCHED(update_priority)(thread);
		}

		pset_lock(pset);

restart:
#ifdef LOOP_COUNT
		loop_count++;
		if (loop_count > max_loop_count[processor->cpu_id]) {
			max_loop_count[processor->cpu_id] = loop_count;
			if (bit_count(loop_count) == 1) {
				kprintf("[%d]%s>max_loop_count = %d\n", processor->cpu_id, __FUNCTION__, loop_count);
			}
		}
#endif
		pending_AST_URGENT = bit_test(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id);
		pending_AST_PREEMPT = bit_test(pset->pending_AST_PREEMPT_cpu_mask, processor->cpu_id);

		processor_state_update_from_thread(processor, thread, true);

		idle_reason = 0;

		processor_t ast_processor = PROCESSOR_NULL;
		processor_t next_rt_processor = PROCESSOR_NULL;
		sched_ipi_type_t ipi_type = SCHED_IPI_NONE;
		sched_ipi_type_t next_rt_ipi_type = SCHED_IPI_NONE;

		assert(processor->state != PROCESSOR_OFF_LINE);

		/*
		 * Bound threads are dispatched to a processor without going through
		 * choose_processor(), so in those cases we must continue trying to dequeue work
		 * as we are the only option.
		 */
		if (!SCHED(processor_bound_count)(processor)) {
			if (!processor->is_recommended) {
				/*
				 * The performance controller has provided a hint to not dispatch more threads,
				 */
				idle_reason = 1;
				goto send_followup_ipi_before_idle;
			} else if (rt_runq_count(pset)) {
				bool ok_to_run_realtime_thread = sched_ok_to_run_realtime_thread(pset, processor, false);
				/* Give the current RT thread a chance to complete */
				ok_to_run_realtime_thread |= (thread->sched_pri >= BASEPRI_RTQUEUES && processor->first_timeslice);
#if CONFIG_SCHED_SMT
				/*
				 * On Intel we want to avoid SMT secondary processors and processor 0
				 * but allow them to be used as backup processors in case the preferred chosen
				 * processor is delayed by interrupts or processor stalls.  So if it is
				 * not ok_to_run_realtime_thread as preferred (sched_ok_to_run_realtime_thread(pset, processor, as_backup=false))
				 * but ok_to_run_realtime_thread as backup (sched_ok_to_run_realtime_thread(pset, processor, as_backup=true))
				 * we delay up to (timeout_count * 10us) to give the preferred processor chance
				 * to grab the thread before the (current) backup processor does.
				 *
				 * timeout_count defaults to 5 but can be tuned using sysctl kern.sched_backup_cpu_timeout_count
				 * on DEVELOPMENT || DEBUG kernels.  It is also adjusted (see above) depending on whether we want to use
				 * cpu0 before secondary cpus or not.
				 */
				if (!ok_to_run_realtime_thread) {
					if (sched_ok_to_run_realtime_thread(pset, processor, true)) {
						if (timeout_count-- > 0) {
							pset_unlock(pset);
							thread_unlock(thread);
							delay(10);
							delay_count++;
							thread_lock(thread);
							pset_lock(pset);
							goto restart;
						}
						ok_to_run_realtime_thread = true;
					}
				}
#endif /* CONFIG_SCHED_SMT */
				if (!ok_to_run_realtime_thread) {
					idle_reason = 2;
					goto send_followup_ipi_before_idle;
				}
			}
#if CONFIG_SCHED_SMT
			else if (processor->processor_primary != processor) {
				/*
				 * Should this secondary SMT processor attempt to find work? For pset runqueue systems,
				 * we should look for work only under the same conditions that choose_processor()
				 * would have assigned work, which is when all primary processors have been assigned work.
				 */
				if ((pset->recommended_bitmask & pset->primary_map & pset->cpu_state_map[PROCESSOR_IDLE]) != 0) {
					/* There are idle primaries */
					idle_reason = 3;
					goto idle;
				}
			}
#endif /* CONFIG_SCHED_SMT */
		}

		/*
		 *	Test to see if the current thread should continue
		 *	to run on this processor.  Must not be attempting to wait, and not
		 *	bound to a different processor, nor be in the wrong
		 *	processor set, nor be forced to context switch by TH_SUSP.
		 *
		 *	Note that there are never any RT threads in the regular runqueue.
		 *
		 *	This code is very insanely tricky.
		 */

		/* i.e. not waiting, not TH_SUSP'ed */
		bool still_running = ((thread->state & (TH_TERMINATE | TH_IDLE | TH_WAIT | TH_RUN | TH_SUSP)) == TH_RUN);

		/*
		 * Threads running on SMT processors are forced to context switch. Don't rebalance realtime threads.
		 * TODO: This should check if it's worth it to rebalance, i.e. 'are there any idle primary processors'
		 *       <rdar://problem/47907700>
		 *
		 * A yielding thread shouldn't be forced to context switch.
		 */

		bool is_yielding         = (*reason & AST_YIELD) == AST_YIELD;

#if CONFIG_SCHED_SMT
		bool needs_smt_rebalance = !is_yielding && thread->sched_pri < BASEPRI_RTQUEUES && processor->processor_primary != processor;
#endif /* CONFIG_SCHED_SMT */

		bool affinity_mismatch   = thread->affinity_set != AFFINITY_SET_NULL && thread->affinity_set->aset_pset != pset;

		bool bound_elsewhere     = thread->bound_processor != PROCESSOR_NULL && thread->bound_processor != processor;

		bool avoid_processor     = !is_yielding && SCHED(avoid_processor_enabled) && SCHED(thread_avoid_processor)(processor, thread, *reason);

		bool ok_to_run_realtime_thread = sched_ok_to_run_realtime_thread(pset, processor, true);

		bool current_thread_can_keep_running = (
			still_running
#if CONFIG_SCHED_SMT
			&& !needs_smt_rebalance
#endif /* CONFIG_SCHED_SMT */
			&& !affinity_mismatch
			&& !bound_elsewhere
			&& !avoid_processor);
		if (current_thread_can_keep_running) {
			/*
			 * This thread is eligible to keep running on this processor.
			 *
			 * RT threads with un-expired quantum stay on processor,
			 * unless there's a valid RT thread with an earlier deadline
			 * and it is still ok_to_run_realtime_thread.
			 */
			if (thread->sched_pri >= BASEPRI_RTQUEUES && processor->first_timeslice) {
				/*
				 * Pick a new RT thread only if ok_to_run_realtime_thread
				 * (but the current thread is allowed to complete).
				 */
				if (ok_to_run_realtime_thread) {
					if (bit_test(pset->rt_pending_spill_cpu_mask, processor->cpu_id)) {
						goto pick_new_rt_thread;
					}
					if (rt_runq_priority(pset) > thread->sched_pri) {
						if (sched_rt_runq_strict_priority) {
							/* The next RT thread is better, so pick it off the runqueue. */
							goto pick_new_rt_thread;
						}

						/*
						 * See if the current lower priority thread can continue to run without causing
						 * the higher priority thread on the runq queue to miss its deadline.
						 */
						thread_t hi_thread = rt_runq_first(&pset->rt_runq);
						if (thread->realtime.computation + hi_thread->realtime.computation + rt_deadline_epsilon >= hi_thread->realtime.constraint) {
							/* The next RT thread is better, so pick it off the runqueue. */
							goto pick_new_rt_thread;
						}
					} else if ((rt_runq_count(pset) > 0) && (rt_deadline_add(rt_runq_earliest_deadline(pset), rt_deadline_epsilon) < thread->realtime.deadline)) {
						/* The next RT thread is better, so pick it off the runqueue. */
						goto pick_new_rt_thread;
					}
					if (other_psets_have_earlier_rt_threads_pending(pset, thread->realtime.deadline)) {
						goto pick_new_rt_thread;
					}
				}

				/* This is still the best RT thread to run. */
				processor->deadline = thread->realtime.deadline;

				SCHED(update_pset_load_average)(pset, 0);

				clear_pending_AST_bits(pset, processor, 1);

				next_rt_processor = PROCESSOR_NULL;
				next_rt_ipi_type = SCHED_IPI_NONE;

				bool pset_unlocked = false;
				next_processor_type_t nptype = none;
#if CONFIG_SCHED_EDGE
				if (rt_pset_has_stealable_threads(pset)) {
					nptype = spill;
					pset_unlocked = rt_choose_next_processor_for_spill_IPI(pset, processor, &next_rt_processor, &next_rt_ipi_type);
				}
#endif /* CONFIG_SCHED_EDGE */
				if (nptype == none && rt_pset_needs_a_followup_IPI(pset)) {
					nptype = followup;
					rt_choose_next_processor_for_followup_IPI(pset, processor, &next_rt_processor, &next_rt_ipi_type);
				}
				if (!pset_unlocked) {
					pset_unlock(pset);
				}

				if (next_rt_processor) {
					KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_NEXT_PROCESSOR) | DBG_FUNC_NONE,
					    next_rt_processor->cpu_id, next_rt_processor->state, nptype, 2);
					sched_ipi_perform(next_rt_processor, next_rt_ipi_type);
				}

				KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_THREAD_SELECT) | DBG_FUNC_END,
				    (uintptr_t)thread_tid(thread), pset->pending_AST_URGENT_cpu_mask, delay_count, 1);
				return thread;
			}

			if ((rt_runq_count(pset) == 0) &&
			    SCHED(processor_queue_has_priority)(processor, thread->sched_pri, TRUE) == FALSE) {
				/* This thread is still the highest priority runnable (non-idle) thread */
				processor->deadline = RT_DEADLINE_NONE;

				SCHED(update_pset_load_average)(pset, 0);

				clear_pending_AST_bits(pset, processor, 2);

				pset_unlock(pset);

				KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_THREAD_SELECT) | DBG_FUNC_END,
				    (uintptr_t)thread_tid(thread), pset->pending_AST_URGENT_cpu_mask, delay_count, 2);
				return thread;
			}
		} else {
			/*
			 * This processor must context switch.
			 * If it's due to a rebalance, we should aggressively find this thread a new home.
			 */
			bool ast_rebalance = affinity_mismatch || bound_elsewhere || avoid_processor;
#if CONFIG_SCHED_SMT
			ast_rebalance = ast_rebalance || needs_smt_rebalance;
#endif /* CONFIG_SCHED_SMT */
			if (ast_rebalance) {
				*reason |= AST_REBALANCE;
			}
		}

#if CONFIG_SCHED_SMT
		bool secondary_forced_idle = ((processor->processor_secondary != PROCESSOR_NULL) &&
		    (thread_no_smt(thread) || (thread->sched_pri >= BASEPRI_RTQUEUES)) &&
		    (processor->processor_secondary->state == PROCESSOR_IDLE));
#endif /* CONFIG_SCHED_SMT */

		/* OK, so we're not going to run the current thread. Look at the RT queue. */
		if (ok_to_run_realtime_thread) {
pick_new_rt_thread:
			/* sched_rt_choose_thread may drop and re-take the processor's pset lock. */
			new_thread = sched_rt_choose_thread(processor);
			pset_assert_locked(pset);
			if (new_thread != THREAD_NULL) {
				processor->deadline = new_thread->realtime.deadline;
				pset_commit_processor_to_new_thread(pset, processor, new_thread);

				clear_pending_AST_bits(pset, processor, 3);

#if CONFIG_SCHED_SMT
				if (processor->processor_secondary != NULL) {
					processor_t sprocessor = processor->processor_secondary;
					if ((sprocessor->state == PROCESSOR_RUNNING) || (sprocessor->state == PROCESSOR_DISPATCHING)) {
						ipi_type = sched_ipi_action(sprocessor, NULL, SCHED_IPI_EVENT_SMT_REBAL);
						ast_processor = sprocessor;
					}
				}
#endif /* CONFIG_SCHED_SMT */
			}
		}

send_followup_ipi_before_idle:
		/* This might not have been cleared if we didn't call sched_rt_choose_thread() */
		rt_clear_pending_spill(processor, 5);
		next_processor_type_t nptype = none;
		bool pset_unlocked = false;
#if CONFIG_SCHED_EDGE
		if (rt_pset_has_stealable_threads(pset)) {
			nptype = spill;
			pset_unlocked = rt_choose_next_processor_for_spill_IPI(pset, processor, &next_rt_processor, &next_rt_ipi_type);
		}
#endif /* CONFIG_SCHED_EDGE */
		if (nptype == none && rt_pset_needs_a_followup_IPI(pset)) {
			nptype = followup;
			rt_choose_next_processor_for_followup_IPI(pset, processor, &next_rt_processor, &next_rt_ipi_type);
		}

		assert(new_thread || !ast_processor);
		if (new_thread || next_rt_processor) {
			if (!pset_unlocked) {
				pset_unlock(pset);
				pset_unlocked = true;
			}
			if (ast_processor == next_rt_processor) {
				ast_processor = PROCESSOR_NULL;
				ipi_type = SCHED_IPI_NONE;
			}

			if (ast_processor) {
				sched_ipi_perform(ast_processor, ipi_type);
			}

			if (next_rt_processor) {
				KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_NEXT_PROCESSOR) | DBG_FUNC_NONE,
				    next_rt_processor->cpu_id, next_rt_processor->state, nptype, 3);
				sched_ipi_perform(next_rt_processor, next_rt_ipi_type);
			}

			if (new_thread) {
				KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_THREAD_SELECT) | DBG_FUNC_END,
				    (uintptr_t)thread_tid(new_thread), pset->pending_AST_URGENT_cpu_mask, delay_count, 3);
				return new_thread;
			}
		}

		if (pset_unlocked) {
			pset_lock(pset);
		}

		if (!pending_AST_URGENT && bit_test(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
			/* Things changed while we dropped the lock */
			goto restart;
		}

		if (processor->is_recommended) {
			bool spill_pending = bit_test(pset->rt_pending_spill_cpu_mask, processor->cpu_id);
			if (sched_ok_to_run_realtime_thread(pset, processor, true) && (spill_pending || rt_runq_count(pset))) {
				/* Things changed while we dropped the lock */
				goto restart;
			}

#if CONFIG_SCHED_SMT
			if ((processor->processor_primary != processor) && (processor->processor_primary->current_pri >= BASEPRI_RTQUEUES)) {
				/* secondary can only run realtime thread */
				if (idle_reason == 0) {
					idle_reason = 4;
				}
				goto idle;
			}
#endif /* CONFIG_SCHED_SMT */
		} else if (!SCHED(processor_bound_count)(processor)) {
			/* processor not recommended and no bound threads */
			if (idle_reason == 0) {
				idle_reason = 5;
			}
			goto idle;
		}

		processor->deadline = RT_DEADLINE_NONE;

		/* No RT threads, so let's look at the regular threads using EEVDF */
		if ((new_thread = SCHED(choose_thread)(processor, MINPRI, current_thread_can_keep_running ? thread : THREAD_NULL, *reason)) != THREAD_NULL) {
			if (new_thread != thread) {
				/* Going to context-switch */
				pset_commit_processor_to_new_thread(pset, processor, new_thread);

				clear_pending_AST_bits(pset, processor, 4);

				ast_processor = PROCESSOR_NULL;
				ipi_type = SCHED_IPI_NONE;

#if CONFIG_SCHED_SMT
				processor_t sprocessor = processor->processor_secondary;
				if (sprocessor != NULL) {
					if (sprocessor->state == PROCESSOR_RUNNING) {
						if (thread_no_smt(new_thread)) {
							ipi_type = sched_ipi_action(sprocessor, NULL, SCHED_IPI_EVENT_SMT_REBAL);
							ast_processor = sprocessor;
						}
					} else if (secondary_forced_idle && !thread_no_smt(new_thread) && pset_has_stealable_threads(pset)) {
						ipi_type = sched_ipi_action(sprocessor, NULL, SCHED_IPI_EVENT_PREEMPT);
						ast_processor = sprocessor;
					}
				}
#endif /* CONFIG_SCHED_SMT */

				pset_unlock(pset);

				if (ast_processor) {
					sched_ipi_perform(ast_processor, ipi_type);
				}
			} else {
				/* Will continue running the current thread */
				clear_pending_AST_bits(pset, processor, 4);
				pset_unlock(pset);
			}

			KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_THREAD_SELECT) | DBG_FUNC_END,
			    (uintptr_t)thread_tid(new_thread), pset->pending_AST_URGENT_cpu_mask, delay_count, 4);
			return new_thread;
		}

		if (processor->must_idle) {
			processor->must_idle = false;
			*reason |= AST_REBALANCE;
			idle_reason = 6;
			goto idle;
		}

		if (SCHED(steal_thread_enabled)(pset)
#if CONFIG_SCHED_SMT
		    && (processor->processor_primary == processor)
#endif /* CONFIG_SCHED_SMT */
		    ) {
			/*
			 * No runnable threads, attempt to steal
			 * from other processors. Returns with pset lock dropped.
			 */

			if ((new_thread = SCHED(steal_thread)(pset)) != THREAD_NULL) {
				pset_lock(pset);
				pset_commit_processor_to_new_thread(pset, processor, new_thread);
				if (!pending_AST_URGENT && bit_test(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
					/*
					 * A realtime thread choose this processor while it was DISPATCHING
					 * and the pset lock was dropped
					 */
					ast_on(AST_URGENT | AST_PREEMPT);
				}

				clear_pending_AST_bits(pset, processor, 5);

				pset_unlock(pset);

				KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_THREAD_SELECT) | DBG_FUNC_END,
				    (uintptr_t)thread_tid(new_thread), pset->pending_AST_URGENT_cpu_mask, delay_count, 5);
				return new_thread;
			}

			/*
			 * If other threads have appeared, shortcut
			 * around again.
			 */
			if (SCHED(processor_bound_count)(processor)) {
				continue;
			}
			if (processor->is_recommended) {
				if (!SCHED(processor_queue_empty)(processor) || (sched_ok_to_run_realtime_thread(pset, processor, true) && (rt_runq_count(pset) > 0))) {
					continue;
				}
			}

			pset_lock(pset);
		}

idle:
		/* Someone selected this processor while we had dropped the lock */
		if ((!pending_AST_URGENT && bit_test(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) ||
		    (!pending_AST_PREEMPT && bit_test(pset->pending_AST_PREEMPT_cpu_mask, processor->cpu_id))) {
			goto restart;
		}

		if ((idle_reason == 0) && current_thread_can_keep_running) {
			/* This thread is the only runnable (non-idle) thread */
			if (thread->sched_pri >= BASEPRI_RTQUEUES) {
				processor->deadline = thread->realtime.deadline;
			} else {
				processor->deadline = RT_DEADLINE_NONE;
			}

			SCHED(update_pset_load_average)(pset, 0);

			clear_pending_AST_bits(pset, processor, 6);

			KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_THREAD_SELECT) | DBG_FUNC_END,
			    (uintptr_t)thread_tid(thread), pset->pending_AST_URGENT_cpu_mask, delay_count, 6);
			pset_unlock(pset);
			return thread;
		}

		/*
		 *	Nothing is runnable, or this processor must be forced idle,
		 *	so set this processor idle if it was running.
		 */
		if ((processor->state == PROCESSOR_RUNNING) || (processor->state == PROCESSOR_DISPATCHING)) {
			pset_update_processor_state(pset, processor, PROCESSOR_IDLE);
			processor_state_update_idle(processor);
		}
		pset_update_rt_stealable_state(pset);

		clear_pending_AST_bits(pset, processor, 7);

		/* Invoked with pset locked, returns with pset unlocked */
		processor->next_idle_short = SCHED(processor_balance)(processor, pset);

		new_thread = processor->idle_thread;
	} while (new_thread == THREAD_NULL);

	KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_THREAD_SELECT) | DBG_FUNC_END,
	    (uintptr_t)thread_tid(new_thread), pset->pending_AST_URGENT_cpu_mask, delay_count, 10 + idle_reason);
	return new_thread;
}

/*
 * thread_invoke
 *
 * Called at splsched with neither thread locked.
 *
 * Perform a context switch and start executing the new thread.
 *
 * Returns FALSE when the context switch didn't happen.
 * The reference to the new thread is still consumed.
 *
 * "self" is what is currently running on the processor,
 * "thread" is the new thread to context switch to
 * (which may be the same thread in some cases)
 */
static boolean_t
thread_invoke(
	thread_t                        self,
	thread_t                        thread,
	ast_t                           reason)
{
	if (__improbable(get_preemption_level() != 0)) {
		int pl = get_preemption_level();
		panic("thread_invoke: preemption_level %d, possible cause: %s",
		    pl, (pl < 0 ? "unlocking an unlocked mutex or spinlock" :
		    "blocking while holding a spinlock, or within interrupt context"));
	}

	thread_continue_t       continuation = self->continuation;
	void                    *parameter   = self->parameter;

	struct recount_snap snap = { 0 };
	recount_snapshot(&snap);
	uint64_t ctime = snap.rsn_time_mach;

	check_monotonic_time(ctime);

#ifdef CONFIG_MACH_APPROXIMATE_TIME
	commpage_update_mach_approximate_time(ctime);
#endif

	if (ctime < thread->last_made_runnable_time) {
		panic("Non-monotonic time: invoke at 0x%llx, runnable at 0x%llx",
		    ctime, thread->last_made_runnable_time);
	}

#if defined(CONFIG_SCHED_TIMESHARE_CORE)
	if (!((thread->state & TH_IDLE) != 0 ||
	    ((reason & AST_HANDOFF) && self->sched_mode == TH_MODE_REALTIME))) {
		sched_timeshare_consider_maintenance(ctime, true);
	}
#endif

	recount_log_switch_thread(&snap);

	processor_t processor = current_processor();

	if (!processor->processor_online) {
		panic("Invalid attempt to context switch an offline processor");
	}

	assert_thread_magic(self);
	assert(self == current_thread());
	thread_assert_runq_null(self);
	assert((self->state & (TH_RUN | TH_TERMINATE2)) == TH_RUN);

	thread_lock(thread);

	assert_thread_magic(thread);
	assert((thread->state & (TH_RUN | TH_WAIT | TH_UNINT | TH_TERMINATE | TH_TERMINATE2)) == TH_RUN);
	assert(thread->bound_processor == PROCESSOR_NULL || thread->bound_processor == processor);
	thread_assert_runq_null(thread);

	/* Update SFI class based on other factors */
	thread->sfi_class = sfi_thread_classify(thread);

	/* Update the same_pri_latency for the thread (used by perfcontrol callouts) */
	thread->same_pri_latency = ctime - thread->last_basepri_change_time;
	/*
	 * In case a base_pri update happened between the timestamp and
	 * taking the thread lock
	 */
	if (ctime <= thread->last_basepri_change_time) {
		thread->same_pri_latency = ctime - thread->last_made_runnable_time;
	}

	/* Allow realtime threads to hang onto a stack. */
	if ((self->sched_mode == TH_MODE_REALTIME) && !self->reserved_stack) {
		self->reserved_stack = self->kernel_stack;
	}

	/* Prepare for spin debugging */
#if SCHED_HYGIENE_DEBUG
	ml_spin_debug_clear(thread);
#endif

	if (continuation != NULL) {
		if (!thread->kernel_stack) {
			/*
			 * If we are using a privileged stack,
			 * check to see whether we can exchange it with
			 * that of the other thread.
			 */
			if (self->kernel_stack == self->reserved_stack && !thread->reserved_stack) {
				goto need_stack;
			}

			/*
			 * Context switch by performing a stack handoff.
			 * Requires both threads to be parked in a continuation.
			 */
			continuation = thread->continuation;
			parameter = thread->parameter;

			processor->active_thread = thread;
			processor_state_update_from_thread(processor, thread, false);

			if (thread->last_processor != processor && thread->last_processor != NULL) {
				if (thread->last_processor->processor_set != processor->processor_set) {
					thread->ps_switch++;
				}
				thread->p_switch++;
			}
			thread->last_processor = processor;
			thread->c_switch++;
			ast_context(thread);

			thread_unlock(thread);

			self->reason = reason;

			processor->last_dispatch = ctime;
			self->last_run_time = ctime;
			timer_update(&thread->runnable_timer, ctime);
			recount_switch_thread(&snap, self, get_threadtask(self));

			KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
			    MACHDBG_CODE(DBG_MACH_SCHED, MACH_STACK_HANDOFF) | DBG_FUNC_NONE,
			    self->reason, (uintptr_t)thread_tid(thread), self->sched_pri, thread->sched_pri, 0);

			if ((thread->chosen_processor != processor) && (thread->chosen_processor != PROCESSOR_NULL)) {
				SCHED_DEBUG_CHOOSE_PROCESSOR_KERNEL_DEBUG_CONSTANT_IST(MACHDBG_CODE(DBG_MACH_SCHED, MACH_MOVED) | DBG_FUNC_NONE,
				    (uintptr_t)thread_tid(thread), (uintptr_t)thread->chosen_processor->cpu_id, 0, 0, 0);
			}

			DTRACE_SCHED2(off__cpu, struct thread *, thread, struct proc *, current_proc());

			SCHED_STATS_CSW(processor, self->reason, self->sched_pri, thread->sched_pri);

#if KPERF
			kperf_off_cpu(self);
#endif /* KPERF */

			/*
			 * This is where we actually switch thread identity,
			 * and address space if required.  However, register
			 * state is not switched - this routine leaves the
			 * stack and register state active on the current CPU.
			 */
			TLOG(1, "thread_invoke: calling stack_handoff\n");
			stack_handoff(self, thread);

			/* 'self' is now off core */
			assert(thread == current_thread_volatile());

			DTRACE_SCHED(on__cpu);

#if KPERF
			kperf_on_cpu(thread, continuation, NULL);
#endif /* KPERF */


			recount_log_switch_thread_on(&snap);

			thread_dispatch(self, thread);

#if KASAN
			/* Old thread's stack has been moved to the new thread, so explicitly
			 * unpoison it. */
			kasan_unpoison_stack(thread->kernel_stack, kernel_stack_size);
#endif

			thread->continuation = thread->parameter = NULL;

			boolean_t enable_interrupts = TRUE;

			/* idle thread needs to stay interrupts-disabled */
			if ((thread->state & TH_IDLE)) {
				enable_interrupts = FALSE;
			}

			assert(continuation);
			call_continuation(continuation, parameter,
			    thread->wait_result, enable_interrupts);
			/*NOTREACHED*/
		} else if (thread == self) {
			/* same thread but with continuation */
			ast_context(self);

			thread_unlock(self);

#if KPERF
			kperf_on_cpu(thread, continuation, NULL);
#endif /* KPERF */

			recount_log_switch_thread_on(&snap);

			KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
			    MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED) | DBG_FUNC_NONE,
			    self->reason, (uintptr_t)thread_tid(thread), self->sched_pri, thread->sched_pri, 0);

#if KASAN
			/* stack handoff to self - no thread_dispatch(), so clear the stack
			 * and free the fakestack directly */
#if KASAN_CLASSIC
			kasan_fakestack_drop(self);
			kasan_fakestack_gc(self);
#endif /* KASAN_CLASSIC */
			kasan_unpoison_stack(self->kernel_stack, kernel_stack_size);
#endif /* KASAN */

			self->continuation = self->parameter = NULL;

			boolean_t enable_interrupts = TRUE;

			/* idle thread needs to stay interrupts-disabled */
			if ((self->state & TH_IDLE)) {
				enable_interrupts = FALSE;
			}

			call_continuation(continuation, parameter,
			    self->wait_result, enable_interrupts);
			/*NOTREACHED*/
		}
	} else {
		/*
		 * Check that the other thread has a stack
		 */
		if (!thread->kernel_stack) {
need_stack:
			if (!stack_alloc_try(thread)) {
				thread_unlock(thread);
				thread_stack_enqueue(thread);
				return FALSE;
			}
		} else if (thread == self) {
			ast_context(self);
			thread_unlock(self);

			KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
			    MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED) | DBG_FUNC_NONE,
			    self->reason, (uintptr_t)thread_tid(thread), self->sched_pri, thread->sched_pri, 0);

			return TRUE;
		}
	}

	/*
	 * Context switch by full context save.
	 */
	processor->active_thread = thread;
	processor_state_update_from_thread(processor, thread, false);

	if (thread->last_processor != processor && thread->last_processor != NULL) {
		if (thread->last_processor->processor_set != processor->processor_set) {
			thread->ps_switch++;
		}
		thread->p_switch++;
	}
	thread->last_processor = processor;
	thread->c_switch++;
	ast_context(thread);

	thread_unlock(thread);

	self->reason = reason;

	processor->last_dispatch = ctime;
	self->last_run_time = ctime;
	timer_update(&thread->runnable_timer, ctime);
	recount_switch_thread(&snap, self, get_threadtask(self));

	KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
	    MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED) | DBG_FUNC_NONE,
	    self->reason, (uintptr_t)thread_tid(thread), self->sched_pri, thread->sched_pri, 0);

	if ((thread->chosen_processor != processor) && (thread->chosen_processor != NULL)) {
		SCHED_DEBUG_CHOOSE_PROCESSOR_KERNEL_DEBUG_CONSTANT_IST(MACHDBG_CODE(DBG_MACH_SCHED, MACH_MOVED) | DBG_FUNC_NONE,
		    (uintptr_t)thread_tid(thread), (uintptr_t)thread->chosen_processor->cpu_id, 0, 0, 0);
	}

	DTRACE_SCHED2(off__cpu, struct thread *, thread, struct proc *, current_proc());

	SCHED_STATS_CSW(processor, self->reason, self->sched_pri, thread->sched_pri);

#if KPERF
	kperf_off_cpu(self);
#endif /* KPERF */

	/*
	 * This is where we actually switch register context,
	 * and address space if required.  We will next run
	 * as a result of a subsequent context switch.
	 *
	 * Once registers are switched and the processor is running "thread",
	 * the stack variables and non-volatile registers will contain whatever
	 * was there the last time that thread blocked. No local variables should
	 * be used after this point, except for the special case of "thread", which
	 * the platform layer returns as the previous thread running on the processor
	 * via the function call ABI as a return register, and "self", which may have
	 * been stored on the stack or a non-volatile register, but a stale idea of
	 * what was on the CPU is newly-accurate because that thread is again
	 * running on the CPU.
	 *
	 * If one of the threads is using a continuation, thread_continue
	 * is used to stitch up its context.
	 *
	 * If we are invoking a thread which is resuming from a continuation,
	 * the CPU will invoke thread_continue next.
	 *
	 * If the current thread is parking in a continuation, then its state
	 * won't be saved and the stack will be discarded. When the stack is
	 * re-allocated, it will be configured to resume from thread_continue.
	 */

	assert(continuation == self->continuation);
	thread = machine_switch_context(self, continuation, thread);
	assert(self == current_thread_volatile());
	TLOG(1, "thread_invoke: returning machine_switch_context: self %p continuation %p thread %p\n", self, continuation, thread);

	assert(continuation == NULL && self->continuation == NULL);

	DTRACE_SCHED(on__cpu);

#if KPERF
	kperf_on_cpu(self, NULL, __builtin_frame_address(0));
#endif /* KPERF */


	/* Previous snap on the old stack is gone. */
	recount_log_switch_thread_on(NULL);

	/* We have been resumed and are set to run. */
	thread_dispatch(thread, self);

	return TRUE;
}

#if defined(CONFIG_SCHED_DEFERRED_AST)
/*
 *	pset_cancel_deferred_dispatch:
 *
 *	Cancels all ASTs that we can cancel for the given processor set
 *	if the current processor is running the last runnable thread in the
 *	system.
 *
 *	This function assumes the current thread is runnable.  This must
 *	be called with the pset unlocked.
 */
static void
pset_cancel_deferred_dispatch(
	processor_set_t         pset,
	processor_t             processor)
{
	processor_t             active_processor = NULL;
	uint32_t                sampled_sched_run_count;

	pset_lock(pset);
	sampled_sched_run_count = os_atomic_load(&sched_run_buckets[TH_BUCKET_RUN], relaxed);

	/*
	 * If we have emptied the run queue, and our current thread is runnable, we
	 * should tell any processors that are still DISPATCHING that they will
	 * probably not have any work to do.  In the event that there are no
	 * pending signals that we can cancel, this is also uninteresting.
	 *
	 * In the unlikely event that another thread becomes runnable while we are
	 * doing this (sched_run_count is atomically updated, not guarded), the
	 * codepath making it runnable SHOULD (a dangerous word) need the pset lock
	 * in order to dispatch it to a processor in our pset.  So, the other
	 * codepath will wait while we squash all cancelable ASTs, get the pset
	 * lock, and then dispatch the freshly runnable thread.  So this should be
	 * correct (we won't accidentally have a runnable thread that hasn't been
	 * dispatched to an idle processor), if not ideal (we may be restarting the
	 * dispatch process, which could have some overhead).
	 */

	if ((sampled_sched_run_count == 1) && (pset->pending_deferred_AST_cpu_mask)) {
		uint64_t dispatching_map = (pset->cpu_state_map[PROCESSOR_DISPATCHING] &
		    pset->pending_deferred_AST_cpu_mask &
		    ~pset->pending_AST_URGENT_cpu_mask);
		for (int cpuid = lsb_first(dispatching_map); cpuid >= 0; cpuid = lsb_next(dispatching_map, cpuid)) {
			active_processor = processor_array[cpuid];
			/*
			 * If a processor is DISPATCHING, it could be because of
			 * a cancelable signal.
			 *
			 * IF the processor is not our
			 * current processor (the current processor should not
			 * be DISPATCHING, so this is a bit paranoid), AND there
			 * is a cancelable signal pending on the processor, AND
			 * there is no non-cancelable signal pending (as there is
			 * no point trying to backtrack on bringing the processor
			 * up if a signal we cannot cancel is outstanding), THEN
			 * it should make sense to roll back the processor state
			 * to the IDLE state.
			 *
			 * If the racey nature of this approach (as the signal
			 * will be arbitrated by hardware, and can fire as we
			 * roll back state) results in the core responding
			 * despite being pushed back to the IDLE state, it
			 * should be no different than if the core took some
			 * interrupt while IDLE.
			 */
			if (active_processor != processor) {
				/*
				 * Squash all of the processor state back to some
				 * reasonable facsimile of PROCESSOR_IDLE.
				 */

				processor_state_update_idle(active_processor);
				active_processor->deadline = RT_DEADLINE_NONE;
				pset_update_processor_state(pset, active_processor, PROCESSOR_IDLE);
				bit_clear(pset->pending_deferred_AST_cpu_mask, active_processor->cpu_id);
				machine_signal_idle_cancel(active_processor);
			}
		}
	}

	pset_unlock(pset);
}
#else
/* We don't support deferred ASTs; everything is candycanes and sunshine. */
#endif

static void
thread_csw_callout(
	thread_t            old,
	thread_t            new,
	uint64_t            timestamp)
{
	perfcontrol_event event = (new->state & TH_IDLE) ? IDLE : CONTEXT_SWITCH;
	uint64_t same_pri_latency = (new->state & TH_IDLE) ? 0 : new->same_pri_latency;
	machine_switch_perfcontrol_context(event, timestamp, 0,
	    same_pri_latency, old, new);
}


/*
 *	thread_dispatch:
 *
 *	Handle threads at context switch.  Re-dispatch other thread
 *	if still running, otherwise update run state and perform
 *	special actions.  Update quantum for other thread and begin
 *	the quantum for ourselves.
 *
 *      "thread" is the old thread that we have switched away from.
 *      "self" is the new current thread that we have context switched to
 *
 *	Called at splsched.
 *
 */
void
thread_dispatch(
	thread_t                thread,
	thread_t                self)
{
	processor_t             processor = self->last_processor;
	bool was_idle = false;
	bool processor_bootstrap = (thread == THREAD_NULL);

	assert(processor == current_processor());
	assert(self == current_thread_volatile());
	assert(thread != self);

	if (thread != THREAD_NULL) {
		/*
		 * Do the perfcontrol callout for context switch.
		 * The reason we do this here is:
		 * - thread_dispatch() is called from various places that are not
		 *   the direct context switch path for eg. processor shutdown etc.
		 *   So adding the callout here covers all those cases.
		 * - We want this callout as early as possible to be close
		 *   to the timestamp taken in thread_invoke()
		 * - We want to avoid holding the thread lock while doing the
		 *   callout
		 * - We do not want to callout if "thread" is NULL.
		 */
		thread_csw_callout(thread, self, processor->last_dispatch);

#if KASAN
		if (thread->continuation != NULL) {
			/*
			 * Thread has a continuation and the normal stack is going away.
			 * Unpoison the stack and mark all fakestack objects as unused.
			 */
#if KASAN_CLASSIC
			kasan_fakestack_drop(thread);
#endif /* KASAN_CLASSIC */
			if (thread->kernel_stack) {
				kasan_unpoison_stack(thread->kernel_stack, kernel_stack_size);
			}
		}


#if KASAN_CLASSIC
		/*
		 * Free all unused fakestack objects.
		 */
		kasan_fakestack_gc(thread);
#endif /* KASAN_CLASSIC */
#endif /* KASAN */

		/*
		 *	If blocked at a continuation, discard
		 *	the stack.
		 */
		if (thread->continuation != NULL && thread->kernel_stack != 0) {
			stack_free(thread);
		}

		if (thread->state & TH_IDLE) {
			was_idle = true;
			KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
			    MACHDBG_CODE(DBG_MACH_SCHED, MACH_DISPATCH) | DBG_FUNC_NONE,
			    (uintptr_t)thread_tid(thread), 0, thread->state,
			    sched_run_buckets[TH_BUCKET_RUN], 0);
		} else {
			int64_t consumed;
			int64_t remainder = 0;

			if (processor->quantum_end > processor->last_dispatch) {
				remainder = processor->quantum_end -
				    processor->last_dispatch;
			}

			consumed = thread->quantum_remaining - remainder;

			if ((thread->reason & AST_LEDGER) == 0) {
				/*
				 * Bill CPU time to both the task and
				 * the individual thread.
				 */
				ledger_credit_thread(thread, thread->t_ledger,
				    task_ledgers.cpu_time, consumed);
				ledger_credit_thread(thread, thread->t_threadledger,
				    thread_ledgers.cpu_time, consumed);
				if (thread->t_bankledger) {
					ledger_credit_thread(thread, thread->t_bankledger,
					    bank_ledgers.cpu_time,
					    (consumed - thread->t_deduct_bank_ledger_time));
				}
				thread->t_deduct_bank_ledger_time = 0;
				if (consumed > 0) {
					/*
					 * This should never be negative, but in traces we are seeing some instances
					 * of consumed being negative.
					 * <rdar://problem/57782596> thread_dispatch() thread CPU consumed calculation sometimes results in negative value
					 */
					SCHED(update_pset_avg_execution_time)(current_processor()->processor_set, consumed, processor->last_dispatch, thread->th_sched_bucket);
				}
			}

			/* For the thread that we just context switched away from, figure
			 * out if we have expired the wq quantum and set the AST if we have
			 */
			if (thread_get_tag(thread) & THREAD_TAG_WORKQUEUE) {
				thread_evaluate_workqueue_quantum_expiry(thread);
			}

			if (__improbable(thread->rwlock_count != 0)) {
				smr_mark_active_trackers_stalled(thread);
			}

			/*
			 * Pairs with task_restartable_ranges_synchronize
			 */
			wake_lock(thread);
			thread_lock(thread);

			/*
			 * Same as ast_check(), in case we missed the IPI
			 */
			thread_reset_pcs_ack_IPI(thread);

			/*
			 * Apply a priority floor if the thread holds a kernel resource
			 * or explicitly requested it.
			 * Do this before checking starting_pri to avoid overpenalizing
			 * repeated rwlock blockers.
			 */
			if (__improbable(thread->rwlock_count != 0)) {
				lck_rw_set_promotion_locked(thread);
			}
			if (__improbable(thread->priority_floor_count != 0)) {
				thread_floor_boost_set_promotion_locked(thread);
			}

			boolean_t keep_quantum = processor->first_timeslice;

			/*
			 * Treat a thread which has dropped priority since it got on core
			 * as having expired its quantum.
			 */
			if (processor->starting_pri > thread->sched_pri) {
				keep_quantum = FALSE;
			}

			/* Compute remainder of current quantum. */
			if (keep_quantum &&
			    processor->quantum_end > processor->last_dispatch) {
				thread->quantum_remaining = (uint32_t)remainder;
			} else {
				thread->quantum_remaining = 0;
			}

			if (thread->sched_mode == TH_MODE_REALTIME) {
				/*
				 *	Cancel the deadline if the thread has
				 *	consumed the entire quantum.
				 */
				if (thread->quantum_remaining == 0) {
					KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_CANCEL_RT_DEADLINE) | DBG_FUNC_NONE,
					    (uintptr_t)thread_tid(thread), thread->realtime.deadline, thread->realtime.computation, 0);
					thread->realtime.deadline = RT_DEADLINE_QUANTUM_EXPIRED;
				}
			} else {
#if defined(CONFIG_SCHED_TIMESHARE_CORE)
				/*
				 *	For non-realtime threads treat a tiny
				 *	remaining quantum as an expired quantum
				 *	but include what's left next time.
				 */
				if (thread->quantum_remaining < min_std_quantum) {
					thread->reason |= AST_QUANTUM;
					thread->quantum_remaining += SCHED(initial_quantum_size)(thread);
				}
#endif /* CONFIG_SCHED_TIMESHARE_CORE */
			}

			/*
			 *	If we are doing a direct handoff then
			 *	take the remainder of the quantum.
			 */
			if ((thread->reason & (AST_HANDOFF | AST_QUANTUM)) == AST_HANDOFF) {
				self->quantum_remaining = thread->quantum_remaining;
				thread->reason |= AST_QUANTUM;
				thread->quantum_remaining = 0;
			}

			thread->computation_metered += (processor->last_dispatch - thread->computation_epoch);

			if (!(thread->state & TH_WAIT)) {
				/*
				 *	Still runnable.
				 */
				thread->last_made_runnable_time = thread->last_basepri_change_time = processor->last_dispatch;

				machine_thread_going_off_core(thread, FALSE, processor->last_dispatch, TRUE);

				ast_t reason = thread->reason;
				sched_options_t options = SCHED_NONE;

				if (reason & AST_REBALANCE) {
					options |= SCHED_REBALANCE;
					if (reason & AST_QUANTUM) {
						/*
						 * Having gone to the trouble of forcing this thread off a less preferred core,
						 * we should force the preferable core to reschedule immediately to give this
						 * thread a chance to run instead of just sitting on the run queue where
						 * it may just be stolen back by the idle core we just forced it off.
						 * But only do this at the end of a quantum to prevent cascading effects.
						 */
						options |= SCHED_STIR_POT;
					}
				}

				if (reason & AST_QUANTUM) {
					options |= SCHED_TAILQ;
				} else if (reason & AST_PREEMPT) {
					options |= SCHED_HEADQ;
				} else {
					options |= (SCHED_PREEMPT | SCHED_TAILQ);
				}

				thread_setrun(thread, options);

				KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
				    MACHDBG_CODE(DBG_MACH_SCHED, MACH_DISPATCH) | DBG_FUNC_NONE,
				    (uintptr_t)thread_tid(thread), thread->reason, thread->state,
				    sched_run_buckets[TH_BUCKET_RUN], 0);

				if (thread->wake_active) {
					thread->wake_active = FALSE;
					thread_unlock(thread);

					thread_wakeup(&thread->wake_active);
				} else {
					thread_unlock(thread);
				}

				wake_unlock(thread);
			} else {
				/*
				 *	Waiting.
				 */
				boolean_t should_terminate = FALSE;
				uint32_t new_run_count;
				int thread_state = thread->state;

				/* Only the first call to thread_dispatch
				 * after explicit termination should add
				 * the thread to the termination queue
				 */
				if ((thread_state & (TH_TERMINATE | TH_TERMINATE2)) == TH_TERMINATE) {
					should_terminate = TRUE;
					thread_state |= TH_TERMINATE2;
				}

				timer_stop(&thread->runnable_timer, processor->last_dispatch);

				thread_state &= ~TH_RUN;
				thread->state = thread_state;

				thread->last_made_runnable_time = thread->last_basepri_change_time = THREAD_NOT_RUNNABLE;
				thread->chosen_processor = PROCESSOR_NULL;

				new_run_count = SCHED(run_count_decr)(thread);

#if CONFIG_SCHED_AUTO_JOIN
				if ((thread->sched_flags & TH_SFLAG_THREAD_GROUP_AUTO_JOIN) != 0) {
					work_interval_auto_join_unwind(thread);
				}
#endif /* CONFIG_SCHED_AUTO_JOIN */

#if CONFIG_SCHED_SFI
				if (thread->reason & AST_SFI) {
					thread->wait_sfi_begin_time = processor->last_dispatch;
				}
#endif
				machine_thread_going_off_core(thread, should_terminate, processor->last_dispatch, FALSE);

				KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
				    MACHDBG_CODE(DBG_MACH_SCHED, MACH_DISPATCH) | DBG_FUNC_NONE,
				    (uintptr_t)thread_tid(thread), thread->reason, thread_state,
				    new_run_count, 0);

				if (thread_state & TH_WAIT_REPORT) {
					(*thread->sched_call)(SCHED_CALL_BLOCK, thread);
				}

				if (thread->wake_active) {
					thread->wake_active = FALSE;
					thread_unlock(thread);

					thread_wakeup(&thread->wake_active);
				} else {
					thread_unlock(thread);
				}

				wake_unlock(thread);

				if (should_terminate) {
					thread_terminate_enqueue(thread);
				}
			}
		}
		/*
		 * The thread could have been added to the termination queue, so it's
		 * unsafe to use after this point.
		 */
		thread = THREAD_NULL;
	}

	int urgency = THREAD_URGENCY_NONE;
	uint64_t latency = 0;

	/* Update (new) current thread and reprogram running timers */
	thread_lock(self);

	if (!(self->state & TH_IDLE)) {
		uint64_t        arg1, arg2;

#if CONFIG_SCHED_SFI
		ast_t                   new_ast;

		new_ast = sfi_thread_needs_ast(self, NULL);

		if (new_ast != AST_NONE) {
			ast_on(new_ast);
		}
#endif

		if (processor->last_dispatch < self->last_made_runnable_time) {
			panic("Non-monotonic time: dispatch at 0x%llx, runnable at 0x%llx",
			    processor->last_dispatch, self->last_made_runnable_time);
		}

		assert(self->last_made_runnable_time <= self->last_basepri_change_time);

		latency = processor->last_dispatch - self->last_made_runnable_time;
		assert(latency >= self->same_pri_latency);

		urgency = thread_get_urgency(self, &arg1, &arg2);

		thread_tell_urgency(urgency, arg1, arg2, latency, self);

		/*
		 *	Start a new CPU limit interval if the previous one has
		 *	expired. This should happen before initializing a new
		 *	quantum.
		 */
		if (cpulimit_affects_quantum &&
		    thread_cpulimit_interval_has_expired(processor->last_dispatch)) {
			thread_cpulimit_restart(processor->last_dispatch);
		}

		/*
		 *	Get a new quantum if none remaining.
		 */
		if (self->quantum_remaining == 0) {
			thread_quantum_init(self, processor->last_dispatch);
		}

		/*
		 *	Set up quantum timer and timeslice.
		 */
		processor->quantum_end = processor->last_dispatch +
		    self->quantum_remaining;

		running_timer_setup(processor, RUNNING_TIMER_QUANTUM, self,
		    processor->quantum_end, processor->last_dispatch);
		if (was_idle) {
			/*
			 * kperf's running timer is active whenever the idle thread for a
			 * CPU is not running.
			 */
			kperf_running_setup(processor, processor->last_dispatch);
		}
		running_timers_activate(processor);
		processor->first_timeslice = TRUE;
	} else {
		if (!processor_bootstrap) {
			running_timers_deactivate(processor);
		}
		processor->first_timeslice = FALSE;
		thread_tell_urgency(THREAD_URGENCY_NONE, 0, 0, 0, self);
	}

	assert(self->block_hint == kThreadWaitNone);
	self->computation_epoch = processor->last_dispatch;
	/*
	 * This relies on the interrupt time being tallied up to the thread in the
	 * exception handler epilogue, which is before AST context where preemption
	 * is considered (and the scheduler is potentially invoked to
	 * context switch, here).
	 */
	self->computation_interrupt_epoch = recount_current_thread_interrupt_time_mach();
	self->reason = AST_NONE;
	processor->starting_pri = self->sched_pri;

	thread_unlock(self);

	machine_thread_going_on_core(self, urgency, latency, self->same_pri_latency,
	    processor->last_dispatch);

#if defined(CONFIG_SCHED_DEFERRED_AST)
	/*
	 * TODO: Can we state that redispatching our old thread is also
	 * uninteresting?
	 */
	if ((os_atomic_load(&sched_run_buckets[TH_BUCKET_RUN], relaxed) == 1) && !(self->state & TH_IDLE)) {
		pset_cancel_deferred_dispatch(processor->processor_set, processor);
	}
#endif
}

/*
 *	thread_block_reason:
 *
 *	Forces a reschedule, blocking the caller if a wait
 *	has been asserted.
 *
 *	If a continuation is specified, then thread_invoke will
 *	attempt to discard the thread's kernel stack.  When the
 *	thread resumes, it will execute the continuation function
 *	on a new kernel stack.
 */
__mockable wait_result_t
thread_block_reason(
	thread_continue_t       continuation,
	void                            *parameter,
	ast_t                           reason)
{
	thread_t        self = current_thread();
	processor_t     processor;
	thread_t        new_thread;
	spl_t           s;

	s = splsched();

	processor = current_processor();

	/* If we're explicitly yielding, force a subsequent quantum */
	if (reason & AST_YIELD) {
		processor->first_timeslice = FALSE;
	}

	/* We're handling all scheduling AST's */
	ast_off(AST_SCHEDULING);

	clear_pending_nonurgent_preemption(processor);

#if PROC_REF_DEBUG
	if ((continuation != NULL) && (get_threadtask(self) != kernel_task)) {
		uthread_assert_zero_proc_refcount(get_bsdthread_info(self));
	}
#endif

#if CONFIG_EXCLAVES
	if (continuation != NULL) {
		assert3u(self->th_exclaves_state & TH_EXCLAVES_STATE_ANY, ==, 0);
	}
#endif /* CONFIG_EXCLAVES */

	self->continuation = continuation;
	self->parameter = parameter;

	if (self->state & ~(TH_RUN | TH_IDLE)) {
		KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
		    MACHDBG_CODE(DBG_MACH_SCHED, MACH_BLOCK),
		    reason, VM_KERNEL_UNSLIDE(continuation), 0, 0, 0);
	}

	do {
		thread_lock(self);
		new_thread = thread_select(self, processor, &reason);
		thread_unlock(self);
	} while (!thread_invoke(self, new_thread, reason));

	splx(s);

	return self->wait_result;
}

/*
 *	thread_block:
 *
 *	Block the current thread if a wait has been asserted.
 */
wait_result_t
thread_block(
	thread_continue_t       continuation)
{
	return thread_block_reason(continuation, NULL, AST_NONE);
}

wait_result_t
thread_block_parameter(
	thread_continue_t       continuation,
	void                            *parameter)
{
	return thread_block_reason(continuation, parameter, AST_NONE);
}

/*
 *	thread_run:
 *
 *	Switch directly from the current thread to the
 *	new thread, handing off our quantum if appropriate.
 *
 *	New thread must be runnable, and not on a run queue.
 *
 *	Called at splsched.
 */
int
thread_run(
	thread_t                        self,
	thread_continue_t       continuation,
	void                            *parameter,
	thread_t                        new_thread)
{
	ast_t reason = AST_NONE;

	if ((self->state & TH_IDLE) == 0) {
		reason = AST_HANDOFF;
	}

	/* Must not get here without a chosen processor */
	assert(new_thread->chosen_processor);

	self->continuation = continuation;
	self->parameter = parameter;

	while (!thread_invoke(self, new_thread, reason)) {
		/* the handoff failed, so we have to fall back to the normal block path */
		processor_t processor = current_processor();

		reason = AST_NONE;

		thread_lock(self);
		new_thread = thread_select(self, processor, &reason);
		thread_unlock(self);
	}

	return self->wait_result;
}

/*
 *	thread_continue:
 *
 *	Called at splsched when a thread first receives
 *	a new stack after a continuation.
 *
 *	Called with THREAD_NULL as the old thread when
 *	invoked by machine_load_context.
 */
void
thread_continue(
	thread_t        thread)
{
	thread_t                self = current_thread();
	thread_continue_t       continuation;
	void                    *parameter;

	DTRACE_SCHED(on__cpu);

	continuation = self->continuation;
	parameter = self->parameter;

	assert(continuation != NULL);

#if KPERF
	kperf_on_cpu(self, continuation, NULL);
#endif


	thread_dispatch(thread, self);

	self->continuation = self->parameter = NULL;

#if SCHED_HYGIENE_DEBUG
	/* Reset interrupt-masked spin debugging timeout */
	ml_spin_debug_clear(self);
#endif

	TLOG(1, "thread_continue: calling call_continuation\n");

	boolean_t enable_interrupts = TRUE;

	/* bootstrap thread, idle thread need to stay interrupts-disabled */
	if (thread == THREAD_NULL || (self->state & TH_IDLE)) {
		enable_interrupts = FALSE;
	}

#if KASAN_TBI
	kasan_unpoison_stack(self->kernel_stack, kernel_stack_size);
#endif /* KASAN_TBI */


	call_continuation(continuation, parameter, self->wait_result, enable_interrupts);
	/*NOTREACHED*/
}

void
thread_quantum_init(thread_t thread, uint64_t now)
{
	uint64_t new_quantum = 0;

	switch (thread->sched_mode) {
	case TH_MODE_REALTIME:
		new_quantum = thread->realtime.computation;
		new_quantum = MIN(new_quantum, max_unsafe_rt_computation);
		break;

	case TH_MODE_FIXED:
		new_quantum = SCHED(initial_quantum_size)(thread);
		new_quantum = MIN(new_quantum, max_unsafe_fixed_computation);
		break;

	default:
		new_quantum = SCHED(initial_quantum_size)(thread);
		break;
	}

	if (cpulimit_affects_quantum) {
		const uint64_t cpulimit_remaining = thread_cpulimit_remaining(now);

		/*
		 * If there's no remaining CPU time, the ledger system will
		 * notice and put the thread to sleep.
		 */
		if (cpulimit_remaining > 0) {
			new_quantum = MIN(new_quantum, cpulimit_remaining);
		}
	}

	assert3u(new_quantum, <, UINT32_MAX);
	assert3u(new_quantum, >, 0);

	thread->quantum_remaining = (uint32_t)new_quantum;
}

uint32_t
sched_timeshare_initial_quantum_size(thread_t thread)
{
	if ((thread != THREAD_NULL) && thread->th_sched_bucket == TH_BUCKET_SHARE_BG) {
		return bg_quantum;
	} else {
		return std_quantum;
	}
}

/*
 *	run_queue_init:
 *
 *	Initialize a run queue before first use.
 */
void
run_queue_init(
	run_queue_t             rq)
{
	rq->highq = NOPRI;
	for (u_int i = 0; i < BITMAP_LEN(NRQS); i++) {
		rq->bitmap[i] = 0;
	}
	rq->urgency = rq->count = 0;
	for (int i = 0; i < NRQS; i++) {
		circle_queue_init(&rq->queues[i]);
	}
}

/*
 *	run_queue_dequeue:
 *
 *	Perform a dequeue operation on a run queue,
 *	and return the resulting thread.
 *
 *	The run queue must be locked (see thread_run_queue_remove()
 *	for more info), and not empty.
 */
thread_t
run_queue_dequeue(
	run_queue_t     rq,
	sched_options_t options)
{
	thread_t        thread;
	circle_queue_t  queue = &rq->queues[rq->highq];

	if (options & SCHED_HEADQ) {
		thread = cqe_dequeue_head(queue, struct thread, runq_links);
	} else {
		thread = cqe_dequeue_tail(queue, struct thread, runq_links);
	}

	assert(thread != THREAD_NULL);
	assert_thread_magic(thread);

	thread_clear_runq(thread);
	SCHED_STATS_RUNQ_CHANGE(&rq->runq_stats, rq->count);
	rq->count--;
	if (SCHED(priority_is_urgent)(rq->highq)) {
		rq->urgency--; assert(rq->urgency >= 0);
	}
	if (circle_queue_empty(queue)) {
		bitmap_clear(rq->bitmap, rq->highq);
		rq->highq = bitmap_first(rq->bitmap, NRQS);
	}

	return thread;
}

/*
 *	run_queue_enqueue:
 *
 *	Perform a enqueue operation on a run queue.
 *
 *	The run queue must be locked (see thread_run_queue_remove()
 *	for more info).
 */
boolean_t
run_queue_enqueue(
	run_queue_t      rq,
	thread_t         thread,
	sched_options_t  options)
{
	circle_queue_t  queue = &rq->queues[thread->sched_pri];
	boolean_t       result = FALSE;

	assert_thread_magic(thread);

	if (circle_queue_empty(queue)) {
		circle_enqueue_tail(queue, &thread->runq_links);

		rq_bitmap_set(rq->bitmap, thread->sched_pri);
		if (thread->sched_pri > rq->highq) {
			rq->highq = thread->sched_pri;
			result = TRUE;
		}
	} else {
		if (options & SCHED_TAILQ) {
			circle_enqueue_tail(queue, &thread->runq_links);
		} else {
			circle_enqueue_head(queue, &thread->runq_links);
		}
	}
	if (SCHED(priority_is_urgent)(thread->sched_pri)) {
		rq->urgency++;
	}
	SCHED_STATS_RUNQ_CHANGE(&rq->runq_stats, rq->count);
	rq->count++;

	return result;
}

/*
 *	run_queue_remove:
 *
 *	Remove a specific thread from a runqueue.
 *
 *	The run queue must be locked.
 */
void
run_queue_remove(
	run_queue_t    rq,
	thread_t       thread)
{
	circle_queue_t  queue = &rq->queues[thread->sched_pri];

	thread_assert_runq_nonnull(thread);
	assert_thread_magic(thread);

	circle_dequeue(queue, &thread->runq_links);
	SCHED_STATS_RUNQ_CHANGE(&rq->runq_stats, rq->count);
	rq->count--;
	if (SCHED(priority_is_urgent)(thread->sched_pri)) {
		rq->urgency--; assert(rq->urgency >= 0);
	}

	if (circle_queue_empty(queue)) {
		/* update run queue status */
		bitmap_clear(rq->bitmap, thread->sched_pri);
		rq->highq = bitmap_first(rq->bitmap, NRQS);
	}

	thread_clear_runq(thread);
}

/*
 *      run_queue_peek
 *
 *      Peek at the runq and return the highest
 *      priority thread from the runq.
 *
 *	The run queue must be locked.
 */
thread_t
run_queue_peek(
	run_queue_t    rq)
{
	if (rq->count > 0) {
		circle_queue_t queue = &rq->queues[rq->highq];
		thread_t thread = cqe_queue_first(queue, struct thread, runq_links);
		assert_thread_magic(thread);
		return thread;
	} else {
		return THREAD_NULL;
	}
}

/*
 *	realtime_setrun:
 *
 *	Dispatch a thread for realtime execution.
 *
 *	Thread must be locked.  Associated pset must
 *	be locked, and is returned unlocked.
 */
static void
realtime_setrun(
	processor_t                     chosen_processor,
	thread_t                        thread)
{
	processor_set_t pset = chosen_processor->processor_set;
	pset_assert_locked(pset);
	bool pset_is_locked = true;

	int n_backup = 0;

	if (thread->realtime.constraint <= rt_constraint_threshold) {
		n_backup = sched_rt_n_backup_processors;
	}
	assert((n_backup >= 0) && (n_backup <= SCHED_MAX_BACKUP_PROCESSORS));

	int existing_backups = bit_count(pset->pending_AST_URGENT_cpu_mask) - rt_runq_count(pset);
	if (existing_backups > 0) {
		n_backup = n_backup - existing_backups;
		if (n_backup < 0) {
			n_backup = 0;
		}
	}

	sched_ipi_type_t ipi_type[SCHED_MAX_BACKUP_PROCESSORS + 1] = {};
	processor_t ipi_processor[SCHED_MAX_BACKUP_PROCESSORS + 1] = {};

	thread->chosen_processor = chosen_processor;

	/* <rdar://problem/15102234> */
	assert(thread->bound_processor == PROCESSOR_NULL);

	rt_runq_insert(chosen_processor, pset, thread);

	processor_t processor = chosen_processor;

	int count = 0;
	for (int i = 0; i <= n_backup; i++) {
		if (i == 0) {
			ipi_type[i] = SCHED_IPI_NONE;
			ipi_processor[i] = processor;
			count++;

			ast_t preempt = AST_NONE;
			if (thread->sched_pri > processor->current_pri) {
				preempt = (AST_PREEMPT | AST_URGENT);
			} else if (thread->sched_pri == processor->current_pri) {
				if (rt_deadline_add(thread->realtime.deadline, rt_deadline_epsilon) < processor->deadline) {
					preempt = (AST_PREEMPT | AST_URGENT);
				}
			}

			if (preempt != AST_NONE) {
				if (processor->state == PROCESSOR_IDLE) {
					if (processor == current_processor()) {
						pset_update_processor_state(pset, processor, PROCESSOR_DISPATCHING);
						ast_on(preempt);

						if ((preempt & AST_URGENT) == AST_URGENT) {
							if (bit_set_if_clear(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
								KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PENDING_AST_URGENT) | DBG_FUNC_START,
								    processor->cpu_id, pset->pending_AST_URGENT_cpu_mask, (uintptr_t)thread_tid(thread), 1);
							}
						}

						if ((preempt & AST_PREEMPT) == AST_PREEMPT) {
							bit_set(pset->pending_AST_PREEMPT_cpu_mask, processor->cpu_id);
						}
					} else {
						ipi_type[i] = sched_ipi_action(processor, thread, SCHED_IPI_EVENT_RT_PREEMPT);
					}
				} else if (processor->state == PROCESSOR_DISPATCHING) {
					if (bit_set_if_clear(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
						KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PENDING_AST_URGENT) | DBG_FUNC_START,
						    processor->cpu_id, pset->pending_AST_URGENT_cpu_mask, (uintptr_t)thread_tid(thread), 2);
					}
				} else {
					if (processor == current_processor()) {
						ast_on(preempt);

						if ((preempt & AST_URGENT) == AST_URGENT) {
							if (bit_set_if_clear(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
								KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PENDING_AST_URGENT) | DBG_FUNC_START,
								    processor->cpu_id, pset->pending_AST_URGENT_cpu_mask, (uintptr_t)thread_tid(thread), 3);
							}
						}

						if ((preempt & AST_PREEMPT) == AST_PREEMPT) {
							bit_set(pset->pending_AST_PREEMPT_cpu_mask, processor->cpu_id);
						}
					} else {
						ipi_type[i] = sched_ipi_action(processor, thread, SCHED_IPI_EVENT_RT_PREEMPT);
					}
				}
			} else {
				/* Selected processor was too busy, just keep thread enqueued and let other processors drain it naturally. */
			}
		} else {
			if (!pset_is_locked) {
				pset_lock(pset);
			}
			ipi_type[i] = SCHED_IPI_NONE;
			ipi_processor[i] = PROCESSOR_NULL;
			rt_choose_next_processor_for_followup_IPI(pset, chosen_processor, &ipi_processor[i], &ipi_type[i]);
			if (ipi_processor[i] == PROCESSOR_NULL) {
				break;
			}
			count++;

			KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_NEXT_PROCESSOR) | DBG_FUNC_NONE,
			    ipi_processor[i]->cpu_id, ipi_processor[i]->state, backup, 1);
#if CONFIG_SCHED_SMT
#define p_is_good(p) (((p)->processor_primary == (p)) && ((sched_avoid_cpu0 != 1) || ((p)->cpu_id != 0)))
			if (n_backup == SCHED_DEFAULT_BACKUP_PROCESSORS_SMT) {
				processor_t p0 = ipi_processor[0];
				processor_t p1 = ipi_processor[1];
				assert(p0 && p1);
				if (p_is_good(p0) && p_is_good(p1)) {
					/*
					 * Both the chosen processor and the first backup are non-cpu0 primaries,
					 * so there is no need for a 2nd backup processor.
					 */
					break;
				}
			}
#endif /* CONFIG_SCHED_SMT */
		}
	}

	if (pset_is_locked) {
		pset_unlock(pset);
	}

	assert((count > 0) && (count <= (n_backup + 1)));
	for (int i = 0; i < count; i++) {
		assert(ipi_processor[i] != PROCESSOR_NULL);
		sched_ipi_perform(ipi_processor[i], ipi_type[i]);
	}
}


sched_ipi_type_t
sched_ipi_deferred_policy(processor_set_t pset, processor_t dst,
    thread_t thread, __unused sched_ipi_event_t event)
{
#if defined(CONFIG_SCHED_DEFERRED_AST)
#if CONFIG_THREAD_GROUPS
	if (thread) {
		struct thread_group *tg = thread_group_get(thread);
		if (thread_group_uses_immediate_ipi(tg)) {
			return SCHED_IPI_IMMEDIATE;
		}
	}
#endif /* CONFIG_THREAD_GROUPS */
	if (!bit_test(pset->pending_deferred_AST_cpu_mask, dst->cpu_id)) {
		return SCHED_IPI_DEFERRED;
	}
#else /* CONFIG_SCHED_DEFERRED_AST */
	(void) thread;
	panic("Request for deferred IPI on an unsupported platform; pset: %p CPU: %d", pset, dst->cpu_id);
#endif /* CONFIG_SCHED_DEFERRED_AST */
	return SCHED_IPI_NONE;
}

/* Requires the destination pset lock to be held */
sched_ipi_type_t
sched_ipi_action(processor_t dst, thread_t thread, sched_ipi_event_t event)
{
	sched_ipi_type_t ipi_type = SCHED_IPI_NONE;
	assert(dst != NULL);

	processor_set_t pset = dst->processor_set;
	if (current_processor() == dst) {
		return SCHED_IPI_NONE;
	}

	bool dst_idle = (dst->state == PROCESSOR_IDLE);
	if (dst_idle) {
		pset_update_processor_state(pset, dst, PROCESSOR_DISPATCHING);
	}

	ipi_type = SCHED(ipi_policy)(dst, thread, dst_idle, event);
	switch (ipi_type) {
	case SCHED_IPI_NONE:
		return SCHED_IPI_NONE;
#if defined(CONFIG_SCHED_DEFERRED_AST)
	case SCHED_IPI_DEFERRED:
		bit_set(pset->pending_deferred_AST_cpu_mask, dst->cpu_id);
		break;
#endif /* CONFIG_SCHED_DEFERRED_AST */
	default:
		if (bit_set_if_clear(pset->pending_AST_URGENT_cpu_mask, dst->cpu_id)) {
			KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PENDING_AST_URGENT) | DBG_FUNC_START,
			    dst->cpu_id, pset->pending_AST_URGENT_cpu_mask, (uintptr_t)thread_tid(thread), 4);
		}
		bit_set(pset->pending_AST_PREEMPT_cpu_mask, dst->cpu_id);
		break;
	}
	return ipi_type;
}

sched_ipi_type_t
sched_ipi_policy(processor_t dst, thread_t thread, boolean_t dst_idle, sched_ipi_event_t event)
{
	sched_ipi_type_t ipi_type = SCHED_IPI_NONE;
	boolean_t deferred_ipi_supported = false;
	processor_set_t pset = dst->processor_set;

#if defined(CONFIG_SCHED_DEFERRED_AST)
	deferred_ipi_supported = true;
#endif /* CONFIG_SCHED_DEFERRED_AST */

	switch (event) {
	case SCHED_IPI_EVENT_SPILL:
	case SCHED_IPI_EVENT_SMT_REBAL:
	case SCHED_IPI_EVENT_REBALANCE:
	case SCHED_IPI_EVENT_BOUND_THR:
	case SCHED_IPI_EVENT_RT_PREEMPT:
		/*
		 * The RT preempt, spill, SMT rebalance, rebalance and the bound thread
		 * scenarios use immediate IPIs always.
		 */
		ipi_type = dst_idle ? SCHED_IPI_IDLE : SCHED_IPI_IMMEDIATE;
		break;
	case SCHED_IPI_EVENT_PREEMPT:
		/* In the preemption case, use immediate IPIs for RT threads */
		if (thread && (thread->sched_pri >= BASEPRI_RTQUEUES)) {
			ipi_type = dst_idle ? SCHED_IPI_IDLE : SCHED_IPI_IMMEDIATE;
			break;
		}

		/*
		 * For Non-RT threads preemption,
		 * If the core is active, use immediate IPIs.
		 * If the core is idle, use deferred IPIs if supported; otherwise immediate IPI.
		 */
		if (deferred_ipi_supported && dst_idle) {
			return sched_ipi_deferred_policy(pset, dst, thread, event);
		}
		ipi_type = dst_idle ? SCHED_IPI_IDLE : SCHED_IPI_IMMEDIATE;
		break;
	default:
		panic("Unrecognized scheduler IPI event type %d", event);
	}
	assert(ipi_type != SCHED_IPI_NONE);
	return ipi_type;
}

void
sched_ipi_perform(processor_t dst, sched_ipi_type_t ipi)
{
	switch (ipi) {
	case SCHED_IPI_NONE:
		break;
	case SCHED_IPI_IDLE:
		machine_signal_idle(dst);
		break;
	case SCHED_IPI_IMMEDIATE:
		cause_ast_check(dst);
		break;
	case SCHED_IPI_DEFERRED:
		machine_signal_idle_deferred(dst);
		break;
	default:
		panic("Unrecognized scheduler IPI type: %d", ipi);
	}
}

#if defined(CONFIG_SCHED_TIMESHARE_CORE)

boolean_t
priority_is_urgent(int priority)
{
	return bitmap_test(sched_preempt_pri, priority) ? TRUE : FALSE;
}

#endif /* CONFIG_SCHED_TIMESHARE_CORE */

/*
 *	processor_setrun:
 *
 *	Dispatch a thread for execution on a
 *	processor.
 *
 *	Thread must be locked.  Associated pset must
 *	be locked, and is returned unlocked.
 */
static void
processor_setrun(
	processor_t                     processor,
	thread_t                        thread,
	sched_options_t                 options)
{
	processor_set_t pset = processor->processor_set;
	pset_assert_locked(pset);
	ast_t preempt = AST_NONE;
	enum { eExitIdle, eInterruptRunning, eDoNothing } ipi_action = eDoNothing;

	sched_ipi_type_t ipi_type = SCHED_IPI_NONE;

	thread->chosen_processor = processor;

	/*
	 *	Set preemption mode.
	 */
#if defined(CONFIG_SCHED_DEFERRED_AST)
	/* TODO: Do we need to care about urgency (see rdar://problem/20136239)? */
#endif
	if (SCHED(priority_is_urgent)(thread->sched_pri) && thread->sched_pri > processor->current_pri) {
		preempt = (AST_PREEMPT | AST_URGENT);
	} else if (processor->current_is_eagerpreempt) {
		preempt = (AST_PREEMPT | AST_URGENT);
	} else if ((thread->sched_mode == TH_MODE_TIMESHARE) && (thread->sched_pri < thread->base_pri)) {
		if (SCHED(priority_is_urgent)(thread->base_pri) && thread->sched_pri > processor->current_pri) {
			preempt = (options & SCHED_PREEMPT)? AST_PREEMPT: AST_NONE;
		} else {
			preempt = AST_NONE;
		}
	} else {
		preempt = (options & SCHED_PREEMPT)? AST_PREEMPT: AST_NONE;
	}

	if ((options & SCHED_STIR_POT) ||
	    ((options & (SCHED_PREEMPT | SCHED_REBALANCE)) == (SCHED_PREEMPT | SCHED_REBALANCE))) {
		/*
		 * Having gone to the trouble of forcing this thread off a less preferred core,
		 * we should force the preferable core to reschedule immediately to give this
		 * thread a chance to run instead of just sitting on the run queue where
		 * it may just be stolen back by the idle core we just forced it off.
		 */
		preempt |= AST_PREEMPT;
	}

	SCHED(processor_enqueue)(processor, thread, options);
	SCHED(update_pset_load_average)(pset, 0);

	if (preempt != AST_NONE) {
		if (processor->state == PROCESSOR_IDLE) {
			ipi_action = eExitIdle;
		} else if (processor->state == PROCESSOR_DISPATCHING) {
			if (bit_set_if_clear(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
				KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PENDING_AST_URGENT) | DBG_FUNC_START,
				    processor->cpu_id, pset->pending_AST_URGENT_cpu_mask, (uintptr_t)thread_tid(thread), 5);
			}
		} else if (processor->state == PROCESSOR_RUNNING &&
		    (thread->sched_pri >= processor->current_pri)) {
			ipi_action = eInterruptRunning;
		}
	} else {
		/*
		 * New thread is not important enough to preempt what is running, but
		 * special processor states may need special handling
		 */
		if (processor->state == PROCESSOR_IDLE) {
			ipi_action = eExitIdle;
		} else if (processor->state == PROCESSOR_DISPATCHING) {
			if (bit_set_if_clear(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
				KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PENDING_AST_URGENT) | DBG_FUNC_START,
				    processor->cpu_id, pset->pending_AST_URGENT_cpu_mask, (uintptr_t)thread_tid(thread), 6);
			}
		}
	}

	if (ipi_action != eDoNothing) {
		if (processor == current_processor()) {
			if (ipi_action == eExitIdle) {
				pset_update_processor_state(pset, processor, PROCESSOR_DISPATCHING);
			}
			if ((preempt = csw_check_locked(processor->active_thread, processor, pset, AST_NONE)) != AST_NONE) {
				ast_on(preempt);
			}

			if ((preempt & AST_URGENT) == AST_URGENT) {
				if (bit_set_if_clear(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
					KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PENDING_AST_URGENT) | DBG_FUNC_START,
					    processor->cpu_id, pset->pending_AST_URGENT_cpu_mask, (uintptr_t)thread_tid(thread), 7);
				}
			} else {
				if (bit_clear_if_set(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
					KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PENDING_AST_URGENT) | DBG_FUNC_END, processor->cpu_id, pset->pending_AST_URGENT_cpu_mask, 0, 7);
				}
			}

			if ((preempt & AST_PREEMPT) == AST_PREEMPT) {
				bit_set(pset->pending_AST_PREEMPT_cpu_mask, processor->cpu_id);
			} else {
				bit_clear(pset->pending_AST_PREEMPT_cpu_mask, processor->cpu_id);
			}
		} else {
			sched_ipi_event_t event = (options & SCHED_REBALANCE) ? SCHED_IPI_EVENT_REBALANCE : SCHED_IPI_EVENT_PREEMPT;
			ipi_type = sched_ipi_action(processor, thread, event);
		}
	}

	pset_unlock(pset);
	sched_ipi_perform(processor, ipi_type);

	if (ipi_action != eDoNothing && processor == current_processor()) {
		ast_t new_preempt = update_pending_nonurgent_preemption(processor, preempt);
		ast_on(new_preempt);
	}
}

/*
 *	choose_next_pset:
 *
 *	Return the next sibling pset containing
 *	available processors.
 *
 *	Returns the original pset if none other is
 *	suitable.
 */
static processor_set_t
choose_next_pset(
	processor_set_t         pset)
{
	processor_set_t         nset = pset;

	do {
		nset = next_pset(nset);

		/*
		 * Sometimes during startup the pset_map can contain a bit
		 * for a pset that isn't fully published in pset_array because
		 * the pset_map read isn't an acquire load.
		 *
		 * In order to avoid needing an acquire barrier here, just bail
		 * out.
		 */
		if (nset == PROCESSOR_SET_NULL) {
			return pset;
		}
	} while (nset->online_processor_count < 1 && nset != pset);

	return nset;
}

#if CONFIG_SCHED_SMT
/*
 *	choose_processor_smt:
 *
 *  SMT-aware implementation of choose_processor.
 */
processor_t
choose_processor_smt(
	processor_set_t         starting_pset,
	processor_t             processor,
	thread_t                thread,
	__unused sched_options_t *options)
{
	processor_set_t pset = starting_pset;
	processor_set_t nset;

	assert(thread->sched_pri <= MAXPRI);

	/*
	 * Prefer the hinted processor, when appropriate.
	 */

	/* Fold last processor hint from secondary processor to its primary */
	if (processor != PROCESSOR_NULL) {
		processor = processor->processor_primary;
	}

	/*
	 * Only consult platform layer if pset is active, which
	 * it may not be in some cases when a multi-set system
	 * is going to sleep.
	 */
	if (pset->online_processor_count) {
		if ((processor == PROCESSOR_NULL) || (processor->processor_set == pset && processor->state == PROCESSOR_IDLE)) {
			processor_t mc_processor = machine_choose_processor(pset, processor);
			if (mc_processor != PROCESSOR_NULL) {
				processor = mc_processor->processor_primary;
			}
		}
	}

	/*
	 * At this point, we may have a processor hint, and we may have
	 * an initial starting pset. If the hint is not in the pset, or
	 * if the hint is for a processor in an invalid state, discard
	 * the hint.
	 */
	if (processor != PROCESSOR_NULL) {
		if (processor->processor_set != pset) {
			processor = PROCESSOR_NULL;
		} else if (!processor->is_recommended) {
			processor = PROCESSOR_NULL;
		} else {
			switch (processor->state) {
			case PROCESSOR_START:
			case PROCESSOR_PENDING_OFFLINE:
			case PROCESSOR_OFF_LINE:
				/*
				 * Hint is for a processor that cannot support running new threads.
				 */
				processor = PROCESSOR_NULL;
				break;
			case PROCESSOR_IDLE:
				/*
				 * Hint is for an idle processor. Assume it is no worse than any other
				 * idle processor. The platform layer had an opportunity to provide
				 * the "least cost idle" processor above.
				 */
				if ((thread->sched_pri < BASEPRI_RTQUEUES) || processor_is_fast_track_candidate_for_realtime_thread(pset, processor)) {
					uint64_t idle_primary_map = (pset->cpu_state_map[PROCESSOR_IDLE] & pset->primary_map & pset->recommended_bitmask);
					uint64_t non_avoided_idle_primary_map = idle_primary_map & ~pset->perfcontrol_cpu_migration_bitmask;
					/*
					 * If the rotation bitmask to force a migration is set for this core and there's an idle core that
					 * that needn't be avoided, don't continue running on the same core.
					 */
					if (!(bit_test(processor->processor_set->perfcontrol_cpu_migration_bitmask, processor->cpu_id) && non_avoided_idle_primary_map != 0)) {
						return processor;
					}
				}
				processor = PROCESSOR_NULL;
				break;
			case PROCESSOR_RUNNING:
			case PROCESSOR_DISPATCHING:
				/*
				 * Hint is for an active CPU. This fast-path allows
				 * realtime threads to preempt non-realtime threads
				 * to regain their previous executing processor.
				 */
				if (thread->sched_pri >= BASEPRI_RTQUEUES) {
					if (processor_is_fast_track_candidate_for_realtime_thread(pset, processor)) {
						return processor;
					}
					processor = PROCESSOR_NULL;
				}

				/* Otherwise, use hint as part of search below */
				break;
			default:
				processor = PROCESSOR_NULL;
				break;
			}
		}
	}

	/*
	 * Iterate through the processor sets to locate
	 * an appropriate processor. Seed results with
	 * a last-processor hint, if available, so that
	 * a search must find something strictly better
	 * to replace it.
	 *
	 * A primary/secondary pair of SMT processors are
	 * "unpaired" if the primary is busy but its
	 * corresponding secondary is idle (so the physical
	 * core has full use of its resources).
	 */

	assert(pset == starting_pset);
	if (thread->sched_pri >= BASEPRI_RTQUEUES) {
		return SCHED(rt_choose_processor)(pset, processor, thread);
	}

	/* No realtime threads from this point on */
	assert(thread->sched_pri < BASEPRI_RTQUEUES);

	integer_t lowest_priority = MAXPRI + 1;
	integer_t lowest_secondary_priority = MAXPRI + 1;
	integer_t lowest_unpaired_primary_priority = MAXPRI + 1;
	integer_t lowest_idle_secondary_priority = MAXPRI + 1;
	integer_t lowest_count = INT_MAX;
	processor_t lp_processor = PROCESSOR_NULL;
	processor_t lp_unpaired_primary_processor = PROCESSOR_NULL;
	processor_t lp_idle_secondary_processor = PROCESSOR_NULL;
	processor_t lp_paired_secondary_processor = PROCESSOR_NULL;
	processor_t lc_processor = PROCESSOR_NULL;

	if (processor != PROCESSOR_NULL) {
		/* All other states should be enumerated above. */
		assert(processor->state == PROCESSOR_RUNNING || processor->state == PROCESSOR_DISPATCHING);
		assert(thread->sched_pri < BASEPRI_RTQUEUES);

		lowest_priority = processor->current_pri;
		lp_processor = processor;

		lowest_count = SCHED(processor_runq_count)(processor);
		lc_processor = processor;
	}

	do {
		/*
		 * Choose an idle processor, in pset traversal order
		 */
		uint64_t idle_primary_map = (pset->cpu_state_map[PROCESSOR_IDLE] & pset->primary_map & pset->recommended_bitmask);
		uint64_t preferred_idle_primary_map = idle_primary_map & pset->perfcontrol_cpu_preferred_bitmask;

		/* there shouldn't be a pending AST if the processor is idle */
		assert((idle_primary_map & pset->pending_AST_URGENT_cpu_mask) == 0);

		/*
		 * Look at the preferred cores first.
		 */
		int cpuid = lsb_next(preferred_idle_primary_map, pset->cpu_preferred_last_chosen);
		if (cpuid < 0) {
			cpuid = lsb_first(preferred_idle_primary_map);
		}
		if (cpuid >= 0) {
			processor = processor_array[cpuid];
			pset->cpu_preferred_last_chosen = cpuid;
			return processor;
		}

		/*
		 * Look at the cores that don't need to be avoided next.
		 */
		if (pset->perfcontrol_cpu_migration_bitmask != 0) {
			uint64_t non_avoided_idle_primary_map = idle_primary_map & ~pset->perfcontrol_cpu_migration_bitmask;
			cpuid = lsb_next(non_avoided_idle_primary_map, pset->cpu_preferred_last_chosen);
			if (cpuid < 0) {
				cpuid = lsb_first(non_avoided_idle_primary_map);
			}
			if (cpuid >= 0) {
				processor = processor_array[cpuid];
				pset->cpu_preferred_last_chosen = cpuid;
				return processor;
			}
		}

		/*
		 * Fall back to any remaining idle cores if none of the preferred ones and non-avoided ones are available.
		 */
		cpuid = lsb_first(idle_primary_map);
		if (cpuid >= 0) {
			processor = processor_array[cpuid];
			return processor;
		}

		/*
		 * Otherwise, enumerate active and idle processors to find primary candidates
		 * with lower priority/etc.
		 */

		uint64_t active_map = ((pset->cpu_state_map[PROCESSOR_RUNNING] | pset->cpu_state_map[PROCESSOR_DISPATCHING]) &
		    pset->recommended_bitmask &
		    ~pset->pending_AST_URGENT_cpu_mask);

		if (SCHED(priority_is_urgent)(thread->sched_pri) == FALSE) {
			active_map &= ~pset->pending_AST_PREEMPT_cpu_mask;
		}

		active_map = bit_ror64(active_map, (pset->last_chosen + 1));
		for (int rotid = lsb_first(active_map); rotid >= 0; rotid = lsb_next(active_map, rotid)) {
			cpuid = ((rotid + pset->last_chosen + 1) & 63);
			processor = processor_array[cpuid];

			integer_t cpri = processor->current_pri;
			processor_t primary = processor->processor_primary;
			if (primary != processor) {
				/* If primary is running a NO_SMT thread, don't choose its secondary */
				if (!((primary->state == PROCESSOR_RUNNING) && processor_active_thread_no_smt(primary))) {
					if (cpri < lowest_secondary_priority) {
						lowest_secondary_priority = cpri;
						lp_paired_secondary_processor = processor;
					}
				}
			} else {
				if (cpri < lowest_priority) {
					lowest_priority = cpri;
					lp_processor = processor;
				}
			}

			integer_t ccount = SCHED(processor_runq_count)(processor);
			if (ccount < lowest_count) {
				lowest_count = ccount;
				lc_processor = processor;
			}
		}

		/*
		 * For SMT configs, these idle secondary processors must have active primary. Otherwise
		 * the idle primary would have short-circuited the loop above
		 */
		uint64_t idle_secondary_map = (pset->cpu_state_map[PROCESSOR_IDLE] &
		    ~pset->primary_map &
		    pset->recommended_bitmask);

		/* there shouldn't be a pending AST if the processor is idle */
		assert((idle_secondary_map & pset->pending_AST_URGENT_cpu_mask) == 0);
		assert((idle_secondary_map & pset->pending_AST_PREEMPT_cpu_mask) == 0);

		for (cpuid = lsb_first(idle_secondary_map); cpuid >= 0; cpuid = lsb_next(idle_secondary_map, cpuid)) {
			processor = processor_array[cpuid];

			processor_t cprimary = processor->processor_primary;

			integer_t primary_pri = cprimary->current_pri;

			/*
			 * TODO: This should also make the same decisions
			 * as secondary_can_run_realtime_thread
			 *
			 * TODO: Keep track of the pending preemption priority
			 * of the primary to make this more accurate.
			 */

			/* If the primary is running a no-smt thread, then don't choose its secondary */
			if (cprimary->state == PROCESSOR_RUNNING &&
			    processor_active_thread_no_smt(cprimary)) {
				continue;
			}

			/*
			 * Find the idle secondary processor with the lowest priority primary
			 *
			 * We will choose this processor as a fallback if we find no better
			 * primary to preempt.
			 */
			if (primary_pri < lowest_idle_secondary_priority) {
				lp_idle_secondary_processor = processor;
				lowest_idle_secondary_priority = primary_pri;
			}

			/* Find the the lowest priority active primary with idle secondary */
			if (primary_pri < lowest_unpaired_primary_priority) {
				/* If the primary processor is offline or starting up, it's not a candidate for this path */
				if (cprimary->state != PROCESSOR_RUNNING &&
				    cprimary->state != PROCESSOR_DISPATCHING) {
					continue;
				}

				if (!cprimary->is_recommended) {
					continue;
				}

				/* if the primary is pending preemption, don't try to re-preempt it */
				if (bit_test(pset->pending_AST_URGENT_cpu_mask, cprimary->cpu_id)) {
					continue;
				}

				if (SCHED(priority_is_urgent)(thread->sched_pri) == FALSE &&
				    bit_test(pset->pending_AST_PREEMPT_cpu_mask, cprimary->cpu_id)) {
					continue;
				}

				lowest_unpaired_primary_priority = primary_pri;
				lp_unpaired_primary_processor = cprimary;
			}
		}

		/*
		 * We prefer preempting a primary processor over waking up its secondary.
		 * The secondary will then be woken up by the preempted thread.
		 */
		if (thread->sched_pri > lowest_unpaired_primary_priority) {
			pset->last_chosen = lp_unpaired_primary_processor->cpu_id;
			return lp_unpaired_primary_processor;
		}

		/*
		 * We prefer preempting a lower priority active processor over directly
		 * waking up an idle secondary.
		 * The preempted thread will then find the idle secondary.
		 */
		if (thread->sched_pri > lowest_priority) {
			pset->last_chosen = lp_processor->cpu_id;
			return lp_processor;
		}

		/*
		 * lc_processor is used to indicate the best processor set run queue
		 * on which to enqueue a thread when all available CPUs are busy with
		 * higher priority threads, so try to make sure it is initialized.
		 */
		if (lc_processor == PROCESSOR_NULL) {
			cpumap_t available_map = pset_available_cpumap(pset);
			cpuid = lsb_first(available_map);
			if (cpuid >= 0) {
				lc_processor = processor_array[cpuid];
				lowest_count = SCHED(processor_runq_count)(lc_processor);
			}
		}

		/*
		 * Move onto the next processor set.
		 *
		 * If all primary processors in this pset are running a higher
		 * priority thread, move on to next pset. Only when we have
		 * exhausted the search for primary processors do we
		 * fall back to secondaries.
		 */
#if CONFIG_SCHED_EDGE
		/*
		 * The edge scheduler expects a CPU to be selected from the pset it passed in
		 * as the starting pset for non-RT workloads. The edge migration algorithm
		 * should already have considered idle CPUs and loads to decide the starting_pset;
		 * which means that this loop can be short-circuted.
		 */
		nset = starting_pset;
#else /* CONFIG_SCHED_EDGE */
		nset = next_pset(pset);
#endif /* CONFIG_SCHED_EDGE */

		if (nset != starting_pset) {
			pset = change_locked_pset(pset, nset);
		}
	} while (nset != starting_pset);

	/*
	 * Make sure that we pick a running processor,
	 * and that the correct processor set is locked.
	 * Since we may have unlocked the candidate processor's
	 * pset, it may have changed state.
	 *
	 * All primary processors are running a higher priority
	 * thread, so the only options left are enqueuing on
	 * the secondary processor that would perturb the least priority
	 * primary, or the least busy primary.
	 */

	/* lowest_priority is evaluated in the main loops above */
	if (lp_idle_secondary_processor != PROCESSOR_NULL) {
		processor = lp_idle_secondary_processor;
	} else if (lp_paired_secondary_processor != PROCESSOR_NULL) {
		processor = lp_paired_secondary_processor;
	} else if (lc_processor != PROCESSOR_NULL) {
		processor = lc_processor;
	} else {
		processor = PROCESSOR_NULL;
	}

	if (processor) {
		pset = change_locked_pset(pset, processor->processor_set);
		/* Check that chosen processor is still usable */
		cpumap_t available_map = pset_available_cpumap(pset);
		if (bit_test(available_map, processor->cpu_id)) {
			pset->last_chosen = processor->cpu_id;
			return processor;
		}

		/* processor is no longer usable */
		processor = PROCESSOR_NULL;
	}

	pset_assert_locked(pset);
	pset_unlock(pset);
	return PROCESSOR_NULL;
}
#else /* !CONFIG_SCHED_SMT */
/*
 *	choose_processor:
 *
 *	Choose a processor for the thread, beginning at
 *	the pset.  Accepts an optional processor hint in
 *	the pset.
 *
 *	Returns a processor, possibly from a different pset.
 *
 *	The thread must be locked.  The pset must be locked,
 *	and the resulting pset is locked on return.
 */
processor_t
choose_processor(
	processor_set_t         starting_pset,
	processor_t             processor,
	thread_t                thread,
	__unused sched_options_t *options)
{
	processor_set_t pset = starting_pset;
	processor_set_t nset;

	assert3u(thread->sched_pri, <=, MAXPRI);

	/*
	 * At this point, we may have a processor hint, and we may have
	 * an initial starting pset. If the hint is not in the pset, or
	 * if the hint is for a processor in an invalid state, discard
	 * the hint.
	 */
	if (processor != PROCESSOR_NULL) {
		if (processor->processor_set != pset) {
			processor = PROCESSOR_NULL;
		} else if (!processor->is_recommended) {
			processor = PROCESSOR_NULL;
		} else {
			switch (processor->state) {
			case PROCESSOR_START:
			case PROCESSOR_PENDING_OFFLINE:
			case PROCESSOR_OFF_LINE:
				/*
				 * Hint is for a processor that cannot support running new threads.
				 */
				processor = PROCESSOR_NULL;
				break;
			case PROCESSOR_IDLE:
				/*
				 * Hint is for an idle processor. Assume it is no worse than any other
				 * idle processor. The platform layer had an opportunity to provide
				 * the "least cost idle" processor above.
				 */
				if ((thread->sched_pri < BASEPRI_RTQUEUES) || processor_is_fast_track_candidate_for_realtime_thread(pset, processor)) {
					uint64_t idle_map = (pset->cpu_state_map[PROCESSOR_IDLE] & pset->recommended_bitmask);
					uint64_t non_avoided_idle_map = idle_map & ~pset->perfcontrol_cpu_migration_bitmask;
					/*
					 * If the rotation bitmask to force a migration is set for this core and there's an idle core that
					 * that needn't be avoided, don't continue running on the same core.
					 */
					if (!(bit_test(processor->processor_set->perfcontrol_cpu_migration_bitmask, processor->cpu_id) && non_avoided_idle_map != 0)) {
						return processor;
					}
				}
				processor = PROCESSOR_NULL;
				break;
			case PROCESSOR_RUNNING:
			case PROCESSOR_DISPATCHING:
				/*
				 * Hint is for an active CPU. This fast-path allows
				 * realtime threads to preempt non-realtime threads
				 * to regain their previous executing processor.
				 */
				if (thread->sched_pri >= BASEPRI_RTQUEUES) {
					if (processor_is_fast_track_candidate_for_realtime_thread(pset, processor)) {
						return processor;
					}
					processor = PROCESSOR_NULL;
				}

				/* Otherwise, use hint as part of search below */
				break;
			default:
				processor = PROCESSOR_NULL;
				break;
			}
		}
	}

	/*
	 * Iterate through the processor sets to locate
	 * an appropriate processor. Seed results with
	 * a last-processor hint, if available, so that
	 * a search must find something strictly better
	 * to replace it.
	 */

	assert(pset == starting_pset);
	if (thread->sched_pri >= BASEPRI_RTQUEUES) {
		return SCHED(rt_choose_processor)(pset, processor, thread);
	}

	/* No realtime threads from this point on */
	assert(thread->sched_pri < BASEPRI_RTQUEUES);

	integer_t lowest_priority = MAXPRI + 1;
	integer_t lowest_count = INT_MAX;
	processor_t lp_processor = PROCESSOR_NULL;
	processor_t lc_processor = PROCESSOR_NULL;

	if (processor != PROCESSOR_NULL) {
		/* All other states should be enumerated above. */
		assert(processor->state == PROCESSOR_RUNNING || processor->state == PROCESSOR_DISPATCHING);
		assert(thread->sched_pri < BASEPRI_RTQUEUES);

		lowest_priority = processor->current_pri;
		lp_processor = processor;

		lowest_count = SCHED(processor_runq_count)(processor);
		lc_processor = processor;
	}


	do {
		/*
		 * Choose an idle processor, in pset traversal order
		 */
		uint64_t idle_map = (pset->cpu_state_map[PROCESSOR_IDLE] & pset->recommended_bitmask);
		uint64_t preferred_idle_map = idle_map & pset->perfcontrol_cpu_preferred_bitmask;

		/* there shouldn't be a pending AST if the processor is idle */
		assert((idle_map & pset->pending_AST_URGENT_cpu_mask) == 0);

		/*
		 * Look at the preferred cores first.
		 */
		int cpuid = lsb_next(preferred_idle_map, pset->cpu_preferred_last_chosen);
		if (cpuid < 0) {
			cpuid = lsb_first(preferred_idle_map);
		}
		if (cpuid >= 0) {
			processor = processor_array[cpuid];
			pset->cpu_preferred_last_chosen = cpuid;
			return processor;
		}

		/*
		 * Look at the cores that don't need to be avoided next.
		 */
		if (pset->perfcontrol_cpu_migration_bitmask != 0) {
			uint64_t non_avoided_idle_map = idle_map & ~pset->perfcontrol_cpu_migration_bitmask;
			cpuid = lsb_next(non_avoided_idle_map, pset->cpu_preferred_last_chosen);
			if (cpuid < 0) {
				cpuid = lsb_first(non_avoided_idle_map);
			}
			if (cpuid >= 0) {
				processor = processor_array[cpuid];
				pset->cpu_preferred_last_chosen = cpuid;
				return processor;
			}
		}

		/*
		 * Fall back to any remaining idle cores if none of the preferred ones and non-avoided ones are available.
		 */
		cpuid = lsb_first(idle_map);
		if (cpuid >= 0) {
			processor = processor_array[cpuid];
			return processor;
		}

		/*
		 * Otherwise, enumerate active and idle processors to find primary candidates
		 * with lower priority/etc.
		 */

		uint64_t active_map = ((pset->cpu_state_map[PROCESSOR_RUNNING] | pset->cpu_state_map[PROCESSOR_DISPATCHING]) &
		    pset->recommended_bitmask &
		    ~pset->pending_AST_URGENT_cpu_mask);

		if (SCHED(priority_is_urgent)(thread->sched_pri) == FALSE) {
			active_map &= ~pset->pending_AST_PREEMPT_cpu_mask;
		}

		active_map = bit_ror64(active_map, (pset->last_chosen + 1));
		for (int rotid = lsb_first(active_map); rotid >= 0; rotid = lsb_next(active_map, rotid)) {
			cpuid = ((rotid + pset->last_chosen + 1) & 63);
			processor = processor_array[cpuid];

			integer_t cpri = processor->current_pri;
			if (cpri < lowest_priority) {
				lowest_priority = cpri;
				lp_processor = processor;
			}

			integer_t ccount = SCHED(processor_runq_count)(processor);
			if (ccount < lowest_count) {
				lowest_count = ccount;
				lc_processor = processor;
			}
		}

		/*
		 * We prefer preempting a lower priority active processor over directly
		 * waking up an idle secondary.
		 * The preempted thread will then find the idle secondary.
		 */
		if (thread->sched_pri > lowest_priority) {
			pset->last_chosen = lp_processor->cpu_id;
			return lp_processor;
		}

		/*
		 * lc_processor is used to indicate the best processor set run queue
		 * on which to enqueue a thread when all available CPUs are busy with
		 * higher priority threads, so try to make sure it is initialized.
		 */
		if (lc_processor == PROCESSOR_NULL) {
			cpumap_t available_map = pset_available_cpumap(pset);
			cpuid = lsb_first(available_map);
			if (cpuid >= 0) {
				lc_processor = processor_array[cpuid];
				lowest_count = SCHED(processor_runq_count)(lc_processor);
			}
		}

		/*
		 * Move onto the next processor set.
		 *
		 * If all primary processors in this pset are running a higher
		 * priority thread, move on to next pset. Only when we have
		 * exhausted the search for primary processors do we
		 * fall back to secondaries.
		 */
#if CONFIG_SCHED_EDGE
		/*
		 * The edge scheduler expects a CPU to be selected from the pset it passed in
		 * as the starting pset for non-RT workloads. The edge migration algorithm
		 * should already have considered idle CPUs and loads to decide the starting_pset;
		 * which means that this loop can be short-circuted.
		 */
		nset = starting_pset;
#else /* CONFIG_SCHED_EDGE */
		nset = next_pset(pset);
#endif /* CONFIG_SCHED_EDGE */

		if (nset != starting_pset) {
			pset = change_locked_pset(pset, nset);
		}
	} while (nset != starting_pset);

	processor = lc_processor;

	if (processor) {
		pset = change_locked_pset(pset, processor->processor_set);
		/* Check that chosen processor is still usable */
		cpumap_t available_map = pset_available_cpumap(pset);
		if (bit_test(available_map, processor->cpu_id)) {
			pset->last_chosen = processor->cpu_id;
			return processor;
		}

		/* processor is no longer usable */
		processor = PROCESSOR_NULL;
	}

	pset_assert_locked(pset);
	pset_unlock(pset);
	return PROCESSOR_NULL;
}
#endif /* !CONFIG_SCHED_SMT */



/*
 * Default implementation of SCHED(choose_node)()
 * for single node systems
 */
pset_node_t
sched_choose_node(__unused thread_t thread)
{
	return &pset_node0;
}

/*
 *	choose_starting_pset:
 *
 *	Choose a starting processor set for the thread.
 *	May return a processor hint within the pset.
 *
 *	Returns a starting processor set, to be used by
 *      choose_processor.
 *
 *	The thread must be locked.  The resulting pset is unlocked on return,
 *      and is chosen without taking any pset locks.
 */
processor_set_t
choose_starting_pset(pset_node_t node, thread_t thread, processor_t *processor_hint)
{
	processor_set_t pset;
	processor_t processor = PROCESSOR_NULL;

	if (thread->affinity_set != AFFINITY_SET_NULL) {
		/*
		 * Use affinity set policy hint.
		 */
		pset = thread->affinity_set->aset_pset;
	} else if (thread->last_processor != PROCESSOR_NULL) {
		/*
		 *	Simple (last processor) affinity case.
		 */
		processor = thread->last_processor;
		pset = processor->processor_set;
	} else {
		/*
		 *	No Affinity case:
		 *
		 *	Utilitize a per task hint to spread threads
		 *	among the available processor sets.
		 * NRG this seems like the wrong thing to do.
		 * See also task->pset_hint = pset in thread_setrun()
		 */
		pset = get_threadtask(thread)->pset_hint;
		if (pset == PROCESSOR_SET_NULL) {
			pset = current_processor()->processor_set;
		}

		pset = choose_next_pset(pset);
	}

	if (!bit_test(node->pset_map, pset->pset_id)) {
		/* pset is not from this node so choose one that is */
		pset = pset_for_id((pset_id_t)lsb_first(node->pset_map));
	}

	if (bit_count(node->pset_map) == 1) {
		/* Only a single pset in this node */
		goto out;
	}

	bool avoid_cpu0 = false;

#if defined(__x86_64__)
	if ((thread->sched_pri >= BASEPRI_RTQUEUES) && sched_avoid_cpu0) {
		/* Avoid the pset containing cpu0 */
		avoid_cpu0 = true;
		/* Assert that cpu0 is in pset 0.  I expect this to be true on __x86_64__ */
		assert(bit_test(pset_for_id(0)->cpu_bitmask, 0));
	}
#endif

	if (thread->sched_pri >= BASEPRI_RTQUEUES) {
		pset_map_t rt_target_map;
#if CONFIG_SCHED_SMT
		rt_target_map = atomic_load(&node->pset_non_rt_primary_map);
		if ((avoid_cpu0 && pset->pset_id == 0) || !bit_test(rt_target_map, pset->pset_id)) {
			if (avoid_cpu0) {
				rt_target_map = bit_ror64(rt_target_map, 1);
			}
			int rotid = lsb_first(rt_target_map);
			if (rotid >= 0) {
				int id = avoid_cpu0 ? ((rotid + 1) & 63) : rotid;
				pset = pset_array[id];
				goto out;
			}
		}
		if (!pset->is_SMT || !sched_allow_rt_smt) {
			/* All psets are full of RT threads - fall back to choose processor to find the furthest deadline RT thread */
			goto out;
		}
#endif /* CONFIG_SCHED_SMT*/
		rt_target_map = atomic_load(&node->pset_non_rt_map);
		if ((avoid_cpu0 && pset->pset_id == 0) || !bit_test(rt_target_map, pset->pset_id)) {
			if (avoid_cpu0) {
				rt_target_map = bit_ror64(rt_target_map, 1);
			}
			int rotid = lsb_first(rt_target_map);
			if (rotid >= 0) {
				int id = avoid_cpu0 ? ((rotid + 1) & 63) : rotid;
				pset = pset_array[id];
				goto out;
			}
		}
		/* All psets are full of RT threads - fall back to choose processor to find the furthest deadline RT thread */
	} else {
		pset_map_t idle_map = atomic_load(&node->pset_idle_map);
		if (!bit_test(idle_map, pset->pset_id)) {
			int next_idle_pset_id = lsb_first(idle_map);
			if (next_idle_pset_id >= 0) {
				pset = pset_array[next_idle_pset_id];
			}
		}
	}

out:
	if ((processor != PROCESSOR_NULL) && (processor->processor_set != pset)) {
		processor = PROCESSOR_NULL;
	}
	if (processor != PROCESSOR_NULL) {
		*processor_hint = processor;
	}

	assert(pset != NULL);
	return pset;
}

/*
 *	thread_setrun:
 *
 *	Dispatch thread for execution, onto an idle
 *	processor or run queue, and signal a preemption
 *	as appropriate.
 *
 *	Thread must be locked.
 */
void
thread_setrun(
	thread_t                        thread,
	sched_options_t                 options)
{
	processor_t                     processor = PROCESSOR_NULL;
	processor_set_t         pset;

	assert((thread->state & (TH_RUN | TH_WAIT | TH_UNINT | TH_TERMINATE | TH_TERMINATE2)) == TH_RUN);
	thread_assert_runq_null(thread);

	simple_lock_assert(&sched_available_cores_lock, LCK_ASSERT_NOTOWNED);

#if CONFIG_PREADOPT_TG
	/* We know that the thread is not in the runq by virtue of being in this
	 * function and the thread is not self since we are running. We can safely
	 * resolve the thread group hierarchy and modify the thread's thread group
	 * here. */
	thread_resolve_and_enforce_thread_group_hierarchy_if_needed(thread);
#endif

	/*
	 *	Update priority if needed.
	 */
	if (SCHED(can_update_priority)(thread)) {
		SCHED(update_priority)(thread);
	}
	thread->sfi_class = sfi_thread_classify(thread);

	if (thread->bound_processor == PROCESSOR_NULL) {
		/*
		 * Unbound case.
		 *
		 * Usually, this loop will only be executed once,
		 * but if CLPC derecommends a processor after it has been chosen,
		 * or if a processor is shut down after it is chosen,
		 * choose_processor() may return NULL, so a retry
		 * may be necessary.  A single retry will usually
		 * be enough, and we can't afford to retry too many times
		 * because interrupts are disabled.
		 */
#define CHOOSE_PROCESSOR_MAX_RETRIES 3
		for (int retry = 0; retry <= CHOOSE_PROCESSOR_MAX_RETRIES; retry++) {
			processor_t processor_hint = PROCESSOR_NULL;
			pset_node_t node = SCHED(choose_node)(thread);
			processor_set_t starting_pset = choose_starting_pset(node, thread, &processor_hint);

			pset_lock(starting_pset);

			processor = SCHED(choose_processor)(starting_pset, processor_hint, thread, &options);
			if (processor != PROCESSOR_NULL) {
				pset = processor->processor_set;
				pset_assert_locked(pset);
				break;
			}
		}
		/*
		 * If choose_processor() still returns NULL,
		 * which is very unlikely, we need a fallback.
		 */
		if (processor == PROCESSOR_NULL) {
			bool unlock_available_cores_lock = false;
			if (sched_all_cpus_offline()) {
				/*
				 * There are no available processors
				 * because we're in final system shutdown.
				 * Enqueue on the master processor and we'll
				 * handle it when it powers back up.
				 */
				processor = master_processor;
			} else if (support_bootcpu_shutdown) {
				/*
				 * Grab the sched_available_cores_lock to select
				 * some available processor and prevent it from
				 * becoming offline while we enqueue the thread.
				 */
				simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);
				unlock_available_cores_lock = true;

				int last_resort_cpu = sched_last_resort_cpu();

				processor = processor_array[last_resort_cpu];
			} else {
				/*
				 * The master processor is never shut down, always safe to choose.
				 */
				processor = master_processor;
			}
			pset = processor->processor_set;
			pset_lock(pset);
			assert((pset_available_cpu_count(pset) > 0) || (processor->state != PROCESSOR_OFF_LINE && processor->is_recommended));
			if (unlock_available_cores_lock) {
				simple_unlock(&sched_available_cores_lock);
			}
		}
		task_t task = get_threadtask(thread);
		if (!(task->t_flags & TF_USE_PSET_HINT_CLUSTER_TYPE)) {
			task->pset_hint = pset; /* NRG this is done without holding the task lock */
		}
		SCHED_DEBUG_CHOOSE_PROCESSOR_KERNEL_DEBUG_CONSTANT_IST(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_CHOOSE_PROCESSOR) | DBG_FUNC_NONE,
		    (uintptr_t)thread_tid(thread), (uintptr_t)-1, processor->cpu_id, processor->state, 0);
		assert((pset_available_cpu_count(pset) > 0) || (processor->state != PROCESSOR_OFF_LINE && processor->is_recommended));
	} else {
		/*
		 *	Bound case:
		 *
		 *	Unconditionally dispatch on the processor.
		 */
		processor = thread->bound_processor;
		pset = processor->processor_set;
		pset_lock(pset);

		SCHED_DEBUG_CHOOSE_PROCESSOR_KERNEL_DEBUG_CONSTANT_IST(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_CHOOSE_PROCESSOR) | DBG_FUNC_NONE,
		    (uintptr_t)thread_tid(thread), (uintptr_t)-2, processor->cpu_id, processor->state, 0);
	}

	/*
	 *	Dispatch the thread on the chosen processor.
	 *	TODO: This should be based on sched_mode, not sched_pri
	 */
	if (thread->sched_pri >= BASEPRI_RTQUEUES) {
		realtime_setrun(processor, thread);
	} else {
		processor_setrun(processor, thread, options);
	}
	/* pset is now unlocked */
	if (thread->bound_processor == PROCESSOR_NULL) {
		SCHED(check_spill)(pset, thread);
	}
}

processor_set_t
task_choose_pset(
	task_t          task)
{
	processor_set_t         pset = task->pset_hint;

	if (pset != PROCESSOR_SET_NULL) {
		pset = choose_next_pset(pset);
	}

	return pset;
}

/*
 *	Check for a preemption point in
 *	the current context.
 *
 *	Called at splsched with thread locked.
 */
ast_t
csw_check(
	thread_t                thread,
	processor_t             processor,
	ast_t                   check_reason)
{
	processor_set_t pset = processor->processor_set;

	assert(thread == processor->active_thread);

	pset_lock(pset);

	processor_state_update_from_thread(processor, thread, true);

	ast_t preempt = csw_check_locked(thread, processor, pset, check_reason);

	/* Acknowledge the IPI if we decided not to preempt */

	if ((preempt & AST_URGENT) == 0) {
		if (bit_clear_if_set(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
			KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PENDING_AST_URGENT) | DBG_FUNC_END, processor->cpu_id, pset->pending_AST_URGENT_cpu_mask, 0, 8);
		}
	}

	if ((preempt & AST_PREEMPT) == 0) {
		bit_clear(pset->pending_AST_PREEMPT_cpu_mask, processor->cpu_id);
	}

	pset_unlock(pset);

	return update_pending_nonurgent_preemption(processor, preempt);
}

void
clear_pending_nonurgent_preemption(processor_t processor)
{
	if (!processor->pending_nonurgent_preemption) {
		return;
	}

	KDBG_RELEASE(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_PREEMPT_TIMER_ACTIVE) | DBG_FUNC_END);

	processor->pending_nonurgent_preemption = false;
	running_timer_clear(processor, RUNNING_TIMER_PREEMPT);
}

ast_t
update_pending_nonurgent_preemption(processor_t processor, ast_t reason)
{
	if ((reason & (AST_URGENT | AST_PREEMPT)) != (AST_PREEMPT)) {
		clear_pending_nonurgent_preemption(processor);
		return reason;
	}

	if (nonurgent_preemption_timer_abs == 0) {
		/* Preemption timer not enabled */
		return reason;
	}

	if (current_thread()->state & TH_IDLE) {
		/* idle threads don't need nonurgent preemption */
		return reason;
	}

	if (processor->pending_nonurgent_preemption) {
		/* Timer is already armed, no need to do it again */
		return reason;
	}

	if (ml_did_interrupt_userspace()) {
		/*
		 * We're preempting userspace here, so we don't need
		 * to defer the preemption.  Force AST_URGENT
		 * so that we can avoid arming this timer without risking
		 * ast_taken_user deciding to spend too long in kernel
		 * space to handle other ASTs.
		 */

		return reason | AST_URGENT;
	}

	/*
	 * We've decided to do a nonurgent preemption when running in
	 * kernelspace. We defer the preemption until reaching userspace boundary
	 * to give a grace period for locks etc to be dropped and to reach
	 * a clean preemption point, so that the preempting thread doesn't
	 * always immediately hit the lock that the waking thread still holds.
	 *
	 * Arm a timer to enforce that the preemption executes within a bounded
	 * time if the thread doesn't block or return to userspace quickly.
	 */

	processor->pending_nonurgent_preemption = true;
	KDBG_RELEASE(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_PREEMPT_TIMER_ACTIVE) | DBG_FUNC_START,
	    reason);

	uint64_t now = mach_absolute_time();

	uint64_t deadline = now + nonurgent_preemption_timer_abs;

	running_timer_enter(processor, RUNNING_TIMER_PREEMPT, NULL,
	    deadline, now);

	return reason;
}

/*
 * Check for preemption at splsched with
 * pset locked and processor as the current
 * processor.
 */
ast_t
csw_check_locked(
	thread_t                thread,
	processor_t             processor,
	processor_set_t         pset,
	ast_t                   check_reason)
{
	assert(processor == current_processor());
	/*
	 * If the current thread is running on a processor that is no longer recommended,
	 * urgently preempt it, at which point thread_select() should
	 * try to idle the processor and re-dispatch the thread to a recommended processor.
	 */
	if (!processor->is_recommended) {
		return check_reason | AST_PREEMPT | AST_URGENT;
	}

	if (bit_test(pset->rt_pending_spill_cpu_mask, processor->cpu_id)) {
		return check_reason | AST_PREEMPT | AST_URGENT;
	}

	if (rt_runq_count(pset) > 0) {
		if ((rt_runq_priority(pset) > processor->current_pri) || !processor->first_timeslice) {
			return check_reason | AST_PREEMPT | AST_URGENT;
		} else if (rt_deadline_add(rt_runq_earliest_deadline(pset), rt_deadline_epsilon) < processor->deadline) {
			return check_reason | AST_PREEMPT | AST_URGENT;
		} else {
			return check_reason | AST_PREEMPT;
		}
	}

	ast_t result = SCHED(processor_csw_check)(processor);
	if (result != AST_NONE) {
		return check_reason | result | (thread_is_eager_preempt(thread) ? AST_URGENT : AST_NONE);
	}

	/*
	 * Same for avoid-processor
	 *
	 * TODO: Should these set AST_REBALANCE?
	 */
	if (SCHED(avoid_processor_enabled) && SCHED(thread_avoid_processor)(processor, thread, check_reason)) {
		return check_reason | AST_PREEMPT;
	}

#if CONFIG_SCHED_SMT
	/*
	 * Even though we could continue executing on this processor, a
	 * secondary SMT core should try to shed load to another primary core.
	 *
	 * TODO: Should this do the same check that thread_select does? i.e.
	 * if no bound threads target this processor, and idle primaries exist, preempt
	 * The case of RT threads existing is already taken care of above
	 */

	if (processor->current_pri < BASEPRI_RTQUEUES &&
	    processor->processor_primary != processor) {
		return check_reason | AST_PREEMPT;
	}
#endif /* CONFIG_SCHED_SMT*/

	if (thread->state & TH_SUSP) {
		return check_reason | AST_PREEMPT;
	}

#if CONFIG_SCHED_SFI
	/*
	 * Current thread may not need to be preempted, but maybe needs
	 * an SFI wait?
	 */
	result = sfi_thread_needs_ast(thread, NULL);
	if (result != AST_NONE) {
		return result;
	}
#endif

	return AST_NONE;
}

/*
 * Handle preemption IPI or IPI in response to setting an AST flag
 * Triggered by cause_ast_check
 * Called at splsched
 */
void
ast_check(processor_t processor)
{
	smr_ack_ipi();

	if (processor->state != PROCESSOR_RUNNING) {
		return;
	}

	SCHED_DEBUG_AST_CHECK_KDBG_RELEASE(MACHDBG_CODE(DBG_MACH_SCHED,
	    MACH_SCHED_AST_CHECK) | DBG_FUNC_START);

	thread_t thread = processor->active_thread;

	assert(thread == current_thread());

	/*
	 * Pairs with task_restartable_ranges_synchronize
	 */
	thread_lock(thread);

	thread_reset_pcs_ack_IPI(thread);

	/*
	 * Propagate thread ast to processor.
	 * (handles IPI in response to setting AST flag)
	 */
	ast_propagate(thread);

	/*
	 * Stash the old urgency and perfctl values to find out if
	 * csw_check updates them.
	 */
	thread_urgency_t old_urgency = processor->current_urgency;
	perfcontrol_class_t old_perfctl_class = processor->current_perfctl_class;

	ast_t preempt;

	if ((preempt = csw_check(thread, processor, AST_NONE)) != AST_NONE) {
		ast_on(preempt);
	}

	if (old_urgency != processor->current_urgency) {
		/*
		 * Urgency updates happen with the thread lock held (ugh).
		 * TODO: This doesn't notice QoS changes...
		 */
		uint64_t urgency_param1, urgency_param2;

		thread_urgency_t urgency = thread_get_urgency(thread, &urgency_param1, &urgency_param2);
		thread_tell_urgency(urgency, urgency_param1, urgency_param2, 0, thread);
	}

	thread_unlock(thread);

	if (old_perfctl_class != processor->current_perfctl_class) {
		/*
		 * We updated the perfctl class of this thread from another core.
		 * Let CLPC know that the currently running thread has a new
		 * class.
		 */

		machine_switch_perfcontrol_state_update(PERFCONTROL_ATTR_UPDATE,
		    mach_approximate_time(), 0, thread);
	}

	SCHED_DEBUG_AST_CHECK_KDBG_RELEASE(MACHDBG_CODE(DBG_MACH_SCHED,
	    MACH_SCHED_AST_CHECK) | DBG_FUNC_END, preempt);
}


void
thread_preempt_expire(
	timer_call_param_t      p0,
	__unused timer_call_param_t      p1)
{
	processor_t processor = p0;

	assert(processor == current_processor());
	assert(p1 == NULL);

	thread_t thread = current_thread();

	/*
	 * This is set and cleared by the current core, so we will
	 * never see a race with running timer expiration
	 */
	assert(processor->pending_nonurgent_preemption);

	clear_pending_nonurgent_preemption(processor);

	thread_lock(thread);

	/*
	 * Check again to see if it's still worth a
	 * context switch, but this time force enable kernel preemption
	 */

	ast_t preempt = csw_check(thread, processor, AST_URGENT);

	if (preempt) {
		ast_on(preempt);
	}

	thread_unlock(thread);

	KDBG_RELEASE(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_PREEMPT_TIMER_ACTIVE), preempt);
}

void
perfcontrol_timer_expire(
	timer_call_param_t          p0,
	__unused timer_call_param_t p1
	)
{
	processor_t processor = p0;
	uint64_t now = mach_absolute_time();
	/* Default behavior is to cancel the timer */
	uint64_t timeout_ticks = EndOfAllTime;
	machine_perfcontrol_running_timer_expire(now, 0, processor->cpu_id, &timeout_ticks);
	if (timeout_ticks == EndOfAllTime) {
		running_timer_clear(processor, RUNNING_TIMER_PERFCONTROL);
	} else {
		uint64_t deadline = now + timeout_ticks;
		running_timer_setup(processor, RUNNING_TIMER_PERFCONTROL, NULL, deadline, now);
	}
}

/*
 *	set_sched_pri:
 *
 *	Set the scheduled priority of the specified thread.
 *
 *	This may cause the thread to change queues.
 *
 *	Thread must be locked.
 */
void
set_sched_pri(
	thread_t        thread,
	int16_t         new_priority,
	set_sched_pri_options_t options)
{
	bool is_current_thread = (thread == current_thread());
	bool removed_from_runq = false;
	bool lazy_update = ((options & SETPRI_LAZY) == SETPRI_LAZY);

	int16_t old_priority = thread->sched_pri;

	/* If we're already at this priority, no need to mess with the runqueue */
	if (new_priority == old_priority) {
#if CONFIG_SCHED_CLUTCH
		/* For the first thread in the system, the priority is correct but
		 * th_sched_bucket is still TH_BUCKET_RUN. Since the clutch
		 * scheduler relies on the bucket being set for all threads, update
		 * its bucket here.
		 */
		if (thread->th_sched_bucket == TH_BUCKET_RUN) {
			assert(thread == vm_pageout_scan_thread);
			SCHED(update_thread_bucket)(thread);
		}
#endif /* CONFIG_SCHED_CLUTCH */

		return;
	}

	if (is_current_thread) {
		assert(thread->state & TH_RUN);
		thread_assert_runq_null(thread);
	} else {
		removed_from_runq = thread_run_queue_remove(thread);
	}

	thread->sched_pri = new_priority;

#if CONFIG_SCHED_CLUTCH
	/*
	 * Since for the clutch scheduler, the thread's bucket determines its runq
	 * in the hierarchy, it is important to update the bucket when the thread
	 * lock is held and the thread has been removed from the runq hierarchy.
	 *
	 * If the thread's bucket has changed, this will consume sched_tick_delta()
	 * in order to account CPU time with the correct scheduling bucket.
	 */
	SCHED(update_thread_bucket)(thread);

#endif /* CONFIG_SCHED_CLUTCH */

	KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_CHANGE_PRIORITY),
	    (uintptr_t)thread_tid(thread),
	    thread->base_pri,
	    thread->sched_pri,
	    thread->sched_usage,
	    0);

	if (removed_from_runq) {
		thread_run_queue_reinsert(thread, SCHED_PREEMPT | SCHED_TAILQ);
	} else if (is_current_thread) {
		processor_t processor = thread->last_processor;
		assert(processor == current_processor());

		thread_urgency_t old_urgency = processor->current_urgency;

		/*
		 * When dropping in priority, check if the thread no longer belongs on core.
		 * If a thread raises its own priority, don't aggressively rebalance it.
		 * <rdar://problem/31699165>
		 *
		 * csw_check does a processor_state_update_from_thread, but
		 * we should do our own if we're being lazy.
		 */
		if (!lazy_update && new_priority < old_priority) {
			ast_t preempt;

			if ((preempt = csw_check(thread, processor, AST_NONE)) != AST_NONE) {
				ast_on(preempt);
			}
		} else {
			processor_state_update_from_thread(processor, thread, false);
		}

		/*
		 * set_sched_pri doesn't alter RT params. We expect direct base priority/QoS
		 * class alterations from user space to occur relatively infrequently, hence
		 * those are lazily handled. QoS classes have distinct priority bands, and QoS
		 * inheritance is expected to involve priority changes.
		 */
		if (processor->current_urgency != old_urgency) {
			uint64_t urgency_param1, urgency_param2;

			thread_urgency_t new_urgency = thread_get_urgency(thread,
			    &urgency_param1, &urgency_param2);

			thread_tell_urgency(new_urgency, urgency_param1,
			    urgency_param2, 0, thread);
		}

		/* TODO: only call this if current_perfctl_class changed */
		uint64_t ctime = mach_approximate_time();
		machine_thread_going_on_core(thread, processor->current_urgency, 0, 0, ctime);
	} else if (thread->state & TH_RUN) {
		processor_t processor = thread->last_processor;

		if (!lazy_update &&
		    processor != PROCESSOR_NULL &&
		    processor != current_processor() &&
		    processor->active_thread == thread) {
			cause_ast_check(processor);
		}
	}
}

/*
 * thread_run_queue_remove_for_handoff
 *
 * Pull a thread or its (recursive) push target out of the runqueue
 * so that it is ready for thread_run()
 *
 * Called at splsched
 *
 * Returns the thread that was pulled or THREAD_NULL if no thread could be pulled.
 * This may be different than the thread that was passed in.
 */
thread_t
thread_run_queue_remove_for_handoff(thread_t thread)
{
	thread_t pulled_thread = THREAD_NULL;

	thread_lock(thread);

	/*
	 * Check that the thread is not bound to a different processor,
	 * NO_SMT flag is not set on the thread, cluster type of
	 * processor matches with thread if the thread is pinned to a
	 * particular cluster and that realtime is not involved.
	 *
	 * Next, pull it off its run queue.  If it doesn't come, it's not eligible.
	 */
	processor_t processor = current_processor();
	if ((thread->bound_processor == PROCESSOR_NULL || thread->bound_processor == processor)
#if CONFIG_SCHED_SMT
	    && (!thread_no_smt(thread))
#endif /* CONFIG_SCHED_SMT */
	    && (processor->current_pri < BASEPRI_RTQUEUES)
	    && (thread->sched_pri < BASEPRI_RTQUEUES)
#if __AMP__
	    && ((thread->th_bound_cluster_id == THREAD_BOUND_CLUSTER_NONE) ||
	    processor->processor_set->pset_id == thread->th_bound_cluster_id)
#endif /* __AMP__ */
	    ) {
		if (thread_run_queue_remove(thread)) {
			pulled_thread = thread;
		}
	}

	thread_unlock(thread);

	return pulled_thread;
}

/*
 * thread_prepare_for_handoff
 *
 * Make the thread ready for handoff.
 * If the thread was runnable then pull it off the runq, if the thread could
 * not be pulled, return NULL.
 *
 * If the thread was woken up from wait for handoff, make sure it is not bound to
 * different processor.
 *
 * Called at splsched
 *
 * Returns the thread that was pulled or THREAD_NULL if no thread could be pulled.
 * This may be different than the thread that was passed in.
 */
thread_t
thread_prepare_for_handoff(thread_t thread, thread_handoff_option_t option)
{
	thread_t pulled_thread = THREAD_NULL;

	if (option & THREAD_HANDOFF_SETRUN_NEEDED) {
		processor_t processor = current_processor();
		thread_lock(thread);

		/*
		 * Check that the thread is not bound to a different processor,
		 * NO_SMT flag is not set on the thread and cluster type of
		 * processor matches with thread if the thread is pinned to a
		 * particular cluster. Call setrun instead if above conditions
		 * are not satisfied.
		 */
		if ((thread->bound_processor == PROCESSOR_NULL || thread->bound_processor == processor)
#if CONFIG_SCHED_SMT
		    && (!thread_no_smt(thread))
#endif /* CONFIG_SCHED_SMT */
#if __AMP__
		    && ((thread->th_bound_cluster_id == THREAD_BOUND_CLUSTER_NONE) ||
		    processor->processor_set->pset_id == thread->th_bound_cluster_id)
#endif /* __AMP__ */
		    ) {
			pulled_thread = thread;
		} else {
			thread_setrun(thread, SCHED_PREEMPT | SCHED_TAILQ);
		}
		thread_unlock(thread);
	} else {
		pulled_thread = thread_run_queue_remove_for_handoff(thread);
	}

	return pulled_thread;
}

/*
 *	thread_run_queue_remove:
 *
 *	Remove a thread from its current run queue and
 *	return TRUE if successful.
 *
 *	Thread must be locked.
 *
 *	If thread->runq is PROCESSOR_NULL, the thread will not re-enter the
 *	run queues because the caller locked the thread.  Otherwise
 *	the thread is on a run queue, but could be chosen for dispatch
 *	and removed by another processor under a different lock, which
 *	will set thread->runq to PROCESSOR_NULL.
 *
 *	Hence the thread select path must not rely on anything that could
 *	be changed under the thread lock after calling this function,
 *	most importantly thread->sched_pri.
 */
boolean_t
thread_run_queue_remove(
	thread_t        thread)
{
	boolean_t removed = FALSE;

	if ((thread->state & (TH_RUN | TH_WAIT)) == TH_WAIT) {
		/* Thread isn't runnable */
		thread_assert_runq_null(thread);
		return FALSE;
	}

	processor_t processor = thread_get_runq(thread);
	if (processor == PROCESSOR_NULL) {
		/*
		 * The thread is either not on the runq,
		 * or is in the midst of being removed from the runq.
		 *
		 * runq is set to NULL under the pset lock, not the thread
		 * lock, so the thread may still be in the process of being dequeued
		 * from the runq. It will wait in invoke for the thread lock to be
		 * dropped.
		 */

		return FALSE;
	}

	if (thread->sched_pri < BASEPRI_RTQUEUES) {
		return SCHED(processor_queue_remove)(processor, thread);
	}

	processor_set_t pset = processor->processor_set;

	pset_lock(pset);

	/*
	 * Must re-read the thread runq after acquiring the pset lock, in
	 * case another core swooped in before us to dequeue the thread.
	 */
	if (thread_get_runq_locked(thread) != PROCESSOR_NULL) {
		/*
		 *	Thread is on the RT run queue and we have a lock on
		 *	that run queue.
		 */
		rt_runq_remove(&pset->rt_runq, thread);
		pset_update_rt_stealable_state(pset);

		removed = TRUE;
	}

	pset_unlock(pset);

	return removed;
}

/*
 * Put the thread back where it goes after a thread_run_queue_remove
 *
 * Thread must have been removed under the same thread lock hold
 *
 * thread locked, at splsched
 */
void
thread_run_queue_reinsert(thread_t thread, sched_options_t options)
{
	thread_assert_runq_null(thread);
	assert(thread->state & (TH_RUN));

	thread_setrun(thread, options);
}

void
sys_override_cpu_throttle(boolean_t enable_override)
{
	if (enable_override) {
		cpu_throttle_enabled = 0;
	} else {
		cpu_throttle_enabled = 1;
	}
}

thread_urgency_t
thread_get_urgency(thread_t thread, uint64_t *arg1, uint64_t *arg2)
{
	uint64_t urgency_param1 = 0, urgency_param2 = 0;
	task_t task = get_threadtask_early(thread);

	thread_urgency_t urgency;

	if (thread == NULL || task == TASK_NULL || (thread->state & TH_IDLE)) {
		urgency_param1 = 0;
		urgency_param2 = 0;

		urgency = THREAD_URGENCY_NONE;
	} else if (thread->sched_mode == TH_MODE_REALTIME) {
		urgency_param1 = thread->realtime.period;
		urgency_param2 = thread->realtime.deadline;

		urgency = THREAD_URGENCY_REAL_TIME;
	} else if (cpu_throttle_enabled &&
	    (thread->sched_pri <= MAXPRI_THROTTLE) &&
	    (thread->base_pri <= MAXPRI_THROTTLE)) {
		/*
		 * Threads that are running at low priority but are not
		 * tagged with a specific QoS are separated out from
		 * the "background" urgency. Performance management
		 * subsystem can decide to either treat these threads
		 * as normal threads or look at other signals like thermal
		 * levels for optimal power/perf tradeoffs for a platform.
		 */
		boolean_t thread_lacks_qos = (proc_get_effective_thread_policy(thread, TASK_POLICY_QOS) == THREAD_QOS_UNSPECIFIED); //thread_has_qos_policy(thread);
		boolean_t task_is_suppressed = (proc_get_effective_task_policy(task, TASK_POLICY_SUP_ACTIVE) == 0x1);

		/*
		 * Background urgency applied when thread priority is
		 * MAXPRI_THROTTLE or lower and thread is not promoted
		 * and thread has a QoS specified
		 */
		urgency_param1 = thread->sched_pri;
		urgency_param2 = thread->base_pri;

		if (thread_lacks_qos && !task_is_suppressed) {
			urgency = THREAD_URGENCY_LOWPRI;
		} else {
			urgency = THREAD_URGENCY_BACKGROUND;
		}
	} else {
		/* For otherwise unclassified threads, report throughput QoS parameters */
		urgency_param1 = proc_get_effective_thread_policy(thread, TASK_POLICY_THROUGH_QOS);
		urgency_param2 = proc_get_effective_task_policy(task, TASK_POLICY_THROUGH_QOS);
		urgency = THREAD_URGENCY_NORMAL;
	}

	if (arg1 != NULL) {
		*arg1 = urgency_param1;
	}
	if (arg2 != NULL) {
		*arg2 = urgency_param2;
	}

	return urgency;
}

perfcontrol_class_t
thread_get_perfcontrol_class(thread_t thread)
{
	/* Special case handling */
	if (thread->state & TH_IDLE) {
		return PERFCONTROL_CLASS_IDLE;
	}

	if (thread->sched_mode == TH_MODE_REALTIME) {
		return PERFCONTROL_CLASS_REALTIME;
	}

	/* perfcontrol_class based on base_pri */
	if (thread->base_pri <= MAXPRI_THROTTLE) {
		return PERFCONTROL_CLASS_BACKGROUND;
	} else if (thread->base_pri <= BASEPRI_UTILITY) {
		return PERFCONTROL_CLASS_UTILITY;
	} else if (thread->base_pri <= BASEPRI_DEFAULT) {
		return PERFCONTROL_CLASS_NONUI;
	} else if (thread->base_pri <= BASEPRI_USER_INITIATED) {
		return PERFCONTROL_CLASS_USER_INITIATED;
	} else if (thread->base_pri <= BASEPRI_FOREGROUND) {
		return PERFCONTROL_CLASS_UI;
	} else {
		if (get_threadtask(thread) == kernel_task) {
			/*
			 * Classify Above UI kernel threads as PERFCONTROL_CLASS_KERNEL.
			 * All other lower priority kernel threads should be treated
			 * as regular threads for performance control purposes.
			 */
			return PERFCONTROL_CLASS_KERNEL;
		}
		return PERFCONTROL_CLASS_ABOVEUI;
	}
}

/*
 *	This is the processor idle loop, which just looks for other threads
 *	to execute.  Processor idle threads invoke this without supplying a
 *	current thread to idle without an asserted wait state.
 *
 *	Returns a the next thread to execute if dispatched directly.
 */

#if 0
#define IDLE_KERNEL_DEBUG_CONSTANT(...) KERNEL_DEBUG_CONSTANT(__VA_ARGS__)
#else
#define IDLE_KERNEL_DEBUG_CONSTANT(...) do { } while(0)
#endif

#if (DEVELOPMENT || DEBUG)
int sched_idle_delay_cpuid = -1;
#endif

thread_t
processor_idle(
	thread_t                        thread,
	processor_t                     processor)
{
	processor_set_t         pset = processor->processor_set;
	struct recount_snap snap = { 0 };

	(void)splsched();

	KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
	    MACHDBG_CODE(DBG_MACH_SCHED, MACH_IDLE) | DBG_FUNC_START,
	    (uintptr_t)thread_tid(thread), 0, 0, 0, 0);

	SCHED_STATS_INC(idle_transitions);
	assert(processor->running_timers_active == false);

	recount_snapshot(&snap);
	recount_processor_idle(&processor->pr_recount, &snap);

	while (1) {
		/*
		 * Ensure that updates to my processor and pset state,
		 * made by the IPI source processor before sending the IPI,
		 * are visible on this processor now (even though we don't
		 * take the pset lock yet).
		 */
		atomic_thread_fence(memory_order_acquire);

		if (processor->state != PROCESSOR_IDLE) {
			break;
		}
		if (bit_test(pset->pending_AST_URGENT_cpu_mask, processor->cpu_id)) {
			break;
		}
#if defined(CONFIG_SCHED_DEFERRED_AST)
		if (bit_test(pset->pending_deferred_AST_cpu_mask, processor->cpu_id)) {
			break;
		}
#endif
		if (bit_test(pset->rt_pending_spill_cpu_mask, processor->cpu_id)) {
			break;
		}

		if (
			processor->is_recommended
#if CONFIG_SCHED_SMT
			&& (processor->processor_primary == processor)
#endif /* CONFIG_SCHED_SMT */
			) {
			if (rt_runq_count(pset)) {
				break;
			}
		} else {
			if (SCHED(processor_bound_count)(processor)) {
				break;
			}
		}

		IDLE_KERNEL_DEBUG_CONSTANT(
			MACHDBG_CODE(DBG_MACH_SCHED, MACH_IDLE) | DBG_FUNC_NONE, (uintptr_t)thread_tid(thread), rt_runq_count(pset), SCHED(processor_runq_count)(processor), -1, 0);

		machine_track_platform_idle(TRUE);

		machine_idle();
		/* returns with interrupts enabled */

		machine_track_platform_idle(FALSE);

#if (DEVELOPMENT || DEBUG)
		if (processor->cpu_id == sched_idle_delay_cpuid) {
			delay(500);
		}
#endif

		(void)splsched();

		atomic_thread_fence(memory_order_acquire);

		IDLE_KERNEL_DEBUG_CONSTANT(
			MACHDBG_CODE(DBG_MACH_SCHED, MACH_IDLE) | DBG_FUNC_NONE, (uintptr_t)thread_tid(thread), rt_runq_count(pset), SCHED(processor_runq_count)(processor), -2, 0);

		/*
		 * Check if we should call sched_timeshare_consider_maintenance() here.
		 * The CPU was woken out of idle due to an interrupt and we should do the
		 * call only if the processor is still idle. If the processor is non-idle,
		 * the threads running on the processor would do the call as part of
		 * context swithing.
		 */
		if (processor->state == PROCESSOR_IDLE) {
			sched_timeshare_consider_maintenance(mach_absolute_time(), true);
		}

		if (!SCHED(processor_queue_empty)(processor)) {
#if CONFIG_SCHED_SMT
			/* Secondary SMT processors respond to directed wakeups
			 * exclusively. Some platforms induce 'spurious' SMT wakeups.
			 */
			if (processor->processor_primary == processor) {
				break;
			}
#else /* CONFIG_SCHED_SMT*/
			break;
#endif /* CONFIG_SCHED_SMT*/
		}
	}

	recount_snapshot(&snap);
	recount_processor_run(&processor->pr_recount, &snap);
	smr_cpu_join(processor, snap.rsn_time_mach);

	ast_t reason = AST_NONE;

	/* We're handling all scheduling AST's */
	ast_off(AST_SCHEDULING);

	/*
	 * thread_select will move the processor from dispatching to running,
	 * or put it in idle if there's nothing to do.
	 */
	thread_t cur_thread = current_thread();

	thread_lock(cur_thread);
	thread_t new_thread = thread_select(cur_thread, processor, &reason);
	thread_unlock(cur_thread);

	assert(processor->running_timers_active == false);

	KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
	    MACHDBG_CODE(DBG_MACH_SCHED, MACH_IDLE) | DBG_FUNC_END,
	    (uintptr_t)thread_tid(thread), processor->state, (uintptr_t)thread_tid(new_thread), reason, 0);

	return new_thread;
}

/*
 *	Each processor has a dedicated thread which
 *	executes the idle loop when there is no suitable
 *	previous context.
 *
 *	This continuation is entered with interrupts disabled.
 */
void
idle_thread(__assert_only void* parameter,
    __unused wait_result_t result)
{
	assert(ml_get_interrupts_enabled() == FALSE);
	assert(parameter == NULL);

	processor_t processor = current_processor();

	smr_cpu_leave(processor, processor->last_dispatch);

	/*
	 * Ensure that anything running in idle context triggers
	 * preemption-disabled checks.
	 */
	disable_preemption_without_measurements();

	/*
	 * Enable interrupts temporarily to handle any pending interrupts
	 * or IPIs before deciding to sleep
	 */
	spllo();

	thread_t new_thread = processor_idle(THREAD_NULL, processor);
	/* returns with interrupts disabled */

	enable_preemption();

	if (new_thread != THREAD_NULL) {
		thread_run(processor->idle_thread,
		    idle_thread, NULL, new_thread);
		/*NOTREACHED*/
	}

	thread_block(idle_thread);
	/*NOTREACHED*/
}

void
idle_thread_create(
	processor_t             processor,
	thread_continue_t       continuation)
{
	kern_return_t   result;
	thread_t                thread;
	spl_t                   s;
	char                    name[MAXTHREADNAMESIZE];

	result = kernel_thread_create(continuation, NULL, MAXPRI_KERNEL, &thread);
	if (result != KERN_SUCCESS) {
		panic("idle_thread_create failed: %d", result);
	}

	snprintf(name, sizeof(name), "idle #%d", processor->cpu_id);
	thread_set_thread_name(thread, name);

	s = splsched();
	thread_lock(thread);
	thread->bound_processor = processor;
	thread->chosen_processor = processor;
	processor->idle_thread = thread;
	thread->sched_pri = thread->base_pri = IDLEPRI;
	thread->state = (TH_RUN | TH_IDLE);
	thread->options |= TH_OPT_IDLE_THREAD;
	thread->last_made_runnable_time = thread->last_basepri_change_time = mach_absolute_time();
	thread_unlock(thread);
	splx(s);

	thread_deallocate(thread);
}

/*
 * sched_startup:
 *
 * Kicks off scheduler services.
 *
 * Called at splsched.
 */
void
sched_startup(void)
{
	kern_return_t   result;
	thread_t                thread;

	simple_lock_init(&sched_vm_group_list_lock, 0);

	result = kernel_thread_start_priority((thread_continue_t)sched_init_thread,
	    NULL, MAXPRI_KERNEL, &thread);
	if (result != KERN_SUCCESS) {
		panic("sched_startup: failed to create sched_init_thread, error %d", result);
	}

	thread_deallocate(thread);

	assert_thread_magic(thread);

	/*
	 * Yield to the sched_init_thread once, to
	 * initialize our own thread after being switched
	 * back to.
	 *
	 * The current thread is the only other thread
	 * active at this point.
	 */
	thread_block(THREAD_CONTINUE_NULL);

	assert_thread_magic(thread);
	
	/* 
	 * After returning from thread_block, the scheduler
	 * initialization is complete and the current thread
	 * has been properly set up.
	 */
	
	KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_INIT) | DBG_FUNC_END,
	    0, 0, 0, 0, 0);
}

#if defined(CONFIG_SCHED_TIMESHARE_CORE)

void
sched_timeshare_maintenance_continue(void)
{
    uint64_t sched_tick_ctime, late_time, sched_tick_delta;
    
    struct sched_update_scan_context scan_context = {
        .earliest_bg_make_runnable_time = UINT64_MAX,
        .earliest_normal_make_runnable_time = UINT64_MAX,
        .earliest_rt_make_runnable_time = UINT64_MAX
    };
    
    sched_tick_ctime = mach_absolute_time();
    
    /* EEVDF maintenance: update min_vruntime across all CPUs */
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        struct eevdf_rq *rq = &PERCPU_DATA(eevdf_rq)[cpu];
        struct eevdf_cfs_rq *cfs_rq = &rq->cfs;
        
        if (cfs_rq->nr_running > 0) {
            eevdf_update_min_vruntime(cfs_rq);
            
            /* Update deadlines for all entities */
            struct rb_node *node;
            for (node = rb_first(&cfs_rq->tasks_timeline); node; node = rb_next(node)) {
                struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
                if (se->deadline < sched_tick_ctime) {
                    /* Missed deadline - adjust */
                    se->deadline = sched_tick_ctime + se->slice;
                    se->virtual_deadline = se->vruntime + se->slice;
                    
                    struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
                    stats->deadline_misses++;
                }
            }
        }
    }
    
    if (__improbable(sched_tick_last_abstime == 0)) {
        sched_tick_last_abstime = sched_tick_ctime;
        late_time = 0;
        sched_tick_delta = 1;
    } else {
        late_time = sched_tick_ctime - sched_tick_last_abstime;
        sched_tick_delta = late_time / sched_tick_interval;
        sched_tick_delta = MAX(sched_tick_delta, 1);
        sched_tick_delta = MIN(sched_tick_delta, SCHED_TICK_MAX_DELTA);
        sched_tick_last_abstime = sched_tick_ctime;
    }
    
    scan_context.sched_tick_last_abstime = sched_tick_last_abstime;
    
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_MAINTENANCE) | DBG_FUNC_START,
        sched_tick_delta, late_time, 0, 0, 0);
    
    os_atomic_add(&sched_tick, (uint32_t)sched_tick_delta, relaxed);
    
    update_vm_info();
    compute_averages(sched_tick_delta);
    
    /* EEVDF thread scan */
    SCHED(thread_update_scan)(&scan_context);
    SCHED(rt_runq_scan)(&scan_context);
    
    uint64_t ctime = mach_absolute_time();
    
    uint64_t bg_max_latency = (ctime > scan_context.earliest_bg_make_runnable_time) ?
        ctime - scan_context.earliest_bg_make_runnable_time : 0;
    uint64_t default_max_latency = (ctime > scan_context.earliest_normal_make_runnable_time) ?
        ctime - scan_context.earliest_normal_make_runnable_time : 0;
    uint64_t realtime_max_latency = (ctime > scan_context.earliest_rt_make_runnable_time) ?
        ctime - scan_context.earliest_rt_make_runnable_time : 0;
    
    machine_max_runnable_latency(bg_max_latency, default_max_latency, realtime_max_latency);
    
    sched_vm_group_maintenance();
    
#if __arm64__
    sched_recommended_cores_maintenance();
#endif /* __arm64__ */
    
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_MAINTENANCE) | DBG_FUNC_END,
        sched_pri_shifts[TH_BUCKET_SHARE_FG], sched_pri_shifts[TH_BUCKET_SHARE_BG],
        sched_pri_shifts[TH_BUCKET_SHARE_UT], sched_pri_shifts[TH_BUCKET_SHARE_DF], 0);
    
    assert_wait((event_t)sched_timeshare_maintenance_continue, THREAD_UNINT);
    thread_block((thread_continue_t)sched_timeshare_maintenance_continue);
}

boolean_t
runq_scan_eevdf(
    struct eevdf_cfs_rq *cfs_rq,
    sched_update_scan_context_t scan_context)
{
    struct rb_node *node;
    boolean_t need_retry = FALSE;
    
    if (cfs_rq->nr_running == 0) {
        return FALSE;
    }
    
    for (node = rb_first(&cfs_rq->tasks_timeline); node; node = rb_next(node)) {
        struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
        thread_t thread = se->thread;
        
        if (thread->sched_stamp != os_atomic_load(&sched_tick, relaxed) &&
            thread->sched_mode == TH_MODE_TIMESHARE) {
            if (thread_update_add_thread(thread) == FALSE) {
                need_retry = TRUE;
                break;
            }
        }
        
        if (cpu_throttle_enabled && 
            ((thread->sched_pri <= MAXPRI_THROTTLE) && (thread->base_pri <= MAXPRI_THROTTLE))) {
            if (thread->last_made_runnable_time < scan_context->earliest_bg_make_runnable_time) {
                scan_context->earliest_bg_make_runnable_time = thread->last_made_runnable_time;
            }
        } else {
            if (thread->last_made_runnable_time < scan_context->earliest_normal_make_runnable_time) {
                scan_context->earliest_normal_make_runnable_time = thread->last_made_runnable_time;
            }
        }
    }
    
    return need_retry;
}

void
thread_update_scan_eevdf(sched_update_scan_context_t scan_context)
{
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        struct eevdf_rq *rq = &PERCPU_DATA(eevdf_rq)[cpu];
        struct eevdf_cfs_rq *cfs_rq = &rq->cfs;
        
        if (cfs_rq->nr_running > 0) {
            if (runq_scan_eevdf(cfs_rq, scan_context)) {
                /* Hit thread limit - process now */
                thread_update_process_threads();
                /* Continue scanning */
                runq_scan_eevdf(cfs_rq, scan_context);
            }
        }
    }
    
    thread_update_process_threads();
}

void
eevdf_update_priority(thread_t thread)
{
    struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
    
    if (!se) {
        return;
    }
    
    uint32_t new_weight = prio_to_weight(thread->sched_pri);
    uint32_t old_weight = se->load_weight;
    
    if (old_weight != new_weight) {
        uint64_t now = mach_absolute_time();
        spl_t s = splsched();
        
        thread_lock(thread);
        
        /* Remove from runqueue if currently enqueued */
        boolean_t was_on_rq = (se->on_rq != 0);
        if (was_on_rq) {
            struct eevdf_cfs_rq *cfs_rq = se->cfs_rq;
            
            eevdf_update_curr(cfs_rq);
            
            rb_erase(&se->run_node, &cfs_rq->tasks_timeline);
            cfs_rq->nr_running--;
            cfs_rq->load_weight -= se->load_weight;
            se->on_rq = 0;
            
            eevdf_update_min_vruntime(cfs_rq);
        }
        
        /* Update weight and slice */
        se->load_weight = new_weight;
        se->slice = EEVDF_MIN_GRANULARITY_NS;
        if (se->load_weight < EEVDF_WEIGHT_NICE_0) {
            se->slice = (se->slice * EEVDF_WEIGHT_NICE_0) / se->load_weight;
        }
        
        /* Update virtual deadline */
        se->virtual_deadline = se->vruntime + se->slice;
        se->deadline = now + se->slice;
        
        /* Reinsert if was on runqueue */
        if (was_on_rq) {
            struct eevdf_cfs_rq *cfs_rq = se->cfs_rq;
            
            __eevdf_add_entity(cfs_rq, se);
            se->on_rq = 1;
            cfs_rq->nr_running++;
            cfs_rq->load_weight += se->load_weight;
            
            eevdf_update_min_vruntime(cfs_rq);
            
            /* Check if preemption needed */
            if (cfs_rq->curr && cfs_rq->curr != se) {
                if (eevdf_entity_before(se, cfs_rq->curr)) {
                    ast_on(AST_PREEMPT);
                }
            }
        }
        
        thread_unlock(thread);
        splx(s);
        
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PRIORITY_CHANGE) | DBG_FUNC_NONE,
            (uintptr_t)thread_tid(thread), thread->sched_pri, old_weight, new_weight, 0);
    }
}

void
eevdf_thread_terminated(thread_t thread)
{
    struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
    
    if (se) {
        spl_t s = splsched();
        
        if (se->on_rq) {
            struct eevdf_cfs_rq *cfs_rq = se->cfs_rq;
            
            rb_erase(&se->run_node, &cfs_rq->tasks_timeline);
            cfs_rq->nr_running--;
            cfs_rq->load_weight -= se->load_weight;
            
            eevdf_update_min_vruntime(cfs_rq);
        }
        
        kfree(se);
        thread->sched_priv = NULL;
        
        splx(s);
    }
}

void
eevdf_thread_fork(thread_t parent, thread_t child)
{
    struct eevdf_sched_entity *parent_se = (struct eevdf_sched_entity *)parent->sched_priv;
    
    if (parent_se) {
        struct eevdf_sched_entity *child_se = kalloc(sizeof(struct eevdf_sched_entity));
        
        if (child_se) {
            child_se->thread = child;
            child_se->cfs_rq = NULL;
            child_se->load_weight = prio_to_weight(child->sched_pri);
            child_se->on_rq = 0;
            child_se->sum_exec_runtime = 0;
            child_se->prev_sum_exec_runtime = 0;
            child_se->vruntime = parent_se->vruntime; /* Inherit vruntime */
            child_se->sched_pri = child->sched_pri;
            child_se->slice = EEVDF_MIN_GRANULARITY_NS;
            
            child->sched_priv = (void *)child_se;
        }
    }
}

void
eevdf_collect_stats(processor_t processor, struct eevdf_stats *out_stats)
{
    struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
    struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
    
    out_stats->total_enqueues = stats->total_enqueues;
    out_stats->total_dequeues = stats->total_dequeues;
    out_stats->total_preemptions = stats->total_preemptions;
    out_stats->deadline_misses = stats->deadline_misses;
    out_stats->load_sum = rq->cfs.load_weight;
    
    /* Calculate min/max vruntime */
    out_stats->vruntime_min = UINT64_MAX;
    out_stats->vruntime_max = 0;
    
    struct rb_node *node;
    for (node = rb_first(&rq->cfs.tasks_timeline); node; node = rb_next(node)) {
        struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
        
        if (se->vruntime < out_stats->vruntime_min) {
            out_stats->vruntime_min = se->vruntime;
        }
        if (se->vruntime > out_stats->vruntime_max) {
            out_stats->vruntime_max = se->vruntime;
        }
    }
}

kern_return_t
sysctl_eevdf_stats(struct sysctl_req *req)
{
    struct eevdf_stats stats = {0};
    
    /* Aggregate stats from all CPUs */
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        processor_t processor = processor_array[cpu];
        if (processor && processor->processor_online) {
            struct eevdf_stats cpu_stats;
            eevdf_collect_stats(processor, &cpu_stats);
            
            stats.total_enqueues += cpu_stats.total_enqueues;
            stats.total_dequeues += cpu_stats.total_dequeues;
            stats.total_preemptions += cpu_stats.total_preemptions;
            stats.deadline_misses += cpu_stats.deadline_misses;
            stats.load_sum += cpu_stats.load_sum;
            
            if (cpu_stats.vruntime_min < stats.vruntime_min) {
                stats.vruntime_min = cpu_stats.vruntime_min;
            }
            if (cpu_stats.vruntime_max > stats.vruntime_max) {
                stats.vruntime_max = cpu_stats.vruntime_max;
            }
        }
    }
    
    return sysctl_handle_opaque(req, &stats, sizeof(stats));
}

void
sched_init_thread(void)
{
    thread_block(THREAD_CONTINUE_NULL);
    
    thread_t thread = current_thread();
    thread_set_thread_name(thread, "sched_maintenance_thread");
    
    sched_maintenance_thread = thread;
    
    /* Initialize EEVDF per-CPU data if not already done */
    static bool eevdf_initialized = false;
    if (!eevdf_initialized) {
        eevdf_init();
        eevdf_initialized = true;
        
        printf("EEVDF Scheduler: initialized with %d CPUs\n", processor_count);
        printf("  min_granularity: %lld ns, latency: %lld ns\n",
            EEVDF_MIN_GRANULARITY_NS, EEVDF_LATENCY_NS);
    }
    
    SCHED(maintenance_continuation)();
}

#if defined(CONFIG_SCHED_TIMESHARE_CORE)

static _Atomic uint64_t sched_maintenance_deadline;
static uint64_t sched_tick_last_abstime;
static uint32_t sched_maintenance_skip_count = 0;
static uint64_t sched_last_maintenance_completion = 0;
static uint64_t sched_maintenance_total_time = 0;
static uint32_t sched_maintenance_iterations = 0;

/*
 * EEVDF-specific maintenance operations
 */
static void
eevdf_perform_depth_maintenance(void)
{
    uint64_t now = mach_absolute_time();
    static uint64_t last_depth_adjust = 0;
    
    /* Adjust tree depth every 100ms */
    if (now - last_depth_adjust > 100 * NSEC_PER_MSEC) {
        for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
            struct eevdf_rq *rq = &PERCPU_DATA(eevdf_rq)[cpu];
            struct eevdf_cfs_rq *cfs_rq = &rq->cfs;
            
            if (cfs_rq->nr_running > 100) {
                /* Rebalance tree if too deep */
                struct rb_root new_root = RB_ROOT;
                struct rb_node *nodes[256];
                int count = 0;
                
                /* Extract all nodes */
                struct rb_node *node;
                while ((node = rb_first(&cfs_rq->tasks_timeline)) != NULL && count < 256) {
                    rb_erase(node, &cfs_rq->tasks_timeline);
                    nodes[count++] = node;
                }
                
                /* Sort by vruntime for better balance */
                for (int i = 0; i < count - 1; i++) {
                    for (int j = 0; j < count - i - 1; j++) {
                        struct eevdf_sched_entity *se1 = rb_entry(nodes[j], struct eevdf_sched_entity, run_node);
                        struct eevdf_sched_entity *se2 = rb_entry(nodes[j+1], struct eevdf_sched_entity, run_node);
                        
                        if (se1->vruntime > se2->vruntime) {
                            struct rb_node *temp = nodes[j];
                            nodes[j] = nodes[j+1];
                            nodes[j+1] = temp;
                        }
                    }
                }
                
                /* Reinsert in sorted order */
                cfs_rq->tasks_timeline = RB_ROOT;
                cfs_rq->rb_leftmost = NULL;
                
                for (int i = 0; i < count; i++) {
                    struct rb_node *node = nodes[i];
                    struct rb_node **link = &cfs_rq->tasks_timeline.rb_node;
                    struct rb_node *parent = NULL;
                    struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
                    
                    while (*link) {
                        parent = *link;
                        struct eevdf_sched_entity *entry = rb_entry(parent, struct eevdf_sched_entity, run_node);
                        
                        if (eevdf_entity_before(se, entry)) {
                            link = &parent->rb_left;
                        } else {
                            link = &parent->rb_right;
                        }
                    }
                    
                    rb_link_node(node, parent, link);
                    rb_insert_color(node, &cfs_rq->tasks_timeline);
                    
                    if (!cfs_rq->rb_leftmost || eevdf_entity_before(se, 
                        rb_entry(cfs_rq->rb_leftmost, struct eevdf_sched_entity, run_node))) {
                        cfs_rq->rb_leftmost = node;
                    }
                }
            }
        }
        last_depth_adjust = now;
    }
}

/*
 * Advanced deadline pressure calculation
 */
static uint64_t
eevdf_calculate_deadline_pressure(processor_set_t pset)
{
    uint64_t total_pressure = 0;
    uint64_t now = mach_absolute_time();
    int cpu_count = 0;
    
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        processor_t processor = processor_array[cpu];
        if (!processor || processor->processor_set != pset) continue;
        
        struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
        struct eevdf_cfs_rq *cfs_rq = &rq->cfs;
        
        if (cfs_rq->nr_running == 0) continue;
        
        uint64_t pressure = 0;
        int threads_scanned = 0;
        
        struct rb_node *node = rb_first(&cfs_rq->tasks_timeline);
        while (node && threads_scanned < 10) { /* Sample first 10 threads */
            struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
            
            if (se->deadline > now) {
                uint64_t time_to_deadline = se->deadline - now;
                uint64_t time_until_deadline_vruntime = se->virtual_deadline - se->vruntime;
                
                if (time_to_deadline < time_until_deadline_vruntime) {
                    /* Under pressure */
                    pressure += (time_until_deadline_vruntime - time_to_deadline) * 100 / 
                               time_until_deadline_vruntime;
                }
            } else {
                /* Already missed deadline - high pressure */
                pressure += 200;
            }
            
            threads_scanned++;
            node = rb_next(node);
        }
        
        if (threads_scanned > 0) {
            total_pressure += pressure / threads_scanned;
            cpu_count++;
        }
    }
    
    return cpu_count > 0 ? total_pressure / cpu_count : 0;
}

/*
 * EEVDF load balancing based on deadline pressure
 */
static void
eevdf_load_balance_by_pressure(pset_node_t node)
{
    uint64_t max_pressure = 0;
    processor_set_t max_pressure_pset = NULL;
    uint64_t min_pressure = UINT64_MAX;
    processor_set_t min_pressure_pset = NULL;
    
    /* Find most and least pressured psets */
    for (int pset_id = lsb_first(node->pset_map); pset_id >= 0; 
         pset_id = lsb_next(node->pset_map, pset_id)) {
        processor_set_t pset = pset_array[pset_id];
        
        uint64_t pressure = eevdf_calculate_deadline_pressure(pset);
        
        if (pressure > max_pressure) {
            max_pressure = pressure;
            max_pressure_pset = pset;
        }
        if (pressure < min_pressure) {
            min_pressure = pressure;
            min_pressure_pset = pset;
        }
    }
    
    /* If pressure differential is significant, migrate threads */
    if (max_pressure_pset && min_pressure_pset && 
        max_pressure > min_pressure * 2 && max_pressure > 100) {
        
        pset_lock(max_pressure_pset);
        pset_lock(min_pressure_pset);
        
        /* Migrate up to 3 threads from high to low pressure */
        int migrated = 0;
        struct eevdf_rq *rq = &PERCPU_DATA(eevdf_rq)[lsb_first(max_pressure_pset->cpu_bitmask)];
        struct eevdf_cfs_rq *cfs_rq = &rq->cfs;
        
        struct rb_node *node = rb_first(&cfs_rq->tasks_timeline);
        while (node && migrated < 3) {
            struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
            thread_t thread = se->thread;
            
            /* Don't migrate bound threads */
            if (thread->bound_processor == PROCESSOR_NULL) {
                /* Remove from source */
                rb_erase(node, &cfs_rq->tasks_timeline);
                cfs_rq->nr_running--;
                cfs_rq->load_weight -= se->load_weight;
                se->on_rq = 0;
                
                /* Choose target processor in low pressure pset */
                int target_cpu = lsb_first(min_pressure_pset->cpu_bitmask & 
                                           min_pressure_pset->recommended_bitmask);
                if (target_cpu >= 0) {
                    processor_t target = processor_array[target_cpu];
                    struct eevdf_rq *target_rq = PERCPU_GET_RELATIVE(eevdf_rq, target, target);
                    
                    se->cfs_rq = &target_rq->cfs;
                    se->vruntime = target_rq->cfs.min_vruntime; /* Reset to avoid starvation */
                    eevdf_place_entity(&target_rq->cfs, se, FALSE);
                    __eevdf_add_entity(&target_rq->cfs, se);
                    se->on_rq = 1;
                    target_rq->cfs.nr_running++;
                    target_rq->cfs.load_weight += se->load_weight;
                    
                    migrated++;
                }
            }
            
            node = rb_next(node);
        }
        
        eevdf_update_min_vruntime(cfs_rq);
        
        pset_unlock(min_pressure_pset);
        pset_unlock(max_pressure_pset);
    }
}

/*
 * Complete timeshare maintenance with EEVDF integration
 */
void
sched_timeshare_maintenance_continue(void)
{
    uint64_t sched_tick_ctime, late_time, sched_tick_delta;
    uint64_t maintenance_start = mach_absolute_time();
    
    struct sched_update_scan_context scan_context = {
        .earliest_bg_make_runnable_time = UINT64_MAX,
        .earliest_normal_make_runnable_time = UINT64_MAX,
        .earliest_rt_make_runnable_time = UINT64_MAX,
        .pset_pressure = 0,
        .total_deadline_misses = 0,
        .avg_vruntime_drift = 0
    };

    sched_tick_ctime = maintenance_start;

    /* Advanced EEVDF maintenance: per-CPU tree optimization */
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        struct eevdf_rq *rq = &PERCPU_DATA(eevdf_rq)[cpu];
        struct eevdf_cfs_rq *cfs_rq = &rq->cfs;
        
        if (cfs_rq->nr_running > 0) {
            spl_t s = splsched();
            
            /* Update current if exists */
            if (cfs_rq->curr) {
                eevdf_update_curr(cfs_rq);
            }
            
            /* Advanced min_vruntime adjustment with anti-starvation */
            uint64_t old_min = cfs_rq->min_vruntime;
            eevdf_update_min_vruntime(cfs_rq);
            
            /* Detect excessive vruntime drift */
            if (cfs_rq->min_vruntime > old_min + EEVDF_LATENCY_NS * 2) {
                scan_context.avg_vruntime_drift += (cfs_rq->min_vruntime - old_min);
                
                /* Severe drift - boost all waiting threads */
                struct rb_node *node;
                for (node = rb_first(&cfs_rq->tasks_timeline); node; node = rb_next(node)) {
                    struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
                    if (se->vruntime < cfs_rq->min_vruntime - EEVDF_LATENCY_NS) {
                        se->vruntime = cfs_rq->min_vruntime - EEVDF_MIN_GRANULARITY_NS;
                    }
                }
            }
            
            /* Deadline miss tracking */
            struct rb_node *node;
            for (node = rb_first(&cfs_rq->tasks_timeline); node; node = rb_next(node)) {
                struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
                
                if (se->deadline < sched_tick_ctime) {
                    scan_context.total_deadline_misses++;
                    
                    /* Recalculate deadline with penalty */
                    se->virtual_deadline = se->vruntime + se->slice * 2; /* Double penalty */
                    se->deadline = sched_tick_ctime + se->slice * 2;
                    
                    /* Update stats */
                    struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
                    stats->deadline_misses++;
                }
            }
            
            splx(s);
        }
    }
    
    /* Perform depth maintenance periodically */
    eevdf_perform_depth_maintenance();

    if (__improbable(sched_tick_last_abstime == 0)) {
        sched_tick_last_abstime = sched_tick_ctime;
        late_time = 0;
        sched_tick_delta = 1;
    } else {
        late_time = sched_tick_ctime - sched_tick_last_abstime;
        sched_tick_delta = late_time / sched_tick_interval;
        sched_tick_delta = MAX(sched_tick_delta, 1);
        sched_tick_delta = MIN(sched_tick_delta, SCHED_TICK_MAX_DELTA);
        
        /* Handle maintenance skips */
        if (late_time > sched_tick_interval * 2) {
            sched_maintenance_skip_count += (late_time / sched_tick_interval) - 1;
        }
        
        sched_tick_last_abstime = sched_tick_ctime;
    }

    scan_context.sched_tick_last_abstime = sched_tick_last_abstime;
    
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_MAINTENANCE) | DBG_FUNC_START,
        sched_tick_delta, late_time, scan_context.total_deadline_misses, 
        sched_maintenance_skip_count, 0);

    os_atomic_add(&sched_tick, (uint32_t)sched_tick_delta, relaxed);

    update_vm_info();
    compute_averages(sched_tick_delta);

    /* EEVDF-specific thread scanning */
    scan_context.pset_pressure = 0;
    for (pset_node_t node = &pset_node0; node != NULL; node = node->node_list) {
        uint64_t node_pressure = 0;
        int pset_count = 0;
        
        for (int pset_id = lsb_first(node->pset_map); pset_id >= 0; 
             pset_id = lsb_next(node->pset_map, pset_id)) {
            processor_set_t pset = pset_array[pset_id];
            uint64_t pset_pressure = eevdf_calculate_deadline_pressure(pset);
            node_pressure += pset_pressure;
            pset_count++;
            
            /* Perform intra-pset load balancing */
            if (pset_pressure > 200) { /* High pressure threshold */
                eevdf_load_balance_by_pressure(node);
            }
        }
        
        if (pset_count > 0) {
            scan_context.pset_pressure += node_pressure / pset_count;
        }
    }

    /* Run the scans */
    SCHED(thread_update_scan)(&scan_context);
    SCHED(rt_runq_scan)(&scan_context);

    uint64_t ctime = mach_absolute_time();

    uint64_t bg_max_latency = (ctime > scan_context.earliest_bg_make_runnable_time) ?
        ctime - scan_context.earliest_bg_make_runnable_time : 0;
    uint64_t default_max_latency = (ctime > scan_context.earliest_normal_make_runnable_time) ?
        ctime - scan_context.earliest_normal_make_runnable_time : 0;
    uint64_t realtime_max_latency = (ctime > scan_context.earliest_rt_make_runnable_time) ?
        ctime - scan_context.earliest_rt_make_runnable_time : 0;

    /* Apply EEVDF-specific latency hints */
    if (scan_context.avg_vruntime_drift > EEVDF_LATENCY_NS * 10) {
        /* Severe drift detected - suggest more aggressive scheduling */
        bg_max_latency = bg_max_latency * 2;
        default_max_latency = default_max_latency * 2;
    }

    machine_max_runnable_latency(bg_max_latency, default_max_latency, realtime_max_latency);

    /* VM group maintenance */
    sched_vm_group_maintenance();

#if __arm64__
    sched_recommended_cores_maintenance();
#endif /* __arm64__ */

    /* Update maintenance statistics */
    sched_last_maintenance_completion = ctime;
    sched_maintenance_total_time += ctime - maintenance_start;
    sched_maintenance_iterations++;

    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_MAINTENANCE) | DBG_FUNC_END,
        sched_pri_shifts[TH_BUCKET_SHARE_FG], sched_pri_shifts[TH_BUCKET_SHARE_BG],
        sched_pri_shifts[TH_BUCKET_SHARE_UT], sched_pri_shifts[TH_BUCKET_SHARE_DF], 
        scan_context.pset_pressure);

    assert_wait((event_t)sched_timeshare_maintenance_continue, THREAD_UNINT);
    thread_block((thread_continue_t)sched_timeshare_maintenance_continue);
    /*NOTREACHED*/
}

static uint64_t sched_maintenance_wakeups;
static uint64_t sched_maintenance_wakeups_skipped = 0;

/*
 * Enhanced maintenance consideration with EEVDF deadline awareness
 */
void
sched_timeshare_consider_maintenance(uint64_t ctime, bool safe_point)
{
    uint64_t deadline = os_atomic_load(&sched_maintenance_deadline, relaxed);
    uint64_t now = ctime;

    /* EEVDF-specific deadline check */
    if (__improbable(now >= deadline)) {
        if (__improbable(current_thread() == sched_maintenance_thread)) {
            return;
        }
        
        /* Check if we should skip due to high deadline pressure */
        uint64_t total_pressure = 0;
        int pset_count = 0;
        
        for (pset_node_t node = &pset_node0; node != NULL; node = node->node_list) {
            for (int pset_id = lsb_first(node->pset_map); pset_id >= 0; 
                 pset_id = lsb_next(node->pset_map, pset_id)) {
                total_pressure += eevdf_calculate_deadline_pressure(pset_array[pset_id]);
                pset_count++;
            }
        }
        
        uint64_t avg_pressure = pset_count > 0 ? total_pressure / pset_count : 0;
        
        /* Skip maintenance if pressure is extremely high (everyone is missing deadlines) */
        if (avg_pressure > 500) { /* 500% pressure means severe deadline misses */
            sched_maintenance_wakeups_skipped++;
            
            /* Reschedule for sooner */
            uint64_t new_deadline = now + sched_tick_interval / 2;
            os_atomic_store(&sched_maintenance_deadline, new_deadline, relaxed);
            return;
        }

        OSMemoryBarrier();

        uint64_t ndeadline = now + sched_tick_interval;

        if (__probable(os_atomic_cmpxchg(&sched_maintenance_deadline, deadline, ndeadline, seq_cst))) {
            thread_wakeup((event_t)sched_timeshare_maintenance_continue);
            sched_maintenance_wakeups++;
            smr_maintenance(ctime);
            
            KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_MAINT_WAKEUP) | DBG_FUNC_NONE,
                sched_maintenance_wakeups, sched_maintenance_wakeups_skipped, avg_pressure, 0, 0);
        }
    }

    smr_cpu_tick(ctime, safe_point);

#if !CONFIG_SCHED_CLUTCH
    /* Load calculation with EEVDF influence */
    uint64_t load_compute_deadline = os_atomic_load_wide(&sched_load_compute_deadline, relaxed);

    if (__improbable(load_compute_deadline && ctime >= load_compute_deadline)) {
        uint64_t new_deadline = 0;
        if (os_atomic_cmpxchg(&sched_load_compute_deadline, load_compute_deadline, new_deadline, relaxed)) {
            /* Weight load calculation by deadline pressure */
            uint64_t pressure_factor = 100;
            
            for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
                struct eevdf_rq *rq = &PERCPU_DATA(eevdf_rq)[cpu];
                if (rq->cfs.nr_running > 0) {
                    pressure_factor = pressure_factor * 2; /* Increase weight if busy */
                }
            }
            
            compute_sched_load_with_weight(pressure_factor);
            new_deadline = ctime + (sched_load_compute_interval_abs * 100 / pressure_factor);
            os_atomic_store_wide(&sched_load_compute_deadline, new_deadline, relaxed);
        }
    }
#endif /* CONFIG_SCHED_CLUTCH */

#if __arm64__
    uint64_t perf_deadline = os_atomic_load(&sched_perfcontrol_callback_deadline, relaxed);

    if (__improbable(perf_deadline && ctime >= perf_deadline)) {
        if (os_atomic_cmpxchg(&sched_perfcontrol_callback_deadline, perf_deadline, 0, relaxed)) {
            /* Include EEVDF stats in perfcontrol callback */
            uint64_t total_deadline_misses = 0;
            for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
                struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
                total_deadline_misses += stats->deadline_misses;
            }
            
            machine_perfcontrol_deadline_passed_with_stats(perf_deadline, total_deadline_misses);
        }
    }
#endif /* __arm64__ */
}

#endif /* CONFIG_SCHED_TIMESHARE_CORE */

/*
 * Advanced sched_init_thread with EEVDF initialization
 */
void
sched_init_thread(void)
{
    uint64_t init_start = mach_absolute_time();
    
    thread_block(THREAD_CONTINUE_NULL);

    thread_t thread = current_thread();
    thread_set_thread_name(thread, "sched_maintenance_thread");

    sched_maintenance_thread = thread;

    /* Comprehensive EEVDF initialization */
    static bool eevdf_initialized = false;
    static uint64_t eevdf_init_time = 0;
    
    if (!eevdf_initialized) {
        spl_t s = splsched();
        
        printf("========================================\n");
        printf("EEVDF Scheduler Initialization Starting\n");
        printf("========================================\n");
        
        /* Initialize core EEVDF structures */
        eevdf_init();
        
        /* Initialize per-CPU statistics */
        for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
            struct eevdf_stats *stats = &PERCPU_DATA(eevdf_stats)[cpu];
            stats->total_enqueues = 0;
            stats->total_dequeues = 0;
            stats->total_preemptions = 0;
            stats->deadline_misses = 0;
            stats->vruntime_max = 0;
            stats->vruntime_min = UINT64_MAX;
            stats->load_sum = 0;
        }
        
        /* Pre-allocate scheduling entities for common threads */
        thread_t threads_to_init[] = {
            sched_maintenance_thread,
            master_processor->idle_thread,
            vm_pageout_scan_thread
        };
        
        for (int i = 0; i < sizeof(threads_to_init)/sizeof(threads_to_init[0]); i++) {
            thread_t t = threads_to_init[i];
            if (t && !t->sched_priv) {
                struct eevdf_sched_entity *se = kalloc(sizeof(struct eevdf_sched_entity));
                if (se) {
                    se->thread = t;
                    se->cfs_rq = NULL;
                    se->load_weight = prio_to_weight(t->sched_pri);
                    se->on_rq = 0;
                    se->sum_exec_runtime = 0;
                    se->prev_sum_exec_runtime = 0;
                    se->vruntime = 0;
                    se->sched_pri = t->sched_pri;
                    se->slice = EEVDF_MIN_GRANULARITY_NS;
                    t->sched_priv = (void *)se;
                }
            }
        }
        
        splx(s);
        
        eevdf_init_time = mach_absolute_time() - init_start;
        eevdf_initialized = true;
        
        printf("EEVDF Scheduler: Successfully initialized\n");
        printf("  Initialization time: %llu ns\n", eevdf_init_time);
        printf("  min_granularity: %lld ns (%lld us)\n", 
            EEVDF_MIN_GRANULARITY_NS, EEVDF_MIN_GRANULARITY_NS / 1000);
        printf("  latency: %lld ns (%lld us)\n", 
            EEVDF_LATENCY_NS, EEVDF_LATENCY_NS / 1000);
        printf("  wakeup_granularity: %lld ns (%lld us)\n", 
            EEVDF_WAKEUP_GRANULARITY_NS, EEVDF_WAKEUP_GRANULARITY_NS / 1000);
        printf("  Processors: %d\n", processor_count);
        printf("  Max CPUs: %d\n", MAX_CPUS);
        printf("========================================\n");
    }

    /* Call the appropriate maintenance continuation */
    SCHED(maintenance_continuation)();

    /*NOTREACHED*/
}

/*
 * EEVDF-enhanced thread update scan
 */
#define THREAD_UPDATE_SIZE 256 /* Increased for EEVDF */
static thread_t thread_update_array[THREAD_UPDATE_SIZE];
static uint32_t thread_update_count = 0;
static uint64_t thread_update_total_vruntime = 0;
static uint32_t thread_update_count_processed = 0;

boolean_t
thread_update_add_thread(thread_t thread)
{
    if (thread_update_count >= THREAD_UPDATE_SIZE) {
        /* Process half the array to make room */
        thread_update_process_threads_partial(THREAD_UPDATE_SIZE / 2);
    }
    
    if (thread_update_count < THREAD_UPDATE_SIZE) {
        thread_update_array[thread_update_count++] = thread;
        thread_reference(thread);
        
        /* Track vruntime for statistics */
        struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
        if (se) {
            thread_update_total_vruntime += se->vruntime;
        }
        
        return TRUE;
    }
    
    return FALSE;
}

void
thread_update_process_threads_partial(uint32_t max_to_process)
{
    uint32_t to_process = MIN(max_to_process, thread_update_count);
    
    for (uint32_t i = 0; i < to_process; i++) {
        thread_t thread = thread_update_array[i];
        if (!thread) continue;
        
        assert_thread_magic(thread);
        
        spl_t s = splsched();
        thread_lock(thread);

        const bool should_report_failsafe = thread_should_report_failsafe(thread);
        const sched_mode_t saved_mode = thread->saved_mode;

        if (!(thread->state & (TH_WAIT)) && 
            thread->sched_stamp != os_atomic_load(&sched_tick, relaxed)) {
            
            /* EEVDF-specific priority update */
            struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
            if (se) {
                uint64_t old_vruntime = se->vruntime;
                SCHED(update_priority)(thread);
                
                /* Track vruntime change */
                if (se->vruntime > old_vruntime + EEVDF_MIN_GRANULARITY_NS * 10) {
                    /* Large jump - may indicate starvation */
                    struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
                    stats->vruntime_max = MAX(stats->vruntime_max, se->vruntime - old_vruntime);
                }
            } else {
                SCHED(update_priority)(thread);
            }
        }
        
        thread_unlock(thread);
        splx(s);

        if (should_report_failsafe) {
            assert((saved_mode & TH_MODE_REALTIME) || (saved_mode & TH_MODE_FIXED));
            uint64_t th_id = thread->thread_id;
            char th_name[MAXTHREADNAMESIZE] = "unknown";
            if (thread_has_thread_name(thread)) {
                thread_get_thread_name(thread, th_name);
            }
            task_t task = get_threadtask(thread);
            assert(task != NULL);
            const char* t_name = task_best_name(task);
            pid_t t_pid = task_pid(task);
            const int quanta = (saved_mode & TH_MODE_REALTIME) ? max_unsafe_rt_quanta : max_unsafe_fixed_quanta;
            const char* mode = (saved_mode & TH_MODE_REALTIME) ? "realtime" : "fixed";
            
            os_log_error(OS_LOG_DEFAULT, 
                "scheduler: thread %s [%llx] in process %s [%d] triggered fail-safe "
                "by spinning for at least %d us at %s priority. EEVDF stats: "
                "vruntime delta may indicate starvation.\n",
                th_name, th_id, t_name, t_pid,
                quanta * (int)sched_get_quantum_us(), mode);
        }

        thread_deallocate(thread);
        thread_update_array[i] = THREAD_NULL;
        thread_update_count_processed++;
    }

    /* Compact the array */
    uint32_t write_idx = 0;
    for (uint32_t read_idx = to_process; read_idx < thread_update_count; read_idx++) {
        if (thread_update_array[read_idx]) {
            thread_update_array[write_idx++] = thread_update_array[read_idx];
        }
    }
    thread_update_count = write_idx;
}

void
thread_update_process_threads(void)
{
    thread_update_process_threads_partial(thread_update_count);
    
    /* Reset statistics */
    thread_update_total_vruntime = 0;
    thread_update_count_processed = 0;
}

/*
 * EEVDF runqueue scanner
 */
boolean_t
runq_scan_eevdf_advanced(
    struct eevdf_cfs_rq *cfs_rq,
    sched_update_scan_context_t scan_context,
    uint32_t *threads_scanned)
{
    struct rb_node *node;
    boolean_t need_retry = FALSE;
    uint32_t scanned = 0;
    
    if (cfs_rq->nr_running == 0) {
        return FALSE;
    }
    
    /* Track tree balance */
    int depth = 0;
    int max_depth = 0;
    
    for (node = rb_first(&cfs_rq->tasks_timeline); node; node = rb_next(node)) {
        scanned++;
        depth++;
        
        struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
        thread_t thread = se->thread;
        
        if (!node->rb_right) {
            max_depth = MAX(max_depth, depth);
            depth = 0;
        }
        
        if (thread->sched_stamp != os_atomic_load(&sched_tick, relaxed) &&
            thread->sched_mode == TH_MODE_TIMESHARE) {
            
            if (thread_update_add_thread(thread) == FALSE) {
                need_retry = TRUE;
                break;
            }
        }
        
        /* Update scan context with EEVDF-specific metrics */
        if (se->deadline < scan_context->sched_tick_last_abstime) {
            scan_context->total_deadline_misses++;
        }
        
        /* Track latency based on priority */
        if (cpu_throttle_enabled && 
            ((thread->sched_pri <= MAXPRI_THROTTLE) && (thread->base_pri <= MAXPRI_THROTTLE))) {
            if (thread->last_made_runnable_time < scan_context->earliest_bg_make_runnable_time) {
                scan_context->earliest_bg_make_runnable_time = thread->last_made_runnable_time;
            }
        } else {
            if (thread->last_made_runnable_time < scan_context->earliest_normal_make_runnable_time) {
                scan_context->earliest_normal_make_runnable_time = thread->last_made_runnable_time;
            }
        }
    }
    
    /* Report tree imbalance */
    if (max_depth > 2 * cfs_rq->nr_running / 10) { /* More than 20% depth */
        /* Tree is unbalanced - will be fixed in maintenance */
        scan_context->avg_vruntime_drift += max_depth * 1000;
    }
    
    if (threads_scanned) {
        *threads_scanned = scanned;
    }
    
    return need_retry;
}

/*
 * RT runqueue scanner with EEVDF integration
 */
void
rt_runq_scan_eevdf(sched_update_scan_context_t scan_context)
{
    for (pset_node_t node = &pset_node0; node != NULL; node = node->node_list) {
        for (int pset_id = lsb_first(node->pset_map); pset_id >= 0; 
             pset_id = lsb_next(node->pset_map, pset_id)) {
            processor_set_t pset = pset_array[pset_id];
            
            spl_t s = splsched();
            pset_lock(pset);
            
            /* Update pset bitmasks for EEVDF */
            atomic_bitmask_store(&node->pset_rt_map, pset->pset_id, 
                (pset->rt_runq.count > 0), memory_order_relaxed);
            
            /* Scan RT runqueue */
            if (pset->rt_runq.count > 0) {
                rt_queue_t rtq = &pset->rt_runq;
                thread_t thread;
                
                rt_queue_iterate(thread, rtq, rt_runq_links) {
                    if (thread->sched_mode == TH_MODE_REALTIME) {
                        if (thread->last_made_runnable_time < scan_context->earliest_rt_make_runnable_time) {
                            scan_context->earliest_rt_make_runnable_time = thread->last_made_runnable_time;
                        }
                        
                        /* Check if RT thread is interfering with EEVDF fairness */
                        if (thread->realtime.deadline < scan_context->sched_tick_last_abstime) {
                            scan_context->total_deadline_misses++;
                        }
                    }
                }
            }
            
            pset_unlock(pset);
            splx(s);
        }
    }
}

/*
 * Eager preemption with EEVDF awareness
 */
bool
thread_is_eager_preempt(thread_t thread)
{
    return thread->sched_flags & TH_SFLAG_EAGERPREEMPT;
}

void
thread_set_eager_preempt(thread_t thread)
{
    spl_t s = splsched();
    thread_lock(thread);

    assert(!thread_is_eager_preempt(thread));

    thread->sched_flags |= TH_SFLAG_EAGERPREEMPT;

    /* EEVDF-specific eager preemption handling */
    struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
    if (se) {
        /* Reduce slice for eager preempt threads to improve responsiveness */
        se->slice = EEVDF_MIN_GRANULARITY_NS / 2;
        se->virtual_deadline = se->vruntime + se->slice;
        
        if (se->on_rq) {
            /* Update in runqueue */
            struct eevdf_cfs_rq *cfs_rq = se->cfs_rq;
            rb_erase(&se->run_node, &cfs_rq->tasks_timeline);
            __eevdf_add_entity(cfs_rq, se);
        }
    }

    if (thread == current_thread()) {
        ast_t ast = csw_check(thread, current_processor(), AST_NONE);
        current_processor()->current_is_eagerpreempt = true;

        thread_unlock(thread);

        if (ast != AST_NONE) {
            thread_block_reason(THREAD_CONTINUE_NULL, NULL, ast);
        }
    } else {
        processor_t last_processor = thread->last_processor;

        if (last_processor != PROCESSOR_NULL &&
            last_processor->state == PROCESSOR_RUNNING &&
            last_processor->active_thread == thread) {
            cause_ast_check(last_processor);
        }

        thread_unlock(thread);
    }

    splx(s);
}

void
thread_clear_eager_preempt(thread_t thread)
{
    spl_t s = splsched();
    thread_lock(thread);

    assert(thread_is_eager_preempt(thread));

    thread->sched_flags &= ~TH_SFLAG_EAGERPREEMPT;

    /* Restore normal EEVDF slice */
    struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
    if (se) {
        se->slice = EEVDF_MIN_GRANULARITY_NS;
        if (se->load_weight < EEVDF_WEIGHT_NICE_0) {
            se->slice = (se->slice * EEVDF_WEIGHT_NICE_0) / se->load_weight;
        }
        se->virtual_deadline = se->vruntime + se->slice;
        
        if (se->on_rq) {
            struct eevdf_cfs_rq *cfs_rq = se->cfs_rq;
            rb_erase(&se->run_node, &cfs_rq->tasks_timeline);
            __eevdf_add_entity(cfs_rq, se);
        }
    }

    if (thread == current_thread()) {
        current_processor()->current_is_eagerpreempt = false;
    }

    thread_unlock(thread);
    splx(s);
}

/*
 * Enhanced scheduling statistics with EEVDF metrics
 */
void
sched_stats_handle_csw(processor_t processor, int reasons, int selfpri, int otherpri)
{
    struct sched_statistics *stats;
    struct eevdf_stats *eevdf_stats;
    boolean_t to_realtime = FALSE;

    stats = PERCPU_GET_RELATIVE(sched_stats, processor, processor);
    eevdf_stats = PERCPU_GET(eevdf_stats);
    
    stats->csw_count++;

    if (otherpri >= BASEPRI_REALTIME) {
        stats->rt_sched_count++;
        to_realtime = TRUE;
    }

    if ((reasons & AST_PREEMPT) != 0) {
        stats->preempt_count++;
        eevdf_stats->total_preemptions++;

        if (selfpri >= BASEPRI_REALTIME) {
            stats->preempted_rt_count++;
        }

        if (to_realtime) {
            stats->preempted_by_rt_count++;
        }
        
        /* Track preemption reason for EEVDF */
        struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
        if (rq->cfs.curr) {
            uint64_t now = mach_absolute_time();
            if (now > rq->cfs.curr->deadline) {
                eevdf_stats->deadline_misses++;
            }
        }
    }
}

void
sched_stats_handle_runq_change(struct runq_stats *stats, int old_count)
{
    uint64_t timestamp = mach_absolute_time();

    stats->count_sum += (timestamp - stats->last_change_timestamp) * old_count;
    stats->last_change_timestamp = timestamp;
    
    /* Update EEVDF load sum */
    struct eevdf_stats *eevdf_stats = PERCPU_GET(eevdf_stats);
    eevdf_stats->load_sum = (eevdf_stats->load_sum * 7 + old_count * 3) / 10; /* EWMA */
}

/*
 * Assembly-callable thread_wakeup
 */
#undef thread_wakeup
void
thread_wakeup(event_t x);

void
thread_wakeup(event_t x)
{
    thread_wakeup_with_result(x, THREAD_AWAKENED);
    
    /* EEVDF wakeup optimization - hint that a thread is now runnable */
    struct eevdf_rq *rq = PERCPU_GET(eevdf_rq);
    if (rq->cfs.nr_running > 0) {
        /* Potential preemption opportunity */
        ast_on(AST_PREEMPT);
    }
}

/*
 * Preemption state with EEVDF awareness
 */
boolean_t
preemption_enabled(void)
{
    return get_preemption_level() == 0 && ml_get_interrupts_enabled();
}

/*
 * Timer deadline tracking initialization
 */
static void
sched_timer_deadline_tracking_init(void)
{
    nanoseconds_to_absolutetime(TIMER_DEADLINE_TRACKING_BIN_1_DEFAULT, 
        &timer_deadline_tracking_bin_1);
    nanoseconds_to_absolutetime(TIMER_DEADLINE_TRACKING_BIN_2_DEFAULT, 
        &timer_deadline_tracking_bin_2);
    
    /* EEVDF-specific timer tracking */
    printf("EEVDF Timer Tracking: bins %llu ns, %llu ns\n",
        timer_deadline_tracking_bin_1, timer_deadline_tracking_bin_2);
}

/*
 * CPU startup verification
 */
static void
check_all_cpus_are_done_starting(processor_start_kind_t start_kind)
{
    for (int cpu_id = 0; cpu_id < machine_info.logical_cpu_max; cpu_id++) {
        processor_t processor = processor_array[cpu_id];
        processor_wait_for_start(processor, start_kind);
        
        /* Initialize EEVDF for this CPU if not already done */
        if (processor && processor->cpu_id == cpu_id) {
            struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
            if (!rq->cfs.processor) {
                eevdf_processor_init(processor);
            }
        }
    }
}

/*
 * Last resort CPU selection with EEVDF awareness
 */
static int
sched_last_resort_cpu(void)
{
    simple_lock_assert(&sched_available_cores_lock, LCK_ASSERT_OWNED);

    int last_resort_cpu = lsb_first(pcs.pcs_effective.pcs_online_cores);

    if (last_resort_cpu == -1) {
        panic("no last resort cpu found!");
    }
    
    /* Prefer CPU with lowest EEVDF load */
    uint32_t min_load = UINT32_MAX;
    int best_cpu = last_resort_cpu;
    
    cpumap_foreach(cpu_id, pcs.pcs_effective.pcs_online_cores) {
        struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor_array[cpu_id], processor_array[cpu_id]);
        if (rq->cfs.load_weight < min_load) {
            min_load = rq->cfs.load_weight;
            best_cpu = cpu_id;
        }
    }

    return best_cpu;
}

/*
 * Assert no processors in transition
 */
static void
assert_no_processors_in_transition_locked(void)
{
    assert(pcs.pcs_in_kernel_sleep == false);

    assert(pcs.pcs_managed_cores ==
        (processor_offline_state_map[PROCESSOR_OFFLINE_RUNNING] |
         processor_offline_state_map[PROCESSOR_OFFLINE_FULLY_OFFLINE]));

    assert(pcs.pcs_effective.pcs_online_cores ==
        processor_offline_state_map[PROCESSOR_OFFLINE_RUNNING]);
}

/*
 * Compute requested powered cores with EEVDF considerations
 */
static struct powered_cores_state
sched_compute_requested_powered_cores(void)
{
    simple_lock_assert(&sched_available_cores_lock, LCK_ASSERT_OWNED);

    struct powered_cores_state output = {
        .pcs_online_cores = pcs.pcs_managed_cores,
        .pcs_powerdown_recommended_cores = pcs.pcs_managed_cores,
        .pcs_tempdown_cores = 0,
    };

    if (!pcs.pcs_init_completed) {
        return output;
    }

    if (pcs.pcs_powerdown_suspend_count) {
        return output;
    } else {
        cpumap_t system_required_powered_cores = pcs.pcs_required_online_pmgr |
            pcs.pcs_required_online_system;

        cpumap_t online_cores_goal;

        if (pcs.pcs_user_online_core_control) {
            output.pcs_powerdown_recommended_cores = pcs.pcs_requested_online_user;
            online_cores_goal = pcs.pcs_requested_online_user | system_required_powered_cores;
        } else {
            cpumap_t clpc_wanted_powered_cores = pcs.pcs_managed_cores;
            clpc_wanted_powered_cores &= pcs.pcs_requested_online_clpc_user;
            clpc_wanted_powered_cores &= pcs.pcs_requested_online_clpc_system;

            output.pcs_powerdown_recommended_cores = clpc_wanted_powered_cores;
            online_cores_goal = clpc_wanted_powered_cores | system_required_powered_cores;
            output.pcs_tempdown_cores = (pcs.pcs_managed_cores & ~clpc_wanted_powered_cores);
        }

        if (online_cores_goal == 0) {
            online_cores_goal = BIT(lsb_first(pcs.pcs_managed_cores));
        }

#if RHODES_CLUSTER_POWERDOWN_WORKAROUND
        cpumap_t workaround_online_cores = 0;
        const ml_topology_info_t* topology = ml_get_topology_info();
        
        for (unsigned int i = 0; i < topology->num_clusters; i++) {
            ml_topology_cluster_t* cluster = &topology->clusters[i];
            if ((cluster->cpu_mask & online_cores_goal) != 0) {
                workaround_online_cores |= cluster->cpu_mask;
            }
        }
        online_cores_goal = workaround_online_cores;
#endif /* RHODES_CLUSTER_POWERDOWN_WORKAROUND */

        output.pcs_online_cores = online_cores_goal;
    }

    return output;
}

/*
 * Check if powered cores need update
 */
static bool
sched_needs_update_requested_powered_cores(void)
{
    if (!pcs.pcs_init_completed) {
        return false;
    }

    struct powered_cores_state requested = sched_compute_requested_powered_cores();
    struct powered_cores_state effective = pcs.pcs_effective;

    if (requested.pcs_powerdown_recommended_cores != effective.pcs_powerdown_recommended_cores ||
        requested.pcs_online_cores != effective.pcs_online_cores ||
        requested.pcs_tempdown_cores != effective.pcs_tempdown_cores) {
        return true;
    }
    
    return false;
}

/*
 * Processor exit from user with EEVDF cleanup
 */
kern_return_t
sched_processor_exit_user(processor_t processor)
{
    assert(processor);

    lck_mtx_assert(&cluster_powerdown_lock, LCK_MTX_ASSERT_OWNED);
    assert(preemption_enabled());

    kern_return_t result;
    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    if (!enable_processor_exit) {
        result = KERN_NOT_SUPPORTED;
        goto unlock;
    }

    if (bit_test(pcs.pcs_required_online_system, processor->cpu_id)) {
        result = KERN_NOT_SUPPORTED;
        goto unlock;
    }

    if (bit_test(pcs.pcs_required_online_pmgr, processor->cpu_id)) {
        result = KERN_FAILURE;
        goto unlock;
    }

    if (pcs.pcs_powerdown_suspend_count) {
        result = KERN_FAILURE;
        goto unlock;
    }

    if (!bit_test(pcs.pcs_requested_online_user, processor->cpu_id)) {
        result = KERN_NODE_DOWN;
        goto unlock;
    }

    if ((pcs.pcs_recommended_cores & pcs.pcs_effective.pcs_online_cores) == BIT(processor->cpu_id)) {
        result = KERN_RESOURCE_SHORTAGE;
        goto unlock;
    }

    result = KERN_SUCCESS;

    if (!pcs.pcs_user_online_core_control) {
        pcs.pcs_user_online_core_control = true;
    }

    bit_clear(pcs.pcs_requested_online_user, processor->cpu_id);

    /* Clean up EEVDF for this processor */
    struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
    if (rq->cfs.nr_running > 0) {
        /* Migrate all threads off this processor */
        struct rb_node *node;
        while ((node = rb_first(&rq->cfs.tasks_timeline)) != NULL) {
            struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
            thread_t thread = se->thread;
            
            rb_erase(node, &rq->cfs.tasks_timeline);
            se->on_rq = 0;
            
            /* Re-enqueue on another processor */
            thread_lock(thread);
            thread_setrun(thread, SCHED_TAILQ);
            thread_unlock(thread);
        }
        rq->cfs.nr_running = 0;
        rq->cfs.load_weight = 0;
    }

    if (sched_needs_update_requested_powered_cores()) {
        threadq = sched_update_powered_cores_drops_lock(REASON_USER, s, threadq);
    }

unlock:
    simple_unlock(&sched_available_cores_lock);
    splx(s);

    pulled_thread_queue_flush(threadq);

    return result;
}

/*
 * Processor start from user with EEVDF initialization
 */
kern_return_t
sched_processor_start_user(processor_t processor)
{
    assert(processor);

    lck_mtx_assert(&cluster_powerdown_lock, LCK_MTX_ASSERT_OWNED);
    assert(preemption_enabled());

    kern_return_t result;
    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    if (!enable_processor_exit) {
        result = KERN_NOT_SUPPORTED;
        goto unlock;
    }

    if (bit_test(pcs.pcs_required_online_system, processor->cpu_id)) {
        result = KERN_NOT_SUPPORTED;
        goto unlock;
    }

#if CONFIG_SCHED_SMT
    if ((sched_enable_smt == 0) && (processor->processor_primary != processor)) {
        result = KERN_FAILURE;
        goto unlock;
    }
#endif /* CONFIG_SCHED_SMT */

    if (pcs.pcs_powerdown_suspend_count) {
        result = KERN_FAILURE;
        goto unlock;
    }

    if (bit_test(pcs.pcs_requested_online_user, processor->cpu_id)) {
        result = KERN_FAILURE;
        goto unlock;
    }

    result = KERN_SUCCESS;

    bit_set(pcs.pcs_requested_online_user, processor->cpu_id);

    /* Initialize EEVDF for this processor if needed */
    struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
    if (!rq->cfs.processor) {
        eevdf_processor_init(processor);
    }

    if (pcs.pcs_requested_online_user == pcs.pcs_managed_cores) {
        pcs.pcs_user_online_core_control = false;
    }

    if (sched_needs_update_requested_powered_cores()) {
        threadq = sched_update_powered_cores_drops_lock(REASON_USER, s, threadq);
    }

unlock:
    simple_unlock(&sched_available_cores_lock);
    splx(s);

    pulled_thread_queue_flush(threadq);

    return result;
}

/*
 * EEVDF-aware powered cores update thread
 */
sched_cond_atomic_t sched_update_powered_cores_wakeup;
thread_t sched_update_powered_cores_thread;
processor_reason_t last_sched_update_powered_cores_continue_reason;

static void OS_NORETURN sched_update_powered_cores_continue(void *param __unused, 
    wait_result_t wr __unused);

/*
 * CPU initialization completion with EEVDF finalization
 */
void
sched_cpu_init_completed(void)
{
    static bool sched_cpu_init_completed_called = false;

    if (!os_atomic_cmpxchg(&sched_cpu_init_completed_called, false, true, relaxed)) {
        panic("sched_cpu_init_completed called twice! %d", sched_cpu_init_completed_called);
    }

    if (SCHED(cpu_init_completed) != NULL) {
        SCHED(cpu_init_completed)();
    }

    SCHED(rt_init_completed)();

    check_all_cpus_are_done_starting(PROCESSOR_FIRST_BOOT);

    lck_mtx_lock(&cluster_powerdown_lock);

    assert(sched_update_powered_cores_thread == THREAD_NULL);

    sched_cond_init(&sched_update_powered_cores_wakeup);

    kern_return_t result = kernel_thread_start_priority(
        sched_update_powered_cores_continue,
        NULL, MAXPRI_KERNEL, &sched_update_powered_cores_thread);
    if (result != KERN_SUCCESS) {
        panic("failed to create sched_update_powered_cores thread");
    }

    thread_set_thread_name(sched_update_powered_cores_thread,
        "sched_update_powered_cores");

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    assert(pcs.pcs_init_completed == false);

    pcs.pcs_managed_cores = pcs.pcs_effective.pcs_online_cores;

    assert(__builtin_popcountll(pcs.pcs_managed_cores) == machine_info.logical_cpu_max);

    pcs.pcs_requested_online_user = pcs.pcs_managed_cores;
    pcs.pcs_requested_online_clpc_system = pcs.pcs_managed_cores;
    pcs.pcs_requested_online_clpc_user = pcs.pcs_managed_cores;

    cpumap_t system_required_cores = 0;

    for (int cpu_id = 0; cpu_id < machine_info.logical_cpu_max; cpu_id++) {
        if (!ml_cpu_can_exit(cpu_id)) {
            bit_set(system_required_cores, cpu_id);
        }
        
        /* Ensure EEVDF is initialized for all CPUs */
        processor_t processor = processor_array[cpu_id];
        struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
        if (!rq->cfs.processor) {
            eevdf_processor_init(processor);
        }
    }

    pcs.pcs_required_online_system = system_required_cores;
    pcs.pcs_effective.pcs_powerdown_recommended_cores = pcs.pcs_managed_cores;

    pcs.pcs_requested = sched_compute_requested_powered_cores();

    assert(pcs.pcs_requested.pcs_powerdown_recommended_cores == pcs.pcs_managed_cores);
    assert(pcs.pcs_requested.pcs_online_cores == pcs.pcs_managed_cores);
    assert(pcs.pcs_requested.pcs_tempdown_cores == 0);

    assert(pcs.pcs_effective.pcs_powerdown_recommended_cores == pcs.pcs_managed_cores);
    assert(pcs.pcs_effective.pcs_online_cores == pcs.pcs_managed_cores);
    assert(pcs.pcs_effective.pcs_tempdown_cores == 0);

    pcs.pcs_init_completed = true;

    simple_unlock(&sched_available_cores_lock);
    splx(s);

    lck_mtx_unlock(&cluster_powerdown_lock);

    resume_cluster_powerdown();
    
    printf("EEVDF: CPU initialization complete - %d cores managed\n", 
        __builtin_popcountll(pcs.pcs_managed_cores));
}

/*
 * Sleep state query
 */
bool
sched_is_in_sleep(void)
{
    return pcs.pcs_in_kernel_sleep || pcs.pcs_wants_kernel_sleep;
}

/*
 * CPU init completion query
 */
bool
sched_is_cpu_init_completed(void)
{
    return pcs.pcs_init_completed;
}

/*
 * Powered cores update thread continuation
 */
static void OS_NORETURN
sched_update_powered_cores_continue(void *param __unused, wait_result_t wr __unused)
{
    uint64_t last_update = 0;
    uint64_t update_count = 0;
    
    sched_cond_ack(&sched_update_powered_cores_wakeup);

    while (true) {
        uint64_t start_time = mach_absolute_time();
        
        lck_mtx_lock(&cluster_powerdown_lock);

        struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

        spl_t s = splsched();
        simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

        bool needs_update = sched_needs_update_requested_powered_cores();

        if (needs_update) {
            assert(pcs.pcs_powerdown_suspend_count == 0);

            processor_reason_t reason = last_sched_update_powered_cores_continue_reason;

            threadq = sched_update_powered_cores_drops_lock(reason, s, threadq);
            
            update_count++;
            last_update = mach_absolute_time();
            
            KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_POWERED_CORES_UPDATE) | DBG_FUNC_NONE,
                update_count, last_update - start_time, reason, 0, 0);
        }

        simple_unlock(&sched_available_cores_lock);
        splx(s);

        pulled_thread_queue_flush(threadq);

        lck_mtx_unlock(&cluster_powerdown_lock);

        if (!needs_update) {
            sched_cond_wait(&sched_update_powered_cores_wakeup, THREAD_UNINT,
                sched_update_powered_cores_continue);
        }
    }
}

/*
 * EEVDF-aware perfcontrol update
 */
void
sched_perfcontrol_update_powered_cores(
    uint64_t requested_powered_cores,
    processor_reason_t reason,
    __unused uint32_t flags)
{
    assert((reason == REASON_CLPC_SYSTEM) || (reason == REASON_CLPC_USER));

#if DEVELOPMENT || DEBUG
    if (flags & (ASSERT_IN_SLEEP | ASSERT_POWERDOWN_SUSPENDED)) {
        if (flags & ASSERT_POWERDOWN_SUSPENDED) {
            assert(pcs.pcs_powerdown_suspend_count > 0);
        }
        if (flags & ASSERT_IN_SLEEP) {
            assert(pcs.pcs_sleep_override_recommended == true);
        }
        return;
    }
#endif

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    cpumap_t requested_cores = requested_powered_cores & pcs.pcs_managed_cores;

    /* Adjust based on EEVDF load */
    if (reason == REASON_CLPC_SYSTEM) {
        /* Consider EEVDF deadline pressure */
        uint64_t total_pressure = 0;
        int cpu_count = 0;
        
        cpumap_foreach(cpu_id, requested_cores) {
            processor_t processor = processor_array[cpu_id];
            struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
            if (rq->cfs.nr_running > 10) {
                /* High load - maybe keep this core */
                total_pressure += rq->cfs.nr_running;
                cpu_count++;
            }
        }
        
        if (cpu_count > 0 && total_pressure / cpu_count > 20) {
            /* Very high load - override and keep more cores */
            requested_cores = pcs.pcs_managed_cores;
        }
        
        pcs.pcs_requested_online_clpc_system = requested_cores;
    } else if (reason == REASON_CLPC_USER) {
        pcs.pcs_requested_online_clpc_user = requested_cores;
    }

    bool needs_update = sched_needs_update_requested_powered_cores();

    if (needs_update) {
        last_sched_update_powered_cores_continue_reason = reason;
    }

    simple_unlock(&sched_available_cores_lock);
    splx(s);

    if (needs_update) {
        sched_cond_signal(&sched_update_powered_cores_wakeup,
            sched_update_powered_cores_thread);
    }
}

/*
 * On-core preemption check with EEVDF
 */
bool
sched_perfcontrol_check_oncore_thread_preemption(
    __unused uint64_t flags,
    int cpu_id __assert_only)
{
    bool ret = false;
    assert(ml_get_interrupts_enabled() == false);

    processor_t processor = current_processor();
    thread_t thread = current_thread();
    assert(processor->cpu_id == cpu_id);

    thread_lock(thread);
    
    /* EEVDF-specific preemption check */
    struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
    struct eevdf_cfs_rq *cfs_rq = &rq->cfs;
    
    if (cfs_rq->nr_running > 0) {
        struct eevdf_sched_entity *next = eevdf_leftmost(cfs_rq);
        struct eevdf_sched_entity *curr = (struct eevdf_sched_entity *)thread->sched_priv;
        
        if (next && curr && next != curr) {
            if (eevdf_entity_before(next, curr)) {
                ret = true;
            }
        }
    }
    
    ast_t preempt = csw_check(thread, processor, AST_NONE);
    if (preempt != AST_NONE) {
        ret = true;
        ast_on(preempt);
        
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_ONCORE_PREEMPT), 
            thread_tid(thread), processor->cpu_id, 
            cfs_rq->nr_running, ret ? 1 : 0, 0);
    }
    
    thread_unlock(thread);

    return ret;
}

/*
 * Suspend cluster powerdown with EEVDF state preservation
 */
static void
suspend_cluster_powerdown_locked(bool for_sleep)
{
    lck_mtx_assert(&cluster_powerdown_lock, LCK_MTX_ASSERT_OWNED);
    kprintf("%s>calling sched_update_powered_cores to suspend powerdown\n", __func__);

    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    assert(pcs.pcs_powerdown_suspend_count >= 0);

    if (for_sleep) {
        assert(!pcs.pcs_wants_kernel_sleep);
        assert(!pcs.pcs_in_kernel_sleep);
        pcs.pcs_wants_kernel_sleep = true;
    }

    pcs.pcs_powerdown_suspend_count++;

    /* Save EEVDF state before powerdown */
    static struct eevdf_state_snapshot {
        uint64_t total_load;
        uint64_t deadline_misses;
        uint64_t avg_vruntime;
    } pre_suspend_state = {0};
    
    pre_suspend_state.total_load = 0;
    pre_suspend_state.deadline_misses = 0;
    pre_suspend_state.avg_vruntime = 0;
    
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
        pre_suspend_state.total_load += stats->load_sum;
        pre_suspend_state.deadline_misses += stats->deadline_misses;
    }

    if (sched_needs_update_requested_powered_cores()) {
        threadq = sched_update_powered_cores_drops_lock(REASON_SYSTEM, s, threadq);
    }

    if (for_sleep) {
        assert(pcs.pcs_wants_kernel_sleep);
        assert(!pcs.pcs_in_kernel_sleep);
        pcs.pcs_in_kernel_sleep = true;

        assert(sched_needs_update_requested_powered_cores() == false);
    }

    simple_unlock(&sched_available_cores_lock);
    splx(s);

    pulled_thread_queue_flush(threadq);

    if (pcs.pcs_init_completed) {
        check_all_cpus_are_done_starting(for_sleep ?
            PROCESSOR_BEFORE_ENTERING_SLEEP : PROCESSOR_CLUSTER_POWERDOWN_SUSPEND);
    }
    
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_CLUSTER_SUSPEND) | DBG_FUNC_NONE,
        pre_suspend_state.total_load, pre_suspend_state.deadline_misses, for_sleep, 0, 0);
}

/*
 * Resume cluster powerdown with EEVDF state restoration
 */
static void
resume_cluster_powerdown_locked(bool for_sleep)
{
    lck_mtx_assert(&cluster_powerdown_lock, LCK_MTX_ASSERT_OWNED);

    if (pcs.pcs_init_completed) {
        check_all_cpus_are_done_starting(for_sleep ?
            PROCESSOR_WAKE_FROM_SLEEP : PROCESSOR_CLUSTER_POWERDOWN_RESUME);
    }

    kprintf("%s>calling sched_update_powered_cores to resume powerdown\n", __func__);

    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    if (pcs.pcs_powerdown_suspend_count <= 0) {
        panic("resume_cluster_powerdown() called with pcs.pcs_powerdown_suspend_count=%d\n", 
            pcs.pcs_powerdown_suspend_count);
    }

    if (for_sleep) {
        assert(pcs.pcs_wants_kernel_sleep);
        assert(pcs.pcs_in_kernel_sleep);
        pcs.pcs_wants_kernel_sleep = false;
    }

    pcs.pcs_powerdown_suspend_count--;

    if (pcs.pcs_powerdown_suspend_count == 0) {
        assert(pcs.pcs_init_completed);

        pcs.pcs_requested_online_user = pcs.pcs_managed_cores;
        pcs.pcs_user_online_core_control = false;

        pcs.pcs_requested_online_clpc_user = pcs.pcs_managed_cores;
        pcs.pcs_requested_online_clpc_system = pcs.pcs_managed_cores;
        
        /* Reset EEVDF stats after resume */
        for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
            struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
            stats->deadline_misses = 0;
        }
    }

    if (sched_needs_update_requested_powered_cores()) {
        threadq = sched_update_powered_cores_drops_lock(REASON_SYSTEM, s, threadq);
    }

    if (for_sleep) {
        assert(!pcs.pcs_wants_kernel_sleep);
        assert(pcs.pcs_in_kernel_sleep);
        pcs.pcs_in_kernel_sleep = false;

        assert(sched_needs_update_requested_powered_cores() == false);
    }

    simple_unlock(&sched_available_cores_lock);
    splx(s);

    pulled_thread_queue_flush(threadq);
}

/*
 * Convert die and cluster to CPU mask
 */
static uint64_t
die_and_cluster_to_cpu_mask(
    __unused unsigned int die_id,
    __unused unsigned int die_cluster_id)
{
#if __arm__ || __arm64__
    const ml_topology_info_t* topology = ml_get_topology_info();
    unsigned int num_clusters = topology->num_clusters;
    for (unsigned int i = 0; i < num_clusters; i++) {
        ml_topology_cluster_t* cluster = &topology->clusters[i];
        if ((cluster->die_id == die_id) &&
            (cluster->die_cluster_id == die_cluster_id)) {
            return cluster->cpu_mask;
        }
    }
#endif
    return 0ull;
}

/*
 * Enable accelerator rail with EEVDF awareness
 */
void
sched_enable_acc_rail(unsigned int die_id, unsigned int die_cluster_id)
{
    uint64_t core_mask = die_and_cluster_to_cpu_mask(die_id, die_cluster_id);

    lck_mtx_lock(&cluster_powerdown_lock);

    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    if (pcs.pcs_init_completed) {
        assert3u(pcs.pcs_managed_cores & core_mask, ==, core_mask);
    }

    assert((pcs.pcs_required_online_pmgr & core_mask) == 0);

    pcs.pcs_required_online_pmgr |= core_mask;

    /* Initialize EEVDF for these cores if needed */
    cpumap_foreach(cpu_id, core_mask) {
        processor_t processor = processor_array[cpu_id];
        struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
        if (!rq->cfs.processor) {
            eevdf_processor_init(processor);
        }
    }

    if (sched_needs_update_requested_powered_cores()) {
        threadq = sched_update_powered_cores_drops_lock(REASON_PMGR_SYSTEM, s, threadq);
    }

    simple_unlock(&sched_available_cores_lock);
    splx(s);

    pulled_thread_queue_flush(threadq);

    lck_mtx_unlock(&cluster_powerdown_lock);
    
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_ACC_RAIL_ENABLE) | DBG_FUNC_NONE,
        core_mask, die_id, die_cluster_id, 0, 0);
}

/*
 * Disable accelerator rail
 */
void
sched_disable_acc_rail(unsigned int die_id, unsigned int die_cluster_id)
{
    uint64_t core_mask = die_and_cluster_to_cpu_mask(die_id, die_cluster_id);

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    assert((pcs.pcs_required_online_pmgr & core_mask) == core_mask);

    if (pcs.pcs_init_completed) {
        assert3u(pcs.pcs_managed_cores & core_mask, ==, core_mask);
    }

    pcs.pcs_required_online_pmgr &= ~core_mask;

    bool needs_update = sched_needs_update_requested_powered_cores();

    if (needs_update) {
        last_sched_update_powered_cores_continue_reason = REASON_PMGR_SYSTEM;
    }

    simple_unlock(&sched_available_cores_lock);
    splx(s);

    if (needs_update) {
        sched_cond_signal(&sched_update_powered_cores_wakeup,
            sched_update_powered_cores_thread);
    }
    
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_ACC_RAIL_DISABLE) | DBG_FUNC_NONE,
        core_mask, die_id, die_cluster_id, 0, 0);
}

/*
 * Public cluster powerdown control
 */
void
suspend_cluster_powerdown(void)
{
    lck_mtx_lock(&cluster_powerdown_lock);
    suspend_cluster_powerdown_locked(false);
    lck_mtx_unlock(&cluster_powerdown_lock);
}

void
resume_cluster_powerdown(void)
{
    lck_mtx_lock(&cluster_powerdown_lock);
    resume_cluster_powerdown_locked(false);
    lck_mtx_unlock(&cluster_powerdown_lock);

#if CONFIG_SCHED_SMT
    if (sched_enable_smt == 0) {
        enable_smt_processors(false);
    }
#endif /* CONFIG_SCHED_SMT */
}

/*
 * User-space cluster powerdown control
 */
LCK_MTX_DECLARE(user_cluster_powerdown_lock, &cluster_powerdown_grp);
static bool user_suspended_cluster_powerdown = false;

kern_return_t
suspend_cluster_powerdown_from_user(void)
{
    kern_return_t ret = KERN_FAILURE;

    lck_mtx_lock(&user_cluster_powerdown_lock);

    if (!user_suspended_cluster_powerdown) {
        suspend_cluster_powerdown();
        user_suspended_cluster_powerdown = true;
        ret = KERN_SUCCESS;
        
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_USER_CLUSTER_SUSPEND) | DBG_FUNC_NONE,
            1, 0, 0, 0, 0);
    }

    lck_mtx_unlock(&user_cluster_powerdown_lock);

    return ret;
}

kern_return_t
resume_cluster_powerdown_from_user(void)
{
    kern_return_t ret = KERN_FAILURE;

    lck_mtx_lock(&user_cluster_powerdown_lock);

    if (user_suspended_cluster_powerdown) {
        resume_cluster_powerdown();
        user_suspended_cluster_powerdown = false;
        ret = KERN_SUCCESS;
        
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_USER_CLUSTER_SUSPEND) | DBG_FUNC_NONE,
            0, 0, 0, 0, 0);
    }

    lck_mtx_unlock(&user_cluster_powerdown_lock);

    return ret;
}

int
get_cluster_powerdown_user_suspended(void)
{
    lck_mtx_lock(&user_cluster_powerdown_lock);

    int ret = (int)user_suspended_cluster_powerdown;

    lck_mtx_unlock(&user_cluster_powerdown_lock);

    return ret;
}

#if DEVELOPMENT || DEBUG
static uint64_t saved_requested_powered_cores = ALL_CORES_POWERED;

void
sched_set_powered_cores(int requested_powered_cores)
{
    processor_reason_t reason = bit_test(requested_powered_cores, 31) ? 
        REASON_CLPC_USER : REASON_CLPC_SYSTEM;
    sched_powered_cores_flags_t flags = requested_powered_cores & POWERED_CORES_OPTIONS_MASK;

    saved_requested_powered_cores = requested_powered_cores;

    requested_powered_cores = bits(requested_powered_cores, 28, 0);

    sched_perfcontrol_update_powered_cores(requested_powered_cores, reason, flags);
}

int
sched_get_powered_cores(void)
{
    return (int)saved_requested_powered_cores;
}

uint64_t
sched_sysctl_get_recommended_cores(void)
{
    return pcs.pcs_recommended_cores;
}
#endif

/*
 * Sleep override for available cores
 */
void
sched_override_available_cores_for_sleep(void)
{
    if (!pcs.pcs_init_completed) {
        panic("Attempting to sleep before all CPUS are registered");
    }

    lck_mtx_lock(&cluster_powerdown_lock);

    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    assert(pcs.pcs_sleep_override_recommended == false);

    pcs.pcs_sleep_override_recommended = true;
    sched_update_recommended_cores_locked(REASON_SYSTEM, 0, threadq);

    simple_unlock(&sched_available_cores_lock);
    splx(s);

    pulled_thread_queue_flush(threadq);

    suspend_cluster_powerdown_locked(true);
}

/*
 * Restore available cores after sleep
 */
void
sched_restore_available_cores_after_sleep(void)
{
    lck_mtx_assert(&cluster_powerdown_lock, LCK_MTX_ASSERT_OWNED);

    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);
    assert(pcs.pcs_sleep_override_recommended == true);

    pcs.pcs_sleep_override_recommended = false;
    sched_update_recommended_cores_locked(REASON_NONE, 0, threadq);

    simple_unlock(&sched_available_cores_lock);
    splx(s);

    pulled_thread_queue_flush(threadq);

    resume_cluster_powerdown_locked(true);

    lck_mtx_unlock(&cluster_powerdown_lock);

#if CONFIG_SCHED_SMT
    if (sched_enable_smt == 0) {
        enable_smt_processors(false);
    }
#endif /* CONFIG_SCHED_SMT */
}

/*
 * Pulled thread queue preparation
 */
struct pulled_thread_queue *
pulled_thread_queue_prepare(void)
{
    struct pulled_thread_queue *threadq;

    if (ml_get_interrupts_enabled() == false) {
        threadq = &current_processor()->processor_threadq_interrupt;
    } else {
        disable_preemption();
        threadq = &current_processor()->processor_threadq;
    }

    assert(threadq->ptq_queue_active == false);
    threadq->ptq_queue_active = true;

    return threadq;
}

/*
 * Pulled thread queue flush with EEVDF re-enqueue
 */
void
pulled_thread_queue_flush(struct pulled_thread_queue *threadq)
{
    assert(!preemption_enabled());

    bool in_interrupt;
    if (threadq == &current_processor()->processor_threadq_interrupt) {
        assert(ml_get_interrupts_enabled() == false);
        in_interrupt = true;
    } else {
        assert3p(threadq, ==, &current_processor()->processor_threadq);
        in_interrupt = false;
    }

    assert(threadq->ptq_queue_active == true);

    if (circle_queue_empty(&threadq->ptq_threadq) && threadq->ptq_needs_smr_cpu_down == 0) {
        threadq->ptq_queue_active = false;
        if (!in_interrupt) {
            enable_preemption();
        }
        return;
    }

    thread_t thread = THREAD_NULL;
    int flushed_threads = 0;
    uint64_t total_vruntime = 0;

#if SCHED_HYGIENE_DEBUG
    uint64_t start_time = ml_get_sched_hygiene_timebase();
#endif /* SCHED_HYGIENE_DEBUG */

    cqe_foreach_element_safe(thread, &threadq->ptq_threadq, wait_links) {
        assert_thread_magic(thread);
        circle_dequeue(&threadq->ptq_threadq, &thread->wait_links);

        spl_t s = splsched();
        thread_lock(thread);

        thread_assert_runq_null(thread);
        assert(thread->state & (TH_RUN));
        
        /* Use EEVDF-aware setrun */
        if (thread->sched_pri < BASEPRI_RTQUEUES) {
            struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
            if (se) {
                total_vruntime += se->vruntime;
            }
        }
        
        thread_setrun(thread, SCHED_TAILQ);

        thread_unlock(thread);
        splx(s);

        flushed_threads++;
    }

#if SCHED_HYGIENE_DEBUG
    uint64_t end_time = ml_get_sched_hygiene_timebase();

    if (waitq_flush_excess_threads && waitq_flush_excess_time_mt &&
        flushed_threads > waitq_flush_excess_threads &&
        (end_time - start_time) > waitq_flush_excess_time_mt) {
        
        if (ml_get_interrupts_enabled() == false) {
            ml_spin_debug_reset(current_thread());
            ml_irq_debug_abandon();
        }
        abandon_preemption_disable_measurement();

        KDBG(MACHDBG_CODE(DBG_MACH_SCHED, MACH_INT_MASKED_RESET),
            flushed_threads, end_time - start_time);
    }
#endif /* SCHED_HYGIENE_DEBUG */

    cpumap_foreach(cpu_id, threadq->ptq_needs_smr_cpu_down) {
        processor_t processor = processor_array[cpu_id];

        spl_t s = splsched();
        smr_cpu_down(processor, SMR_CPU_REASON_IGNORED);
        splx(s);
    }
    threadq->ptq_needs_smr_cpu_down = 0;

    assert(circle_queue_empty(&threadq->ptq_threadq));
    assert(threadq->ptq_queue_active == true);
    threadq->ptq_queue_active = false;

    if (!in_interrupt) {
        enable_preemption();
    }
    
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PULLED_QUEUE_FLUSH) | DBG_FUNC_NONE,
        flushed_threads, total_vruntime / (flushed_threads ? flushed_threads : 1), 0, 0, 0);
}

/*
 * Pulled thread queue enqueue
 */
void
pulled_thread_queue_enqueue(
    struct pulled_thread_queue *threadq,
    thread_t thread)
{
    assert(threadq == &current_processor()->processor_threadq ||
        threadq == &current_processor()->processor_threadq_interrupt);
    assert(threadq->ptq_queue_active == true);
    assert(!preemption_enabled());

    circle_enqueue_tail(&threadq->ptq_threadq, &thread->wait_links);
}

/*
 * Pulled thread queue needs SMR CPU down
 */
void
pulled_thread_queue_needs_smr_cpu_down(
    struct pulled_thread_queue *threadq,
    int cpu_id)
{
    assert(threadq == &current_processor()->processor_threadq ||
        threadq == &current_processor()->processor_threadq_interrupt);

    assert(threadq->ptq_queue_active == true);
    assert(!preemption_enabled());

    bit_set(threadq->ptq_needs_smr_cpu_down, cpu_id);
}

#if __arm__ || __arm64__

uint64_t perfcontrol_failsafe_maintenance_runnable_time;
uint64_t perfcontrol_failsafe_activation_time;
uint64_t perfcontrol_failsafe_deactivation_time;

#define FAILSAFE_NAME_LEN 33
char perfcontrol_failsafe_name[FAILSAFE_NAME_LEN];
int perfcontrol_failsafe_pid;
uint64_t perfcontrol_failsafe_tid;
uint64_t perfcontrol_failsafe_thread_timer_at_start;
uint64_t perfcontrol_failsafe_thread_timer_last_seen;
uint64_t perfcontrol_failsafe_recommended_at_trigger;
uint64_t perfcontrol_failsafe_eevdf_misses = 0;

/*
 * Perfcontrol update recommended cores with EEVDF failsafe
 */
void
sched_perfcontrol_update_recommended_cores_reason(
    uint64_t recommended_cores,
    processor_reason_t reason,
    __unused uint32_t flags)
{
    assert(preemption_enabled());

    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    if (reason == REASON_CLPC_SYSTEM) {
        pcs.pcs_requested_recommended_clpc_system = recommended_cores;
    } else {
        assert(reason == REASON_CLPC_USER);
        pcs.pcs_requested_recommended_clpc_user = recommended_cores;
    }

    pcs.pcs_requested_recommended_clpc = pcs.pcs_requested_recommended_clpc_system &
        pcs.pcs_requested_recommended_clpc_user;

    sysctl_sched_recommended_cores = pcs.pcs_requested_recommended_clpc;

    sched_update_recommended_cores_locked(reason, 0, threadq);

    simple_unlock(&sched_available_cores_lock);
    splx(s);

    pulled_thread_queue_flush(threadq);
}

void
sched_perfcontrol_update_recommended_cores(uint32_t recommended_cores)
{
    sched_perfcontrol_update_recommended_cores_reason(recommended_cores, REASON_CLPC_USER, 0);
}

/*
 * Consider recommended cores failsafe with EEVDF metrics
 */
void
sched_consider_recommended_cores(uint64_t ctime, thread_t cur_thread)
{
    assert(ml_get_interrupts_enabled() == false);

    if (__improbable(pcs.pcs_recommended_clpc_failsafe_active)) {
        uint64_t cur_th_time = recount_current_thread_time_mach();

        simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

        if (pcs.pcs_recommended_clpc_failsafe_active &&
            cur_thread->thread_id == perfcontrol_failsafe_tid) {
            perfcontrol_failsafe_thread_timer_last_seen = cur_th_time;
        }

        simple_unlock(&sched_available_cores_lock);
        return;
    }

    if (__probable(bit_count(pcs.pcs_requested_recommended_clpc) >= processor_count)) {
        return;
    }

    uint64_t too_long_ago = ctime - perfcontrol_failsafe_starvation_threshold;

    thread_t m_thread = sched_maintenance_thread;

    if (__probable(m_thread->last_made_runnable_time >= too_long_ago)) {
        return;
    }

    thread_lock(m_thread);

    if (thread_get_runq(m_thread) == PROCESSOR_NULL ||
        (m_thread->state & (TH_RUN | TH_WAIT)) != TH_RUN ||
        m_thread->last_made_runnable_time >= too_long_ago) {
        thread_unlock(m_thread);
        return;
    }

    uint64_t maintenance_runnable_time = m_thread->last_made_runnable_time;
    thread_unlock(m_thread);

    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    if (pcs.pcs_recommended_clpc_failsafe_active) {
        simple_unlock(&sched_available_cores_lock);
        pulled_thread_queue_flush(threadq);
        return;
    }

    /* Check EEVDF deadline misses as additional failsafe trigger */
    uint64_t total_deadline_misses = 0;
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
        total_deadline_misses += stats->deadline_misses;
    }

    KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
        MACHDBG_CODE(DBG_MACH_SCHED, MACH_REC_CORES_FAILSAFE) | DBG_FUNC_START,
        pcs.pcs_requested_recommended_clpc, maintenance_runnable_time, 
        total_deadline_misses, 0, 0);

    pcs.pcs_recommended_clpc_failsafe_active = true;
    perfcontrol_failsafe_activation_time = mach_absolute_time();
    perfcontrol_failsafe_maintenance_runnable_time = maintenance_runnable_time;
    perfcontrol_failsafe_recommended_at_trigger = pcs.pcs_requested_recommended_clpc;
    perfcontrol_failsafe_eevdf_misses = total_deadline_misses;

    task_t task = get_threadtask(cur_thread);
    perfcontrol_failsafe_pid = task_pid(task);
    strlcpy(perfcontrol_failsafe_name, proc_name_address(get_bsdtask_info(task)), 
        sizeof(perfcontrol_failsafe_name));

    perfcontrol_failsafe_tid = cur_thread->thread_id;

    uint64_t recent_computation = (ctime - cur_thread->computation_epoch) + 
        cur_thread->computation_metered;
    uint64_t last_seen = recount_current_thread_time_mach();

    perfcontrol_failsafe_thread_timer_at_start = last_seen - recent_computation;
    perfcontrol_failsafe_thread_timer_last_seen = last_seen;

    sched_update_recommended_cores_locked(REASON_SYSTEM, 0, threadq);

    simple_unlock(&sched_available_cores_lock);

    pulled_thread_queue_flush(threadq);
}

/*
 * Recommended cores maintenance with EEVDF
 */
static void
sched_recommended_cores_maintenance(void)
{
    if (__probable(!pcs.pcs_recommended_clpc_failsafe_active)) {
        return;
    }

    uint64_t ctime = mach_absolute_time();

    boolean_t print_diagnostic = FALSE;
    char p_name[FAILSAFE_NAME_LEN] = "";

    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    if (!pcs.pcs_recommended_clpc_failsafe_active) {
        goto out;
    }

    if ((ctime - perfcontrol_failsafe_activation_time) < 
        perfcontrol_failsafe_starvation_threshold) {
        goto out;
    }

    /* Check if EEVDF has stabilized */
    uint64_t current_deadline_misses = 0;
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
        current_deadline_misses += stats->deadline_misses;
    }
    
    if (current_deadline_misses > perfcontrol_failsafe_eevdf_misses * 2) {
        /* Still getting worse - keep failsafe active */
        goto out;
    }

    int pid = perfcontrol_failsafe_pid;
    uint64_t tid = perfcontrol_failsafe_tid;

    uint64_t thread_usage = perfcontrol_failsafe_thread_timer_last_seen -
        perfcontrol_failsafe_thread_timer_at_start;
    uint64_t rec_cores_before = perfcontrol_failsafe_recommended_at_trigger;
    uint64_t rec_cores_after = pcs.pcs_requested_recommended_clpc;
    uint64_t failsafe_duration = ctime - perfcontrol_failsafe_activation_time;
    strlcpy(p_name, perfcontrol_failsafe_name, sizeof(p_name));

    print_diagnostic = TRUE;

    perfcontrol_failsafe_deactivation_time = ctime;
    pcs.pcs_recommended_clpc_failsafe_active = false;

    sched_update_recommended_cores_locked(REASON_SYSTEM, 0, threadq);

    KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
        MACHDBG_CODE(DBG_MACH_SCHED, MACH_REC_CORES_FAILSAFE) | DBG_FUNC_END,
        pcs.pcs_requested_recommended_clpc, failsafe_duration, 
        current_deadline_misses, 0, 0);

out:
    simple_unlock(&sched_available_cores_lock);
    splx(s);

    pulled_thread_queue_flush(threadq);

    if (print_diagnostic) {
        uint64_t failsafe_duration_ms = 0, thread_usage_ms = 0;

        absolutetime_to_nanoseconds(failsafe_duration, &failsafe_duration_ms);
        failsafe_duration_ms = failsafe_duration_ms / NSEC_PER_MSEC;

        absolutetime_to_nanoseconds(thread_usage, &thread_usage_ms);
        thread_usage_ms = thread_usage_ms / NSEC_PER_MSEC;

        printf("recommended core failsafe kicked in for %lld ms "
            "likely due to %s[%d] thread 0x%llx spending "
            "%lld ms on cpu at realtime priority - "
            "new recommendation: 0x%llx -> 0x%llx, "
            "EEVDF deadline misses: %lld\n",
            failsafe_duration_ms, p_name, pid, tid, thread_usage_ms,
            rec_cores_before, rec_cores_after, 
            perfcontrol_failsafe_eevdf_misses);
    }
}

#endif /* __arm64__ */

/*
 * Check if all CPUs are offline
 */
bool
sched_all_cpus_offline(void)
{
    return pcs.pcs_effective.pcs_online_cores == 0;
}

/*
 * Assert not last online CPU
 */
void
sched_assert_not_last_online_cpu(__assert_only int cpu_id)
{
    assertf(pcs.pcs_effective.pcs_online_cores != BIT(cpu_id),
        "attempting to shut down the last online CPU!");
}

/*
 * Processor change mode with EEVDF updates
 */
void
sched_processor_change_mode_locked(processor_t processor, processor_mode_t pcm_mode, bool set)
{
    simple_lock_assert(&sched_available_cores_lock, LCK_ASSERT_OWNED);
    pset_assert_locked(processor->processor_set);

    switch (pcm_mode) {
    case PCM_RECOMMENDED:
        if (set) {
            assert(!processor->is_recommended);
            assert(!bit_test(pcs.pcs_recommended_cores, processor->cpu_id));

            processor->is_recommended = true;
            bit_set(pcs.pcs_recommended_cores, processor->cpu_id);

            if (processor->processor_online) {
                os_atomic_inc(&processor_avail_count_user, relaxed);
#if CONFIG_SCHED_SMT
                if (processor->processor_primary == processor) {
                    os_atomic_inc(&primary_processor_avail_count_user, relaxed);
                }
#endif /* CONFIG_SCHED_SMT */
            }
        } else {
            assert(processor->is_recommended);
            assert(bit_test(pcs.pcs_recommended_cores, processor->cpu_id));

            processor->is_recommended = false;
            bit_clear(pcs.pcs_recommended_cores, processor->cpu_id);

            if (processor->processor_online) {
                os_atomic_dec(&processor_avail_count_user, relaxed);
#if CONFIG_SCHED_SMT
                if (processor->processor_primary == processor) {
                    os_atomic_dec(&primary_processor_avail_count_user, relaxed);
                }
#endif /* CONFIG_SCHED_SMT */
            }
        }
        break;
    case PCM_TEMPORARY:
        if (set) {
            assert(!processor->shutdown_temporary);
            assert(!bit_test(pcs.pcs_effective.pcs_tempdown_cores, processor->cpu_id));

            processor->shutdown_temporary = true;
            bit_set(pcs.pcs_effective.pcs_tempdown_cores, processor->cpu_id);

            if (!processor->processor_online) {
                goto counts_up;
            }
        } else {
            assert(processor->shutdown_temporary);
            assert(bit_test(pcs.pcs_effective.pcs_tempdown_cores, processor->cpu_id));

            processor->shutdown_temporary = false;
            bit_clear(pcs.pcs_effective.pcs_tempdown_cores, processor->cpu_id);

            if (!processor->processor_online) {
                goto counts_down;
            }
        }
        break;
    case PCM_ONLINE:
        if (set) {
            assert(!processor->processor_online);
            assert(!bit_test(pcs.pcs_effective.pcs_online_cores, processor->cpu_id));
            processor->processor_online = true;
            bit_set(pcs.pcs_effective.pcs_online_cores, processor->cpu_id);

            if (!processor->shutdown_temporary) {
                goto counts_up;
            }
        } else {
            assert(processor->processor_online);
            assert(bit_test(pcs.pcs_effective.pcs_online_cores, processor->cpu_id));
            processor->processor_online = false;
            bit_clear(pcs.pcs_effective.pcs_online_cores, processor->cpu_id);

            if (!processor->shutdown_temporary) {
                goto counts_down;
            }
        }
        break;
    default:
        panic("unknown mode %d", pcm_mode);
    }

    return;

counts_up:
    ml_cpu_up_update_counts(processor->cpu_id);

    os_atomic_inc(&processor_avail_count, relaxed);

    if (processor->is_recommended) {
        os_atomic_inc(&processor_avail_count_user, relaxed);
#if CONFIG_SCHED_SMT
        if (processor->processor_primary == processor) {
            os_atomic_inc(&primary_processor_avail_count_user, relaxed);
        }
#endif /* CONFIG_SCHED_SMT */
    }
    commpage_update_active_cpus();

    return;

counts_down:
    ml_cpu_down_update_counts(processor->cpu_id);

    os_atomic_dec(&processor_avail_count, relaxed);

    if (processor->is_recommended) {
        os_atomic_dec(&processor_avail_count_user, relaxed);
#if CONFIG_SCHED_SMT
        if (processor->processor_primary == processor) {
            os_atomic_dec(&primary_processor_avail_count_user, relaxed);
        }
#endif /* CONFIG_SCHED_SMT */
    }
    commpage_update_active_cpus();

    return;
}

bool
sched_mark_processor_online(processor_t processor, __assert_only processor_reason_t reason)
{
    assert(processor == current_processor());

    processor_set_t pset = processor->processor_set;
    uint64_t now = mach_absolute_time();

    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);
    pset_lock(pset);

    /* Boot CPU coming online for the first time, either at boot or after sleep */
    bool is_first_online_processor = sched_all_cpus_offline();
    if (is_first_online_processor) {
        assert(processor == master_processor);
        
        /* Reset EEVDF global state on first processor */
        for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
            struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
            stats->deadline_misses = 0;
            stats->total_enqueues = 0;
            stats->total_dequeues = 0;
        }
    }

    assert((processor != master_processor) || (reason == REASON_SYSTEM) || support_bootcpu_shutdown);

    /* Initialize EEVDF for this processor */
    struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
    if (!rq->cfs.processor) {
        eevdf_processor_init(processor);
    } else {
        /* Reset runqueue state */
        rq->cfs.nr_running = 0;
        rq->cfs.load_weight = 0;
        rq->cfs.min_vruntime = now;
        rq->cfs.tasks_timeline = RB_ROOT;
        rq->cfs.rb_leftmost = NULL;
        rq->cfs.curr = NULL;
    }

    sched_processor_change_mode_locked(processor, PCM_ONLINE, true);

    assert(processor->processor_offline_state == PROCESSOR_OFFLINE_STARTING ||
        processor->processor_offline_state == PROCESSOR_OFFLINE_STARTED_NOT_RUNNING ||
        processor->processor_offline_state == PROCESSOR_OFFLINE_FINAL_SYSTEM_SLEEP);

    processor_update_offline_state_locked(processor, PROCESSOR_OFFLINE_STARTED_NOT_WAITED);

    ++pset->online_processor_count;
    
    /* Mark as RUNNING - not DISPATCHING to avoid idle thread issues */
    pset_update_processor_state(pset, processor, PROCESSOR_RUNNING);

    if (processor->is_recommended) {
        SCHED(pset_made_schedulable)(pset);
        
        /* Update EEVDF node maps */
        atomic_bit_set(&pset->node->pset_eevdf_online_map, pset->pset_id, memory_order_relaxed);
    }

    /* Initialize EEVDF load tracking */
    rq->last_tick = now;
    rq->cfs.last_update = now;
    rq->cfs.exec_clock = 0;

    SCHED(update_pset_load_average)(pset, 0);
    pset_update_rt_stealable_state(pset);

    pset_unlock(pset);

    smr_cpu_up(processor, SMR_CPU_REASON_OFFLINE);

    /* Log online event with EEVDF stats */
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_CPU_ONLINE) | DBG_FUNC_NONE,
        processor->cpu_id, reason, rq->cfs.nr_running, is_first_online_processor, 0);

    simple_unlock(&sched_available_cores_lock);
    splx(s);

    pulled_thread_queue_flush(threadq);

    return is_first_online_processor;
}

void
sched_mark_processor_offline(processor_t processor, bool is_final_system_sleep)
{
    assert(processor == current_processor());
    
    uint64_t offline_start = mach_absolute_time();
    struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
    
    /* Capture pre-offline EEVDF state */
    uint32_t pre_offline_threads = rq->cfs.nr_running;
    uint32_t pre_offline_load = rq->cfs.load_weight;
    uint64_t pre_offline_min_vruntime = rq->cfs.min_vruntime;

    struct pulled_thread_queue *threadq = pulled_thread_queue_prepare();

    processor_set_t pset = processor->processor_set;

    spl_t s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    assert(bit_test(pcs.pcs_effective.pcs_online_cores, processor->cpu_id));
    assert(processor->processor_offline_state == PROCESSOR_OFFLINE_BEGIN_SHUTDOWN);

    if (!is_final_system_sleep) {
        /*
         * We can't shut down the last available core!
         * Force recommend another CPU if this is the last one.
         */
        if ((pcs.pcs_effective.pcs_online_cores & pcs.pcs_recommended_cores) == BIT(processor->cpu_id)) {
            sched_update_recommended_cores_locked(REASON_SYSTEM, BIT(processor->cpu_id), threadq);
        }

        /* If we're still the last one, something went wrong. */
        if ((pcs.pcs_effective.pcs_online_cores & pcs.pcs_recommended_cores) == BIT(processor->cpu_id)) {
            panic("shutting down the last available core! online: 0x%llx rec: 0x%llx",
                pcs.pcs_effective.pcs_online_cores,
                pcs.pcs_recommended_cores);
        }
    }

    pset_lock(pset);
    assert(processor->state == PROCESSOR_RUNNING);
    assert(processor->processor_inshutdown);
    
    /* Migrate all EEVDF threads off this processor */
    if (rq->cfs.nr_running > 0) {
        struct rb_node *node;
        while ((node = rb_first(&rq->cfs.tasks_timeline)) != NULL) {
            struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
            thread_t thread = se->thread;
            
            rb_erase(node, &rq->cfs.tasks_timeline);
            se->on_rq = 0;
            se->cfs_rq = NULL;
            
            /* Re-enqueue on another processor */
            thread_lock(thread);
            thread_setrun(thread, SCHED_TAILQ | SCHED_REBALANCE);
            thread_unlock(thread);
            
            KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_THREAD_MIGRATE) | DBG_FUNC_NONE,
                thread_tid(thread), processor->cpu_id, se->vruntime, se->deadline, 0);
        }
        
        rq->cfs.nr_running = 0;
        rq->cfs.load_weight = 0;
        rq->cfs.tasks_timeline = RB_ROOT;
        rq->cfs.rb_leftmost = NULL;
    }

    pset_update_processor_state(pset, processor, PROCESSOR_PENDING_OFFLINE);
    --pset->online_processor_count;

    sched_processor_change_mode_locked(processor, PCM_ONLINE, false);

    /* Update node maps */
    atomic_bit_clear(&pset->node->pset_eevdf_online_map, pset->pset_id, memory_order_relaxed);

    if (is_final_system_sleep) {
        assert3u(pcs.pcs_effective.pcs_online_cores, ==, 0);
        assert(processor == master_processor);
        assert(sched_all_cpus_offline());

        processor_update_offline_state_locked(processor, PROCESSOR_OFFLINE_FINAL_SYSTEM_SLEEP);
    } else {
        processor_update_offline_state_locked(processor, PROCESSOR_OFFLINE_PENDING_OFFLINE);
    }

    simple_unlock(&sched_available_cores_lock);

    /* Shutdown scheduler queues */
    SCHED(processor_queue_shutdown)(processor, threadq);
    SCHED(rt_queue_shutdown)(processor, threadq);

    splx(s);

    pulled_thread_queue_flush(threadq);

    /* Log offline event */
    uint64_t offline_duration = mach_absolute_time() - offline_start;
    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_CPU_OFFLINE) | DBG_FUNC_NONE,
        processor->cpu_id, pre_offline_threads, pre_offline_load, offline_duration, is_final_system_sleep);
}

static void
sched_update_recommended_cores_locked(
    processor_reason_t reason,
    cpumap_t core_going_offline,
    struct pulled_thread_queue *threadq)
{
    simple_lock_assert(&sched_available_cores_lock, LCK_ASSERT_OWNED);

    cpumap_t recommended_cores = pcs.pcs_requested_recommended_clpc;

    if (pcs.pcs_init_completed) {
        recommended_cores &= pcs.pcs_effective.pcs_powerdown_recommended_cores;
    }

    /* EEVDF pressure-based override */
    uint64_t total_deadline_pressure = 0;
    int online_cpus = 0;
    
    cpumap_foreach(cpu_id, pcs.pcs_effective.pcs_online_cores) {
        processor_t proc = processor_array[cpu_id];
        struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, proc, proc);
        
        if (rq->cfs.nr_running > 0) {
            uint64_t pressure = 0;
            struct rb_node *node = rb_first(&rq->cfs.tasks_timeline);
            if (node) {
                struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
                if (se->deadline < mach_absolute_time()) {
                    pressure = 1000; /* High pressure */
                } else {
                    pressure = (rq->cfs.nr_running * 100) / (online_cpus + 1);
                }
            }
            total_deadline_pressure += pressure;
            online_cpus++;
        }
    }
    
    /* If deadline pressure is extremely high, override recommendations */
    if (online_cpus > 0 && (total_deadline_pressure / online_cpus) > 500) {
        recommended_cores = pcs.pcs_managed_cores;
        KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_PRESSURE_OVERRIDE) | DBG_FUNC_NONE,
            total_deadline_pressure / online_cpus, recommended_cores, 0, 0, 0);
    }

    if (pcs.pcs_sleep_override_recommended || pcs.pcs_recommended_clpc_failsafe_active) {
        KERNEL_DEBUG_CONSTANT_IST(KDEBUG_TRACE,
            MACHDBG_CODE(DBG_MACH_SCHED, MACH_REC_CORES_FAILSAFE) | DBG_FUNC_NONE,
            recommended_cores,
            sched_maintenance_thread->last_made_runnable_time, 
            total_deadline_pressure / (online_cpus ? online_cpus : 1), 0, 0);

        recommended_cores = pcs.pcs_managed_cores;
    }

    /* Ensure at least one core is recommended */
    if (bit_count(recommended_cores & pcs.pcs_effective.pcs_online_cores & ~core_going_offline) == 0) {
        int last_resort;

        if (!support_bootcpu_shutdown) {
            last_resort = master_processor->cpu_id;
        } else {
            last_resort = lsb_first(pcs.pcs_effective.pcs_online_cores & ~core_going_offline);

            if (last_resort == -1) {
                panic("%s> no last resort cpu found: 0x%llx 0x%llx",
                    __func__, pcs.pcs_effective.pcs_online_cores, core_going_offline);
            }
        }

        bit_set(recommended_cores, last_resort);
    }

    if (pcs.pcs_recommended_cores == recommended_cores) {
        return;
    }

    KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_UPDATE_REC_CORES) | DBG_FUNC_START,
        recommended_cores,
        pcs.pcs_recommended_clpc_failsafe_active, pcs.pcs_sleep_override_recommended, 
        total_deadline_pressure / (online_cpus ? online_cpus : 1));

    cpumap_t needs_exit_idle_mask = 0x0;

    /* First set recommended cores */
    foreach_node(node) {
        foreach_pset_id(pset_id, node) {
            processor_set_t pset = pset_for_id((pset_id_t)pset_id);

            cpumap_t changed_recommendations = (recommended_cores & pset->cpu_bitmask) ^ pset->recommended_bitmask;
            cpumap_t newly_recommended = changed_recommendations & recommended_cores;

            if (newly_recommended == 0) {
                continue;
            }

            pset_lock(pset);

            cpumap_foreach(cpu_id, newly_recommended) {
                processor_t processor = processor_array[cpu_id];

                sched_processor_change_mode_locked(processor, PCM_RECOMMENDED, true);
                processor->last_recommend_reason = reason;

                if (pset->recommended_bitmask == 0) {
                    atomic_bit_set(&pset->node->pset_recommended_map, pset->pset_id, memory_order_relaxed);
                }
                bit_set(pset->recommended_bitmask, processor->cpu_id);

                /* Initialize EEVDF for newly recommended processor */
                struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
                if (!rq->cfs.processor) {
                    eevdf_processor_init(processor);
                }

                if (processor->state == PROCESSOR_IDLE) {
                    if (processor != current_processor()) {
                        bit_set(needs_exit_idle_mask, processor->cpu_id);
                    }
                    pset_update_processor_state(pset, processor, PROCESSOR_DISPATCHING);
                }

                if (processor->processor_online) {
                    SCHED(pset_made_schedulable)(pset);
                }
            }
            
            SCHED(update_pset_load_average)(pset, 0);
            pset_update_rt_stealable_state(pset);

            pset_unlock(pset);

            cpumap_foreach(cpu_id, newly_recommended) {
                smr_cpu_up(processor_array[cpu_id], SMR_CPU_REASON_IGNORED);
            }
        }
    }

    /* Now shutdown not recommended cores */
    foreach_node(node) {
        foreach_pset_id(pset_id, node) {
            processor_set_t pset = pset_array[pset_id];

            cpumap_t changed_recommendations = (recommended_cores & pset->cpu_bitmask) ^ pset->recommended_bitmask;
            cpumap_t newly_unrecommended = changed_recommendations & ~recommended_cores;

            if (newly_unrecommended == 0) {
                continue;
            }

            cpumap_foreach(cpu_id, newly_unrecommended) {
                processor_t processor = processor_array[cpu_id];
                sched_ipi_type_t ipi_type = SCHED_IPI_NONE;

                pset_lock(pset);

                /* Migrate EEVDF threads off this core before derecommending */
                struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, processor, processor);
                if (rq->cfs.nr_running > 0) {
                    struct rb_node *node;
                    while ((node = rb_first(&rq->cfs.tasks_timeline)) != NULL) {
                        struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
                        thread_t thread = se->thread;
                        
                        rb_erase(node, &rq->cfs.tasks_timeline);
                        se->on_rq = 0;
                        
                        pulled_thread_queue_enqueue(threadq, thread);
                    }
                    rq->cfs.nr_running = 0;
                    rq->cfs.load_weight = 0;
                }

                sched_processor_change_mode_locked(processor, PCM_RECOMMENDED, false);

                if (reason != REASON_NONE) {
                    processor->last_derecommend_reason = reason;
                }
                bit_clear(pset->recommended_bitmask, processor->cpu_id);
                pset_update_rt_stealable_state(pset);
                
                if (pset->recommended_bitmask == 0) {
                    atomic_bit_clear(&pset->node->pset_recommended_map, pset->pset_id, memory_order_relaxed);
                }

                if ((processor->state == PROCESSOR_RUNNING) || (processor->state == PROCESSOR_DISPATCHING)) {
                    ipi_type = SCHED_IPI_IMMEDIATE;
                }
                
                SCHED(processor_queue_shutdown)(processor, threadq);
                SCHED(rt_queue_shutdown)(processor, threadq);

                if (ipi_type == SCHED_IPI_NONE) {
                    pulled_thread_queue_needs_smr_cpu_down(threadq, cpu_id);
                } else if (processor == current_processor()) {
                    ast_on(AST_PREEMPT);
                } else {
                    sched_ipi_perform(processor, ipi_type);
                }
            }
        }
    }

    if (pcs.pcs_init_completed) {
        assert3u(pcs.pcs_recommended_cores, ==, recommended_cores);
    }

#if defined(__x86_64__)
    commpage_update_active_cpus();
#endif

    /* Issue all pending IPIs */
    cpumap_foreach(cpu_id, needs_exit_idle_mask) {
        processor_t processor = processor_array[cpu_id];
        machine_signal_idle(processor);
    }

    KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_SCHED_UPDATE_REC_CORES) | DBG_FUNC_END,
        needs_exit_idle_mask, total_deadline_pressure / (online_cpus ? online_cpus : 1), 0, 0);
}

static __result_use_check struct pulled_thread_queue *
sched_update_powered_cores_drops_lock(
    processor_reason_t requested_reason,
    spl_t caller_s,
    struct pulled_thread_queue *threadq)
{
    lck_mtx_assert(&cluster_powerdown_lock, LCK_MTX_ASSERT_OWNED);
    simple_lock_assert(&sched_available_cores_lock, LCK_ASSERT_OWNED);

    assert(ml_get_interrupts_enabled() == false);
    assert(caller_s == true);

    assert_no_processors_in_transition_locked();

    pcs.pcs_in_flight_reason = requested_reason;

    struct powered_cores_state requested = sched_compute_requested_powered_cores();
    struct powered_cores_state effective = pcs.pcs_effective;

    /* Capture EEVDF state before changes */
    uint64_t pre_change_load = 0;
    uint64_t pre_change_deadline_misses = 0;
    
    cpumap_foreach(cpu_id, pcs.pcs_effective.pcs_online_cores) {
        processor_t proc = processor_array[cpu_id];
        struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
        pre_change_load += stats->load_sum;
        pre_change_deadline_misses += stats->deadline_misses;
    }

    KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_UPDATE_POWERED_CORES) | DBG_FUNC_START,
        requested.pcs_online_cores, requested_reason, pre_change_load, effective.pcs_online_cores);

    cpumap_t newly_online_cores = (requested.pcs_online_cores ^
        effective.pcs_online_cores) & requested.pcs_online_cores;

    cpumap_t newly_offline_cores = (requested.pcs_online_cores ^
        effective.pcs_online_cores) & ~requested.pcs_online_cores;

    cpumap_t newly_recommended_cores = (requested.pcs_powerdown_recommended_cores ^
        effective.pcs_powerdown_recommended_cores) & requested.pcs_powerdown_recommended_cores;

    cpumap_t newly_derecommended_cores = (requested.pcs_powerdown_recommended_cores ^
        effective.pcs_powerdown_recommended_cores) & ~requested.pcs_powerdown_recommended_cores;

    cpumap_t newly_temporary_cores = (requested.pcs_tempdown_cores ^
        effective.pcs_tempdown_cores) & requested.pcs_tempdown_cores;

    cpumap_t newly_nontemporary_cores = (requested.pcs_tempdown_cores ^
        effective.pcs_tempdown_cores) & ~requested.pcs_tempdown_cores;

    cpumap_t newly_online_and_derecommended = newly_online_cores & newly_derecommended_cores;

    pcs.pcs_requested = requested;

    pcs.pcs_effective.pcs_powerdown_recommended_cores |= newly_recommended_cores;
    pcs.pcs_effective.pcs_powerdown_recommended_cores &= ~newly_online_and_derecommended;

    sched_update_recommended_cores_locked(requested_reason, 0, threadq);

    simple_unlock(&sched_available_cores_lock);
    splx(caller_s);

    pulled_thread_queue_flush(threadq);

    assert(ml_get_interrupts_enabled() == true);
    assert(preemption_enabled());

    /* Power up new cores */
    cpumap_t started_cores = 0ull;
    foreach_node(node) {
        foreach_pset_id(pset_id, node) {
            processor_set_t pset = pset_array[pset_id];

            spl_t s = splsched();
            pset_lock(pset);
            cpumap_t pset_newly_online = newly_online_cores & pset->cpu_bitmask;

            __assert_only cpumap_t pset_online_cores =
                pset->cpu_state_map[PROCESSOR_START] |
                pset->cpu_state_map[PROCESSOR_IDLE] |
                pset->cpu_state_map[PROCESSOR_DISPATCHING] |
                pset->cpu_state_map[PROCESSOR_RUNNING];
            assert((pset_online_cores & pset_newly_online) == 0);

            pset_unlock(pset);
            splx(s);

            if (pset_newly_online == 0) {
                continue;
            }
            
            cpumap_foreach(cpu_id, pset_newly_online) {
                processor_t proc = processor_array[cpu_id];
                
                /* Initialize EEVDF for new core */
                struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, proc, proc);
                eevdf_processor_init(proc);
                
                processor_start_reason(proc, requested_reason);
                bit_set(started_cores, cpu_id);
            }
        }
    }

    /* Wait for cores to finish starting */
    cpumap_foreach(cpu_id, started_cores) {
        processor_wait_for_start(processor_array[cpu_id], PROCESSOR_POWERED_CORES_CHANGE);
    }

    /* Update temporary status */
    spl_t s;
    s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    foreach_node(node) {
        foreach_pset_id(pset_id, node) {
            processor_set_t pset = pset_array[pset_id];

            pset_lock(pset);

            cpumap_t pset_newly_temporary = newly_temporary_cores & pset->cpu_bitmask;
            cpumap_foreach(cpu_id, pset_newly_temporary) {
                sched_processor_change_mode_locked(processor_array[cpu_id], PCM_TEMPORARY, true);
            }

            pset_unlock(pset);
        }
    }

    foreach_node(node) {
        foreach_pset_id(pset_id, node) {
            processor_set_t pset = pset_array[pset_id];

            pset_lock(pset);

            cpumap_t pset_newly_nontemporary = newly_nontemporary_cores & pset->cpu_bitmask;
            cpumap_foreach(cpu_id, pset_newly_nontemporary) {
                sched_processor_change_mode_locked(processor_array[cpu_id], PCM_TEMPORARY, false);
            }

            pset_unlock(pset);
        }
    }

    simple_unlock(&sched_available_cores_lock);
    splx(s);

    /* Power down cores */
    foreach_node(node) {
        foreach_pset_id(pset_id, node) {
            processor_set_t pset = pset_array[pset_id];

            s = splsched();
            pset_lock(pset);

            cpumap_t pset_newly_offline = newly_offline_cores & pset->cpu_bitmask;
            __assert_only cpumap_t pset_powered_cores =
                pset->cpu_state_map[PROCESSOR_START] |
                pset->cpu_state_map[PROCESSOR_IDLE] |
                pset->cpu_state_map[PROCESSOR_DISPATCHING] |
                pset->cpu_state_map[PROCESSOR_RUNNING];
            assert((pset_powered_cores & pset_newly_offline) == pset_newly_offline);

            pset_unlock(pset);
            splx(s);

            if (pset_newly_offline == 0) {
                continue;
            }

            cpumap_foreach(cpu_id, pset_newly_offline) {
                processor_t proc = processor_array[cpu_id];
                
                /* Migrate any remaining EEVDF threads */
                struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, proc, proc);
                if (rq->cfs.nr_running > 0) {
                    struct rb_node *node;
                    while ((node = rb_first(&rq->cfs.tasks_timeline)) != NULL) {
                        struct eevdf_sched_entity *se = rb_entry(node, struct eevdf_sched_entity, run_node);
                        thread_t thread = se->thread;
                        
                        rb_erase(node, &rq->cfs.tasks_timeline);
                        se->on_rq = 0;
                        
                        /* Will be re-enqueued by caller */
                    }
                }
                
                processor_exit_reason(proc, requested_reason, false);
            }
        }
    }

    assert(ml_get_interrupts_enabled() == true);
    assert(preemption_enabled());

    threadq = pulled_thread_queue_prepare();

    s = splsched();
    simple_lock(&sched_available_cores_lock, LCK_GRP_NULL);

    assert(s == caller_s);

    pcs.pcs_effective.pcs_powerdown_recommended_cores &= ~newly_derecommended_cores;

    sched_update_recommended_cores_locked(requested_reason, 0, threadq);

    pcs.pcs_previous_reason = requested_reason;

    assert_no_processors_in_transition_locked();

    assert3u(pcs.pcs_requested.pcs_online_cores, ==, pcs.pcs_effective.pcs_online_cores);
    assert3u(pcs.pcs_requested.pcs_tempdown_cores, ==, pcs.pcs_effective.pcs_tempdown_cores);
    assert3u(pcs.pcs_requested.pcs_powerdown_recommended_cores, ==, pcs.pcs_effective.pcs_powerdown_recommended_cores);

    /* Calculate post-change EEVDF stats */
    uint64_t post_change_load = 0;
    cpumap_foreach(cpu_id, pcs.pcs_effective.pcs_online_cores) {
        processor_t proc = processor_array[cpu_id];
        struct eevdf_stats *stats = PERCPU_GET(eevdf_stats);
        post_change_load += stats->load_sum;
    }

    KTRC(MACHDBG_CODE(DBG_MACH_SCHED, MACH_UPDATE_POWERED_CORES) | DBG_FUNC_END,
        post_change_load, pre_change_deadline_misses, 0, 0);
        
    return threadq;
}

void
thread_set_options(uint32_t thopt)
{
    spl_t x;
    thread_t t = current_thread();

    x = splsched();
    thread_lock(t);

    t->options |= thopt;
    
    /* EEVDF-specific option handling */
    if (thopt & TH_OPT_EAGER_PREEMPT) {
        struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)t->sched_priv;
        if (se) {
            /* Reduce slice for eager preemption */
            se->slice = EEVDF_MIN_GRANULARITY_NS / 2;
            se->virtual_deadline = se->vruntime + se->slice;
            
            KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_EAGER_PREEMPT_SET) | DBG_FUNC_NONE,
                thread_tid(t), se->slice, se->virtual_deadline, 0, 0);
        }
    }

    thread_unlock(t);
    splx(x);
}

void
thread_set_pending_block_hint(thread_t thread, block_hint_t block_hint)
{
    thread->pending_block_hint = block_hint;
    
    /* EEVDF may adjust based on block hint */
    if (block_hint == kThreadWaitNone) {
        struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
        if (se && se->on_rq) {
            /* Thread is runnable, ensure it's in the right position */
            se->vruntime = se->cfs_rq->min_vruntime - EEVDF_MIN_GRANULARITY_NS;
            se->virtual_deadline = se->vruntime + se->slice;
        }
    }
}

uint32_t
qos_max_parallelism(int qos, uint64_t options)
{
    uint32_t base = SCHED(qos_max_parallelism)(qos, options);
    
    /* EEVDF may adjust parallelism based on load */
    if (options & QOS_PARALLELISM_EEVDF_AWARE) {
        uint64_t total_load = 0;
        int online_cpus = 0;
        
        for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
            processor_t proc = processor_array[cpu];
            if (proc && proc->processor_online && proc->is_recommended) {
                struct eevdf_rq *rq = PERCPU_GET_RELATIVE(eevdf_rq, proc, proc);
                total_load += rq->cfs.load_weight;
                online_cpus++;
            }
        }
        
        if (online_cpus > 0) {
            uint32_t avg_load = total_load / online_cpus;
            if (avg_load > EEVDF_WEIGHT_NICE_0 * 2) {
                /* High load - reduce parallelism to prevent thrashing */
                base = (base * 3) / 4;
            }
        }
    }
    
    return base;
}

uint32_t
sched_qos_max_parallelism(__unused int qos, uint64_t options)
{
    host_basic_info_data_t hinfo;
    mach_msg_type_number_t count = HOST_BASIC_INFO_COUNT;

    assert((options & QOS_PARALLELISM_CLUSTER_SHARED_RESOURCE) == 0);

    __assert_only kern_return_t kret = host_info(host_self(), HOST_BASIC_INFO,
        (host_info_t)&hinfo, &count);
    assert(kret == KERN_SUCCESS);

    uint32_t base_count;
    if (options & QOS_PARALLELISM_COUNT_LOGICAL) {
        base_count = hinfo.logical_cpu;
    } else {
        base_count = hinfo.physical_cpu;
    }
    
    /* EEVDF-aware adjustment */
    if (options & QOS_PARALLELISM_EEVDF_FAIR) {
        /* Ensure fair distribution across EEVDF domains */
        uint32_t eevdf_domains = 0;
        for (pset_node_t node = &pset_node0; node != NULL; node = node->node_list) {
            if (atomic_load(&node->pset_eevdf_online_map)) {
                eevdf_domains++;
            }
        }
        if (eevdf_domains > 0) {
            base_count = (base_count + eevdf_domains - 1) / eevdf_domains;
        }
    }
    
    return base_count;
}

int sched_allow_NO_SMT_threads = 1;

#if CONFIG_SCHED_SMT
bool
thread_no_smt(thread_t thread)
{
    bool no_smt = sched_allow_NO_SMT_threads &&
           (thread->bound_processor == PROCESSOR_NULL) &&
           ((thread->sched_flags & TH_SFLAG_NO_SMT) || 
            (get_threadtask(thread)->t_flags & TF_NO_SMT));
    
    if (no_smt) {
        struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
        if (se && se->on_rq) {
            /* NO_SMT threads get larger slices on primary cores */
            se->slice = EEVDF_MIN_GRANULARITY_NS * 2;
            se->virtual_deadline = se->vruntime + se->slice;
        }
    }
    
    return no_smt;
}

bool
processor_active_thread_no_smt(processor_t processor)
{
    bool no_smt = sched_allow_NO_SMT_threads && 
                  !processor->current_is_bound && 
                  processor->current_is_NO_SMT;
    
    if (no_smt && processor->processor_primary != processor) {
        /* Force NO_SMT threads off secondary cores */
        ast_on(AST_PREEMPT);
    }
    
    return no_smt;
}
#endif /* CONFIG_SCHED_SMT */

#if CONFIG_SCHED_SMT
bool
thread_get_no_smt(void)
{
    return current_thread()->sched_flags & TH_SFLAG_NO_SMT;
}

void
task_set_no_smt(task_t task)
{
    if (!system_is_SMT) {
        return;
    }

    if (task == TASK_NULL) {
        task = current_task();
    }

    task_lock(task);
    task->t_flags |= TF_NO_SMT;
    task_unlock(task);
    
    /* Apply to all threads in task */
    thread_t thread;
    queue_iterate(&task->threads, thread, task_threads) {
        spl_t s = splsched();
        thread_lock(thread);
        thread->sched_flags |= TH_SFLAG_NO_SMT;
        
        /* Update EEVDF slice for NO_SMT */
        struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
        if (se) {
            se->slice = EEVDF_MIN_GRANULARITY_NS * 2;
            if (se->on_rq) {
                struct eevdf_cfs_rq *cfs_rq = se->cfs_rq;
                rb_erase(&se->run_node, &cfs_rq->tasks_timeline);
                se->virtual_deadline = se->vruntime + se->slice;
                __eevdf_add_entity(cfs_rq, se);
            }
        }
        
        thread_unlock(thread);
        splx(s);
    }
}

#if DEBUG || DEVELOPMENT
void
sysctl_task_set_no_smt(char no_smt)
{
    if (!system_is_SMT) {
        return;
    }

    task_t task = current_task();

    task_lock(task);
    if (no_smt == '1') {
        task->t_flags |= TF_NO_SMT;
        task_unlock(task);
        task_set_no_smt(task);
    } else {
        task->t_flags &= ~TF_NO_SMT;
        task_unlock(task);
        
        /* Revert EEVDF slices */
        thread_t thread;
        queue_iterate(&task->threads, thread, task_threads) {
            spl_t s = splsched();
            thread_lock(thread);
            thread->sched_flags &= ~TH_SFLAG_NO_SMT;
            
            struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
            if (se) {
                se->slice = EEVDF_MIN_GRANULARITY_NS;
                if (se->load_weight < EEVDF_WEIGHT_NICE_0) {
                    se->slice = (se->slice * EEVDF_WEIGHT_NICE_0) / se->load_weight;
                }
                if (se->on_rq) {
                    struct eevdf_cfs_rq *cfs_rq = se->cfs_rq;
                    rb_erase(&se->run_node, &cfs_rq->tasks_timeline);
                    se->virtual_deadline = se->vruntime + se->slice;
                    __eevdf_add_entity(cfs_rq, se);
                }
            }
            
            thread_unlock(thread);
            splx(s);
        }
    }
}

char
sysctl_task_get_no_smt(void)
{
    task_t task = current_task();

    if (task->t_flags & TF_NO_SMT) {
        return '1';
    }
    return '0';
}
#endif /* DEVELOPMENT || DEBUG */

#else /* CONFIG_SCHED_SMT */

void
task_set_no_smt(__unused task_t task)
{
    return;
}

#if DEBUG || DEVELOPMENT
void
sysctl_task_set_no_smt(__unused char no_smt)
{
    return;
}

char
sysctl_task_get_no_smt(void)
{
    return '1';
}
#endif /* DEBUG || DEVELOPMENT */
#endif /* CONFIG_SCHED_SMT */

#if DEVELOPMENT || DEBUG

int32_t
sysctl_get_bound_cpuid(void)
{
    int32_t cpuid = -1;
    thread_t self = current_thread();

    processor_t processor = self->bound_processor;
    if (processor == NULL) {
        cpuid = -1;
    } else {
        cpuid = processor->cpu_id;
    }

    return cpuid;
}

kern_return_t
sysctl_thread_bind_cpuid(int32_t cpuid)
{
    processor_t processor = PROCESSOR_NULL;

    if (cpuid == -1) {
        goto unbind;
    }

    if (cpuid < 0 || cpuid >= MAX_SCHED_CPUS) {
        return KERN_INVALID_VALUE;
    }

    processor = processor_array[cpuid];
    if (processor == PROCESSOR_NULL) {
        return KERN_INVALID_VALUE;
    }

unbind:
    thread_bind(processor);
    thread_block(THREAD_CONTINUE_NULL);
    
    /* Log EEVDF state after binding change */
    thread_t self = current_thread();
    struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)self->sched_priv;
    if (se) {
        printf("EEVDF: Thread bound to CPU %d - vruntime: %llu, deadline: %llu\n",
            cpuid, se->vruntime, se->deadline);
    }
    
    return KERN_SUCCESS;
}

#if __AMP__

static char
pset_cluster_type_to_name_char(pset_cluster_type_t pset_type)
{
    switch (pset_type) {
    case PSET_AMP_E:
        return 'E';
    case PSET_AMP_P:
        return 'P';
    default:
        panic("Unexpected AMP pset cluster type %d", pset_type);
    }
}

#endif /* __AMP__ */

char
sysctl_get_task_cluster_type(void)
{
#if __AMP__
    task_t task = current_task();
    processor_set_t pset_hint = task->pset_hint;

    if (!pset_hint) {
        return '0';
    }
    return pset_cluster_type_to_name_char(pset_hint->pset_cluster_type);
#else /* !__AMP__ */
    return '0';
#endif /* __AMP__ */
}

#if __AMP__
char
sysctl_get_bound_cluster_type(void)
{
    thread_t self = current_thread();

    if (self->th_bound_cluster_id == THREAD_BOUND_CLUSTER_NONE) {
        return '0';
    }
    pset_cluster_type_t pset_type = pset_array[self->th_bound_cluster_id]->pset_cluster_type;
    return pset_cluster_type_to_name_char(pset_type);
}

static processor_set_t
find_pset_of_type(pset_cluster_type_t t)
{
    for (pset_node_t node = &pset_node0; node != NULL; node = node->node_list) {
        if (node->pset_cluster_type != t) {
            continue;
        }

        processor_set_t pset = PROCESSOR_SET_NULL;
        for (int pset_id = lsb_first(node->pset_map); pset_id >= 0; 
             pset_id = lsb_next(node->pset_map, pset_id)) {
            pset = pset_array[pset_id];
            if (pset_is_recommended(pset)) {
                assert(pset->pset_cluster_type == t);
                return pset;
            }
        }
        return pset;
    }

    return PROCESSOR_SET_NULL;
}
#endif /* __AMP__ */

kern_return_t
sysctl_task_set_cluster_type(char cluster_type)
{
#if __AMP__
    kern_return_t kr;
    task_t task = current_task();
    pset_cluster_type_t pset_cluster_type;
    
    kr = pset_cluster_type_from_name_char(cluster_type, &pset_cluster_type);
    if (kr == KERN_SUCCESS) {
        processor_set_t pset_hint = find_pset_of_type(pset_cluster_type);
        if (pset_hint) {
            task_lock(task);
            task->t_flags |= TF_USE_PSET_HINT_CLUSTER_TYPE;
            task->pset_hint = pset_hint;
            task_unlock(task);

            /* Update EEVDF for all threads in task */
            thread_t thread;
            queue_iterate(&task->threads, thread, task_threads) {
                spl_t s = splsched();
                thread_lock(thread);
                
                struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
                if (se) {
                    /* Adjust vruntime based on cluster type */
                    if (pset_cluster_type == PSET_AMP_E) {
                        se->vruntime -= EEVDF_MIN_GRANULARITY_NS;
                    } else if (pset_cluster_type == PSET_AMP_P) {
                        se->vruntime += EEVDF_MIN_GRANULARITY_NS / 2;
                    }
                    
                    KERNEL_DEBUG_CONSTANT(MACHDBG_CODE(DBG_MACH_SCHED, MACH_CLUSTER_SET) | DBG_FUNC_NONE,
                        thread_tid(thread), pset_cluster_type, se->vruntime, 0, 0);
                }
                
                thread_unlock(thread);
                splx(s);
            }

            thread_block(THREAD_CONTINUE_NULL);
            return KERN_SUCCESS;
        }
    }
    return KERN_INVALID_ARGUMENT;
#else
    (void)cluster_type;
    return KERN_SUCCESS;
#endif
}

kern_return_t
sysctl_clutch_thread_group_cpu_time_for_thread(thread_t thread,
    int sched_bucket, uint64_t *cpu_stats)
{
#if CONFIG_SCHED_CLUTCH
    kern_return_t kr = sched_clutch_thread_group_cpu_time_for_thread(thread, sched_bucket, cpu_stats);
    
    /* Add EEVDF-specific stats */
    struct eevdf_sched_entity *se = (struct eevdf_sched_entity *)thread->sched_priv;
    if (se && kr == KERN_SUCCESS) {
        cpu_stats[5] = se->vruntime;          /* Current vruntime */
        cpu_stats[6] = se->virtual_deadline;  /* Virtual deadline */
        cpu_stats[7] = se->slice;              /* Time slice */
        cpu_stats[8] = se->load_weight;        /* Load weight */
    }
    
    return kr;
#else /* !CONFIG_SCHED_CLUTCH */
    (void)thread;
    (void)sched_bucket;
    (void)cpu_stats;
    return KERN_NOT_SUPPORTED;
#endif /* !CONFIG_SCHED_CLUTCH */
}

#endif /* DEVELOPMENT || DEBUG */
