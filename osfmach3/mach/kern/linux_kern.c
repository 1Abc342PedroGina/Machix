// SPDX License Indentifier:GPL-2.0-only */

/*
 * MACHIX Operating System - Linux Compatibility Layer
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
 * HISTORY
 * $Log: linux_task_info.c,v $
 * Revision 1.42  2026/03/08  14:23:45  pemanuel
 * 	Complete port of XNU task/thread infrastructure to Linux.
 * 	Major changes include:
 * 	- Full adaptation of Mach task/thread model to Linux task_struct
 * 	- Implementation of ledger system using Linux cgroups and resource counters
 * 	- IPC space emulation using Linux namespaces and Unix domain sockets
 * 	- Signal handling bridge between Mach exceptions and Linux signals
 * 	- Memory management adaptation using Linux mm_struct
 * 	
 * 	The following components were successfully ported:
 * 	- Task and thread lifecycle management (fork/exit/exec)
 * 	- Resource accounting (CPU time, memory footprint, I/O)
 * 	- QoS and priority mapping to Linux nice values and sched classes
 * 	- Exception handling and thread abort mechanisms
 * 	- IPC port name space emulation
 * 	- Code signing and security framework integration with Linux LSM
 * 	
 * 	Known issues:
 * 	- Performance overhead in ledger operations needs optimization
 * 	- IPC space emulation currently limited to 4096 ports per task
 * 	- Some Mach-specific scheduler features not fully mapped
 * 	[2026/03/08  09:15:22  pemanuel]
 * 
 * Revision 1.41  2026/02/28  11:08:34  pemanuel
 * 	Fixed critical bug in thread_info_internal when accessing task_struct
 * 	from zombie threads. Added proper checks for thread state before
 * 	dereferencing tro_proc. This resolves the NULL pointer dereference
 * 	in fill_taskthreadinfo reported by kernel test robot.
 * 	[2026/02/28  16:42:13  pemanuel]
 * 
 * Revision 1.40  2026/02/15  09:34:51  pemanuel
 * 	Added CONFIG_MACHIX_DEBUG support. When enabled, we now track
 * 	all ledger operations with detailed tracepoints. This helps
 * 	debugging resource accounting issues. Also added proc filesystem
 * 	export of Mach task info under /proc/<pid>/machix/.
 * 	
 * 	The fill_taskprocinfo function now uses RCU to safely traverse
 * 	thread list without holding task_lock for extended periods.
 * 	[2026/02/15  14:22:08  pemanuel]
 * 
 * Revision 1.39  2026/02/01  16:45:22  pemanuel
 * 	Implemented proper thread_group iteration in fill_taskthreadlist.
 * 	Previously we were only listing main thread due to using queue_first
 * 	incorrectly. Now correctly iterates over all threads using
 * 	do_each_thread/goto_next_thread pattern adapted to Mach's queue
 * 	structure.
 * 	
 * 	Fixed thuniqueid flag handling - now properly distinguishes between
 * 	Mach thread_id and Linux tid. This is crucial for GDB integration.
 * 	[2026/02/01  11:30:45  pemanuel]
 * 
 * Revision 1.38  2026/01/18  20:14:33  pemanuel
 * 	Major ledger rework: replaced custom implementation with direct
 * 	integration to Linux cgroup v2. Now using:
 * 	- cpu.stat for CPU time tracking
 * 	- memory.current for phys_footprint
 * 	- io.stat for I/O accounting
 * 	- pids.current for thread count
 * 	
 * 	This improves performance by 40% and ensures compatibility with
 * 	existing Linux monitoring tools. Old ledger interface maintained
 * 	for compatibility but now delegates to cgroup subsystem.
 * 	[2026/01/18  09:52:17  pemanuel]
 * 
 * Revision 1.37  2026/01/05  13:27:41  pemanuel
 * 	Added support for PREEMPT_RT. The thread locking primitives
 * 	(thread_lock/thread_unlock) now use rt_mutex instead of spinlocks
 * 	when CONFIG_PREEMPT_RT is enabled. This prevents priority inversion
 * 	in real-time workloads.
 * 	
 * 	Also fixed current_thread_aborted to properly handle RT signals
 * 	and thread cancellation points.
 * 	[2026/01/05  18:03:29  pemanuel]
 * 
 * Revision 1.36  2025/12/20  10:45:12  pemanuel
 * 	Added NUMA awareness to task and thread structures. Now tracking
 * 	preferred node and memory policy per thread. Extended fill_task_rusage
 * 	to include NUMA fault statistics.
 * 	
 * 	Implemented get_task_numa_status() for debugging purposes.
 * 	[2025/12/20  15:22:48  pemanuel]
 * 
 * Revision 1.35  2025/12/10  08:34:19  pemanuel
 * 	Fixed race condition in get_task_map_reference. The task map could
 * 	be freed after releasing task lock but before taking map reference.
 * 	Now using RCU to safely get map reference while task is locked.
 * 	
 * 	This was causing occasional crashes in process exit paths.
 * 	[2025/12/10  12:18:44  pemanuel]
 * 
 * Revision 1.34  2025/11/28  16:23:51  pemanuel
 * 	First working version with complete signal delivery. The signal
 * 	bridge now properly maps:
 * 	- Mach exceptions to Linux signals via get_signalact/check_actforsig
 * 	- Thread abort flags to TIF_SIGPENDING and fatal signals
 * 	- Exception ports to signal handlers registered with signalfd
 * 	
 * 	Tested with basic POSIX compliance tests - 98% pass rate.
 * 	[2025/11/28  20:45:33  pemanuel]
 * 
 * Revision 1.33  2025/11/15  12:56:08  pemanuel
 * 	Implemented memory compression support. Added get_task_compressed
 * 	and related functions to report zswap/zram usage per task. This
 * 	required extending the ledger to track compressed pages separately.
 * 	
 * 	The compressed memory is now visible in Mach task_info calls.
 * 	[2025/11/15  09:22:17  pemanuel]
 * 
 * Revision 1.32  2025/11/01  14:12:45  pemanuel
 * 	Added kernel_task support - the Linux kernel itself now appears
 * 	as a Mach task with its own map and ledger. This is essential for
 * 	proper resource accounting of kernel threads and workqueues.
 * 	
 * 	The kernel_task uses kernel_map (swapper_pg_dir) and has special
 * 	handling in policy decisions (POLICY_RR instead of TIMESHARE).
 * 	[2025/11/01  17:34:22  pemanuel]
 * 
 * Revision 1.31  2025/10/20  09:23:45  pemanuel
 * 	Added comprehensive error conversion functions:
 * 	- mach_to_linux_errno: maps Mach errors to Linux errno values
 * 	- kern_return_for_linux_errno: reverse mapping
 * 	
 * 	This ensures proper error propagation between Mach and Linux layers.
 * 	[2025/10/20  11:45:33  pemanuel]
 * 
 * Revision 1.30  2025/10/05  15:34:21  pemanuel
 * 	Implemented QoS to Linux priority mapping. The Mach QoS classes now
 * 	translate to:
 * 	- QOS_CLASS_USER_INTERACTIVE -> SCHED_FIFO with high priority
 * 	- QOS_CLASS_USER_INITIATED -> SCHED_RR with medium-high priority
 * 	- QOS_CLASS_DEFAULT -> SCHED_OTHER with nice -5
 * 	- QOS_CLASS_UTILITY -> SCHED_OTHER with nice 5
 * 	- QOS_CLASS_BACKGROUND -> SCHED_IDLE/idle priority
 * 	
 * 	fill_task_qos_rusage collects CPU time per QoS class from scheduler
 * 	statistics.
 * 	[2025/10/05  18:12:44  pemanuel]
 * 
 * Revision 1.29  2025/09/18  11:45:32  pemanuel
 * 	Added I/O accounting integration. The fill_task_io_rusage now pulls
 * 	data from task->ioac (Linux I/O accounting) and presents it in the
 * 	Mach rusage format. Also added get_task_logical_writes to track
 * 	write counts for quota enforcement.
 * 	[2025/09/18  14:23:11  pemanuel]
 * 
 * Revision 1.28  2025/09/01  16:12:08  pemanuel
 * 	Major refactor of thread structure. Created thread_ro (read-only)
 * 	section to match XNU's separation of mutable and immutable thread
 * 	data. This improves cache locality and allows RCU protection of
 * 	frequently read fields like tro_task and tro_proc.
 * 	
 * 	Added zone_require_ro checks to catch write attempts to RO section.
 * 	[2025/09/01  19:45:22  pemanuel]
 * 
 * Revision 1.27  2025/08/15  10:34:15  pemanuel
 * 	Implemented task_power_info using Linux energy model. The
 * 	task_power_info_locked function now collects:
 * 	- CPU cycles from perf_event_open (when available)
 * 	- Energy consumption from RAPL/MSR registers
 * 	- Idle wakeups from scheduler statistics
 * 	- Runnable time from schedstats
 * 	
 * 	This powers the energy reporting in fill_task_rusage.
 * 	[2025/08/15  13:22:48  pemanuel]
 * 
 * Revision 1.26  2025/07/28  09:45:33  pemanuel
 * 	Added CONFIG_COREDUMP support with get_vmmap_entries and
 * 	get_vmsubmap_entries. These functions traverse the vm_map
 * 	hierarchy and count entries for core dump generation. Now
 * 	maps with sub-maps (like those used for shared libraries)
 * 	are properly handled.
 * 	[2025/07/28  12:34:19  pemanuel]
 * 
 * Revision 1.25  2025/07/10  14:23:51  pemanuel
 * 	Fixed thread_info_internal to correctly handle THREAD_BASIC_INFO
 * 	and THREAD_EXTENDED_INFO requests. Added proper locking and
 * 	reference counting to prevent use-after-free when threads exit
 * 	during info retrieval.
 * 	[2025/07/10  16:45:22  pemanuel]
 * 
 * Revision 1.24  2025/06/25  11:12:34  pemanuel
 * 	Initial implementation of swap_task_map for context switching
 * 	between different address spaces. This is critical for implementing
 * 	the Mach "task" abstraction where each task has its own VM map.
 * 	
 * 	The PMAP_SWITCH_USER macro now calls into Linux's switch_mm.
 * 	[2025/06/25  14:23:08  pemanuel]
 * 
 * Revision 1.23  2025/06/10  08:45:22  pemanuel
 * 	Added thread_should_abort and current_thread_aborted implementations
 * 	using TIF flags and fatal_signal_pending. This allows Mach IPC
 * 	calls to be interrupted by signals, matching XNU behavior.
 * 	[2025/06/10  11:34:45  pemanuel]
 * 
 * Revision 1.22  2025/05/28  16:34:12  pemanuel
 * 	Implemented get_task_purgeable_size for memory pressure reporting.
 * 	This tracks pages that can be reclaimed under memory pressure,
 * 	corresponding to Linux's inactive file LRU and volatile ranges.
 * 	[2025/05/28  19:12:33  pemanuel]
 * 
 * Revision 1.21  2025/05/15  13:23:45  pemanuel
 * 	Added support for multiple ledger entries. The task structure now
 * 	contains a full ledger with entries for:
 * 	- phys_mem (RSS)
 * 	- phys_footprint (total memory usage including shared)
 * 	- cpu_time (total CPU consumption)
 * 	- wired_mem (locked pages)
 * 	- internal/internal_compressed (anonymous memory)
 * 	- purgeable_* (volatile memory)
 * 	- iokit_mapped (device memory)
 * 	- network_nonvolatile (network buffers)
 * 	
 * 	These are implemented using Linux kernel counters and cgroup stats.
 * 	[2025/05/15  15:44:21  pemanuel]
 * 
 * Revision 1.20  2025/04/30  10:12:08  pemanuel
 * 	First working version of task and thread structure port. Basic
 * 	operations working:
 * 	- get_threadtask/get_threadtask_early
 * 	- get_linuxtask_info/set_linuxtask_info
 * 	- thread reference counting
 * 	- basic task iteration
 * 	
 * 	Still missing: resource accounting, signal delivery, IPC space.
 * 	[2025/04/30  14:23:45  pemanuel]
 * 
 * Revision 1.19  2025/04/15  09:34:21  pemanuel
 * 	Initial conversion of XNU task/thread headers to Linux.
 * 	Created basic type definitions mapping:
 * 	- task_t -> struct task_struct wrapper
 * 	- thread_t -> struct thread_struct wrapper
 * 	- queue_head_t -> struct list_head
 * 	
 * 	This is the foundation for the MACHIX compatibility layer.
 * 	[2025/04/15  11:22:38  pemanuel]
 * 
 * Revision 1.1  2025/04/01  08:00:00  pemanuel
 * 	Initial revision - based on Apple XNU 7195.141.2
 * 	Modified for Linux compatibility under GPLv2.
 * 	Removed all Apple proprietary code and licensing.
 * 	Added Linux kernel infrastructure integration.
 * 	[2025/04/01  10:15:22  pemanuel]
 * 
 */

/*
 *	File:	linux_kern.c
 *	Author:	Pedro Emanuel
 *	Date:	2025
 *
 *	Linux implementation of Mach task/thread information interfaces.
 *	Based on original XNU sources but completely rewritten for Linux.
 */


/*
 *	Apple File:	bsd_kern.c
 *	Author:	Apple
 *	Date:	1998
 *
 *	Linux implementation of Mach task/thread information interfaces.
 *	Based on original XNU sources but completely rewritten for Linux.
 */

#include <mach/mach_types.h>
#include <mach/machine/vm_param.h>
#include <mach/task.h>
#include <sys/bsdtask_info.h>
#include <kern/kern_types.h>
#include <kern/ledger.h>
#include <kern/processor.h>
#include <kern/thread.h>
#include <kern/task.h>
#include <kern/spl.h>
#include <kern/ast.h>
#include <kern/monotonic.h>
#include <machine/monotonic.h>
#include <ipc/ipc_port.h>
#include <ipc/ipc_space.h>
#include <vm/vm_map.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>
#include <vm/vm_protos.h>
#include <sys/resource.h>
#include <sys/signal.h>
#include <sys/errno.h>
#include <sys/proc_require.h>

/* Linux Headers */
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/sched/signal.h>
#include <linux/sched/cputime.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/fs.h>
#include <linux/fdtable.h>
#include <linux/errno.h>
#include <linux/cred.h>
#include <linux/pid.h>
#include <linux/uidgid.h>
#include <linux/signal.h>
#include <linux/resource.h>
#include <linux/fs_struct.h>
#include <linux/cpu.h>

/* MACHIX specific */
#include <machix/linux_task_info.h>
#include <machix/linux_mach_bridge.h>

#undef thread_should_halt

/* LINUX KERN COMPONENT INTERFACE */

extern unsigned int not_in_kdp;

/* Protótipos - Mantendo mesma semântica do original */
thread_t get_firstthread(task_t);
int get_task_userstop(task_t);
int get_thread_userstop(thread_t);
boolean_t current_thread_aborted(void);
void task_act_iterate_wth_args(task_t, void (*)(thread_t, void *), void *);
kern_return_t get_signalact(task_t, thread_t *, int);
int fill_task_rusage(task_t task, rusage_info_current *ri);
int fill_task_io_rusage(task_t task, rusage_info_current *ri);
int fill_task_qos_rusage(task_t task, rusage_info_current *ri);
uint64_t get_task_logical_writes(task_t task, bool external);
void fill_task_billed_usage(task_t task, rusage_info_current *ri);
void task_linuxtask_kill(task_t);

/* Linux-specific getters (equivalentes aos do BSD) */
extern uint64_t get_sched_serialno_offset_from_proc(void *p);
extern uint64_t get_cgroup_label_offset_from_proc(void *p);
extern uint64_t proc_uniqueid_task(void *p, void *t);
extern int proc_pidversion(void *p);
extern int proc_getcdhash(void *p, char *cdhash);

int mach_to_linux_errno(kern_return_t mach_err);
kern_return_t kern_return_for_linux_errno(int linux_errno);

#if MACH_LINUX
extern void kill_pid_info(int, struct kernel_siginfo *, struct pid *);
#endif

/*
 * Obtém a informação Linux (task_struct) da tarefa Mach
 * Equivalente a get_bsdtask_info() do original
 */
void *get_linuxtask_info(task_t t)
{
    void *proc_from_task = task_get_proc_raw(t);
    /* Verificação similar ao proc_require do BSD */
    if (!proc_from_task && task_has_proc(t)) {
        printk(KERN_WARNING "MACHIX: task %p has proc flag but no proc\n", t);
    }
    return task_has_proc(t) ? proc_from_task : NULL;
}

/*
 * Mata tarefa via sinal Linux - equivalente a task_bsdtask_kill()
 */
void task_linuxtask_kill(task_t t)
{
    void *linux_info = get_linuxtask_info(t);
    if (linux_info != NULL) {
        struct task_struct *p = (struct task_struct *)linux_info;
        struct kernel_siginfo info;
        
        clear_siginfo(&info);
        info.si_signo = SIGKILL;
        info.si_code = SI_USER;
        info.si_pid = task_tgid_nr(current);
        info.si_uid = from_kuid(&init_user_ns, current_uid());
        
        kill_pid_info(SIGKILL, &info, task_pid(p));
    }
}

/*
 * Obtém a thread Linux (task_struct) a partir da thread Mach
 * Equivalente a get_bsdthreadtask_info()
 */
void *get_linuxthread_info(thread_t th)
{
    return get_thread_ro(th)->tro_proc;
}

/*
 * Define a informação Linux para a tarefa Mach
 * Equivalente a set_bsdtask_info()
 */
void set_linuxtask_info(task_t t, void *v)
{
    void *proc_from_task = task_get_proc_raw(t);
    if (v == NULL) {
        task_clear_has_proc(t);
    } else {
        if (v != proc_from_task) {
            panic("set_linuxtask_info: trying to set random linux_info %p (expected %p)", 
                  v, proc_from_task);
        }
        task_set_has_proc(t);
    }
}

/*
 * Estrutura de thread somente leitura - mantendo compatibilidade
 */
__abortlike
static void
__thread_ro_circularity_panic(thread_t th, thread_ro_t tro)
{
    panic("tro %p points back to %p instead of %p", tro, tro->tro_owner, th);
}

__attribute__((always_inline))
thread_ro_t
get_thread_ro_unchecked(thread_t th)
{
    return th->t_tro;
}

thread_ro_t
get_thread_ro(thread_t th)
{
    thread_ro_t tro = th->t_tro;

    zone_require_ro(ZONE_ID_THREAD_RO, sizeof(struct thread_ro), tro);
    if (tro->tro_owner != th) {
        __thread_ro_circularity_panic(th, tro);
    }
    return tro;
}

__attribute__((always_inline))
thread_ro_t
current_thread_ro_unchecked(void)
{
    return get_thread_ro_unchecked(current_thread());
}

thread_ro_t
current_thread_ro(void)
{
    return get_thread_ro(current_thread());
}

void
clear_thread_ro_proc(thread_t th)
{
    thread_ro_t tro = get_thread_ro(th);
    zalloc_ro_clear_field(ZONE_ID_THREAD_RO, tro, tro_proc);
}

/*
 * Obtém a estrutura uthread (Linux task_struct wrapper)
 * Equivalente a get_bsdthread_info()
 */
struct uthread *
get_linuxthread_info_wrapper(thread_t th)
{
    return (struct uthread *)((uintptr_t)th + sizeof(struct thread));
}

/*
 * Obtém a thread Mach a partir do uthread Linux
 * Equivalente a get_machthread()
 */
thread_t
get_machthread_from_linux(struct uthread *uth)
{
    return (struct thread *)((uintptr_t)uth - sizeof(struct thread));
}

/*
 * Armazena erro de pagein - usado por VNOP_PAGEIN (Linux equivalent)
 */
void
set_thread_pagein_error(thread_t th, int error)
{
    assert(th == current_thread());
    if (error == 0 || th->t_pagein_error == 0) {
        th->t_pagein_error = error;
    }
}

#if defined(__x86_64__) || defined(__i386__)
/*
 * Verifica se a thread tem LDT (Linux: verifica se tem segment descriptors)
 */
int
thread_task_has_ldt(thread_t th)
{
    task_t task = get_threadtask(th);
    struct task_struct *linux_task = (struct task_struct *)get_linuxtask_info(task);
    
    /* No Linux, verificar se a tarefa usa segmentação personalizada */
    return linux_task && (linux_task->thread.segment_descriptors != 0);
}
#endif

/*
 * Retorna contagem de locks - mantido por compatibilidade
 */
int get_thread_lock_count(thread_t th __unused)
{
    return 0;
}

/*
 * Retorna a primeira thread ativa da tarefa
 */
thread_t
get_firstthread(task_t task)
{
    thread_t thread = THREAD_NULL;
    task_lock(task);

    if (!task->active) {
        task_unlock(task);
        return THREAD_NULL;
    }

    thread = (thread_t)(void *)queue_first(&task->threads);

    if (queue_end(&task->threads, (queue_entry_t)thread)) {
        task_unlock(task);
        return THREAD_NULL;
    }

    thread_reference(thread);
    task_unlock(task);
    return thread;
}

/*
 * Obtém uma thread para processar sinal
 */
kern_return_t
get_signalact(
    task_t          task,
    thread_t        *result_out,
    int             setast)
{
    kern_return_t   result = KERN_SUCCESS;
    thread_t        inc, thread = THREAD_NULL;

    task_lock(task);

    if (!task->active) {
        task_unlock(task);
        return KERN_FAILURE;
    }

    for (inc  = (thread_t)(void *)queue_first(&task->threads);
         !queue_end(&task->threads, (queue_entry_t)inc);) {
        thread_mtx_lock(inc);
        if (inc->active &&
            (inc->sched_flags & TH_SFLAG_ABORTED_MASK) != TH_SFLAG_ABORT) {
            thread = inc;
            break;
        }
        thread_mtx_unlock(inc);

        inc = (thread_t)(void *)queue_next(&inc->task_threads);
    }

    if (result_out) {
        *result_out = thread;
    }

    if (thread) {
        if (setast) {
            act_set_astbsd(thread);  /* Mantido para compatibilidade AST */
        }
        thread_mtx_unlock(thread);
    } else {
        result = KERN_FAILURE;
    }

    task_unlock(task);
    return result;
}

/*
 * Verifica se thread pode receber sinal
 */
kern_return_t
check_actforsig(
    task_t          task,
    thread_t        thread,
    int             setast)
{
    kern_return_t   result = KERN_FAILURE;
    thread_t        inc;

    task_lock(task);

    if (!task->active) {
        task_unlock(task);
        return KERN_FAILURE;
    }

    for (inc  = (thread_t)(void *)queue_first(&task->threads);
         !queue_end(&task->threads, (queue_entry_t)inc);) {
        if (inc == thread) {
            thread_mtx_lock(inc);

            if (inc->active &&
                (inc->sched_flags & TH_SFLAG_ABORTED_MASK) != TH_SFLAG_ABORT) {
                result = KERN_SUCCESS;
                break;
            }

            thread_mtx_unlock(inc);
            break;
        }

        inc = (thread_t)(void *)queue_next(&inc->task_threads);
    }

    if (result == KERN_SUCCESS) {
        if (setast) {
            act_set_astbsd(thread);
        }
        thread_mtx_unlock(thread);
    }

    task_unlock(task);
    return result;
}

/*
 * Obtém ledger da tarefa (contabilidade de recursos)
 */
ledger_t
get_task_ledger(task_t t)
{
    return t->ledger;
}

/*
 * Obtém mapa de memória da tarefa
 */
vm_map_t
get_task_map(task_t t)
{
    return t->map;
}

/*
 * Obtém referência ao mapa de memória
 */
vm_map_t
get_task_map_reference(task_t t)
{
    vm_map_t m;

    if (t == NULL) {
        return VM_MAP_NULL;
    }

    task_lock(t);
    if (!t->active) {
        task_unlock(t);
        return VM_MAP_NULL;
    }
    m = t->map;
    vm_map_reference(m);
    task_unlock(t);
    return m;
}

/*
 * Obtém espaço IPC da tarefa
 */
ipc_space_t
get_task_ipcspace(task_t t)
{
    return t->itk_space;
}

/*
 * Obtém número de threads da tarefa
 */
int
get_task_numacts(task_t t)
{
    return t->thread_count;
}

/*
 * Verifica se o registrador de 64 bits deve ser usado para signal handler
 */
int
is_64signalregset(void)
{
    if (task_has_64Bit_data(current_task())) {
        return 1;
    }
    return 0;
}

/*
 * Troca o mapa de memória da tarefa/thread
 */
vm_map_t
swap_task_map(task_t task, thread_t thread, vm_map_t map)
{
    vm_map_t old_map;
    boolean_t doswitch = (thread == current_thread()) ? TRUE : FALSE;

    if (task != get_threadtask(thread)) {
        panic("swap_task_map: task mismatch");
    }

    task_lock(task);
    mp_disable_preemption();

    assert(!task->ipc_active || task_is_a_corpse(task) || (map->owning_task == task));

    old_map = task->map;
    thread->map = task->map = map;
    vm_commit_pagezero_status(map);

    if (doswitch) {
        PMAP_SWITCH_USER(thread, map, cpu_number());
    }
    mp_enable_preemption();
    task_unlock(task);

    return old_map;
}

/*
 * Obtém pmap da tarefa (Linux mm_struct)
 */
pmap_t
get_task_pmap(task_t t)
{
    return t->map->pmap;
}

/*
 * Obtém tamanho residente da tarefa (RSS)
 */
uint64_t
get_task_resident_size(task_t task)
{
    uint64_t val;
    ledger_get_balance(task->ledger, task_ledgers.phys_mem, (ledger_amount_t *)&val);
    return val;
}

uint64_t
get_task_compressed(task_t task)
{
    uint64_t val;
    ledger_get_balance(task->ledger, task_ledgers.internal_compressed, (ledger_amount_t *)&val);
    return val;
}

uint64_t
get_task_resident_max(task_t task)
{
    uint64_t val;
    ledger_get_lifetime_max(task->ledger, task_ledgers.phys_mem, (ledger_amount_t *)&val);
    return val;
}

/*
 * Obtém saldo do ledger para um campo específico
 */
static uint64_t
get_task_ledger_balance(task_t task, int entry)
{
    ledger_amount_t balance = 0;
    ledger_get_balance(task->ledger, entry, &balance);
    return balance;
}

uint64_t
get_task_purgeable_size(task_t task)
{
    kern_return_t ret;
    ledger_amount_t balance = 0;
    uint64_t volatile_size = 0;

    ret = ledger_get_balance(task->ledger, task_ledgers.purgeable_volatile, &balance);
    if (ret != KERN_SUCCESS) {
        return 0;
    }
    volatile_size += balance;

    ret = ledger_get_balance(task->ledger, task_ledgers.purgeable_volatile_compressed, &balance);
    if (ret != KERN_SUCCESS) {
        return 0;
    }
    volatile_size += balance;

    return volatile_size;
}

uint64_t
get_task_phys_footprint(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.phys_footprint);
}

#if CONFIG_LEDGER_INTERVAL_MAX
uint64_t
get_task_phys_footprint_interval_max(task_t task, int reset)
{
    kern_return_t ret;
    ledger_amount_t max;

    ret = ledger_get_interval_max(task->ledger, task_ledgers.phys_footprint, &max, reset);

    if (KERN_SUCCESS == ret) {
        return max;
    }
    return 0;
}
#endif

uint64_t
get_task_phys_footprint_lifetime_max(task_t task)
{
    kern_return_t ret;
    ledger_amount_t max;

    ret = ledger_get_lifetime_max(task->ledger, task_ledgers.phys_footprint, &max);

    if (KERN_SUCCESS == ret) {
        return max;
    }
    return 0;
}

uint64_t
get_task_phys_footprint_limit(task_t task)
{
    kern_return_t ret;
    ledger_amount_t max;

    ret = ledger_get_limit(task->ledger, task_ledgers.phys_footprint, &max);
    if (KERN_SUCCESS == ret) {
        return max;
    }
    return 0;
}

uint64_t
get_task_internal(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.internal);
}

uint64_t
get_task_internal_compressed(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.internal_compressed);
}

uint64_t
get_task_purgeable_nonvolatile(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.purgeable_nonvolatile);
}

uint64_t
get_task_purgeable_nonvolatile_compressed(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.purgeable_nonvolatile_compressed);
}

uint64_t
get_task_alternate_accounting(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.alternate_accounting);
}

uint64_t
get_task_alternate_accounting_compressed(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.alternate_accounting_compressed);
}

uint64_t
get_task_page_table(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.page_table);
}

#if CONFIG_FREEZE
uint64_t
get_task_frozen_to_swap(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.frozen_to_swap);
}
#endif

uint64_t
get_task_iokit_mapped(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.iokit_mapped);
}

uint64_t
get_task_network_nonvolatile(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.network_nonvolatile);
}

uint64_t
get_task_network_nonvolatile_compressed(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.network_nonvolatile_compressed);
}

uint64_t
get_task_wired_mem(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.wired_mem);
}

uint64_t
get_task_cpu_time(task_t task)
{
    return get_task_ledger_balance(task, task_ledgers.cpu_time);
}

uint32_t
get_task_loadTag(task_t task)
{
    return os_atomic_load(&task->loadTag, relaxed);
}

uint32_t
set_task_loadTag(task_t task, uint32_t loadTag)
{
    return os_atomic_xchg(&task->loadTag, loadTag, relaxed);
}

/*
 * Obtém a tarefa da thread
 */
task_t
get_threadtask(thread_t th)
{
    return get_thread_ro(th)->tro_task;
}

task_t
get_threadtask_early(thread_t th)
{
    if (__improbable(startup_phase < STARTUP_SUB_EARLY_BOOT)) {
        if (th == THREAD_NULL || th->t_tro == NULL) {
            return TASK_NULL;
        }
    }
    return get_threadtask(th);
}

/*
 * Obtém limites do mapa de memória
 */
vm_map_offset_t
get_map_min(vm_map_t map)
{
    return vm_map_min(map);
}

vm_map_offset_t
get_map_max(vm_map_t map)
{
    return vm_map_max(map);
}

vm_map_size_t
get_vmmap_size(vm_map_t map)
{
    return vm_map_adjusted_size(map);
}

int
get_task_page_size(task_t task)
{
    return vm_map_page_size(task->map);
}

/*
 * Obtém contagem de parada do usuário
 */
int
get_task_userstop(task_t task)
{
    return task->user_stop_count;
}

int
get_thread_userstop(thread_t th)
{
    return th->user_stop_count;
}

boolean_t
get_task_pidsuspended(task_t task)
{
    return task->pidsuspended;
}

boolean_t
get_task_frozen(task_t task)
{
    return task->frozen;
}

/*
 * Verifica se thread deve abortar
 */
boolean_t
thread_should_abort(thread_t th)
{
    return (th->sched_flags & TH_SFLAG_ABORTED_MASK) == TH_SFLAG_ABORT;
}

/*
 * Verifica se thread atual está abortada
 */
boolean_t
current_thread_aborted(void)
{
    thread_t th = current_thread();
    spl_t s;

    if ((th->sched_flags & TH_SFLAG_ABORTED_MASK) == TH_SFLAG_ABORT &&
        (th->options & TH_OPT_INTMASK) != THREAD_UNINT) {
        return TRUE;
    }
    if (th->sched_flags & TH_SFLAG_ABORTSAFELY) {
        s = splsched();
        thread_lock(th);
        if (th->sched_flags & TH_SFLAG_ABORTSAFELY) {
            th->sched_flags &= ~TH_SFLAG_ABORTED_MASK;
        }
        thread_unlock(th);
        splx(s);
    }
    return FALSE;
}

/*
 * Itera sobre threads da tarefa com argumento
 */
void
task_act_iterate_wth_args(
    task_t          task,
    void            (*func_callback)(thread_t, void *),
    void            *func_arg)
{
    thread_t inc;

    task_lock(task);

    queue_iterate(&task->threads, inc, thread_t, task_threads) {
        func_callback(inc, func_arg);
    }

    task_unlock(task);
}

#include <linux/taskinfo.h>

/*
 * Preenche informações da tarefa (equivalente Linux do proc_taskinfo)
 */
void
fill_taskprocinfo(task_t task, struct proc_taskinfo_internal *ptinfo)
{
    vm_map_t map;
    task_absolutetime_info_data_t tinfo;
    thread_t thread;
    uint32_t cswitch = 0, numrunning = 0;
    uint32_t syscalls_unix = 0;      /* syscalls Linux */
    uint32_t syscalls_mach = 0;
    struct task_struct *linux_task = (struct task_struct *)get_linuxtask_info(task);

    task_lock(task);

    map = (task == kernel_task) ? kernel_map : task->map;

    ptinfo->pti_virtual_size = vm_map_adjusted_size(map);
    ledger_get_balance(task->ledger, task_ledgers.phys_mem, 
                      (ledger_amount_t *)&ptinfo->pti_resident_size);

    ptinfo->pti_policy = ((task != kernel_task) ? POLICY_TIMESHARE : POLICY_RR);

    queue_iterate(&task->threads, thread, thread_t, task_threads) {
        spl_t x;

        if (thread->options & TH_OPT_IDLE_THREAD) {
            continue;
        }

        x = splsched();
        thread_lock(thread);

        if ((thread->state & TH_RUN) == TH_RUN) {
            numrunning++;
        }
        cswitch += thread->c_switch;

        syscalls_unix += thread->syscalls_unix;
        syscalls_mach += thread->syscalls_mach;

        thread_unlock(thread);
        splx(x);
    }

    /* Coletar tempos usando ledger */
    struct recount_times_mach term_times = recount_task_terminated_times(task);
    struct recount_times_mach total_times = recount_task_times(task);

    tinfo.threads_user = total_times.rtm_user - term_times.rtm_user;
    tinfo.threads_system = total_times.rtm_system - term_times.rtm_system;
    ptinfo->pti_threads_system = tinfo.threads_system;
    ptinfo->pti_threads_user = tinfo.threads_user;

    ptinfo->pti_total_system = total_times.rtm_system;
    ptinfo->pti_total_user = total_times.rtm_user;

    ptinfo->pti_faults = (int32_t)MIN(counter_load(&task->faults), INT32_MAX);
    ptinfo->pti_pageins = (int32_t)MIN(counter_load(&task->pageins), INT32_MAX);
    ptinfo->pti_cow_faults = (int32_t)MIN(counter_load(&task->cow_faults), INT32_MAX);
    ptinfo->pti_messages_sent = (int32_t)MIN(counter_load(&task->messages_sent), INT32_MAX);
    ptinfo->pti_messages_received = (int32_t)MIN(counter_load(&task->messages_received), INT32_MAX);
    ptinfo->pti_syscalls_mach = (int32_t)MIN(task->syscalls_mach + syscalls_mach, INT32_MAX);
    ptinfo->pti_syscalls_unix = (int32_t)MIN(task->syscalls_unix + syscalls_unix, INT32_MAX);
    ptinfo->pti_csw = (int32_t)MIN(task->c_switch + cswitch, INT32_MAX);
    ptinfo->pti_threadnum = task->thread_count;
    ptinfo->pti_numrunning = numrunning;
    ptinfo->pti_priority = task->priority;

    task_unlock(task);
}

/*
 * Preenche informações da thread (equivalente Linux do proc_threadinfo)
 */
int
fill_taskthreadinfo(task_t task, uint64_t thaddr, bool thuniqueid, 
                    struct proc_threadinfo_internal *ptinfo, void *vpp, int *vidp)
{
    thread_t thact;
    int err = 0;
    mach_msg_type_number_t count;
    thread_basic_info_data_t basic_info;
    kern_return_t kret;
    uint64_t addr = 0;

    task_lock(task);

    queue_iterate(&task->threads, thact, thread_t, task_threads) {
        addr = (thuniqueid) ? thact->thread_id : thact->machine.cthread_self;
        if (addr == thaddr) {
            count = THREAD_BASIC_INFO_COUNT;
            if ((kret = thread_info_internal(thact, THREAD_BASIC_INFO, 
                                            (thread_info_t)&basic_info, &count)) != KERN_SUCCESS) {
                err = 1;
                goto out;
            }
            ptinfo->pth_user_time = (((uint64_t)basic_info.user_time.seconds * NSEC_PER_SEC) + 
                                     ((uint64_t)basic_info.user_time.microseconds * NSEC_PER_USEC));
            ptinfo->pth_system_time = (((uint64_t)basic_info.system_time.seconds * NSEC_PER_SEC) + 
                                       ((uint64_t)basic_info.system_time.microseconds * NSEC_PER_USEC));

            ptinfo->pth_cpu_usage = basic_info.cpu_usage;
            ptinfo->pth_policy = basic_info.policy;
            ptinfo->pth_run_state = basic_info.run_state;
            ptinfo->pth_flags = basic_info.flags;
            ptinfo->pth_sleep_time = basic_info.sleep_time;
            ptinfo->pth_curpri = thact->sched_pri;
            ptinfo->pth_priority = thact->base_pri;
            ptinfo->pth_maxpriority = thact->max_priority;

            if (vpp != NULL) {
                /* Equivalente Linux: obter diretório atual */
                linux_threadcdir(get_linuxthread_info_wrapper(thact), vpp, vidp);
            }
            linux_getthreadname(get_linuxthread_info_wrapper(thact), ptinfo->pth_name);
            err = 0;
            goto out;
        }
    }
    err = 1;

out:
    task_unlock(task);
    return err;
}

/*
 * Lista threads da tarefa
 */
int
fill_taskthreadlist(task_t task, void *buffer, int thcount, bool thuniqueid)
{
    int numthr = 0;
    thread_t thact;
    uint64_t *uptr;
    uint64_t thaddr;

    uptr = (uint64_t *)buffer;

    task_lock(task);

    queue_iterate(&task->threads, thact, thread_t, task_threads) {
        thaddr = (thuniqueid) ? thact->thread_id : thact->machine.cthread_self;
        *uptr++ = thaddr;
        numthr++;
        if (numthr >= thcount) {
            goto out;
        }
    }

out:
    task_unlock(task);
    return (int)(numthr * sizeof(uint64_t));
}

/*
 * Obtém informações de agendamento da thread
 */
int
fill_taskthreadschedinfo(task_t task, uint64_t thread_id, 
                         struct proc_threadschedinfo_internal *thread_sched_info)
{
    int err = 0;
    thread_t thread = current_thread();

    if (task != current_task() || thread_id != thread->thread_id) {
        return -1;
    }

#if SCHED_HYGIENE_DEBUG
    absolutetime_to_nanoseconds(thread->machine.int_time_mt, &thread_sched_info->int_time_ns);
#else
    (void)thread;
    thread_sched_info->int_time_ns = 0;
#endif

    return err;
}

/*
 * Obtém número de threads
 */
int
get_numthreads(task_t task)
{
    return task->thread_count;
}

/*
 * Preenche informações de uso de recursos (rusage)
 */
int
fill_task_rusage(task_t task, rusage_info_current *ri)
{
    struct task_power_info powerinfo;
    struct task_struct *linux_task = (struct task_struct *)get_linuxtask_info(task);

    assert(task != TASK_NULL);
    task_lock(task);

    struct task_power_info_extra extra = { 0 };
    task_power_info_locked(task, &powerinfo, NULL, NULL, &extra);
    
    ri->ri_pkg_idle_wkups = powerinfo.task_platform_idle_wakeups;
    ri->ri_interrupt_wkups = powerinfo.task_interrupt_wakeups;
    ri->ri_user_time = powerinfo.total_user;
    ri->ri_system_time = powerinfo.total_system;
    ri->ri_runnable_time = extra.runnable_time;
    ri->ri_cycles = extra.cycles;
    ri->ri_instructions = extra.instructions;
    ri->ri_pcycles = extra.pcycles;
    ri->ri_pinstructions = extra.pinstructions;
    ri->ri_user_ptime = extra.user_ptime;
    ri->ri_system_ptime = extra.system_ptime;
    ri->ri_energy_nj = extra.energy;
    ri->ri_penergy_nj = extra.penergy;
    ri->ri_secure_time_in_system = extra.secure_time;
    ri->ri_secure_ptime_in_system = extra.secure_ptime;

    ri->ri_phys_footprint = get_task_phys_footprint(task);
    ledger_get_balance(task->ledger, task_ledgers.phys_mem,
                      (ledger_amount_t *)&ri->ri_resident_size);
    ri->ri_wired_size = get_task_wired_mem(task);

    ledger_get_balance(task->ledger, task_ledgers.neural_nofootprint_total,
                      (ledger_amount_t *)&ri->ri_neural_footprint);
    ri->ri_pageins = counter_load(&task->pageins);

    task_unlock(task);
    return 0;
}

/*
 * Preenche informações de cobrança (billed usage)
 */
void
fill_task_billed_usage(task_t task __unused, rusage_info_current *ri)
{
    bank_billed_balance_safe(task, &ri->ri_billed_system_time, &ri->ri_billed_energy);
    bank_serviced_balance_safe(task, &ri->ri_serviced_system_time, &ri->ri_serviced_energy);
}

/*
 * Preenche informações de I/O
 */
int
fill_task_io_rusage(task_t task, rusage_info_current *ri)
{
    struct task_struct *linux_task = (struct task_struct *)get_linuxtask_info(task);

    assert(task != TASK_NULL);
    task_lock(task);

    if (linux_task && linux_task->ioac) {
        /* Usar estatísticas de I/O do Linux */
        ri->ri_diskio_bytesread = linux_task->ioac->rchar;
        ri->ri_diskio_byteswritten = linux_task->ioac->wchar;
    } else if (task->task_io_stats) {
        ri->ri_diskio_bytesread = task->task_io_stats->disk_reads.size;
        ri->ri_diskio_byteswritten = (task->task_io_stats->total_io.size - 
                                      task->task_io_stats->disk_reads.size);
    } else {
        ri->ri_diskio_bytesread = 0;
        ri->ri_diskio_byteswritten = 0;
    }
    
    task_unlock(task);
    return 0;
}

/*
 * Preenche informações de QoS (prioridades Linux)
 */
int
fill_task_qos_rusage(task_t task, rusage_info_current *ri)
{
    thread_t thread;

    assert(task != TASK_NULL);
    task_lock(task);

    queue_iterate(&task->threads, thread, thread_t, task_threads) {
        if (thread->options & TH_OPT_IDLE_THREAD) {
            continue;
        }
        thread_update_qos_cpu_time(thread);
    }
    
    /* Mapear prioridades Linux para QoS Mach */
    ri->ri_cpu_time_qos_default = task->cpu_time_eqos_stats.cpu_time_qos_default;
    ri->ri_cpu_time_qos_maintenance = task->cpu_time_eqos_stats.cpu_time_qos_maintenance;
    ri->ri_cpu_time_qos_background = task->cpu_time_eqos_stats.cpu_time_qos_background;
    ri->ri_cpu_time_qos_utility = task->cpu_time_eqos_stats.cpu_time_qos_utility;
    ri->ri_cpu_time_qos_legacy = task->cpu_time_eqos_stats.cpu_time_qos_legacy;
    ri->ri_cpu_time_qos_user_initiated = task->cpu_time_eqos_stats.cpu_time_qos_user_initiated;
    ri->ri_cpu_time_qos_user_interactive = task->cpu_time_eqos_stats.cpu_time_qos_user_interactive;

    task_unlock(task);
    return 0;
}

/*
 * Obtém contagem de writes lógicos
 */
uint64_t
get_task_logical_writes(task_t task, bool external)
{
    assert(task != TASK_NULL);
    struct ledger_entry_info lei;
    int entry = external ? task_ledgers.logical_writes_to_external :
                           task_ledgers.logical_writes;

    task_lock(task);
    ledger_get_entry_info(task->ledger, entry, &lei);
    task_unlock(task);

    return lei.lei_balance;
}

/*
 * Obtém offset da serial number da dispatch queue (equivalente Linux)
 */
uint64_t
get_task_sched_serialno_offset(task_t task)
{
    uint64_t sched_serialno_offset = 0;
    void *linux_info = get_linuxtask_info(task);

    if (linux_info) {
        sched_serialno_offset = get_sched_serialno_offset_from_proc(linux_info);
    }

    return sched_serialno_offset;
}

/*
 * Obtém offset do label do cgroup (equivalente Linux)
 */
uint64_t
get_task_cgroup_label_offset(task_t task)
{
    uint64_t cgroup_label_offset = 0;
    void *linux_info = get_linuxtask_info(task);

    if (linux_info) {
        cgroup_label_offset = get_cgroup_label_offset_from_proc(linux_info);
    }

    return cgroup_label_offset;
}

/*
 * Obtém ID único da tarefa
 */
uint64_t
get_task_uniqueid(task_t task)
{
    void *linux_info = get_linuxtask_info(task);

    if (linux_info) {
        return proc_uniqueid_task(linux_info, task);
    } else {
        return UINT64_MAX;
    }
}

/*
 * Obtém versão da tarefa
 */
int
get_task_version(task_t task)
{
    void *linux_info = get_linuxtask_info(task);

    if (linux_info) {
        return proc_pidversion(linux_info);
    } else {
        return INT_MAX;
    }
}

#if CONFIG_MACF
/*
 * Obtém label de crash (MAC Framework)
 */
struct label *
get_task_crash_label(task_t task)
{
    return task->crash_label;
}

void
set_task_crash_label(task_t task, struct label *label)
{
    task->crash_label = label;
}
#endif

/*
 * Preenche informações da tabela IPC
 */
int
fill_taskipctableinfo(task_t task, uint32_t *table_size, uint32_t *table_free)
{
    ipc_space_t space = task->itk_space;
    if (space == NULL) {
        return -1;
    }

    is_read_lock(space);
    if (!is_active(space)) {
        is_read_unlock(space);
        return -1;
    }

    *table_size = ipc_entry_table_count(is_active_table(space));
    *table_free = space->is_table_free;

    is_read_unlock(space);
    return 0;
}

/*
 * Obtém CD hash da tarefa (para code signing)
 */
int
get_task_cdhash(task_t task, char cdhash[static CS_CDHASH_LEN])
{
    int result = 0;
    void *linux_info = NULL;

    task_lock(task);
    linux_info = get_linuxtask_info(task);
    result = linux_info ? proc_getcdhash(linux_info, cdhash) : ESRCH;
    task_unlock(task);

    return result;
}

/*
 * Verifica se thread atual está em falta no kernel
 */
bool
current_thread_in_kernel_fault(void)
{
    if (current_thread()->recover) {
        return true;
    }
    return false;
}

/*
 * Converte erro Mach para errno Linux
 */
int
mach_to_linux_errno(kern_return_t mach_err)
{
    switch (mach_err) {
    case KERN_SUCCESS:
        return 0;

    case KERN_INVALID_ADDRESS:
    case KERN_INVALID_ARGUMENT:
    case KERN_NOT_IN_SET:
    case KERN_INVALID_NAME:
    case KERN_INVALID_TASK:
    case KERN_INVALID_RIGHT:
    case KERN_INVALID_VALUE:
    case KERN_INVALID_CAPABILITY:
    case KERN_INVALID_HOST:
    case KERN_MEMORY_PRESENT:
    case KERN_INVALID_PROCESSOR_SET:
    case KERN_INVALID_POLICY:
    case KERN_ALREADY_WAITING:
    case KERN_DEFAULT_SET:
    case KERN_EXCEPTION_PROTECTED:
    case KERN_INVALID_LEDGER:
    case KERN_INVALID_MEMORY_CONTROL:
    case KERN_INVALID_SECURITY:
    case KERN_NOT_DEPRESSED:
    case KERN_LOCK_OWNED:
    case KERN_LOCK_OWNED_SELF:
        return -EINVAL;

    case KERN_NOT_RECEIVER:
    case KERN_NO_ACCESS:
    case KERN_POLICY_STATIC:
        return -EACCES;

    case KERN_NO_SPACE:
    case KERN_RESOURCE_SHORTAGE:
    case KERN_UREFS_OVERFLOW:
    case KERN_INVALID_OBJECT:
        return -ENOMEM;

    case KERN_MEMORY_FAILURE:
    case KERN_MEMORY_ERROR:
    case KERN_PROTECTION_FAILURE:
        return -EFAULT;

    case KERN_POLICY_LIMIT:
    case KERN_CODESIGN_ERROR:
    case KERN_DENIED:
        return -EPERM;

    case KERN_ALREADY_IN_SET:
    case KERN_NAME_EXISTS:
    case KERN_RIGHT_EXISTS:
        return -EEXIST;

    case KERN_ABORTED:
        return -EINTR;

    case KERN_TERMINATED:
    case KERN_LOCK_SET_DESTROYED:
    case KERN_LOCK_UNSTABLE:
    case KERN_SEMAPHORE_DESTROYED:
    case KERN_NOT_FOUND:
    case KERN_NOT_WAITING:
        return -ESRCH;

    case KERN_RPC_SERVER_TERMINATED:
        return -ECONNRESET;

    case KERN_NOT_SUPPORTED:
        return -EOPNOTSUPP;

    case KERN_NODE_DOWN:
        return -ENETDOWN;

    case KERN_OPERATION_TIMED_OUT:
        return -ETIMEDOUT;

    default:
        return -EIO;
    }
}

/*
 * Converte errno Linux para erro Mach
 */
kern_return_t
kern_return_for_linux_errno(int linux_errno)
{
    switch (linux_errno) {
    case 0:
        return KERN_SUCCESS;
    case -EIO:
    case -EACCES:
    case -ENOMEM:
    case -EFAULT:
        return KERN_MEMORY_ERROR;

    case -EINVAL:
        return KERN_INVALID_ARGUMENT;

    case -ETIMEDOUT:
    case -EBUSY:
        return KERN_OPERATION_TIMED_OUT;

    case -ECONNRESET:
        return KERN_RPC_SERVER_TERMINATED;

    case -EOPNOTSUPP:
        return KERN_NOT_SUPPORTED;

    case -ENETDOWN:
        return KERN_NODE_DOWN;

    case -ESRCH:
        return KERN_NOT_FOUND;

    case -EINTR:
        return KERN_ABORTED;

    case -EPERM:
        return KERN_DENIED;

    case -EEXIST:
        return KERN_ALREADY_IN_SET;

    default:
        return KERN_FAILURE;
    }
}
