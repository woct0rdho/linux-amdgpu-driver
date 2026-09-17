// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Copyright 2023-2026 Advanced Micro Devices, Inc.
 *
 * PC Sampling support for GFX11.5 (SQ_CMD host-trap via trigger_pc_sample_trap).
 *
 * The ioctl contract follows the upstream KFD PC sampling implementation this
 * file was ported from: the revision below is the one whose ioctl semantics
 * (methods, types, flags and query behavior) are implemented here. The upstream
 * change log grows gfx9/gfx12 support on top of that revision, which this tree
 * does not implement; gfx11_5 host trap and stochastic sampling are the
 * additions made here.
 */

#include "kfd_priv.h"
#include "amdgpu_amdkfd.h"
#include "kfd_pc_sampling.h"
#include "kfd_debug.h"
#include "kfd_device_queue_manager.h"

#include <linux/bitops.h>

#define KFD_IOCTL_PCS_MAJOR_VERSION	1
#define KFD_IOCTL_PCS_MINOR_VERSION	5

struct supported_pc_sample_info {
	uint32_t ip_version;
	const struct kfd_pc_sample_info *sample_info;
};

static const struct kfd_pc_sample_info sample_info_hosttrap_11_5 = {
	0, 1, ~0ULL, 0, KFD_IOCTL_PCS_METHOD_HOSTTRAP, KFD_IOCTL_PCS_TYPE_TIME_US };

static const struct kfd_pc_sample_info sample_info_stoch_cycle_11_5 = {
	0, 256, (1ULL << 31), KFD_IOCTL_PCS_FLAG_POWER_OF_2,
	KFD_IOCTL_PCS_METHOD_STOCHASTIC, KFD_IOCTL_PCS_TYPE_CLOCK_CYCLES };

/*
 * Only the revision that has been validated is enabled: gfx1151 reports
 * IP_VERSION(11, 5, 1). The sibling gfx11_5 revisions use the same
 * programming interface, but have not been tested.
 */
static struct supported_pc_sample_info supported_formats[] = {
	{ IP_VERSION(11, 5, 1), &sample_info_hosttrap_11_5 },
	{ IP_VERSION(11, 5, 1), &sample_info_stoch_cycle_11_5 },
};

/*
 * Resolve the VMID of the process that owns a PC sampling session.
 *
 * Upstream passes node->vm_info.last_vmid_kfd to trigger_pc_sample_trap(), but
 * none of the upstream implementations look at the VMID: they trap a wave slot
 * and let the first-level trap handler check the per-process TMA flags, which
 * gfx9/gfx12 provide. The gfx11 first-level trap handler has no such check, so
 * gfx11_5 filters the trap by VMID instead (SQ_CMD.CHECK_VMID/VM_ID) and must
 * therefore target the VMID of the process that is sampling. The VMID can be
 * assigned only when the process gets its first queue, so it is resolved here
 * on demand and cached in hosttrap_entry.target_vmid.
 */
static uint32_t kfd_pc_sampling_lookup_vmid_by_pasid(struct kfd_node *node,
						     uint32_t owner_pasid)
{
	uint32_t vmid;

	if (!owner_pasid || !node->kfd2kgd->get_atc_vmid_pasid_mapping_info)
		return 0;

	/* KFD VMIDs are always allocated from the KFD VMID range */
	for (vmid = node->vm_info.first_vmid_kfd;
	     vmid <= node->vm_info.last_vmid_kfd; vmid++) {
		uint16_t queried_pasid = 0;

		if (node->kfd2kgd->get_atc_vmid_pasid_mapping_info(node->adev,
				vmid, &queried_pasid) &&
		    queried_pasid == owner_pasid)
			return vmid;
	}

	return 0;
}

static int kfd_pc_sample_thread(void *param)
{
	struct amdgpu_device *adev;
	struct kfd_node *node = param;
	uint32_t timeout = 0;
	ktime_t next_trap_time;
	bool need_wait;
	uint32_t inst;
	uint32_t target_vmid;
	uint32_t trap_count = 0;

	mutex_lock(&node->pcs_data.mutex);
	if (node->pcs_data.hosttrap_entry.base.active_count &&
	    node->pcs_data.hosttrap_entry.base.pc_sample_info.interval &&
	    node->kfd2kgd->trigger_pc_sample_trap) {
		switch (node->pcs_data.hosttrap_entry.base.pc_sample_info.type) {
		case KFD_IOCTL_PCS_TYPE_TIME_US:
			timeout = (uint32_t)node->pcs_data.hosttrap_entry.base.pc_sample_info.interval;
			break;
		default:
			pr_debug("PC Sampling type %d not supported.",
				 node->pcs_data.hosttrap_entry.base.pc_sample_info.type);
		}
	}
	mutex_unlock(&node->pcs_data.mutex);
	if (!timeout)
		goto exit_wait;

	adev = node->adev;
	need_wait = false;
	allow_signal(SIGKILL);

	pr_debug("pcs: thread started interval_us=%u\n", timeout);

	/*
	 * signal_pending(current) rather than a dereference of
	 * pc_sample_thread: kthread_run() may run the thread before
	 * kfd_pc_sample_thread_start() publishes the pointer.
	 */
	while (!kthread_should_stop() &&
	       !amdgpu_in_reset(adev) &&
	       !signal_pending(current)) {
		if (!need_wait) {
			next_trap_time = ktime_add_us(ktime_get_raw(), timeout);

			target_vmid = READ_ONCE(node->pcs_data.hosttrap_entry.target_vmid);
			if (!target_vmid) {
				uint32_t owner_pasid = READ_ONCE(
					node->pcs_data.hosttrap_entry.owner_pasid);
				uint32_t resolved_vmid =
					kfd_pc_sampling_lookup_vmid_by_pasid(node, owner_pasid);

				if (resolved_vmid) {
					WRITE_ONCE(node->pcs_data.hosttrap_entry.target_vmid,
						   resolved_vmid);
					target_vmid = resolved_vmid;
					pr_debug("pcs: resolved vmid=%u for pasid=%u (kfd range %u-%u)\n",
						 target_vmid, owner_pasid,
						 node->vm_info.first_vmid_kfd,
						 node->vm_info.last_vmid_kfd);
				}
			}

			if (target_vmid) {
				for_each_inst(inst, node->xcc_mask)
					node->kfd2kgd->trigger_pc_sample_trap(adev, target_vmid,
						&node->pcs_data.hosttrap_entry.target_simd,
						&node->pcs_data.hosttrap_entry.target_wave_slot,
						node->pcs_data.hosttrap_entry.base.pc_sample_info.method,
						inst);
				trap_count++;
				pr_debug_ratelimited("pcs: triggered a host trap with vmid=%u\n",
						     target_vmid);
			}

			need_wait = true;
		} else {
			ktime_t wait_time;
			s64 wait_ns, wait_us;

			wait_time = ktime_sub(next_trap_time, ktime_get_raw());
			wait_ns = ktime_to_ns(wait_time);
			wait_us = ktime_to_us(wait_time);
			if (wait_ns >= 10000) {
				usleep_range(wait_us - 10, wait_us);
			} else {
				schedule();
				if (wait_ns <= 0)
					need_wait = false;
			}
		}
	}

	node->pcs_data.hosttrap_entry.target_simd = 0;
	node->pcs_data.hosttrap_entry.target_wave_slot = 0;

	pr_debug("pcs: thread exiting, sent %u traps\n", trap_count);

exit_wait:
	/*
	 * Wait for kthread_stop() from the stop path rather than exiting
	 * immediately. Upstream returns here and clears pc_sample_thread, which
	 * makes the later kthread_stop() dereference a NULL pointer whenever the
	 * thread stopped by itself (SIGKILL or a GPU reset); parking instead
	 * keeps the pointer valid until the single stop path clears it.
	 */
	while (!kthread_should_stop())
		schedule_timeout_uninterruptible(msecs_to_jiffies(100));

	return 0;
}

static int kfd_pc_sample_thread_start(struct kfd_node *node)
{
	char thread_name[16];
	int ret = 0;

	snprintf(thread_name, 16, "pcs_%d", node->adev->ddev.render->index);
	node->pcs_data.hosttrap_entry.pc_sample_thread =
		kthread_run(kfd_pc_sample_thread, node, thread_name);

	if (IS_ERR(node->pcs_data.hosttrap_entry.pc_sample_thread)) {
		ret = PTR_ERR(node->pcs_data.hosttrap_entry.pc_sample_thread);
		node->pcs_data.hosttrap_entry.pc_sample_thread = NULL;
		pr_debug("Failed to create pc sample thread for %s with ret = %d.",
			 thread_name, ret);
	}

	return ret;
}

static int kfd_pc_sample_query_cap(struct kfd_process_device *pdd,
				   struct kfd_ioctl_pc_sample_args *user_args)
{
	uint64_t sample_offset;
	int num_method = 0;
	int ret;
	int i;
	const uint32_t user_num_sample_info = user_args->num_sample_info;

	user_args->version = KFD_IOCTL_PCS_MAJOR_VERSION << 16 | KFD_IOCTL_PCS_MINOR_VERSION;

	for (i = 0; i < ARRAY_SIZE(supported_formats); i++)
		if (KFD_GC_VERSION(pdd->dev) == supported_formats[i].ip_version)
			num_method++;

	if (!num_method) {
		pr_debug("PC Sampling not supported on GC_HWIP:0x%x.",
			 pdd->dev->adev->ip_versions[GC_HWIP][0]);
		return -EOPNOTSUPP;
	}

	ret = 0;
	mutex_lock(&pdd->dev->pcs_data.mutex);
	if (user_args->flags != KFD_IOCTL_PCS_QUERY_TYPE_FULL &&
	    (pdd->dev->pcs_data.hosttrap_entry.base.use_count ||
	     pdd->dev->pcs_data.stoch_entry.base.use_count)) {
		user_args->num_sample_info = 0;

		/* If we already have a session, restrict returned list to current method */
		if (pdd->dev->pcs_data.stoch_entry.base.use_count) {
			user_args->num_sample_info++;
			if (user_args->sample_info_ptr &&
			    user_args->num_sample_info <= user_num_sample_info) {
				ret = copy_to_user((void __user *)user_args->sample_info_ptr,
					&pdd->dev->pcs_data.stoch_entry.base.pc_sample_info,
					sizeof(struct kfd_pc_sample_info));
				user_args->sample_info_ptr +=
					sizeof(struct kfd_pc_sample_info);
			}
		}

		if (pdd->dev->pcs_data.hosttrap_entry.base.use_count) {
			user_args->num_sample_info++;
			if (user_args->sample_info_ptr &&
			    user_args->num_sample_info <= user_num_sample_info)
				ret |= copy_to_user((void __user *)user_args->sample_info_ptr,
					&pdd->dev->pcs_data.hosttrap_entry.base.pc_sample_info,
					sizeof(struct kfd_pc_sample_info));
		}
		mutex_unlock(&pdd->dev->pcs_data.mutex);
		return ret ? -EFAULT : 0;
	}
	mutex_unlock(&pdd->dev->pcs_data.mutex);

	user_args->num_sample_info = num_method;

	if (!user_args->sample_info_ptr || !user_num_sample_info) {
		/*
		 * User application is querying the size of buffer needed. Application
		 * will allocate required buffer size and send a second query.
		 */
		return 0;
	} else if (user_num_sample_info < num_method) {
		pr_debug("ASIC requires space for %d kfd_pc_sample_info entries.", num_method);
		return -ENOSPC;
	}

	sample_offset = user_args->sample_info_ptr;
	for (i = 0; i < ARRAY_SIZE(supported_formats); i++) {
		if (KFD_GC_VERSION(pdd->dev) == supported_formats[i].ip_version) {
			ret = copy_to_user((void __user *)sample_offset,
				supported_formats[i].sample_info,
				sizeof(struct kfd_pc_sample_info));
			if (ret) {
				pr_debug("Failed to copy PC sampling info to user.");
				return -EFAULT;
			}
			sample_offset += sizeof(struct kfd_pc_sample_info);
		}
	}

	return 0;
}

static int kfd_pc_sample_start(struct kfd_process_device *pdd,
			       struct pc_sampling_entry *pcs_entry)
{
	bool pc_sampling_start = false;
	int ret = 0;

	pcs_entry->enabled = true;
	mutex_lock(&pdd->dev->pcs_data.mutex);

	kfd_process_set_trap_pc_sampling_flag(&pdd->qpd, pcs_entry->method, true);

	if (pcs_entry->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
		if (!pdd->dev->pcs_data.hosttrap_entry.base.active_count)
			pc_sampling_start = true;

		pdd->dev->pcs_data.hosttrap_entry.owner_pasid = pdd->pasid;
		pdd->dev->pcs_data.hosttrap_entry.base.active_count++;
	} else { /* KFD_IOCTL_PCS_METHOD_STOCHASTIC */
		if (!pdd->dev->pcs_data.stoch_entry.base.active_count)
			pc_sampling_start = true;

		pdd->dev->pcs_data.stoch_entry.base.active_count++;
	}
	mutex_unlock(&pdd->dev->pcs_data.mutex);

	while (pc_sampling_start) {
		if (pcs_entry->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
			/* Non-NULL means a thread stop is still in progress */
			if (READ_ONCE(pdd->dev->pcs_data.hosttrap_entry.pc_sample_thread)) {
				usleep_range(1000, 2000);
			} else {
				ret = kfd_pc_sample_thread_start(pdd->dev);
				break;
			}
		} else { /* KFD_IOCTL_PCS_METHOD_STOCHASTIC */
			uint32_t inst;

			if (pdd->dev->kfd2kgd->setup_stoch_sampling)
				for_each_inst(inst, pdd->dev->xcc_mask)
					pdd->dev->kfd2kgd->setup_stoch_sampling(
						pdd->dev->adev, pdd->dev->compute_vmid_bitmap,
						true,
						pdd->dev->pcs_data.stoch_entry.base.pc_sample_info.type,
						pdd->dev->pcs_data.stoch_entry.base.pc_sample_info.interval,
						inst);
			break;
		}
	}

	if (ret && pc_sampling_start) {
		pcs_entry->enabled = false;
		mutex_lock(&pdd->dev->pcs_data.mutex);
		if (pcs_entry->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
			pdd->dev->pcs_data.hosttrap_entry.base.active_count--;
			if (!pdd->dev->pcs_data.hosttrap_entry.base.active_count) {
				pdd->dev->pcs_data.hosttrap_entry.owner_pasid = 0;
			}
		} else {
			pdd->dev->pcs_data.stoch_entry.base.active_count--;
		}
		mutex_unlock(&pdd->dev->pcs_data.mutex);
		kfd_process_set_trap_pc_sampling_flag(&pdd->qpd,
						      pcs_entry->method, false);
	}
	return ret;
}

static int kfd_pc_sample_stop(struct kfd_process_device *pdd,
			      struct pc_sampling_entry *pcs_entry)
{
	bool pc_sampling_stop = false;

	pcs_entry->enabled = false;
	mutex_lock(&pdd->dev->pcs_data.mutex);
	if (pcs_entry->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
		pdd->dev->pcs_data.hosttrap_entry.base.active_count--;
		if (!pdd->dev->pcs_data.hosttrap_entry.base.active_count)
			pc_sampling_stop = true;
	} else { /* KFD_IOCTL_PCS_METHOD_STOCHASTIC */
		pdd->dev->pcs_data.stoch_entry.base.active_count--;
		if (!pdd->dev->pcs_data.stoch_entry.base.active_count)
			pc_sampling_stop = true;
	}
	mutex_unlock(&pdd->dev->pcs_data.mutex);

	kfd_process_set_trap_pc_sampling_flag(&pdd->qpd, pcs_entry->method, false);
	remap_queue(pdd->dev->dqm,
		    KFD_UNMAP_QUEUES_FILTER_ALL_QUEUES, 0);

	if (pc_sampling_stop) {
		if (pcs_entry->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP) {
			struct task_struct *t =
				READ_ONCE(pdd->dev->pcs_data.hosttrap_entry.pc_sample_thread);

			if (t) {
				kthread_stop(t);
				WRITE_ONCE(pdd->dev->pcs_data.hosttrap_entry.pc_sample_thread,
					   NULL);
			}
		} else { /* KFD_IOCTL_PCS_METHOD_STOCHASTIC */
			uint32_t inst;

			if (pdd->dev->kfd2kgd->setup_stoch_sampling)
				for_each_inst(inst, pdd->dev->xcc_mask)
					pdd->dev->kfd2kgd->setup_stoch_sampling(
						pdd->dev->adev, pdd->dev->compute_vmid_bitmap,
						false,
						pdd->dev->pcs_data.stoch_entry.base.pc_sample_info.type,
						0, inst);
		}
	}

	return 0;
}

static int kfd_pc_sample_create(struct kfd_process_device *pdd,
				struct kfd_ioctl_pc_sample_args *user_args)
{
	struct kfd_pc_sample_info *supported_format = NULL;
	struct kfd_pc_sample_info user_info;
	struct pc_sampling_entry *pcs_entry;
	int ret;
	int i;
	struct kfd_dev_pc_sampling_data *pcs_data;

	if (user_args->num_sample_info != 1)
		return -EINVAL;

	ret = copy_from_user(&user_info, (void __user *)user_args->sample_info_ptr,
			     sizeof(struct kfd_pc_sample_info));
	if (ret) {
		pr_debug("Failed to copy PC sampling info from user\n");
		return -EFAULT;
	}

	/*
	 * The host trap interval is truncated to u32 when the sampling thread
	 * converts it to microseconds, so reject anything larger up front.
	 */
	if (user_info.interval > UINT_MAX)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(supported_formats); i++) {
		if (KFD_GC_VERSION(pdd->dev) == supported_formats[i].ip_version &&
		    user_info.method == supported_formats[i].sample_info->method &&
		    user_info.type == supported_formats[i].sample_info->type &&
		    user_info.interval <= supported_formats[i].sample_info->interval_max &&
		    user_info.interval >= supported_formats[i].sample_info->interval_min) {
			supported_format =
				(struct kfd_pc_sample_info *)supported_formats[i].sample_info;
			break;
		}
	}

	if (!supported_format) {
		pr_debug("Sampling format is not supported!");
		return -EOPNOTSUPP;
	}

	if (supported_format->flags == KFD_IOCTL_PCS_FLAG_POWER_OF_2 &&
	    user_info.interval & (user_info.interval - 1)) {
		pr_debug("Sampling interval's power is unmatched!");
		return -EINVAL;
	}

	mutex_lock(&pdd->dev->pcs_data.mutex);
	pcs_data = user_info.method == KFD_IOCTL_PCS_METHOD_HOSTTRAP ?
		&pdd->dev->pcs_data.hosttrap_entry.base : &pdd->dev->pcs_data.stoch_entry.base;

	if (pcs_data->use_count &&
	    memcmp(&pcs_data->pc_sample_info,
		   &user_info, sizeof(user_info))) {
		ret = copy_to_user((void __user *)user_args->sample_info_ptr,
			&pcs_data->pc_sample_info,
			sizeof(struct kfd_pc_sample_info));
		mutex_unlock(&pdd->dev->pcs_data.mutex);
		return ret ? -EFAULT : -EEXIST;
	}

	pcs_entry = kzalloc(sizeof(*pcs_entry), GFP_KERNEL);
	if (!pcs_entry) {
		mutex_unlock(&pdd->dev->pcs_data.mutex);
		return -ENOMEM;
	}

	i = idr_alloc_cyclic(&pdd->dev->pcs_data.sampling_idr,
			     pcs_entry, 1, 0, GFP_KERNEL);
	if (i < 0) {
		mutex_unlock(&pdd->dev->pcs_data.mutex);
		kfree(pcs_entry);
		return i;
	}

	if (!pcs_data->use_count)
		pcs_data->pc_sample_info = user_info;
	pcs_data->use_count++;

	mutex_unlock(&pdd->dev->pcs_data.mutex);

	pcs_entry->pdd = pdd;
	pcs_entry->method = supported_format->method;
	user_args->trace_id = (uint32_t)i;

	/*
	 * Set SPI_GDBG_PER_VMID_CNTL.TRAP_EN so that TTMP registers are valid in
	 * the sampling data. p->runtime_info.ttmp_setup is cleared when the user
	 * application calls runtime_disable on exit.
	 */
	kfd_dbg_enable_ttmp_setup(pdd->process);
	pdd->process->pc_sampling_ref++;

	pr_debug("alloc pcs_entry = %p, trace_id = 0x%x method = %d on gpu 0x%x",
		 pcs_entry, i, pcs_entry->method, pdd->dev->id);

	return 0;
}

static int kfd_pc_sample_destroy(struct kfd_process_device *pdd, uint32_t trace_id,
				 struct pc_sampling_entry *pcs_entry)
{
	struct kfd_dev_pc_sampling_data *pcs_data;

	pr_debug("free pcs_entry = %p, trace_id = 0x%x on gpu 0x%x",
		 pcs_entry, trace_id, pdd->dev->id);

	pdd->process->pc_sampling_ref--;
	mutex_lock(&pdd->dev->pcs_data.mutex);
	pcs_data = pcs_entry->method == KFD_IOCTL_PCS_METHOD_HOSTTRAP ?
		&pdd->dev->pcs_data.hosttrap_entry.base : &pdd->dev->pcs_data.stoch_entry.base;

	pcs_data->use_count--;
	if (!pcs_data->use_count)
		memset(&pcs_data->pc_sample_info, 0,
		       sizeof(struct kfd_pc_sample_info));

	idr_remove(&pdd->dev->pcs_data.sampling_idr, trace_id);
	mutex_unlock(&pdd->dev->pcs_data.mutex);

	kfree(pcs_entry);
	return 0;
}

/*
 * Stop a node's host-trap sampling kthread.
 *
 * kfd_pc_sample_thread() only leaves its loop when it is explicitly stopped (or
 * when it is signalled), so the thread must not outlive its node: the thread
 * dereferences the node and runs code that lives in this module, and neither
 * the node memory nor the module can be released while it is still running.
 * kfd_cleanup_nodes() calls this for every node before tearing the node down.
 */
void kfd_pc_sample_stop_thread(struct kfd_node *node)
{
	struct task_struct *thread;

	mutex_lock(&node->pcs_data.mutex);
	thread = READ_ONCE(node->pcs_data.hosttrap_entry.pc_sample_thread);
	WRITE_ONCE(node->pcs_data.hosttrap_entry.pc_sample_thread, NULL);
	mutex_unlock(&node->pcs_data.mutex);

	if (!thread) {
		pr_debug("kfd: no PC sampling thread to stop for node %d\n",
			 node->node_id);
		return;
	}

	kthread_stop(thread);
	pr_debug("kfd: stopped PC sampling thread for node %d\n", node->node_id);
}

void kfd_pc_sample_release(struct kfd_process_device *pdd)
{
	struct pc_sampling_entry *pcs_entry;
	struct idr *idp;
	uint32_t id;

	/* force to release all PC sampling task for this process */
	idp = &pdd->dev->pcs_data.sampling_idr;
	do {
		pcs_entry = NULL;
		mutex_lock(&pdd->dev->pcs_data.mutex);
		idr_for_each_entry(idp, pcs_entry, id) {
			if (pcs_entry->pdd != pdd)
				continue;
			break;
		}
		mutex_unlock(&pdd->dev->pcs_data.mutex);
		if (pcs_entry) {
			if (pcs_entry->enabled)
				kfd_pc_sample_stop(pdd, pcs_entry);
			kfd_pc_sample_destroy(pdd, id, pcs_entry);
		}
	} while (pcs_entry);
}

int kfd_pc_sample(struct kfd_process_device *pdd,
		  struct kfd_ioctl_pc_sample_args *args)
{
	struct pc_sampling_entry *pcs_entry;

	if (args->op != KFD_IOCTL_PCS_OP_QUERY_CAPABILITIES &&
	    args->op != KFD_IOCTL_PCS_OP_CREATE) {
		mutex_lock(&pdd->dev->pcs_data.mutex);
		pcs_entry = idr_find(&pdd->dev->pcs_data.sampling_idr,
				     args->trace_id);
		mutex_unlock(&pdd->dev->pcs_data.mutex);

		/*
		 * pcs_entry is only for this pc sampling process, which has
		 * kfd_process->mutex protected here.
		 */
		if (!pcs_entry || pcs_entry->pdd != pdd)
			return -EINVAL;
	} else if (pdd->process->debug_trap_enabled) {
		pr_debug("Cannot have PC Sampling and debug trap simultaneously");
		return -EBUSY;
	}

	switch (args->op) {
	case KFD_IOCTL_PCS_OP_QUERY_CAPABILITIES:
		return kfd_pc_sample_query_cap(pdd, args);
	case KFD_IOCTL_PCS_OP_CREATE:
		return kfd_pc_sample_create(pdd, args);
	case KFD_IOCTL_PCS_OP_DESTROY:
		if (pcs_entry->enabled)
			return -EBUSY;
		return kfd_pc_sample_destroy(pdd, args->trace_id, pcs_entry);
	case KFD_IOCTL_PCS_OP_START:
		if (pcs_entry->enabled)
			return -EALREADY;
		return kfd_pc_sample_start(pdd, pcs_entry);
	case KFD_IOCTL_PCS_OP_STOP:
		if (!pcs_entry->enabled)
			return -EALREADY;
		return kfd_pc_sample_stop(pdd, pcs_entry);
	}

	return -EINVAL;
}
