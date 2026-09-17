/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * Copyright 2023 Advanced Micro Devices, Inc.
 */

#ifndef KFD_PC_SAMPLING_H_
#define KFD_PC_SAMPLING_H_

#include "kfd_priv.h"

int kfd_pc_sample(struct kfd_process_device *pdd,
		  struct kfd_ioctl_pc_sample_args *args);
void kfd_pc_sample_release(struct kfd_process_device *pdd);
void kfd_pc_sample_stop_thread(struct kfd_node *node);

#endif /* KFD_PC_SAMPLING_H_ */
