/*
 * The host's required interface for the models that are not yet created
 * through the Devicetree runtime (targets/<target>/: models ported from a
 * target, hakoniwalab/athrill-device#13).
 *
 * athrill-device declares it; the host (the Athrill target) implements
 * athrill_device_host(), returning its AthrillDeviceTargetOps table with the
 * REQUIRED operations. Models call the host only through it, never the
 * Athrill core or the target directly.
 *
 * Models created by the Devicetree runtime get the same table through
 * AthrillDeviceCreateArgs instead; when the ported models are created that
 * way too (step 4 of #13), this header goes.
 *
 * Copyright 2026 Hakoniwa Laboratory
 * SPDX-License-Identifier: MIT
 */
#ifndef ATHRILL_DEVICE_HOST_H
#define ATHRILL_DEVICE_HOST_H

#include "athrill_device_if.h"

/* Implemented by the host. Every operation takes NULL as its context. */
const AthrillDeviceTargetOps *athrill_device_host(void);

/* Short forms: one call through the table each. */
static inline uint64 athrill_device_host_clock(void)
{
	return athrill_device_host()->time_now(NULL);
}

static inline void athrill_device_host_raise(uint32 irq)
{
	AthrillDeviceIrqLine line;

	line.value = irq;
	(void)athrill_device_host()->irq_set(NULL, line, TRUE);
}

static inline std_bool athrill_device_host_irq_pending(uint32 irq)
{
	AthrillDeviceIrqLine line;

	line.value = irq;
	return athrill_device_host()->irq_is_active(NULL, line);
}

#endif
