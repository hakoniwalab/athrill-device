/* SPDX-License-Identifier: MIT */
#include "athrill_device_if.h"
#include <stdlib.h>

/* TEST_PLUGIN_OLD_MINOR: a plugin built against ABI 1.9, whose tables did not
 * change since; it must still load (hakoniwalab/athrill-device#52). */
#ifdef TEST_PLUGIN_OLD_MINOR
#define TEST_PLUGIN_MINOR	9U
#else
#define TEST_PLUGIN_MINOR	ATHRILL_DEVICE_ABI_MINOR
#endif

static sint32 create(const AthrillDeviceCreateArgs *args, void **instance)
{
	(void)args;
	*instance = malloc(1U);
	return *instance != NULL ? 0 : -1;
}
static void destroy(void *instance)
{
	free(instance);
}

static const AthrillDeviceModelOps ops = {
#ifdef TEST_PLUGIN_BAD_ABI
	2U,
#else
	ATHRILL_DEVICE_ABI_MAJOR,
#endif
	TEST_PLUGIN_MINOR, sizeof(AthrillDeviceModelOps),
	"test-plugin-model", "1", NULL, create, destroy,
	NULL, NULL, NULL, NULL, NULL, NULL, NULL
};
static const char *const compatibles[] = { "vendor,plugin" };
static const AthrillDeviceModelRegistration registrations[] = {
	{ compatibles, 1U, &ops }
};

#if defined(__GNUC__)
__attribute__((visibility("default")))
#endif
const AthrillDevicePlugin athrill_device_plugin_v1 = {
	ATHRILL_DEVICE_PLUGIN_MAGIC, ATHRILL_DEVICE_ABI_MAJOR,
	TEST_PLUGIN_MINOR, sizeof(AthrillDevicePlugin),
	"test-plugin", "1", registrations, 1U
};
