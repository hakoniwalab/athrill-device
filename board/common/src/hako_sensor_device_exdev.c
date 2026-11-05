/*
 * Athrill EXDEV glue of the Hakoniwa sensor device.
 *
 * memory.txt:  DEV, 0x20000000, <path to libhako_sensor_device>
 *
 * Device configuration (athrill3 -d):
 *   HAKO_SENSOR_DEVICE_IRQ               PLIC source of the data-ready interrupt (default 5)
 *   HAKO_SENSOR_DEVICE_ASSET             Hakoniwa asset name: samples come from Hakoniwa
 *   HAKO_SENSOR_DEVICE_ROBOT             robot name of the PDU channels
 *   HAKO_SENSOR_DEVICE_SENSOR_CHANNEL    channel of hil_sensor (HakoHilSensor)
 *   HAKO_SENSOR_DEVICE_GPS_CHANNEL       channel of hil_gps (HakoHilGps)
 *   HAKO_SENSOR_DEVICE_ACTUATOR_CHANNEL  channel of hil_actuator (HakoHilActuatorControls)
 *   HAKO_SENSOR_DEVICE_TEST_PERIOD       without Hakoniwa: every N usec of simulated time, a
 *                                        sample of a vehicle at rest (GPS every 25th)
 *   HAKO_SENSOR_DEVICE_TEST_LAT_DEG      the test vehicle (all eight needed with TEST_PERIOD):
 *   HAKO_SENSOR_DEVICE_TEST_LON_DEG        GPS latitude and longitude (deg), altitude (m),
 *   HAKO_SENSOR_DEVICE_TEST_ALT_M          pressure (hPa) and temperature (degC) of the
 *   HAKO_SENSOR_DEVICE_TEST_PRESSURE_HPA   barometer, magnetic field north, east, down
 *   HAKO_SENSOR_DEVICE_TEST_TEMPERATURE_C  (gauss). Level and facing north. PX4 stores a
 *   HAKO_SENSOR_DEVICE_TEST_MAG_N_GAUSS    barometer offset and the magnetic declination
 *   HAKO_SENSOR_DEVICE_TEST_MAG_E_GAUSS    at its first boot: give the values of the place
 *   HAKO_SENSOR_DEVICE_TEST_MAG_D_GAUSS    where the vehicle will fly.
 *
 * Samples are latched between CPU batches, at their exact simulated time:
 * supply_clock() caps the next batch at the clock of the next sample.
 *
 * With Hakoniwa the device moves on when data arrives: it waits (blocking
 * Athrill) for a hil_sensor newer than the last one, runs the CPU up to its
 * time_usec (relative to the first sample's), latches it (and hil_gps if that changed), raises the interrupt,
 * writes the motor values the CPU committed last to hil_actuator with that
 * time, and waits for the next one. Drone PRO steps its physics when it
 * reads hil_actuator and then writes the next hil_sensor, so the order of
 * everything is fixed by the data, not by wall-clock time.
 */
#include "athrill_exdev.h"
#include "hako_sensor_device.h"
#include "hako_sensor_device_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define DEFAULT_CPU_MHZ 100U
#define TEST_GPS_EVERY 25U

static HakoSensorDevice device;
static AthrillExDevOperationType *athrill;
static uint32 irq = HAKO_SENSOR_DEVICE_PLIC_SOURCE;
static uint32 cpu_mhz = DEFAULT_CPU_MHZ;
static uint32 test_period_usec;
static uint64 test_count;

typedef struct {
	double lat_deg;
	double lon_deg;
	double alt_m;
	double pressure_hpa;
	double temperature_c;
	double mag_gauss[3];
} TestVehicle;
static TestVehicle test_vehicle;
static uint8 register_space[HAKO_SENSOR_DEVICE_SIZE];

static bool hakoniwa_mode;
static bool has_pending;
static bool pending_has_gps;
static HakoSensorSample pending_sensor;
static HakoGpsSample pending_gps;
static uint64 last_time_usec;
/*
 * Drone PRO's HIL time (as for PX4 SITL) starts at its wall-clock start time;
 * Athrill's starts at 0. The difference is fixed at the first sample, so
 * every later sample lands at its own time relative to the first.
 */
static bool has_time_offset;
static uint64 time_offset_usec;

static uint32 offset_of(const MpuAddressRegionType *region, AthrillAddrType address)
{
	return (uint32)(address - region->start);
}

static Std_ReturnType get_data8(MpuAddressRegionType *region, CoreIdType core_id, AthrillAddrType address, uint8 *data)
{
	uint32 offset = offset_of(region, address);

	(void)core_id;
	*data = (uint8)(hako_sensor_device_read(&device, offset & ~3U) >> (8U * (offset & 3U)));
	return STD_E_OK;
}

static Std_ReturnType get_data16(MpuAddressRegionType *region, CoreIdType core_id, AthrillAddrType address, uint16 *data)
{
	uint32 offset = offset_of(region, address);

	(void)core_id;
	*data = (uint16)(hako_sensor_device_read(&device, offset & ~3U) >> (8U * (offset & 2U)));
	return STD_E_OK;
}

static Std_ReturnType get_data32(MpuAddressRegionType *region, CoreIdType core_id, AthrillAddrType address, uint32 *data)
{
	(void)core_id;
	*data = hako_sensor_device_read(&device, offset_of(region, address));
	return STD_E_OK;
}

/* The registers are written 32 bits at a time only. */
static Std_ReturnType put_data8(MpuAddressRegionType *region, CoreIdType core_id, AthrillAddrType address, uint8 data)
{
	(void)region;
	(void)core_id;
	(void)address;
	(void)data;
	return STD_E_SEGV;
}

static Std_ReturnType put_data16(MpuAddressRegionType *region, CoreIdType core_id, AthrillAddrType address, uint16 data)
{
	(void)region;
	(void)core_id;
	(void)address;
	(void)data;
	return STD_E_SEGV;
}

static Std_ReturnType put_data32(MpuAddressRegionType *region, CoreIdType core_id, AthrillAddrType address, uint32 data)
{
	(void)core_id;
	hako_sensor_device_write(&device, offset_of(region, address), data);
	return STD_E_OK;
}

/* No direct pointer: every access goes through the register behaviour. */
static Std_ReturnType get_pointer(MpuAddressRegionType *region, CoreIdType core_id, AthrillAddrType address, uint8 **data)
{
	(void)region;
	(void)core_id;
	(void)address;
	(void)data;
	return STD_E_SEGV;
}

static MpuAddressRegionOperationType operations = {
	get_data8, get_data16, get_data32, put_data8, put_data16, put_data32, get_pointer
};

static void raise(uint32 bits)
{
	if ((bits != 0U) && (athrill != NULL) && (athrill->intr.raise_intr != NULL)) {
		athrill->intr.raise_intr(irq);
	}
}

/*
 * The test vehicle of the device configuration (HAKO_SENSOR_DEVICE_TEST_*), at
 * rest, level, facing north. A small repeating dither stands in for sensor
 * noise: PX4 takes a sensor whose value never changes as stale.
 */
static void test_feed(uint64 time_usec)
{
	static const float dither[5] = { -0.002F, 0.001F, 0.0F, 0.002F, -0.001F };
	HakoSensorSample sensor;
	HakoGpsSample gps;
	float d = dither[test_count % 5U];

	memset(&sensor, 0, sizeof(sensor));
	sensor.time_usec = time_usec;
	sensor.acc[0] = d;
	sensor.acc[1] = -d;
	sensor.acc[2] = -9.80665F + d;
	sensor.gyro[0] = d * 0.1F;
	sensor.gyro[1] = -d * 0.1F;
	sensor.gyro[2] = d * 0.05F;
	sensor.mag[0] = (float)test_vehicle.mag_gauss[0] + d;
	sensor.mag[1] = (float)test_vehicle.mag_gauss[1] - d;
	sensor.mag[2] = (float)test_vehicle.mag_gauss[2] + d;
	sensor.abs_pressure = (float)test_vehicle.pressure_hpa + d * 10.0F;
	sensor.temperature = (float)test_vehicle.temperature_c;
	sensor.fields_updated = HAKO_FIELDS_IMU | HAKO_FIELDS_MAG | HAKO_FIELDS_BARO;
	raise(hako_sensor_device_latch_sensor(&device, &sensor));
	if ((test_count % TEST_GPS_EVERY) == 0U) {
		memset(&gps, 0, sizeof(gps));
		gps.time_usec = time_usec;
		gps.lat = (int32_t)(test_vehicle.lat_deg * 1e7 + (test_vehicle.lat_deg < 0.0 ? -0.5 : 0.5));
		gps.lon = (int32_t)(test_vehicle.lon_deg * 1e7 + (test_vehicle.lon_deg < 0.0 ? -0.5 : 0.5));
		gps.alt = (int32_t)(test_vehicle.alt_m * 1e3 + (test_vehicle.alt_m < 0.0 ? -0.5 : 0.5));
		gps.eph = 100;
		gps.epv = 150;
		gps.satellites_visible = 12;
		gps.fix_type = 3;
		raise(hako_sensor_device_latch_gps(&device, &gps));
	}
	test_count++;
}

/*
 * A number of the device configuration. Athrill keeps a whole decimal
 * number as a value (two's complement in 32 bits) and anything else, such as
 * 47.641468, as a string.
 */
static bool get_number(AthrillExDevOperationType *ops, const char *key, double *number)
{
	char *text;
	char *end;
	uint32 value;

	if (ops->param.get_devcfg_value(key, &value) == STD_E_OK) {
		*number = (double)(int32_t)value;
		return true;
	}
	if (ops->param.get_devcfg_string(key, &text) == STD_E_OK) {
		*number = strtod(text, &end);
		if ((end != text) && (*end == '\0')) {
			return true;
		}
		printf("ERROR: hako_sensor_device: %s is not a number: %s\n", key, text);
		return false;
	}
	printf("ERROR: hako_sensor_device: %s is needed with HAKO_SENSOR_DEVICE_TEST_PERIOD\n", key);
	return false;
}

static bool load_test_vehicle(AthrillExDevOperationType *ops)
{
	bool ok = true;

	ok = get_number(ops, "HAKO_SENSOR_DEVICE_TEST_LAT_DEG", &test_vehicle.lat_deg) && ok;
	ok = get_number(ops, "HAKO_SENSOR_DEVICE_TEST_LON_DEG", &test_vehicle.lon_deg) && ok;
	ok = get_number(ops, "HAKO_SENSOR_DEVICE_TEST_ALT_M", &test_vehicle.alt_m) && ok;
	ok = get_number(ops, "HAKO_SENSOR_DEVICE_TEST_PRESSURE_HPA", &test_vehicle.pressure_hpa) && ok;
	ok = get_number(ops, "HAKO_SENSOR_DEVICE_TEST_TEMPERATURE_C", &test_vehicle.temperature_c) && ok;
	ok = get_number(ops, "HAKO_SENSOR_DEVICE_TEST_MAG_N_GAUSS", &test_vehicle.mag_gauss[0]) && ok;
	ok = get_number(ops, "HAKO_SENSOR_DEVICE_TEST_MAG_E_GAUSS", &test_vehicle.mag_gauss[1]) && ok;
	ok = get_number(ops, "HAKO_SENSOR_DEVICE_TEST_MAG_D_GAUSS", &test_vehicle.mag_gauss[2]) && ok;
	return ok;
}

static void wait_for_sample(void)
{
	while (!has_pending) {
		if (!hako_sensor_link_poll_events()) {
			(void)usleep(1000);
			continue;
		}
		if (hako_sensor_link_read(last_time_usec, &pending_sensor, &pending_gps, &pending_has_gps)) {
			has_pending = true;
		}
		else {
			(void)usleep(20);
		}
	}
}

static void hakoniwa_supply_clock(DeviceClockType *clock)
{
	HakoActuatorSample actuator;
	uint64 next;

	for (;;) {
		wait_for_sample();
		if (!has_time_offset) {
			time_offset_usec = pending_sensor.time_usec - (clock->clock / cpu_mhz);
			has_time_offset = true;
		}
		next = (pending_sensor.time_usec - time_offset_usec) * cpu_mhz;
		if (clock->clock < next) {
			if ((next - clock->clock) < clock->min_intr_interval) {
				clock->min_intr_interval = next - clock->clock;
			}
			return;
		}
		raise(hako_sensor_device_latch_sensor(&device, &pending_sensor));
		if (pending_has_gps) {
			raise(hako_sensor_device_latch_gps(&device, &pending_gps));
		}
		last_time_usec = pending_sensor.time_usec;
		has_pending = false;
		if (!hako_sensor_device_committed(&device, &actuator)) {
			memset(&actuator, 0, sizeof(actuator));
		}
		actuator.time_usec = last_time_usec;
		(void)hako_sensor_link_write_actuator(&actuator);
		hako_sensor_link_notify_time(last_time_usec);
	}
}

static void devinit(MpuAddressRegionType *region, AthrillExDevOperationType *ops)
{
	HakoSensorLinkConfig link;
	char *text;
	uint32 value;

	(void)region;
	athrill = ops;
	hako_sensor_device_reset(&device);
	if (ops->param.get_devcfg_value("HAKO_SENSOR_DEVICE_IRQ", &value) == STD_E_OK) {
		irq = value;
	}
	if (ops->param.get_devcfg_value("DEVICE_CPU_FREQ", &value) == STD_E_OK) {
		cpu_mhz = value;
	}
	if (ops->param.get_devcfg_value("HAKO_SENSOR_DEVICE_TEST_PERIOD", &value) == STD_E_OK) {
		if (!load_test_vehicle(ops)) {
			printf("ERROR: hako_sensor_device: no test vehicle, no samples\n");
		}
		else {
			test_period_usec = value;
			printf("hako_sensor_device: test vehicle every %u usec at %.7f %.7f %.3f m, %.3f hPa\n",
				value, test_vehicle.lat_deg, test_vehicle.lon_deg, test_vehicle.alt_m, test_vehicle.pressure_hpa);
		}
	}
	if (ops->param.get_devcfg_string("HAKO_SENSOR_DEVICE_ASSET", &text) != STD_E_OK) {
		return;
	}
	memset(&link, 0, sizeof(link));
	link.asset_name = text;
	if (ops->param.get_devcfg_string("HAKO_SENSOR_DEVICE_ROBOT", &text) == STD_E_OK) {
		link.robot_name = text;
	}
	if ((ops->param.get_devcfg_value("HAKO_SENSOR_DEVICE_SENSOR_CHANNEL", &link.sensor_channel) != STD_E_OK)
		|| (ops->param.get_devcfg_value("HAKO_SENSOR_DEVICE_GPS_CHANNEL", &link.gps_channel) != STD_E_OK)
		|| (ops->param.get_devcfg_value("HAKO_SENSOR_DEVICE_ACTUATOR_CHANNEL", &link.actuator_channel) != STD_E_OK)
		|| (link.robot_name == NULL)) {
		printf("ERROR: hako_sensor_device: HAKO_SENSOR_DEVICE_ROBOT and the three channels are needed\n");
		return;
	}
	if (!hako_sensor_link_initialize(&link)) {
		printf("ERROR: hako_sensor_device: can not register Hakoniwa asset %s\n", link.asset_name);
		return;
	}
	printf("hako_sensor_device: Hakoniwa asset %s, robot %s, channels sensor %u gps %u actuator %u\n",
		link.asset_name, link.robot_name, link.sensor_channel, link.gps_channel, link.actuator_channel);
	hakoniwa_mode = true;
}

static void cleanup(void)
{
	if (hakoniwa_mode) {
		hako_sensor_link_cleanup();
	}
}

static void supply_clock(DeviceClockType *clock)
{
	uint64 period;
	uint64 next;

	if (hakoniwa_mode) {
		hakoniwa_supply_clock(clock);
		return;
	}
	if (test_period_usec == 0U) {
		return;
	}
	period = (uint64)test_period_usec * cpu_mhz;
	next = (test_count + 1U) * period;
	if (clock->clock >= next) {
		test_feed(next / cpu_mhz);
		next += period;
	}
	if ((next - clock->clock) < clock->min_intr_interval) {
		clock->min_intr_interval = next - clock->clock;
	}
}

ATHRILL_EXDEV_EXPORT AthrillExDeviceType athrill_ex_device = {
	.header.magicno = ATHRILL_EXTERNAL_DEVICE_MAGICNO,
	.header.version = ATHRILL_EXTERNAL_DEVICE_VERSION,
	.header.memory_size = HAKO_SENSOR_DEVICE_SIZE,
	.datap = (char *)register_space,
	.ops = &operations,
	.devinit = devinit,
	.supply_clock = supply_clock,
	.cleanup = cleanup,
};
