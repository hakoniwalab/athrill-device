#include "hako_sensor_device_link.h"

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

namespace {

uint32_t floatBits(float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

void printSensor(const HakoSensorSample& sensor, uint64_t relative_time)
{
    std::printf(
        "sensor time=%" PRIu64
        " acc=%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32
        " gyro=%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32
        " mag=%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32
        " pressure=%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32
        " fields=%08" PRIx32 " id=%02" PRIx8 "\n",
        relative_time,
        floatBits(sensor.acc[0]), floatBits(sensor.acc[1]), floatBits(sensor.acc[2]),
        floatBits(sensor.gyro[0]), floatBits(sensor.gyro[1]), floatBits(sensor.gyro[2]),
        floatBits(sensor.mag[0]), floatBits(sensor.mag[1]), floatBits(sensor.mag[2]),
        floatBits(sensor.abs_pressure), floatBits(sensor.diff_pressure), floatBits(sensor.pressure_alt),
        floatBits(sensor.temperature), sensor.fields_updated, sensor.id);
}

void printGps(const HakoGpsSample& gps, uint64_t epoch)
{
    std::printf(
        "gps time=%" PRIu64 " lat=%" PRId32 " lon=%" PRId32 " alt=%" PRId32
        " eph=%" PRIu16 " epv=%" PRIu16 " vel=%" PRIu16
        " vn=%" PRId16 " ve=%" PRId16 " vd=%" PRId16 " cog=%" PRIu16
        " satellites=%" PRIu8 " id=%" PRIu8 " yaw=%" PRIu8 " fix=%" PRIu8 "\n",
        gps.time_usec - epoch, gps.lat, gps.lon, gps.alt, gps.eph, gps.epv, gps.vel,
        gps.vn, gps.ve, gps.vd, gps.cog, gps.satellites_visible, gps.id, gps.yaw, gps.fix_type);
}

HakoActuatorSample fixedActuator(uint64_t time_usec, uint64_t sample_index)
{
    static const float levels[] = { 0.0F, 0.125F, 0.25F, 0.5F, 0.75F };
    HakoActuatorSample actuator{};
    actuator.time_usec = time_usec;
    for (uint32_t channel = 0; channel < HAKO_ACTUATOR_CHANNELS; channel++) {
        actuator.controls[channel] = levels[(sample_index + channel) % (sizeof(levels) / sizeof(levels[0]))];
    }
    return actuator;
}

void printActuator(const HakoActuatorSample& actuator, uint64_t relative_time)
{
    std::printf("actuator time=%" PRIu64 " controls=", relative_time);
    for (uint32_t channel = 0; channel < HAKO_ACTUATOR_CHANNELS; channel++) {
        std::printf("%s%08" PRIx32, channel == 0 ? "" : ",", floatBits(actuator.controls[channel]));
    }
    std::printf(" mode=%02" PRIx8 " flags=%016" PRIx64 "\n", actuator.mode, actuator.flags);
}

}  // namespace

int main(int argc, char **argv)
{
    const bool hakoniwa_mode = argc > 1 && std::strcmp(argv[1], "--hakoniwa") == 0;
    if ((!hakoniwa_mode && argc != 4) || (hakoniwa_mode && argc != 9)) {
        std::fprintf(stderr,
            "usage: %s AIRFRAME_PATH SAMPLE_COUNT SENSOR_PERIOD_USEC\n"
            "       %s --hakoniwa ASSET ROBOT SENSOR_CH GPS_CH ACTUATOR_CH SAMPLE_COUNT SENSOR_PERIOD_USEC\n",
            argv[0], argv[0]);
        return 2;
    }
    const int count_argument = hakoniwa_mode ? 7 : 2;
    const uint64_t sample_count = std::strtoull(argv[count_argument], nullptr, 10);
    const uint64_t sensor_period_usec = std::strtoull(argv[count_argument + 1], nullptr, 10);
    if (sample_count == 0 || sensor_period_usec == 0) {
        std::fprintf(stderr, "sample count and period must be nonzero\n");
        return 2;
    }

    HakoSensorLinkConfig config{};
    if (hakoniwa_mode) {
        config.asset_name = argv[2];
        config.robot_name = argv[3];
        config.sensor_channel = static_cast<uint32_t>(std::strtoul(argv[4], nullptr, 10));
        config.gps_channel = static_cast<uint32_t>(std::strtoul(argv[5], nullptr, 10));
        config.actuator_channel = static_cast<uint32_t>(std::strtoul(argv[6], nullptr, 10));
    }
    else {
        config.aircraft_config_path = argv[1];
    }
    if (!hako_sensor_link_initialize(&config)) {
        return 1;
    }

    HakoSensorSample sensor{};
    HakoGpsSample gps{};
    bool has_gps = false;
    uint64_t last_time_usec = 0;
    uint64_t epoch = 0;
    int result = 0;
    for (uint64_t sample_index = 0; sample_index < sample_count; sample_index++) {
        while (!hako_sensor_link_poll_events()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        while (!hako_sensor_link_read(last_time_usec, &sensor, &gps, &has_gps)) {
            std::this_thread::sleep_for(std::chrono::microseconds(20));
        }
        if (sample_index == 0) {
            epoch = sensor.time_usec;
        }
        else if (sensor.time_usec - last_time_usec != sensor_period_usec) {
            std::fprintf(stderr, "sample %" PRIu64 " interval is %" PRIu64 ", expected %" PRIu64 "\n",
                sample_index, sensor.time_usec - last_time_usec, sensor_period_usec);
            result = 1;
            break;
        }
        const uint64_t relative_time = sensor.time_usec - epoch;
        printSensor(sensor, relative_time);
        if (has_gps) {
            printGps(gps, epoch);
        }
        const HakoActuatorSample actuator = fixedActuator(sensor.time_usec, sample_index);
        printActuator(actuator, relative_time);
        if (!hako_sensor_link_write_actuator(&actuator)) {
            std::fprintf(stderr, "failed to write actuator %" PRIu64 "\n", sample_index);
            result = 1;
            break;
        }
        last_time_usec = sensor.time_usec;
        hako_sensor_link_notify_time(relative_time);
    }

    hako_sensor_link_cleanup();
    return result;
}
