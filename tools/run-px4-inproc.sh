#!/bin/sh
# Run the PX4 STM32 example with the aircraft inside Athrill. The example
# directory must be a disposable copy with its first-boot fram.bin and sd.img.
set -eu

usage() {
    echo "usage: $0 EXAMPLE_DIR PX4_ELF ATHRILL3 INPROC_DEVICE HAKO_DRONE_ROOT AIRFRAME_CONFIG" >&2
    echo "environment: ATHRILL_TICKS=-1 HAKO_AIRCRAFT_WORKER=1 HAKO_SENSOR_DEVICE_TRACE_PATH=<file>" >&2
    exit 2
}

[ "$#" -eq 6 ] || usage

absolute() {
    path=$1
    directory=$(dirname "$path")
    filename=$(basename "$path")
    printf '%s/%s\n' "$(cd "$directory" && pwd)" "$filename"
}

example_dir=$(absolute "$1")
px4_elf=$(absolute "$2")
athrill=$(absolute "$3")
device=$(absolute "$4")
drone_root=$(absolute "$5")
airframe=$(absolute "$6")
script_dir=$(cd "$(dirname "$0")" && pwd)
device_tree=$(absolute "${DEVICE_CONFIG_DEVICE_TREE:-$script_dir/../boards/pixhawk4/pixhawk4-hakoniwa.dtb}")
ticks=${ATHRILL_TICKS:--1}
worker=${HAKO_AIRCRAFT_WORKER:-1}
trace=${HAKO_SENSOR_DEVICE_TRACE_PATH:-}

case "$worker" in
    0|1) ;;
    *) echo "HAKO_AIRCRAFT_WORKER must be 0 or 1" >&2; exit 2 ;;
esac

[ -f "$example_dir/device_config.txt" ] || usage
[ -f "$example_dir/memory.txt" ] || usage
[ -f "$px4_elf" ] || usage
[ -x "$athrill" ] || usage
[ -f "$device" ] || usage
[ -d "$drone_root" ] || usage
[ -e "$airframe" ] || usage

device_config=$example_dir/device_config-inproc.txt
memory=$example_dir/memory-inproc.txt
config_tmp=$device_config.tmp.$$
memory_tmp=$memory.tmp.$$
cleanup_temporary() {
    rm -f "$config_tmp" "$memory_tmp"
}
trap cleanup_temporary EXIT INT TERM

# The in-process link gets its channel layout from the airframe. Remove only
# transport-specific lines; preserve the board, UART and MAVLink settings.
awk -v fram="$example_dir/fram.bin" -v sdcard="$example_dir/sd.img" '
    $1 == "DEVICE_CONFIG_STM32_FRAM_FILE" { print $1, fram; next }
    $1 == "DEVICE_CONFIG_STM32_SDCARD_IMAGE" { print $1, sdcard; next }
    $1 == "HAKO_SENSOR_DEVICE_ASSET" { next }
    $1 == "HAKO_SENSOR_DEVICE_ROBOT" { next }
    $1 == "HAKO_SENSOR_DEVICE_SENSOR_CHANNEL" { next }
    $1 == "HAKO_SENSOR_DEVICE_GPS_CHANNEL" { next }
    $1 == "HAKO_SENSOR_DEVICE_ACTUATOR_CHANNEL" { next }
    $1 == "HAKO_AIRCRAFT_CONFIG_PATH" { next }
    $1 == "HAKO_AIRCRAFT_WORKER" { next }
    $1 == "HAKO_SENSOR_DEVICE_TRACE_PATH" { next }
    $1 == "DEVICE_CONFIG_DEVICE_TREE" { next }
    $1 == "DEVICE_CONFIG_DEVICE_PLUGIN_0" { next }
    $1 == "DEVICE_CONFIG_STM32_UART2_TCP_PORT" { next }
    { print }
' "$example_dir/device_config.txt" >"$config_tmp"
printf 'DEVICE_CONFIG_DEVICE_TREE %s\n' "$device_tree" >>"$config_tmp"
printf 'DEVICE_CONFIG_DEVICE_PLUGIN_0 hakoniwa%%2Csensor-device=%s\n' "$device" >>"$config_tmp"
printf 'HAKO_AIRCRAFT_CONFIG_PATH %s\n' "$airframe" >>"$config_tmp"
printf 'HAKO_AIRCRAFT_WORKER %s\n' "$worker" >>"$config_tmp"
if [ -n "$trace" ]; then
    trace=$(absolute "$trace")
    printf 'HAKO_SENSOR_DEVICE_TRACE_PATH %s\n' "$trace" >>"$config_tmp"
fi
mv "$config_tmp" "$device_config"

awk '$1 != "DEV," || $2 != "0x60000000,"' "$example_dir/memory.txt" >"$memory_tmp"
mv "$memory_tmp" "$memory"

trap - EXIT INT TERM
echo "PX4 on Athrill with the in-process aircraft."
echo "TCP 5760 is disabled; use a configured file or other non-TCP control sink if needed."
# Release configs name the MuJoCo XML relative to the release root. The two
# writable board images above are absolute, so Athrill can safely run here.
cd "$drone_root"
exec "$athrill" -c1 -m "$memory" -d "$device_config" -t "$ticks" "$px4_elf"
