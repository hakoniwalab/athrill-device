#!/bin/sh
# Run the two-process PX4 STM32 reference: aircraft service, Athrill, then
# hako-cmd start. The example directory must be a disposable prepared copy.
set -eu

usage() {
    echo "usage: $0 EXAMPLE_DIR PX4_ELF ATHRILL3 SENSOR_DEVICE HAKO_DRONE_ROOT HAKO_ROOT AIRFRAME_CONFIG" >&2
    echo "environment: ATHRILL_TICKS=-1 HAKO_CONFIG_PATH=<file> HAKO_SENSOR_DEVICE_TRACE_PATH=<file>" >&2
    exit 2
}

[ "$#" -eq 7 ] || usage

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
hako_root=$(absolute "$6")
airframe=$(absolute "$7")
script_dir=$(cd "$(dirname "$0")" && pwd)
device_tree=$(absolute "${DEVICE_CONFIG_DEVICE_TREE:-$script_dir/../boards/pixhawk4/pixhawk4.dtb}")
ticks=${ATHRILL_TICKS:--1}
trace=${HAKO_SENSOR_DEVICE_TRACE_PATH:-}
host=${HAKO_SERVICE_HOST:-127.0.0.1}
port=${HAKO_SERVICE_PORT:-4560}
timeout_seconds=${HAKO_START_TIMEOUT_SECONDS:-60}
pdu_definition=${HAKO_PDU_DEFINITION:-$drone_root/config/pdudef/drone-pdudef-athrill-px4.json}
pdu_definition=$(absolute "$pdu_definition")
service=$drone_root/mac/mac-main_hako_aircraft_service_px4
hako_cmd=$hako_root/bin/hako-cmd
service_log=${HAKO_AIRCRAFT_SERVICE_LOG:-$example_dir/hakoniwa-aircraft-service.log}

[ -f "$example_dir/device_config.txt" ] || usage
[ -f "$example_dir/memory.txt" ] || usage
[ -f "$px4_elf" ] || usage
[ -x "$athrill" ] || usage
[ -f "$device" ] || usage
[ -x "$service" ] || usage
[ -x "$hako_cmd" ] || usage
[ -e "$airframe" ] || usage
[ -f "$pdu_definition" ] || usage

device_config=$example_dir/device_config-hakoniwa-run.txt
memory=$example_dir/memory-hakoniwa-run.txt
config_tmp=$device_config.tmp.$$
memory_tmp=$memory.tmp.$$
service_pid=
athrill_pid=

library_path=$hako_root/lib
if [ -n "${DYLD_LIBRARY_PATH:-}" ]; then
    library_path=$library_path:$DYLD_LIBRARY_PATH
fi
DYLD_LIBRARY_PATH=$library_path
export DYLD_LIBRARY_PATH

cleanup() {
    set +e
    rm -f "$config_tmp" "$memory_tmp"
    if [ -n "$athrill_pid" ] && kill -0 "$athrill_pid" 2>/dev/null; then
        kill "$athrill_pid" 2>/dev/null
        wait "$athrill_pid" 2>/dev/null
    fi
    "$hako_cmd" stop >/dev/null 2>&1
    if [ -n "$service_pid" ] && kill -0 "$service_pid" 2>/dev/null; then
        kill "$service_pid" 2>/dev/null
        wait "$service_pid" 2>/dev/null
    fi
    "$hako_cmd" recover >/dev/null 2>&1
}
on_signal() {
    exit 130
}
trap cleanup EXIT
trap on_signal INT TERM

awk '
    $1 == "HAKO_SENSOR_DEVICE_TRACE_PATH" { next }
    $1 == "DEVICE_CONFIG_DEVICE_TREE" { next }
    $1 == "DEVICE_CONFIG_DEVICE_PLUGIN_0" { next }
    $1 == "DEVICE_CONFIG_STM32_UART2_TCP_PORT" { next }
    { print }
' "$example_dir/device_config.txt" >"$config_tmp"
printf 'DEVICE_CONFIG_DEVICE_TREE %s\n' "$device_tree" >>"$config_tmp"
printf 'DEVICE_CONFIG_DEVICE_PLUGIN_0 hakoniwa%%2Csensor-device=%s\n' "$device" >>"$config_tmp"
if [ -n "$trace" ]; then
    trace=$(absolute "$trace")
    printf 'HAKO_SENSOR_DEVICE_TRACE_PATH %s\n' "$trace" >>"$config_tmp"
fi
mv "$config_tmp" "$device_config"

asset=$(awk '$1 == "HAKO_SENSOR_DEVICE_ASSET" { print $2; exit }' "$device_config")
[ -n "$asset" ] || usage

awk '$1 != "DEV," || $2 != "0x60000000,"' "$example_dir/memory.txt" >"$memory_tmp"
mv "$memory_tmp" "$memory"

# recover removes only a stopped runtime. It makes a previous interrupted run
# explicit without sending signals to any process not started by this script.
"$hako_cmd" recover >/dev/null 2>&1 || true

(
    cd "$drone_root"
    exec env -u DYLD_LIBRARY_PATH \
        "$service" "$host" "$port" "$airframe" "$pdu_definition" \
        --no-mujoco-viewer --real-sleep-msec 0
) >"$service_log" 2>&1 &
service_pid=$!

attempts=$((timeout_seconds * 10))
i=0
while ! grep -q 'HakoniwaDroneService::registerService() done' "$service_log" 2>/dev/null; do
    if ! kill -0 "$service_pid" 2>/dev/null; then
        echo "aircraft service exited; see $service_log" >&2
        exit 1
    fi
    i=$((i + 1))
    if [ "$i" -ge "$attempts" ]; then
        echo "timed out waiting for the aircraft service; see $service_log" >&2
        exit 1
    fi
    sleep 0.1
done

(
    cd "$example_dir"
    exec "$athrill" -c1 -m "$memory" -d "$device_config" -t "$ticks" "$px4_elf"
) &
athrill_pid=$!

i=0
while ! "$hako_cmd" ls 2>/dev/null | grep -F -q "$asset"; do
    if ! kill -0 "$athrill_pid" 2>/dev/null; then
        echo "Athrill exited before registering asset $asset" >&2
        exit 1
    fi
    i=$((i + 1))
    if [ "$i" -ge "$attempts" ]; then
        echo "timed out waiting for Athrill asset $asset" >&2
        exit 1
    fi
    sleep 0.1
done

"$hako_cmd" start
echo "PX4 and the Hakoniwa aircraft service are running."
echo "TCP 5760 is disabled; use a configured file or other non-TCP control sink if needed."
set +e
wait "$athrill_pid"
result=$?
set -e
athrill_pid=
exit "$result"
