#!/usr/bin/env bash
#
# HorseShitBot pre-flight diagnostics.
#
# Default:
#   Passive checks only. Does NOT write to motors or command motion.
#
# Active ROS check:
#   ./scripts/robot_check.sh --active
#
# --active starts mks_bus_node and gamepad_teleop_node only.
# It NEVER starts wheel_driver_node and NEVER sends a speed/move command.
# Note: mks_bus_node initializes detected motors (mode/microsteps/enable).
#
# ADC check:
#   ./scripts/robot_check.sh --active --adc
#
set -o pipefail

ACTIVE=false
CHECK_ADC=false

for arg in "$@"; do
    case "$arg" in
        --active) ACTIVE=true ;;
        --adc) CHECK_ADC=true ;;
        *)
            echo "Usage: $0 [--active] [--adc]"
            exit 2
            ;;
    esac
done

if [ "$CHECK_ADC" = true ] && [ "$ACTIVE" = false ]; then
    echo "ERROR: --adc requires --active"
    exit 2
fi

PASS=0
WARN=0
FAIL=0
PIDS=""

ok() {
    echo "[PASS] $*"
    PASS=$((PASS + 1))
}

warn() {
    echo "[WARN] $*"
    WARN=$((WARN + 1))
}

fail() {
    echo "[FAIL] $*"
    FAIL=$((FAIL + 1))
}

section() {
    echo
    echo "============================================================"
    echo "$*"
    echo "============================================================"
}

cleanup() {
    if [ -n "$PIDS" ]; then
        echo
        echo "Stopping temporary diagnostic ROS nodes..."

        for pid in $PIDS; do
            # Every diagnostic process is started with setsid, so its PID
            # is also its process-group ID. Kill the complete ROS process
            # tree, not only the `ros2 run` wrapper.
            kill -TERM -- "-$pid" 2>/dev/null ||                 kill -TERM "$pid" 2>/dev/null || true
        done

        sleep 1

        for pid in $PIDS; do
            if kill -0 "$pid" 2>/dev/null; then
                kill -KILL -- "-$pid" 2>/dev/null ||                     kill -KILL "$pid" 2>/dev/null || true
            fi
        done

        wait 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
PARAMS="$REPO_DIR/src/horseshitbot/config/params.yaml"
BATTERY_PARAMS="$REPO_DIR/src/horseshitbot/config/battery_modbus.yaml"

cd "$REPO_DIR" || exit 1

section "HORSESHITBOT PRE-FLIGHT"

echo "Host   : $(hostname)"
echo "User   : $(whoami)"
echo "Repo   : $REPO_DIR"
echo "Branch : $(git branch --show-current 2>/dev/null || echo unknown)"
echo "Commit : $(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo "IP     : $(hostname -I 2>/dev/null || true)"

section "1. GIT / SOFTWARE"

BRANCH="$(git branch --show-current 2>/dev/null || true)"
if [ "$BRANCH" = "integration/robot-bringup" ]; then
    ok "Correct integration branch"
else
    warn "Current branch is '$BRANCH', expected integration/robot-bringup"
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    ok "ROS 2 Humble found"
else
    fail "/opt/ros/humble/setup.bash missing"
fi

if [ -f "$REPO_DIR/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$REPO_DIR/install/setup.bash"
    ok "Workspace install found"
else
    fail "install/setup.bash missing — rebuild workspace"
fi

if ros2 pkg prefix horseshitbot >/dev/null 2>&1; then
    ok "horseshitbot ROS package available"
else
    fail "horseshitbot ROS package not found"
fi

if python3 -c 'import pymodbus' >/dev/null 2>&1; then
    ok "pymodbus import works"
else
    fail "Python pymodbus unavailable"
fi

if python3 -c 'import evdev' >/dev/null 2>&1; then
    ok "evdev import works"
else
    fail "Python evdev unavailable — gamepad node cannot read controller"
fi

section "2. SERIAL / RS485"

if [ -e /dev/mksbus ]; then
    ok "/dev/mksbus exists -> $(readlink -f /dev/mksbus)"
else
    fail "/dev/mksbus does not exist"
fi

if [ -r /dev/mksbus ] && [ -w /dev/mksbus ]; then
    ok "Current user can read/write /dev/mksbus"
else
    fail "Current user lacks read/write permission for /dev/mksbus"
fi

echo
echo "Serial devices:"
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/mksbus 2>/dev/null || true

if lsusb 2>/dev/null | grep -Eqi '1a86:7523|CH340|QinHeng'; then
    ok "CH340 USB-RS485 adapter detected"
else
    warn "CH340 USB-RS485 adapter not identified by lsusb"
fi

section "3. CONFIGURATION"

if grep -Eq 'port:[[:space:]]*"/dev/mksbus"' "$PARAMS"; then
    ok "MKS configured for /dev/mksbus"
else
    fail "MKS port is not /dev/mksbus"
fi

if grep -Eq 'baud:[[:space:]]*19200' "$PARAMS"; then
    ok "MKS configured for 19200 baud"
else
    fail "Expected MKS baud 19200"
fi

if grep -Eq 'motor_ids:[[:space:]]*\[1,[[:space:]]*2,[[:space:]]*3,[[:space:]]*4,[[:space:]]*5,[[:space:]]*6\]' "$PARAMS"; then
    ok "Configured motor IDs: 1–6"
else
    warn "Motor ID configuration differs from expected 1–6"
fi

if grep -Eq 'update_hz:[[:space:]]*50' "$PARAMS"; then
    ok "Wheel control update rate is 50 Hz"
else
    warn "Wheel update rate is not 50 Hz"
fi

if grep -Eq 'slave_id:[[:space:]]*33' "$BATTERY_PARAMS"; then
    ok "N43IC04 configured for Modbus ID 33"
else
    warn "ADC is not configured for verified Modbus ID 33"
fi

section "4. GAMEPAD / INPUT"

echo "Input groups for current user:"
id -nG

python3 - <<'PY'
import sys

try:
    from evdev import InputDevice, ecodes, list_devices
except Exception as exc:
    print(f"[FAIL] Cannot import evdev: {exc}")
    sys.exit(2)

found = []

for path in list_devices():
    try:
        dev = InputDevice(path)
        caps = dev.capabilities()
        has_abs = ecodes.EV_ABS in caps
        has_key = ecodes.EV_KEY in caps

        abs_codes = []
        if has_abs:
            for item in caps[ecodes.EV_ABS]:
                code = item[0] if isinstance(item, tuple) else item
                abs_codes.append(code)

        looks_like_controller = (
            has_abs
            and has_key
            and ecodes.ABS_X in abs_codes
            and ecodes.ABS_Y in abs_codes
        )

        print(
            f"  {path}: {dev.name}"
            + ("  <-- controller candidate" if looks_like_controller else "")
        )

        if looks_like_controller:
            found.append((path, dev.name))

        dev.close()

    except PermissionError:
        print(f"  {path}: permission denied")
    except Exception as exc:
        print(f"  {path}: {exc}")

if found:
    print(f"[PASS] Controller candidate(s): {found}")
    sys.exit(0)

print("[WARN] No evdev game controller currently detected")
sys.exit(1)
PY

GAMEPAD_RESULT=$?
if [ "$GAMEPAD_RESULT" -eq 0 ]; then
    PASS=$((PASS + 1))
elif [ "$GAMEPAD_RESULT" -eq 1 ]; then
    WARN=$((WARN + 1))
else
    FAIL=$((FAIL + 1))
fi

if command -v bluetoothctl >/dev/null 2>&1; then
    echo
    echo "Bluetooth controller:"
    bluetoothctl show 2>/dev/null | grep -E 'Controller|Powered|Discoverable|Pairable' || true
    echo
    echo "Connected Bluetooth devices:"
    bluetoothctl devices 2>/dev/null || true
    ok "bluetoothctl available"
else
    warn "bluetoothctl not installed"
fi

section "5. EXISTING ROBOT PROCESSES"

EXISTING="$(pgrep -af 'ros2 (run|launch) horseshitbot|mks_bus_node|wheel_driver_node|gamepad_teleop_node' || true)"

if [ -n "$EXISTING" ]; then
    warn "HorseShitBot/ROS processes already running:"
    echo "$EXISTING"
else
    ok "No existing HorseShitBot control process found"
fi

if [ "$ACTIVE" = false ]; then
    section "RESULT"
    echo "Passive pre-flight complete."
    echo
    echo "PASS: $PASS"
    echo "WARN: $WARN"
    echo "FAIL: $FAIL"
    echo
    echo "No ROS hardware node was started."
    echo "No motor command was sent."

    if [ "$FAIL" -gt 0 ]; then
        exit 1
    fi
    exit 0
fi

section "6. ACTIVE ROS CHECK"

if [ -n "$EXISTING" ]; then
    fail "Cannot run --active while robot control processes already exist"
else
    echo "Starting ONLY:"
    echo "  - mks_bus_node"
    echo "  - gamepad_teleop_node"
    echo
    echo "wheel_driver_node is NOT started."
    echo "No speed or movement command will be sent."
    echo
    echo "NOTE: mks_bus_node initializes detected motors:"
    echo "      mode=4, microsteps, enable."
    echo

    setsid ros2 run horseshitbot mks_bus_node \
        --ros-args \
        --params-file "$PARAMS" \
        > /tmp/hsb_mks_check.log 2>&1 &
    MKS_PID=$!
    PIDS="$PIDS $MKS_PID"

    setsid ros2 run horseshitbot gamepad_teleop_node \
        --ros-args \
        --params-file "$PARAMS" \
        > /tmp/hsb_gamepad_check.log 2>&1 &
    GP_PID=$!
    PIDS="$PIDS $GP_PID"

    echo "Waiting for diagnostic nodes..."
    sleep 4

    if kill -0 "$MKS_PID" 2>/dev/null; then
        ok "mks_bus_node is running"
    else
        fail "mks_bus_node exited"
        cat /tmp/hsb_mks_check.log
    fi

    if kill -0 "$GP_PID" 2>/dev/null; then
        ok "gamepad_teleop_node is running"
    else
        fail "gamepad_teleop_node exited"
        cat /tmp/hsb_gamepad_check.log
    fi

    echo
    echo "--- MKS startup log ---"
    cat /tmp/hsb_mks_check.log

    echo
    echo "--- Gamepad startup log ---"
    cat /tmp/hsb_gamepad_check.log

    echo
    echo "--- ROS services ---"
    ros2 service list | grep -E '^/mks/|^/modbus/' || true

    if ros2 service list | grep -qx '/mks/scan'; then
        ok "/mks/scan service available"

        echo
        echo "--- Explicit motor scan ---"
        ros2 service call \
            /mks/scan \
            std_srvs/srv/Trigger \
            "{}"
    else
        fail "/mks/scan service unavailable"
    fi

    echo
    echo "--- MKS cached status ---"
    if timeout 5 ros2 topic echo --once /mks_bus/status; then
        ok "/mks_bus/status received"
    else
        fail "No /mks_bus/status received"
    fi

    echo
    echo "--- Gamepad status ---"
    if timeout 5 ros2 topic echo --once /gamepad/status; then
        ok "/gamepad/status received"
    else
        fail "No /gamepad/status received"
    fi
fi

if [ "$CHECK_ADC" = true ]; then
    section "7. ADC RAW CHANNEL CHECK"

    echo "ADC expectation:"
    echo "  Model     : N43IC04"
    echo "  Modbus ID : 33"
    echo "  Registers : 0-3 (raw channels 1-4)"
    echo "  Bus       : same /dev/mksbus @ 19200"
    echo

    if ros2 service list | grep -qx '/modbus/read_holding_register'; then
        ok "Shared Modbus read service available"

        echo
        echo "--- Shared-bus raw ADC reads ---"
        if python3 "$REPO_DIR/test_scripts/adc_shared_bus_test.py" \
            --device-id 33 \
            --rounds 1 \
            --delay 0.1; then
            ok "All four raw ADC channels responded"
        else
            fail "One or more raw ADC channel reads failed"
        fi
    else
        fail "/modbus/read_holding_register service unavailable"
    fi
fi

section "FINAL RESULT"

echo "PASS: $PASS"
echo "WARN: $WARN"
echo "FAIL: $FAIL"

echo
echo "IMPORTANT:"
echo "  wheel_driver_node was NOT started."
echo "  No speed/move command was issued by this script."

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
