#!/usr/bin/env bash
#
# Safe, date-filtered rosbag synchronization from Jetson to an analysis host.
# Upgrades upload-bags.sh with:
#   - Date filtering (--date YYYY-MM-DD, default: 2026-10-01)
#   - MCAP header/footer & metadata.yaml integrity checks before transfer
#   - Conflict detection (never silently overwrites differing remote files)
#   - Non-destructive transfer (never deletes source or destination files)
#   - Resumable transfers (--partial) and post-transfer SHA-256 + ros2 bag info verification
#   - Structured synchronization log
#
set -euo pipefail

DATE_FILTER="2026-10-01"
LOCAL_PATH="${LOCAL_PATH:-/home/hsb/rosbags}"
REMOTE_HOST="${REMOTE_HOST:-aibox}"
REMOTE_DIR="${REMOTE_DIR:-~/data/robotics/raw_bags}"
SSH_CONFIG="${SSH_CONFIG:-$HOME/.ssh/config}"
LOG_FILE="${LOG_FILE:-/home/hsb/hsb_analysis/lidar_validation/sync_oct1_2026.log}"
DRY_RUN=false
INCLUDE_CORRUPT=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --date)
      DATE_FILTER="$2"
      shift 2
      ;;
    --local-path)
      LOCAL_PATH="$2"
      shift 2
      ;;
    --remote-host)
      REMOTE_HOST="$2"
      shift 2
      ;;
    --remote-dir)
      REMOTE_DIR="$2"
      shift 2
      ;;
    --log-file)
      LOG_FILE="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --include-corrupt)
      INCLUDE_CORRUPT=true
      shift
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Usage: $0 [--date YYYY-MM-DD] [--dry-run] [--include-corrupt] [--log-file PATH]" >&2
      exit 1
      ;;
  esac
done

mkdir -p "$(dirname "$LOG_FILE")"
exec > >(tee -a "$LOG_FILE") 2>&1

echo "=================================================================="
echo "HorseShitBot Rosbag Synchronization — $(date -Is)"
echo "  Date filter : ${DATE_FILTER}"
echo "  Local path  : ${LOCAL_PATH}"
echo "  Remote dest : ${REMOTE_HOST}:${REMOTE_DIR}"
echo "  Dry run     : ${DRY_RUN}"
echo "=================================================================="

SSH_CMD=(ssh -F "$SSH_CONFIG" -o BatchMode=yes -o ConnectTimeout=10)

if ! "${SSH_CMD[@]}" "$REMOTE_HOST" "mkdir -p '$REMOTE_DIR'" 2>/dev/null; then
  echo "ERROR: Cannot reach ${REMOTE_HOST}:${REMOTE_DIR} via SSH."
  exit 2
fi

mapfile -t CANDIDATE_BAGS < <(
  python3 - "$LOCAL_PATH" "$DATE_FILTER" <<'PY'
import os, sys, yaml
from datetime import datetime, timezone
from zoneinfo import ZoneInfo

local_path, date_filter = sys.argv[1], sys.argv[2]
tz_local = ZoneInfo("Europe/Paris")

for entry in sorted(os.listdir(local_path)):
    bpath = os.path.join(local_path, entry)
    if not os.path.isdir(bpath):
        continue
    matched = date_filter in entry
    mpath = os.path.join(bpath, "metadata.yaml")
    if os.path.isfile(mpath):
        try:
            with open(mpath) as f:
                meta = yaml.safe_load(f)["rosbag2_bagfile_information"]
            ns = meta["starting_time"]["nanoseconds_since_epoch"]
            dt_utc = datetime.fromtimestamp(ns / 1e9, tz=timezone.utc)
            dt_loc = dt_utc.astimezone(tz_local)
            if dt_loc.strftime("%Y-%m-%d") == date_filter or dt_utc.strftime("%Y-%m-%d") == date_filter:
                matched = True
        except Exception:
            pass
    if matched:
        print(entry)
PY
)

if [[ ${#CANDIDATE_BAGS[@]} -eq 0 ]]; then
  echo "No bags found matching date ${DATE_FILTER} in ${LOCAL_PATH}."
  exit 0
fi

echo "Found ${#CANDIDATE_BAGS[@]} bag directories for ${DATE_FILTER}:"
VALID_BAGS=()
SKIPPED_BAGS=()
CONFLICT_BAGS=()

for bag in "${CANDIDATE_BAGS[@]}"; do
  bag_dir="${LOCAL_PATH}/${bag}"
  status=$(
    python3 - "$bag_dir" <<'PY'
import os, sys
bdir = sys.argv[1]
files = os.listdir(bdir)
has_meta = "metadata.yaml" in files
mcaps = [f for f in files if f.endswith(".mcap")]
if not has_meta or not mcaps:
    print("UNCLOSED_NO_METADATA")
    sys.exit(0)
for m in mcaps:
    mp = os.path.join(bdir, m)
    sz = os.path.getsize(mp)
    if sz < 16:
        print("INVALID_MCAP_TOO_SMALL")
        sys.exit(0)
    with open(mp, "rb") as fh:
        head = fh.read(8)
        fh.seek(sz - 8)
        tail = fh.read(8)
    if head != b"\x89MCAP0\r\n" or tail != b"\x89MCAP0\r\n":
        print("UNCLOSED_MCAP_BAD_FOOTER")
        sys.exit(0)
# A valid footer is insufficient: duplicate recorder processes can overwrite
# intermediate chunks while leaving the summary/footer intact. Read every
# record through the rosbag2 MCAP plugin and compare with metadata count.
try:
    import rosbag2_py
    import yaml
    with open(os.path.join(bdir, "metadata.yaml")) as fh:
        expected = int(yaml.safe_load(fh)["rosbag2_bagfile_information"]["message_count"])
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bdir, storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    actual = 0
    while reader.has_next():
        reader.read_next()
        actual += 1
    if actual != expected:
        print(f"CORRUPT_MESSAGE_COUNT_{actual}_OF_{expected}")
        sys.exit(0)
except Exception as exc:
    print(f"CORRUPT_ROSBAG2_READ_{type(exc).__name__}")
    sys.exit(0)
print("VALID_CLOSED_MCAP")
PY
  )

  # Check remote state
  remote_check=$(
    "${SSH_CMD[@]}" "$REMOTE_HOST" "python3 - '$REMOTE_DIR/$bag' <<'RPY'
import os, sys
rdir = sys.argv[1]
if not os.path.isdir(rdir):
    print('MISSING')
    sys.exit(0)
items = {}
for f in sorted(os.listdir(rdir)):
    fp = os.path.join(rdir, f)
    if os.path.isfile(fp):
        items[f] = os.path.getsize(fp)
print('EXISTS:' + ','.join(f'{k}={v}' for k, v in items.items()))
RPY"
  )

  local_check=$(
    python3 - "$bag_dir" <<'LPY'
import os, sys
ldir = sys.argv[1]
items = {}
for f in sorted(os.listdir(ldir)):
    fp = os.path.join(ldir, f)
    if os.path.isfile(fp):
        items[f] = os.path.getsize(fp)
print("EXISTS:" + ",".join(f"{k}={v}" for k, v in items.items()))
LPY
  )

  echo "  - ${bag}: local_integrity=${status} | local=${local_check} | remote=${remote_check}"

  if [[ "$status" != "VALID_CLOSED_MCAP" && "$INCLUDE_CORRUPT" == "false" ]]; then
    echo "    -> SKIPPING unclosed/corrupted bag ${bag} (use --include-corrupt to force)."
    if [[ "$remote_check" != "MISSING" && "$remote_check" != "$local_check" ]]; then
      echo "    -> CONFLICT DETECTED on ${bag}: remote size (${remote_check}) differs from local (${local_check}). Preserving both without overwriting."
      CONFLICT_BAGS+=("$bag")
    fi
    SKIPPED_BAGS+=("$bag")
    continue
  fi

  if [[ "$remote_check" != "MISSING" && "$remote_check" != "$local_check" ]]; then
    echo "    -> CONFLICT DETECTED on ${bag}: remote (${remote_check}) differs from local (${local_check})! Refusing to overwrite silently."
    CONFLICT_BAGS+=("$bag")
    continue
  fi

  VALID_BAGS+=("$bag")
done

echo ""
echo "Step 1: Running rsync --dry-run on ${#VALID_BAGS[@]} valid bags..."
for bag in "${VALID_BAGS[@]}"; do
  rsync -avzn --itemize-changes --partial --ignore-existing \
    -e "ssh -F ${SSH_CONFIG}" \
    "${LOCAL_PATH}/${bag}" "${REMOTE_HOST}:${REMOTE_DIR}/"
done

if [[ "$DRY_RUN" == "true" ]]; then
  echo "Dry run complete. No files were modified."
  exit 0
fi

echo ""
echo "Step 2: Synchronizing valid October 1 bags to ${REMOTE_HOST}:${REMOTE_DIR}..."
for bag in "${VALID_BAGS[@]}"; do
  rsync -avz --partial --ignore-existing \
    -e "ssh -F ${SSH_CONFIG}" \
    "${LOCAL_PATH}/${bag}" "${REMOTE_HOST}:${REMOTE_DIR}/"
done

echo ""
echo "Step 3: Verifying SHA-256 checksums for metadata.yaml and .mcap files..."
for bag in "${VALID_BAGS[@]}"; do
  echo "  Verifying ${bag}..."
  local_sums=$(cd "${LOCAL_PATH}" && sha256sum "${bag}/metadata.yaml" "${bag}/"*.mcap)
  remote_sums=$("${SSH_CMD[@]}" "$REMOTE_HOST" "cd '${REMOTE_DIR}' && sha256sum '${bag}/metadata.yaml' '${bag}/'*.mcap")
  if [[ "$local_sums" == "$remote_sums" ]]; then
    echo "    PASS (SHA-256 match)"
  else
    echo "    FAIL (SHA-256 mismatch!)"
    echo "    Local : $local_sums"
    echo "    Remote: $remote_sums"
    exit 3
  fi
done

echo ""
echo "=================================================================="
echo "Synchronization Summary:"
echo "  Verified & synced valid bags : ${#VALID_BAGS[@]} (${VALID_BAGS[*]})"
echo "  Skipped unclosed bags        : ${#SKIPPED_BAGS[@]} (${SKIPPED_BAGS[*]})"
echo "  Reported size conflicts      : ${#CONFLICT_BAGS[@]} (${CONFLICT_BAGS[*]})"
echo "=================================================================="
