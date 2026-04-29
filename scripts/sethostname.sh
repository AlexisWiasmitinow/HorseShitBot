#!/usr/bin/env bash
#
# Set the hostname of the Jetson (persists across reboots).
#
# Usage:
#   ./scripts/sethostname.sh <new-hostname>
#
set -eo pipefail

if [ -z "$1" ]; then
  echo "Usage: $0 <new-hostname>"
  exit 1
fi

NEW_HOST="$1"
OLD_HOST="$(hostname)"

sudo hostnamectl set-hostname "$NEW_HOST"

# Update /etc/hosts so localhost resolution doesn't break
if grep -q "127.0.1.1" /etc/hosts; then
  sudo sed -i "s/127\.0\.1\.1.*/127.0.1.1\t$NEW_HOST/" /etc/hosts
else
  echo -e "127.0.1.1\t$NEW_HOST" | sudo tee -a /etc/hosts > /dev/null
fi

echo "Hostname changed: $OLD_HOST  →  $NEW_HOST"
echo "Changes take effect immediately (no reboot needed)."
