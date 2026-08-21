#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
RULES_SRC="${SCRIPT_DIR}/99-boxbot-devices.rules"
RULES_DST="/etc/udev/rules.d/99-boxbot-devices.rules"
DOCKERARGS_FILE="${HOME}/.isaac_ros_dev-dockerargs"

if [[ ! -f "${RULES_SRC}" ]]; then
    echo "Missing ${RULES_SRC}" >&2
    exit 1
fi

for group_name in dialout gpio i2c video plugdev; do
    if ! getent group "${group_name}" >/dev/null; then
        sudo groupadd -f "${group_name}"
    fi
done

HOST_USER="${SUDO_USER:-${USER}}"
sudo usermod -aG dialout,gpio,i2c,video,plugdev "${HOST_USER}"

sudo mkdir -p /dev/boxbot
sudo cp "${RULES_SRC}" "${RULES_DST}"
sudo udevadm control --reload-rules
sudo udevadm trigger

touch "${DOCKERARGS_FILE}"
sed -i '\|^-v /run/udev:/run/udev:ro$|d' "${DOCKERARGS_FILE}"
if ! grep -Fxq '-v /dev/boxbot:/dev/boxbot' "${DOCKERARGS_FILE}"; then
    printf '\n-v /dev/boxbot:/dev/boxbot\n' >> "${DOCKERARGS_FILE}"
fi

echo "Installed ${RULES_DST}"
echo "Added ${HOST_USER} to dialout,gpio,i2c,video,plugdev."
echo "Updated ${DOCKERARGS_FILE} to mount /dev/boxbot into Isaac ROS containers."
echo "Log out and back in for new host group membership to apply."
