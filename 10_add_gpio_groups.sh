#!/bin/bash
# Runs inside the Isaac ROS container as root, after ${USERNAME} exists.
set -u

USERNAME="${USERNAME:-admin}"

ensure_group_name() {
    local group_name="$1"

    getent group "${group_name}" >/dev/null || groupadd -f "${group_name}"
    usermod -aG "${group_name}" "${USERNAME}" 2>/dev/null || true
}

ensure_gid_group() {
    local wanted_name="$1"
    local gid="$2"
    local existing

    existing="$(getent group "${gid}" | cut -d: -f1 || true)"
    if [[ -n "${existing}" ]]; then
        echo "${existing}"
        return 0
    fi

    if getent group "${wanted_name}" >/dev/null; then
        wanted_name="${wanted_name}-${gid}"
    fi

    groupadd -g "${gid}" "${wanted_name}" 2>/dev/null || true
    getent group "${gid}" | cut -d: -f1
}

grant_device_family() {
    local group_name="$1"
    local mode="$2"
    shift 2

    local devices=()
    local pattern
    for pattern in "$@"; do
        while IFS= read -r -d '' dev; do
            devices+=("${dev}")
        done < <(find /dev -maxdepth 1 -type c -name "${pattern}" -print0 2>/dev/null)
    done

    [[ ${#devices[@]} -gt 0 ]] || return 0

    local gid
    gid="$(stat -c '%g' "${devices[0]}")"

    local actual_group
    actual_group="$(ensure_gid_group "${group_name}" "${gid}")"
    usermod -aG "${actual_group}" "${USERNAME}" 2>/dev/null || true

    local dev
    for dev in "${devices[@]}"; do
        chgrp "${actual_group}" "${dev}" 2>/dev/null || true
        chmod "${mode}" "${dev}" 2>/dev/null || true
    done

    echo "Granted ${USERNAME} ${actual_group} access to: ${devices[*]}"
}

for group_name in dialout gpio i2c video plugdev; do
    ensure_group_name "${group_name}"
done

grant_device_family i2c 0660 'i2c-*'
grant_device_family gpio 0660 'gpiochip*'
grant_device_family dialout 0660 'ttyTHS*' 'ttyUSB*' 'ttyACM*'
grant_device_family video 0660 'video*'
