#!/bin/bash
# This runs inside the container as root, after ${USERNAME} exists.
set -e

for g in dialout gpio i2c; do
    # If host passed the group (via --group-add) it already exists here.
    # Otherwise create it so we can still add the user.
    getent group "$g" >/dev/null || groupadd -f "$g"

    adduser --quiet "${USERNAME}" "$g" || true
done
# sudo chmod 666 /dev/ttyTHS1
# sudo chmod 666 /dev/ttyUSB0
# sudo chmod 666 /dev/ttyACM0
# sudo chmod 666 /dev/gpiochip0
# sudo chmod 666 /dev/gpiochip1
# sudo chmod 666 /dev/gpiochip2
gid=$(stat -c '%g' /dev/gpiochip0)   # whatever gid the BSP uses
getent group gpio >/dev/null || groupadd -g "$gid" gpio
usermod -aG gpio,dialout,i2c ${USERNAME}
chown root:gpio /dev/gpiochip*
chmod 660      /dev/gpiochip*