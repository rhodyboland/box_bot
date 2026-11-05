# Boxbot 40-pin header — JetPack 7.2.1

Prepared on 5 November 2026 for Orin Nano Super / L4T R39.2.1.

| Physical pins | Function | Linux interface |
| --- | --- | --- |
| 3 SDA, 5 SCL | BNO085 I2C, address 0x4A | `/dev/i2c-7` |
| 8 TX, 10 RX | Hoverboard UART, 115200 baud, no hardware flow control | `/dev/ttyTHS1` |
| 7 | Hoverboard power sense input | Jetson.GPIO BOARD pin 7 |
| 11 | Hoverboard power-switch output | Jetson.GPIO BOARD pin 11 |

NVIDIA Jetson-IO reports I2C8 on pins 3/5 and UARTA on pins 8/10 already
enabled in the board configuration. Linux numbers that I2C controller as bus 7.
No extra I2C/UART overlay or speculative DMA workaround was added.

The existing `gpio7-11-uart8-10.dts` only configures GPIO pins 7 and 11;
its historical name includes UART, which continues to use the board defaults.
Pin 7 keeps tristate=1, input-enable=1, Schmitt=1; pin 11 keeps tristate=0
and input-enable=1, matching the checked-in configuration.

## Next boot staged

The source was compiled with `dtc -@`. Its output is installed at
`/boot/gpio7-11-uart8-10.dtbo`. NVIDIA Jetson-IO selected a new `JetsonIO`
boot entry with:

- FDT: `/boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb`
- OVERLAYS: `/boot/gpio7-11-uart8-10.dtbo`
- Original `primary` boot entry retained.
- Backup: `/boot/extlinux/extlinux.conf.boxbot-pre-lyrical`

The exact selected DTB plus installed overlay merged successfully using
`fdtoverlay` into a temporary file. The I2C controller at `c250000` and UART at
`3100000` remain `okay`. No live GPIO, UART traffic, reboot, or robot launch
was performed. Changes take effect on the next boot.

To reproduce on a matching fresh installation, after reviewing the source:

```bash
dtc -@ -I dts -O dtb -o /tmp/boxbot-header.dtbo gpio7-11-uart8-10.dts
sudo cp -a /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.before-boxbot
sudo install -m 0644 /tmp/boxbot-header.dtbo /boot/gpio7-11-uart8-10.dtbo
sudo python3 /opt/nvidia/jetson-io/config-by-hardware.py -n 'GPIO7/11 + UART8/10'
```

After installation in the robot and the next boot, verify the applied tree,
IMU detection on bus 7, UART feedback and device permissions before enabling
motor commands or power-switch pulses. Device nodes and offline DT validation
do not establish electrical communication. Those checks remain pending.

To roll back, select `primary` at the boot menu, or restore the saved
`extlinux.conf` and reboot. Keep the backup available.

Reference: [NVIDIA R39.2 expansion-header configuration](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/HR/ConfiguringTheJetsonExpansionHeaders.html).
