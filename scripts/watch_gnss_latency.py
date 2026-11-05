#!/usr/bin/env python3
"""Read-only GNSS epoch age and USB backlog monitor; run in the ROS workspace."""
import argparse
from datetime import datetime, timezone
import time

import rclpy
from ublox_ubx_msgs.msg import UBXMonComms, UBXNavPVT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Seconds to watch; zero watches until Ctrl-C.')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('boxbot_gnss_latency_watch')
    latest, ports, subscriptions = {}, {}, []

    def pvt(msg, receiver):
        if not (msg.valid_date and msg.valid_time):
            return
        epoch = datetime(msg.year, msg.month, msg.day, msg.hour, msg.min,
                         msg.sec, tzinfo=timezone.utc).timestamp() + msg.nano * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        latest[receiver] = (epoch, stamp, time.monotonic())

    def comms(msg, receiver):
        # MON-COMMS port 0x0300 is USB on these receivers.
        ports[receiver] = (next((p for p in msg.ports if p.port_id == 0x0300), None),
                           time.monotonic())

    def report():
        now, mono = time.time(), time.monotonic()
        lines = []
        for receiver in ('gps_rover', 'gps_moving_base'):
            sample = latest.get(receiver)
            if sample is None:
                lines.append(f'{receiver}: waiting for valid UTC PVT')
                continue
            epoch, stamp, received = sample
            line = (f'{receiver}: epoch_age={now - epoch:.3f}s '
                    f'header_age={now - stamp:.3f}s '
                    f'last_rx={mono - received:.1f}s')
            port, port_received = ports.get(receiver, (None, 0.0))
            if port is not None:
                line += (f' USB_TX={port.tx_usage}% pending={port.tx_pending}B '
                         f'peak={port.tx_peak_usage}% skipped={port.skipped} '
                         f'comms_age={mono - port_received:.1f}s')
            else:
                line += ' USB_COMMS=waiting'
            lines.append(line)
        print(' | '.join(lines), flush=True)

    for receiver in ('gps_rover', 'gps_moving_base'):
        subscriptions.append(node.create_subscription(
            UBXNavPVT, f'/{receiver}/navpvt',
            lambda msg, name=receiver: pvt(msg, name), 10))
        subscriptions.append(node.create_subscription(
            UBXMonComms, f'/{receiver}/ubx_mon_comms',
            lambda msg, name=receiver: comms(msg, name), 10))
    timer = node.create_timer(1.0, report)
    started = time.monotonic()
    try:
        while rclpy.ok() and (args.duration <= 0 or time.monotonic() - started < args.duration):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
