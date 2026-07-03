#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT, NavRELPOSNED, RxmRTCM

try:
    from ublox_msgs.msg import NavRELPOSNED9
except ImportError:
    NavRELPOSNED9 = None


FIX_TYPES = {
    0: 'none',
    1: 'dead_reckoning',
    2: '2d',
    3: '3d',
    4: 'gnss_dead_reckoning',
    5: 'time_only',
}

CARRIER_SOLUTIONS = {
    0: 'none',
    1: 'float',
    2: 'fixed',
}


def _pvt_carrier_solution(flags):
    return CARRIER_SOLUTIONS.get((flags & 0xC0) >> 6, 'reserved')


def _relpos_carrier_solution(flags):
    return CARRIER_SOLUTIONS.get((flags >> 3) & 0x03, 'reserved')


def _yes_no(value):
    return 'yes' if value else 'no'


class GpsStatusLogger(Node):
    def __init__(self):
        super().__init__('gps_status_logger')

        self.declare_parameter('period', 10.0)
        self.declare_parameter('rover_fix_topic', '/gps_rover/fix')
        self.declare_parameter('rover_navpvt_topic', '/gps_rover/navpvt')
        self.declare_parameter('moving_base_fix_topic', '/gps_moving_base/fix')
        self.declare_parameter('moving_base_navpvt_topic', '/gps_moving_base/navpvt')
        self.declare_parameter('relposned_topic', '/gps/relposned')
        self.declare_parameter('rover_rxmrtcm_topic', '/gps_rover/rxmrtcm')
        self.declare_parameter('moving_base_rxmrtcm_topic', '/gps_moving_base/rxmrtcm')

        self._rover_fix = None
        self._rover = None
        self._moving_base_fix = None
        self._moving_base = None
        self._relpos = None
        self._rover_rxmrtcm = None
        self._moving_base_rxmrtcm = None

        self.create_subscription(
            NavSatFix,
            self.get_parameter('rover_fix_topic').value,
            self._rover_fix_callback,
            10,
        )
        self.create_subscription(
            NavPVT,
            self.get_parameter('rover_navpvt_topic').value,
            self._rover_callback,
            10,
        )
        self.create_subscription(
            NavSatFix,
            self.get_parameter('moving_base_fix_topic').value,
            self._moving_base_fix_callback,
            10,
        )
        self.create_subscription(
            NavPVT,
            self.get_parameter('moving_base_navpvt_topic').value,
            self._moving_base_callback,
            10,
        )
        self.create_subscription(
            RxmRTCM,
            self.get_parameter('rover_rxmrtcm_topic').value,
            self._rover_rxmrtcm_callback,
            10,
        )
        self.create_subscription(
            RxmRTCM,
            self.get_parameter('moving_base_rxmrtcm_topic').value,
            self._moving_base_rxmrtcm_callback,
            10,
        )
        relpos_msg_type = NavRELPOSNED9 if NavRELPOSNED9 is not None else NavRELPOSNED
        self.create_subscription(
            relpos_msg_type,
            self.get_parameter('relposned_topic').value,
            self._relpos_callback,
            10,
        )

        period = float(self.get_parameter('period').value)
        self.create_timer(period, self._log_status)
        self.get_logger().info(f'GPS status summary every {period:.1f}s')

    def _rover_fix_callback(self, msg):
        self._rover_fix = msg

    def _rover_callback(self, msg):
        self._rover = msg

    def _moving_base_fix_callback(self, msg):
        self._moving_base_fix = msg

    def _moving_base_callback(self, msg):
        self._moving_base = msg

    def _relpos_callback(self, msg):
        self._relpos = msg

    def _rover_rxmrtcm_callback(self, msg):
        self._rover_rxmrtcm = msg

    def _moving_base_rxmrtcm_callback(self, msg):
        self._moving_base_rxmrtcm = msg

    def _format_fix(self, label, msg):
        if msg is None:
            return f'{label}: waiting'

        status = int(msg.status.status)
        status_text = {
            -1: 'no_fix',
            0: 'fix',
            1: 'sbas',
            2: 'gbas',
        }.get(status, f'unknown({status})')
        h_cov = float(msg.position_covariance[0])

        return (
            f'{label}: fix={status_text} pvt=waiting '
            f'lat={msg.latitude:.7f} lon={msg.longitude:.7f} '
            f'h_cov={h_cov:.3f}'
        )

    def _format_pvt(self, label, msg, fix_msg):
        if msg is None:
            return self._format_fix(label, fix_msg)

        flags = int(msg.flags)
        fix_type = FIX_TYPES.get(int(msg.fix_type), f'unknown({msg.fix_type})')
        carr = _pvt_carrier_solution(flags)
        diff = _yes_no(flags & 0x02)
        h_acc_m = float(msg.h_acc) / 1000.0
        v_acc_m = float(msg.v_acc) / 1000.0

        return (
            f'{label}: fix={fix_type} carr={carr} diff={diff} '
            f'sv={msg.num_sv} h_acc={h_acc_m:.2f}m v_acc={v_acc_m:.2f}m '
            f'flags={flags}'
        )

    def _format_relpos(self):
        msg = self._relpos
        if msg is None:
            return 'baseline: waiting'

        flags = int(msg.flags)
        length_m = float(msg.rel_pos_length) * 0.01 + float(msg.rel_pos_hp_length) * 0.0001
        heading_deg = math.fmod(float(msg.rel_pos_heading) * 1.0e-5, 360.0)
        heading_acc_deg = float(msg.acc_heading) * 1.0e-5
        carr = _relpos_carrier_solution(flags)
        valid = _yes_no(flags & 0x04)
        diff = _yes_no(flags & 0x02)

        return (
            f'baseline: valid={valid} carr={carr} diff={diff} '
            f'length={length_m:.3f}m heading={heading_deg:.2f}deg '
            f'heading_acc={heading_acc_deg:.2f}deg flags={flags}'
        )

    def _format_rxmrtcm(self, label, msg):
        if msg is None:
            return f'{label}_rtcm: waiting'

        crc = 'bad_crc' if int(msg.flags) & int(msg.FLAGS_CRC_FAILED) else 'ok'
        return (
            f'{label}_rtcm: {crc} msg={msg.msg_type} '
            f'ref={msg.ref_station} flags={int(msg.flags)}'
        )

    def _log_status(self):
        self.get_logger().info(
            ' | '.join([
                self._format_pvt('rover', self._rover, self._rover_fix),
                self._format_pvt(
                    'moving_base',
                    self._moving_base,
                    self._moving_base_fix
                ),
                self._format_relpos(),
                self._format_rxmrtcm('rover', self._rover_rxmrtcm),
                self._format_rxmrtcm('moving_base', self._moving_base_rxmrtcm),
                f'rtcm_subs={self.count_subscribers("/rtcm")}',
            ])
        )


def main():
    rclpy.init()
    node = GpsStatusLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
