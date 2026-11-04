#!/usr/bin/env python3

import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu, NavSatFix
from ublox_ubx_msgs.msg import UBXNavPVT, UBXNavRelPosNED, UBXNavSat, UBXRxmCor, UBXSecSig
from gnss_helpers import pvt_flags, relpos_flags


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


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GpsStatusLogger(Node):
    def __init__(self):
        super().__init__('gps_status_logger')

        self.declare_parameter('period', 10.0)
        self.declare_parameter('rover_fix_topic', '/gps_rover/fix')
        self.declare_parameter('rover_navpvt_topic', '/gps_rover/navpvt')
        self.declare_parameter('rover_navsat_topic', '/gps_rover/navsat')
        self.declare_parameter('rover_secsig_topic', '/gps_rover/secsig')
        self.declare_parameter('moving_base_fix_topic', '/gps_moving_base/fix')
        self.declare_parameter('moving_base_navpvt_topic', '/gps_moving_base/navpvt')
        self.declare_parameter('moving_base_navsat_topic', '/gps_moving_base/navsat')
        self.declare_parameter('moving_base_secsig_topic', '/gps_moving_base/secsig')
        self.declare_parameter('relposned_topic', '/gps/relposned')
        self.declare_parameter('rover_rxmcor_topic', '/gps_rover/rxmcor')
        self.declare_parameter('moving_base_rxmcor_topic', '/gps_moving_base/rxmcor')
        self.declare_parameter('corrected_heading_topic', '/gps/heading_corrected')

        self._rover_fix = None
        self._rover_fix_time = None
        self._rover = None
        self._rover_time = None
        self._rover_navsat = None
        self._rover_navsat_time = None
        self._rover_secsig = None
        self._rover_secsig_time = None
        self._moving_base_fix = None
        self._moving_base_fix_time = None
        self._moving_base = None
        self._moving_base_time = None
        self._moving_base_navsat = None
        self._moving_base_navsat_time = None
        self._moving_base_secsig = None
        self._moving_base_secsig_time = None
        self._relpos = None
        self._relpos_time = None
        self._rover_rxmcor = None
        self._rover_rxmcor_time = None
        self._moving_base_rxmcor = None
        self._moving_base_rxmcor_time = None
        self._corrected_heading = None
        self._corrected_heading_time = None

        self.create_subscription(
            NavSatFix,
            self.get_parameter('rover_fix_topic').value,
            self._rover_fix_callback,
            10,
        )
        self.create_subscription(
            UBXNavPVT,
            self.get_parameter('rover_navpvt_topic').value,
            self._rover_callback,
            10,
        )
        self.create_subscription(
            UBXNavSat,
            self.get_parameter('rover_navsat_topic').value,
            self._rover_navsat_callback,
            10,
        )
        self.create_subscription(
            UBXSecSig,
            self.get_parameter('rover_secsig_topic').value,
            self._rover_secsig_callback,
            10,
        )
        self.create_subscription(
            NavSatFix,
            self.get_parameter('moving_base_fix_topic').value,
            self._moving_base_fix_callback,
            10,
        )
        self.create_subscription(
            UBXNavPVT,
            self.get_parameter('moving_base_navpvt_topic').value,
            self._moving_base_callback,
            10,
        )
        self.create_subscription(
            UBXNavSat,
            self.get_parameter('moving_base_navsat_topic').value,
            self._moving_base_navsat_callback,
            10,
        )
        self.create_subscription(
            UBXSecSig,
            self.get_parameter('moving_base_secsig_topic').value,
            self._moving_base_secsig_callback,
            10,
        )
        self.create_subscription(
            UBXRxmCor,
            self.get_parameter('rover_rxmcor_topic').value,
            self._rover_rxmcor_callback,
            10,
        )
        self.create_subscription(
            UBXRxmCor,
            self.get_parameter('moving_base_rxmcor_topic').value,
            self._moving_base_rxmcor_callback,
            10,
        )
        self.create_subscription(
            Imu,
            self.get_parameter('corrected_heading_topic').value,
            self._corrected_heading_callback,
            10,
        )
        self.create_subscription(
            UBXNavRelPosNED,
            self.get_parameter('relposned_topic').value,
            self._relpos_callback,
            10,
        )

        period = float(self.get_parameter('period').value)
        self.create_timer(period, self._log_status)
        self.get_logger().info(f'GPS status summary every {period:.1f}s')

    def _rover_fix_callback(self, msg):
        self._rover_fix = msg
        self._rover_fix_time = self.get_clock().now()

    def _rover_callback(self, msg):
        self._rover = msg
        self._rover_time = self.get_clock().now()

    def _rover_navsat_callback(self, msg):
        self._rover_navsat = msg
        self._rover_navsat_time = self.get_clock().now()

    def _rover_secsig_callback(self, msg):
        self._rover_secsig = msg
        self._rover_secsig_time = self.get_clock().now()

    def _moving_base_fix_callback(self, msg):
        self._moving_base_fix = msg
        self._moving_base_fix_time = self.get_clock().now()

    def _moving_base_callback(self, msg):
        self._moving_base = msg
        self._moving_base_time = self.get_clock().now()

    def _moving_base_navsat_callback(self, msg):
        self._moving_base_navsat = msg
        self._moving_base_navsat_time = self.get_clock().now()

    def _moving_base_secsig_callback(self, msg):
        self._moving_base_secsig = msg
        self._moving_base_secsig_time = self.get_clock().now()

    def _relpos_callback(self, msg):
        self._relpos = msg
        self._relpos_time = self.get_clock().now()

    def _rover_rxmcor_callback(self, msg):
        self._rover_rxmcor = msg
        self._rover_rxmcor_time = self.get_clock().now()

    def _moving_base_rxmcor_callback(self, msg):
        self._moving_base_rxmcor = msg
        self._moving_base_rxmcor_time = self.get_clock().now()

    def _corrected_heading_callback(self, msg):
        self._corrected_heading = msg
        self._corrected_heading_time = self.get_clock().now()

    def _age(self, stamp):
        if stamp is None:
            return 'age=never'
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return f'age={age:.1f}s'

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

    def _format_pvt(self, label, msg, fix_msg, stamp):
        if msg is None:
            return self._format_fix(label, fix_msg)

        flags = pvt_flags(msg)
        fix_type = FIX_TYPES.get(int(msg.gps_fix.fix_type), f'unknown({msg.gps_fix.fix_type})')
        carr = _pvt_carrier_solution(flags)
        diff = _yes_no(flags & 0x02)
        h_acc_m = float(msg.h_acc) / 1000.0
        v_acc_m = float(msg.v_acc) / 1000.0

        return (
            f'{label}: fix={fix_type} carr={carr} diff={diff} '
            f'sv={msg.num_sv} h_acc={h_acc_m:.2f}m v_acc={v_acc_m:.2f}m '
            f'flags={flags} {self._age(stamp)}'
        )

    def _format_relpos(self):
        msg = self._relpos
        if msg is None:
            return 'baseline: waiting'

        flags = relpos_flags(msg)
        length_m = float(msg.rel_pos_length) * 0.01 + float(msg.rel_pos_hp_length) * 0.0001
        heading_deg = math.fmod(float(msg.rel_pos_heading) * 1.0e-5, 360.0)
        heading_acc_deg = float(msg.acc_heading) * 1.0e-5
        carr = _relpos_carrier_solution(flags)
        valid = _yes_no(flags & 0x04)
        diff = _yes_no(flags & 0x02)

        return (
            f'baseline: valid={valid} carr={carr} diff={diff} '
            f'length={length_m:.3f}m heading={heading_deg:.2f}deg '
            f'heading_acc={heading_acc_deg:.2f}deg flags={flags} '
            f'{self._age(self._relpos_time)}'
        )

    def _format_rxmcor(self, label, msg, stamp):
        if msg is None:
            return f'{label}_cor: waiting'
        status = msg.status_info
        error = {0: 'unknown', 1: 'ok', 2: 'error'}.get(status.err_status, 'reserved')
        used = {0: 'unknown', 1: 'no', 2: 'yes'}.get(status.msg_used, 'reserved')
        return (f'{label}_cor: {error} used={used} protocol={status.protocol} '
                f'msg={msg.msg_type} ref={status.correction_id} {self._age(stamp)}')

    def _format_navsat(self, label, msg, stamp):
        if msg is None:
            return f'{label}_sat: waiting'

        used = 0
        carrier_locked = 0
        cno_sum = 0
        cno_count = 0
        for sv in msg.sv_info:
            quality = sv.flags.quality_ind
            if sv.flags.sv_used:
                used += 1
                cno_sum += int(sv.cno)
                cno_count += 1
            if quality >= 5:
                carrier_locked += 1

        avg_cno = cno_sum / cno_count if cno_count else 0.0
        return (
            f'{label}_sat: sv={msg.num_svs} used={used} '
            f'carr_lock={carrier_locked} avg_cno={avg_cno:.1f}dBHz '
            f'{self._age(stamp)}'
        )

    def _format_secsig(self, label, msg, stamp):
        if msg is None:
            return f'{label}_security: waiting'
        return (f'{label}_security: jam={msg.jamming_state} '
                f'jam_detection={bool(msg.jam_det_enabled)} '
                f'spoof={msg.spoofing_state} spoof_detection={bool(msg.spf_det_enabled)} '
                f'{self._age(stamp)}')

    def _format_corrected_heading(self):
        msg = self._corrected_heading
        if msg is None:
            return 'heading_corrected: waiting'

        yaw_deg = math.degrees(_yaw_from_quaternion(msg.orientation))
        cov = msg.orientation_covariance[8]
        return (
            f'heading_corrected: yaw={yaw_deg:.2f}deg cov={cov:.4f} '
            f'{self._age(self._corrected_heading_time)}'
        )

    def _log_status(self):
        self.get_logger().info(
            ' | '.join([
                self._format_pvt(
                    'rover',
                    self._rover,
                    self._rover_fix,
                    self._rover_time,
                ),
                self._format_pvt(
                    'moving_base',
                    self._moving_base,
                    self._moving_base_fix,
                    self._moving_base_time,
                ),
                self._format_relpos(),
                self._format_corrected_heading(),
                self._format_navsat(
                    'rover',
                    self._rover_navsat,
                    self._rover_navsat_time,
                ),
                self._format_navsat(
                    'moving_base',
                    self._moving_base_navsat,
                    self._moving_base_navsat_time,
                ),
                self._format_secsig(
                    'rover',
                    self._rover_secsig,
                    self._rover_secsig_time,
                ),
                self._format_secsig(
                    'moving_base',
                    self._moving_base_secsig,
                    self._moving_base_secsig_time,
                ),
                self._format_rxmcor(
                    'rover',
                    self._rover_rxmcor,
                    self._rover_rxmcor_time,
                ),
                self._format_rxmcor(
                    'moving_base',
                    self._moving_base_rxmcor,
                    self._moving_base_rxmcor_time,
                ),
                f'rtcm_subs={self.count_subscribers("/rtcm")}',
            ])
        )


def main():
    rclpy.init()
    node = GpsStatusLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
