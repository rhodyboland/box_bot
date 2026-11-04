#!/usr/bin/env python3
"""Convert matching X20P PVT/HPPOSLLH epochs to reliable ROS fix/velocity topics."""
import math
import signal

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_ubx_msgs.msg import UBXNavCov, UBXNavHPPosLLH, UBXNavPVT


def make_fix(pvt, hp, cov=None):
    fix = NavSatFix()
    fix.header = hp.header
    fix.status.service = (NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
                          | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO)
    valid = (pvt.gnss_fix_ok and pvt.gps_fix.fix_type in (2, 3, 4)
             and not pvt.invalid_llh
             and not any((hp.invalid_lon, hp.invalid_lat, hp.invalid_height,
                          hp.invalid_lon_hp, hp.invalid_lat_hp, hp.invalid_height_hp)))
    fix.status.status = NavSatStatus.STATUS_NO_FIX
    if not valid:
        fix.latitude = fix.longitude = fix.altitude = math.nan
        return fix
    fix.status.status = NavSatStatus.STATUS_FIX
    if pvt.diff_soln:
        fix.status.status = (NavSatStatus.STATUS_GBAS_FIX if pvt.carr_soln.status in (1, 2)
                             else NavSatStatus.STATUS_SBAS_FIX)
    fix.latitude = hp.lat * 1e-7 + hp.lat_hp * 1e-9
    fix.longitude = hp.lon * 1e-7 + hp.lon_hp * 1e-9
    fix.altitude = hp.height * 1e-3 + hp.height_hp * 1e-4
    if cov is not None and cov.itow == hp.itow and cov.pos_cor_valid:
        fix.position_covariance = [cov.pos_cov_ee, cov.pos_cov_ne, -cov.pos_cov_ed,
                                   cov.pos_cov_ne, cov.pos_cov_nn, -cov.pos_cov_nd,
                                   -cov.pos_cov_ed, -cov.pos_cov_nd, cov.pos_cov_dd]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
    else:
        fix.position_covariance[0] = fix.position_covariance[4] = (hp.h_acc * 1e-4) ** 2
        fix.position_covariance[8] = (hp.v_acc * 1e-4) ** 2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    return fix


class GnssNavsat(Node):
    def __init__(self):
        super().__init__('navsat')
        self._epochs = {}
        self._fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self._vel_pub = self.create_publisher(TwistWithCovarianceStamped, 'fix_velocity', 10)
        for kind, msg_type, topic in (
                ('pvt', UBXNavPVT, 'navpvt'),
                ('hp', UBXNavHPPosLLH, 'hpposllh'),
                ('cov', UBXNavCov, 'navcov')):
            self.create_subscription(msg_type, topic,
                                     lambda msg, kind=kind: self._receive(kind, msg), 10)

    def _receive(self, kind, msg):
        epoch = self._epochs.setdefault(msg.itow, {})
        epoch[kind] = msg
        if 'pvt' in epoch and 'hp' in epoch and not epoch.get('published'):
            pvt, hp = epoch['pvt'], epoch['hp']
            self._fix_pub.publish(make_fix(pvt, hp, epoch.get('cov')))
            if pvt.gnss_fix_ok and pvt.gps_fix.fix_type in (2, 3, 4):
                vel = TwistWithCovarianceStamped()
                vel.header = pvt.header
                # Velocity is ENU; its reference frame is not the antenna body frame.
                vel.header.frame_id = 'enu'
                vel.twist.twist.linear.x = pvt.vel_e * 1e-3
                vel.twist.twist.linear.y = pvt.vel_n * 1e-3
                vel.twist.twist.linear.z = -pvt.vel_d * 1e-3
                for index in (0, 7, 14):
                    vel.twist.covariance[index] = (pvt.s_acc * 1e-3) ** 2
                for index in (21, 28, 35):
                    vel.twist.covariance[index] = 1e6
                self._vel_pub.publish(vel)
            epoch['published'] = True
        # Bound the cache even when one of the required outputs is unavailable.
        while len(self._epochs) > 16:
            del self._epochs[next(iter(self._epochs))]


def main():
    rclpy.init()
    node = GnssNavsat()
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
