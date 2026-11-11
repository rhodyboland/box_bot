"""Exercise X20P field scaling, validity gates, and diagnostic consumers in ROS."""
import math
import struct
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import rclpy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2
from imu_tilt_cloud_filter import ImuTiltCloudFilter
from ublox_ubx_msgs.msg import (
    UBXNavCov, UBXNavHPPosLLH, UBXNavPVT, UBXNavRelPosNED, UBXNavSat,
    UBXNavStatus, UBXRxmCor, UBXSecSig, SatInfo,
)
from gnss_helpers import heading_yaw, relpos_flags
from gnss_navsat import GnssNavsat, make_fix
from gps_status_logger import GpsStatusLogger
from imu_yaw_offset import ImuYawOffset
from motion_chain_logger import MotionChainLogger


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class TestGnssMigration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def messages(self):
        pvt, hp = UBXNavPVT(), UBXNavHPPosLLH()
        pvt.itow = hp.itow = 1234
        pvt.gnss_fix_ok = True
        pvt.gps_fix.fix_type = 3
        pvt.diff_soln = True
        pvt.carr_soln.status = 2
        hp.header.frame_id = 'gps'
        hp.lat, hp.lat_hp = -274000001, -50
        hp.lon, hp.lon_hp = 1530000001, 25
        hp.height, hp.height_hp = 23456, -5
        hp.h_acc, hp.v_acc = 100, 200
        return pvt, hp

    def test_lyrical_pointcloud_buffer_roundtrip(self):
        node = ImuTiltCloudFilter()
        try:
            cloud = node._create_xyz_cloud(Header(frame_id='camera'), [(1.0, 0.0, 0.3)])
            decoded = deserialize_message(serialize_message(cloud), PointCloud2)
            self.assertEqual(decoded.header.frame_id, 'base_link')
            self.assertEqual(decoded.row_step, 12)
            x, y, z = struct.unpack_from('<fff', decoded.data)
            self.assertAlmostEqual(x, 1.0)
            self.assertAlmostEqual(y, 0.0)
            self.assertAlmostEqual(z, 0.3)
        finally:
            node.destroy_node()

    def test_stamped_velocity_logging(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(MotionChainLogger, '_output_path', return_value=str(Path(directory) / 'motion.csv')):
                node = MotionChainLogger()
            try:
                command = TwistStamped()
                command.header.stamp = node.get_clock().now().to_msg()
                command.twist.linear.x = 0.25
                command.twist.angular.z = -0.3
                with patch.object(node, '_latest_msg', return_value=(command, node.get_clock().now())):
                    row = {}
                    node._add_cmd(row, node.get_clock().now())
                self.assertEqual(row['cmd_vx_mps'], 0.25)
                self.assertEqual(row['cmd_wz_radps'], -0.3)
            finally:
                node.destroy_node()

    def test_high_precision_and_accuracy_units(self):
        pvt, hp = self.messages()
        fix = make_fix(pvt, hp)
        self.assertAlmostEqual(fix.latitude, -27.40000015)
        self.assertAlmostEqual(fix.longitude, 153.000000125)
        self.assertAlmostEqual(fix.altitude, 23.4555)
        self.assertAlmostEqual(fix.position_covariance[0], 0.0001)
        self.assertAlmostEqual(fix.position_covariance[8], 0.0004)
        self.assertEqual(fix.status.status, NavSatFix().status.STATUS_GBAS_FIX)
        self.assertEqual(fix.header.frame_id, 'gps')

    def test_covariance_ned_to_enu_and_epoch(self):
        pvt, hp = self.messages()
        cov = UBXNavCov(itow=hp.itow, pos_cor_valid=True,
                        pos_cov_nn=1.0, pos_cov_ee=2.0, pos_cov_dd=3.0,
                        pos_cov_ne=0.1, pos_cov_nd=0.2, pos_cov_ed=0.3)
        fix = make_fix(pvt, hp, cov)
        for actual, expected in zip(fix.position_covariance,
                                    [2., .1, -.3, .1, 1., -.2, -.3, -.2, 3.]):
            self.assertAlmostEqual(actual, expected, places=6)
        cov.itow += 1
        self.assertEqual(make_fix(pvt, hp, cov).position_covariance_type,
                         NavSatFix.COVARIANCE_TYPE_APPROXIMATED)

    def test_differential_fix_without_carrier_is_not_reported_as_rtk(self):
        pvt, hp = self.messages()
        pvt.carr_soln.status = 0
        self.assertEqual(make_fix(pvt, hp).status.status, 1)
        pvt.diff_soln = False
        self.assertEqual(make_fix(pvt, hp).status.status, 0)

    def test_invalid_position_never_reports_a_fix(self):
        for invalid in ('gnss_fix_ok', 'invalid_llh', 'hp_invalid'):
            pvt, hp = self.messages()
            if invalid == 'gnss_fix_ok':
                pvt.gnss_fix_ok = False
            elif invalid == 'invalid_llh':
                pvt.invalid_llh = True
            else:
                hp.invalid_lat_hp = True
            fix = make_fix(pvt, hp)
            self.assertEqual(fix.status.status, -1)
            self.assertTrue(math.isnan(fix.latitude))

    def test_out_of_order_epochs_and_bounded_cache(self):
        node = GnssNavsat()
        try:
            node._fix_pub, node._vel_pub = Capture(), Capture()
            pvt, hp = self.messages()
            node._receive('hp', hp)
            pvt.itow += 1
            node._receive('pvt', pvt)
            self.assertEqual(len(node._fix_pub.messages), 0)
            pvt.itow = hp.itow
            node._receive('pvt', pvt)
            node._receive('hp', hp)
            self.assertEqual(len(node._fix_pub.messages), 1)
            for itow in range(100):
                node._receive('pvt', UBXNavPVT(itow=itow))
            self.assertLessEqual(len(node._epochs), 16)
        finally:
            node.destroy_node()

    def test_heading_front_base_rear_rover_and_invalid_gate(self):
        node = ImuYawOffset()
        try:
            node._pub, node._raw_pub = Capture(), Capture()
            node._yaw_offset = math.pi
            rel = UBXNavRelPosNED(rel_pos_length=40, rel_pos_heading=18000000,
                                 acc_heading=10000)
            node._relpos_callback(rel)
            self.assertEqual(len(node._pub.messages), 0)
            rel.gnss_fix_ok = rel.diff_soln = rel.rel_pos_valid = True
            rel.rel_pos_heading_valid = True
            rel.carr_soln.status = 2
            self.assertEqual(relpos_flags(rel) & 0x117, 0x117)
            self.assertAlmostEqual(heading_yaw(rel), -math.pi / 2)
            node._relpos_callback(rel)
            self.assertEqual(len(node._pub.messages), 1)
            q = node._pub.messages[0].orientation
            self.assertAlmostEqual(2 * math.atan2(q.z, q.w), math.pi / 2)
            rel.rel_pos_length = 90
            node._relpos_callback(rel)
            self.assertEqual(len(node._pub.messages), 1)
        finally:
            node.destroy_node()

    def test_native_diagnostics_and_csv_fields(self):
        status_node = GpsStatusLogger()
        with tempfile.TemporaryDirectory() as temp:
            with patch.object(MotionChainLogger, '_output_path', return_value=temp + '/motion.csv'):
                motion = MotionChainLogger()
            try:
                pvt, hp = self.messages()
                cor = UBXRxmCor()
                sat = UBXNavSat()
                sat.sv_info = [SatInfo(cno=42)]
                sat.sv_info[0].flags.sv_used = True
                sat.sv_info[0].flags.quality_ind = 5
                sat.num_svs = 1
                self.assertIn('fixed', status_node._format_pvt('rover', pvt, None, None))
                self.assertIn('unknown', status_node._format_rxmcor('rover', cor, None))
                self.assertIn('used=1', status_node._format_navsat('rover', sat, None))
                self.assertIn('jam=', status_node._format_secsig('rover', UBXSecSig(), None))
                status_node._relpos = UBXNavRelPosNED()
                self.assertIn('length=', status_node._format_relpos())
                for key, msg in [('rover_pvt', pvt), ('moving_base_pvt', pvt),
                                 ('rover_status', UBXNavStatus()), ('rover_cor', cor),
                                 ('relpos', UBXNavRelPosNED()), ('rover_cov', UBXNavCov())]:
                    motion._store(key, msg)
                row = {}
                motion._add_gps(row, motion.get_clock().now())
                motion._add_relpos(row, motion.get_clock().now())
                motion._add_nav_cov(row, motion.get_clock().now())
                self.assertFalse(set(row) - set(motion._fields))
            finally:
                motion.destroy_node()
                status_node.destroy_node()


if __name__ == '__main__':
    unittest.main()


def test_ntrip_main_handles_normal_shutdown_exceptions():
    from unittest.mock import MagicMock
    from rclpy.executors import ExternalShutdownException
    import rtk2go_ntrip_client as ntrip

    for exception in (KeyboardInterrupt, ExternalShutdownException):
        node = MagicMock()
        with patch.object(ntrip.rclpy, 'init'), \
                patch.object(ntrip, 'Rtk2GoNtripClient', return_value=node), \
                patch.object(ntrip.rclpy, 'spin', side_effect=exception), \
                patch.object(ntrip.rclpy, 'ok', return_value=False), \
                patch.object(ntrip.rclpy, 'shutdown') as shutdown:
            ntrip.main()
        node.destroy_node.assert_called_once()
        shutdown.assert_not_called()
