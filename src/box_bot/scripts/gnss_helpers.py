"""UBX units and diagnostic flag reconstruction for ublox_ubx_msgs."""
import math


def pvt_flags(msg):
    return (int(msg.gnss_fix_ok) | (int(msg.diff_soln) << 1)
            | (int(msg.psm.state) << 2) | (int(msg.head_veh_valid) << 5)
            | (int(msg.carr_soln.status) << 6))


def relpos_flags(msg):
    return (int(msg.gnss_fix_ok) | (int(msg.diff_soln) << 1)
            | (int(msg.rel_pos_valid) << 2) | (int(msg.carr_soln.status) << 3)
            | (int(msg.is_moving) << 5) | (int(msg.ref_pos_miss) << 6)
            | (int(msg.ref_obs_miss) << 7) | (int(msg.rel_pos_heading_valid) << 8)
            | (int(msg.rel_pos_normalized) << 9))


def baseline_length(msg):
    return msg.rel_pos_length * 0.01 + msg.rel_pos_hp_length * 0.0001


def heading_yaw(msg):
    """u-blox clockwise-from-north baseline heading to ROS ENU radians."""
    return math.pi / 2.0 - math.radians(msg.rel_pos_heading * 1e-5)
