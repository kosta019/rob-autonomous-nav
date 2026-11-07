# Test sin robot de scan_slam: el sello de /scan_slam es el de /scan desplazado stamp_offset (también al cruzar un segundo),
# y el resto del mensaje no cambia.
# Uso (con ROS cargado):  python3 -m pytest autonomous_nav/test/test_scan_slam.py -v
import os
import sys
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from builtin_interfaces.msg import Time as Stamp  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from autonomous_nav.scan_slam import ScanSlam, resellar  # noqa: E402


def test_resta_una_decima_cruzando_el_segundo():
    s = resellar(Stamp(sec=10, nanosec=50_000_000), -0.10)
    assert (s.sec, s.nanosec) == (9, 950_000_000)


def test_callback_publica_el_mismo_scan_con_el_sello_corregido():
    publicados = []
    n = SimpleNamespace(offset=-0.10, publisher=SimpleNamespace(publish=publicados.append))
    msg = LaserScan()
    msg.header.stamp = Stamp(sec=100, nanosec=300_000_000)
    msg.ranges = [1.0, 2.0]
    ScanSlam.scan_callback(n, msg)
    assert len(publicados) == 1
    assert (publicados[0].header.stamp.sec, publicados[0].header.stamp.nanosec) == (100, 200_000_000)
    assert list(publicados[0].ranges) == [1.0, 2.0]
