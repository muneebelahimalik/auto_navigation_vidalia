"""gnss_logger: port discovery and topic selection (no farm-ng SDK needed)."""
import socket

from scripts import gnss_logger as g


def test_listening_port_is_discovered():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    s.listen(1)
    try:
        assert s.getsockname()[1] in g.listening_tcp_ports()
    finally:
        s.close()


def test_gps_topic_selection():
    assert g.is_gps_topic("/pvt", "service_name=gps")
    assert g.is_gps_topic("/state", "service_name=gnss")
    assert not g.is_gps_topic("/state", "service_name=canbus")
    assert not g.is_gps_topic("/rgb", "service_name=oak0")
