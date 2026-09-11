import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
import udp_client  # noqa: E402


class UdpClientTests(unittest.TestCase):
    def test_packet_round_trip(self):
        encoded = udp_client.encode_packet(udp_client.MSG_FORCE, 42, b"\x03")
        packet = udp_client.decode_packet(encoded)
        self.assertEqual(packet.message_type, udp_client.MSG_FORCE)
        self.assertEqual(packet.sequence, 42)
        self.assertEqual(packet.payload, b"\x03")

    def test_crc_rejection(self):
        encoded = bytearray(udp_client.encode_packet(udp_client.MSG_HELLO, 7))
        encoded[3] ^= 1
        with self.assertRaises(ValueError):
            udp_client.decode_packet(encoded)

    def test_wire_sizes(self):
        self.assertEqual(udp_client.HEADER.size, 12)
        self.assertEqual(udp_client.MOTOR_PAYLOAD.size, 28)
        self.assertEqual(udp_client.TELEMETRY_PAYLOAD.size, 68)


if __name__ == "__main__":
    unittest.main()
