#!/usr/bin/env python3
"""SDK-free NatNet client for ROS 2 (replacement for mocap_optitrack_client).

Reads NatNet 3.x/4.x UDP packets straight from Motive (no libNatNet.so, so it
runs on arm64) and publishes mocap_optitrack_interfaces/RigidBodyArray, the same
message and topic the original C++ client publishes.

Reuses the original config file (natnetclient.yaml, node name "natnet_client").

Run:
  python3 natnet_py_client.py --ros-args --params-file <path>/natnetclient.yaml
Parser self-test (no ROS or Motive needed):
  python3 natnet_py_client.py --selftest
"""
import socket
import struct
import sys
import threading
import time

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from mocap_optitrack_interfaces.msg import RigidBody, RigidBodyArray
except ImportError:  # lets --selftest run on a machine without ROS
    rclpy = None
    Node = object
    ExternalShutdownException = KeyboardInterrupt

NAT_PING = 0
NAT_SERVERINFO = 1
NAT_FRAMEOFDATA = 7


def read_cstr(buf, off):
    end = buf.index(b"\x00", off)
    return buf[off:end].decode("utf-8", "replace"), end + 1


def parse_server_info(payload):
    """Payload of NAT_SERVERINFO: name[256], app version[4], NatNet version[4]."""
    major, minor = payload[260], payload[261]
    return major, minor


def parse_rigid_bodies(payload, version):
    """Parse a NAT_FRAMEOFDATA payload up to and including the rigid bodies.

    Returns (frame_number, [(id, (x, y, z), (qx, qy, qz, qw), mean_error, valid)]).
    Layout (NatNet 3+): frame number, marker sets, unlabeled markers, rigid bodies.
    """
    major, minor = version
    if major < 3:
        raise ValueError("NatNet %d.%d is not supported (need >= 3.0)" % (major, minor))
    # NatNet >= 4.1 writes a section size in bytes (int32) after each section count.
    has_sizes = (major, minor) >= (4, 1)
    off = 0
    (frame_no,) = struct.unpack_from("<i", payload, off)
    off += 4

    def read_count():
        nonlocal off
        (n,) = struct.unpack_from("<i", payload, off)
        off += 8 if has_sizes else 4
        return n

    n_sets = read_count()
    for _ in range(n_sets):
        _, off = read_cstr(payload, off)
        (n_mk,) = struct.unpack_from("<i", payload, off)
        off += 4 + 12 * n_mk

    n_other = read_count()
    off += 12 * n_other

    n_rb = read_count()
    bodies = []
    for _ in range(n_rb):
        (rb_id,) = struct.unpack_from("<i", payload, off)
        off += 4
        x, y, z, qx, qy, qz, qw = struct.unpack_from("<7f", payload, off)
        off += 28
        (err,) = struct.unpack_from("<f", payload, off)
        off += 4
        (params,) = struct.unpack_from("<h", payload, off)
        off += 2
        bodies.append((rb_id, (x, y, z), (qx, qy, qz, qw), err, bool(params & 0x01)))
    return frame_no, bodies


class NatNetClient(Node):
    def __init__(self):
        super().__init__("natnet_client")
        d = self.declare_parameter
        self.server = d("server_address", "192.168.0.245").value
        self.unicast = d("connection_type", 0).value == 1
        self.mcast = d("multi_cast_address", "239.255.42.99").value
        self.cmd_port = d("server_command_port", 1510).value
        self.data_port = d("server_data_port", 1511).value
        topic = d("pub_topic", "mocap_rigid_bodies").value
        self.frame_id = d("frame_id", "world").value
        self.local_ip = d("local_address", "0.0.0.0").value
        forced = d("natnet_version", [0, 0]).value  # [0, 0] = autodetect
        # Parameters present in the original yaml but unused here:
        d("marker_topic", "mocap_markers")
        d("record", False)
        d("take_name", "")

        self.version = tuple(forced) if forced[0] else None
        self.pub = self.create_publisher(RigidBodyArray, topic, 10)
        self.running = True
        self.n_frames = 0
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.create_timer(2.0, self.report)

    def report(self):
        self.get_logger().info("frames received: %d (version %s)" % (self.n_frames, self.version))

    def open_sockets(self):
        data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        data.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        data.bind(("", self.data_port))
        if not self.unicast:
            mreq = struct.pack("4s4s", socket.inet_aton(self.mcast), socket.inet_aton(self.local_ip))
            data.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        data.settimeout(1.0)
        cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        cmd.bind((self.local_ip, 0))
        cmd.settimeout(1.0)
        return data, cmd

    def ping(self, cmd):
        """Ask the server for its NatNet version (also registers us for unicast)."""
        cmd.sendto(struct.pack("<HH", NAT_PING, 5) + b"Ping\x00", (self.server, self.cmd_port))
        try:
            pkt, _ = cmd.recvfrom(65535)
        except socket.timeout:
            return
        msg_id, size = struct.unpack_from("<HH", pkt, 0)
        if msg_id == NAT_SERVERINFO and self.version is None:
            self.version = parse_server_info(pkt[4:4 + size])
            self.get_logger().info("Server NatNet version %d.%d" % self.version)

    def loop(self):
        data, cmd = self.open_sockets()
        last_ping = 0.0
        while self.running and rclpy.ok():
            now = time.time()
            if self.version is None or (self.unicast and now - last_ping > 5.0):
                self.ping(cmd)  # in unicast mode, keep re-announcing ourselves
                last_ping = now
            try:
                pkt, _ = data.recvfrom(65535)
            except socket.timeout:
                continue
            if len(pkt) < 4 or self.version is None:
                continue
            msg_id, size = struct.unpack_from("<HH", pkt, 0)
            if msg_id != NAT_FRAMEOFDATA:
                continue
            try:
                _, bodies = parse_rigid_bodies(pkt[4:4 + size], self.version)
            except (struct.error, ValueError, IndexError) as e:
                self.get_logger().error("Frame parse failed: %s" % e, throttle_duration_sec=2.0)
                continue
            try:
                self.publish(bodies)
            except Exception:  # context gone during shutdown
                if self.running and rclpy.ok():
                    raise
                break
        data.close()
        cmd.close()

    def publish(self, bodies):
        msg = RigidBodyArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for rb_id, p, q, err, valid in bodies:
            rb = RigidBody()
            rb.id = rb_id
            rb.valid = valid
            rb.mean_error = float(err)
            rb.header = msg.header
            rb.pose_stamped.header = msg.header
            rb.pose_stamped.pose.position.x = float(p[0])
            rb.pose_stamped.pose.position.y = float(p[1])
            rb.pose_stamped.pose.position.z = float(p[2])
            rb.pose_stamped.pose.orientation.x = float(q[0])
            rb.pose_stamped.pose.orientation.y = float(q[1])
            rb.pose_stamped.pose.orientation.z = float(q[2])
            rb.pose_stamped.pose.orientation.w = float(q[3])
            msg.rigid_bodies.append(rb)
        self.pub.publish(msg)
        self.n_frames += 1


# A real NAT_FRAMEOFDATA packet (incl. 4-byte header) captured from Motive 3 / NatNet 4.2:
# marker sets "Go2 Bots" and "all" (5 markers each), one rigid body (id 1006, tracked).
REAL_FRAME_4_2 = bytes.fromhex(
    "0700af01a3a40200020000008d000000476f3220426f74730005000000bb19c03f28bec73e6d0ddc3f"
    "2547d83ff096c73e2e7cdc3f04a2c93f963edc3e488eb83fcf25d03f4b1adc3e695fb93f5b3ccd3f"
    "bf28d83eb9c8a63f616c6c0005000000bb19c03f28bec73e6d0ddc3f2547d83ff096c73e2e7cdc3f"
    "04a2c93f963edc3e488eb83fcf25d03f4b1adc3e695fb93f5b3ccd3fbf28d83eb9c8a63f00000000"
    "000000000100000026000000ee03000079b0cc3fdd2fd33e7568c33f47fae3baa766cd3e36f543bc"
    "e2796abf7bef793a01000000000000000000000000000000000005000000820000000100ee03bb19"
    "c03f28bec73e6d0ddc3f2a08713c4a02d6cf8e3a0200ee032547d83ff096c73e2e7cdc3f8ed8803c"
    "4a02b4a4913a0300ee0304a2c93f963edc3e488eb83f4927613c4a02d4db5b3a0400ee03cf25d03f"
    "4b1adc3e695fb93fc00e833c4a0277728f3a0500ee035b3ccd3fbf28d83eb9c8a63f0c0a753c4a02"
    "71a0633a000000000000000000000000000000000000000000000000d55e23cd241f9e40129cba77"
    "030000005084bb7703000000fba6bb77030000000000000000000000000000000000")


def selftest():
    """Check the parser on a synthetic NatNet 3.x frame and a real NatNet 4.2 frame."""
    p = struct.pack("<i", 42)                                   # frame number
    p += struct.pack("<i", 1) + b"all\x00" + struct.pack("<i", 2) + struct.pack("<6f", *range(6))
    p += struct.pack("<i", 1) + struct.pack("<3f", 9, 9, 9)     # unlabeled markers
    p += struct.pack("<i", 2)                                   # two rigid bodies
    p += struct.pack("<i7ffh", 1, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.001, 1)
    p += struct.pack("<i7ffh", 2, 4.0, 5.0, 6.0, 0.0, 0.0, 0.7071, 0.7071, 0.002, 0)
    p += b"\x00" * 40                                           # trailing data is ignored
    frame, bodies = parse_rigid_bodies(p, (3, 1))
    assert frame == 42 and len(bodies) == 2, (frame, bodies)
    assert bodies[0][0] == 1 and bodies[0][1] == (1.0, 2.0, 3.0) and bodies[0][4] is True
    assert bodies[1][0] == 2 and bodies[1][4] is False
    assert abs(bodies[1][2][3] - 0.7071) < 1e-6

    msg_id, size = struct.unpack_from("<HH", REAL_FRAME_4_2, 0)
    assert msg_id == NAT_FRAMEOFDATA
    frame, bodies = parse_rigid_bodies(REAL_FRAME_4_2[4:4 + size], (4, 2))
    assert len(bodies) == 1 and bodies[0][0] == 1006 and bodies[0][4] is True, bodies
    q = bodies[0][2]
    assert abs(sum(c * c for c in q) - 1.0) < 1e-3, q           # unit quaternion
    assert all(abs(c) < 10 for c in bodies[0][1]), bodies[0][1]  # position in metres
    print("selftest OK (synthetic NatNet 3.1 frame + real Motive NatNet 4.2 frame)")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    if rclpy is None:
        sys.exit("rclpy / mocap_optitrack_interfaces not found: source ROS and ~/swarm_ws/install/setup.bash")
    rclpy.init(args=sys.argv)
    node = NatNetClient()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.running = False
    node.thread.join(timeout=2.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
