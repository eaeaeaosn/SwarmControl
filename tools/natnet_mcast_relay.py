#!/usr/bin/env python3
"""NatNet multicast -> unicast relay (run on the macOS host, not in the VM).

Why: UTM/macOS "bridged" networking over Wi-Fi forwards unicast to the VM but
drops multicast, so Motive's multicast stream (239.255.42.99:1511) never reaches
the Linux VM. The Mac itself does receive it, so this script joins the group on
the Mac and re-sends every packet as unicast UDP to the VM.

Usage (macOS Terminal; standard library only):
    python3 natnet_mcast_relay.py --vm <VM_IP>
Options:
    --vm IP            VM address (required; `ip -4 -br addr` inside the VM)
    --vm-port N        port on the VM (default 1511, what natnet_py_client binds)
    --mcast ADDR       Motive multicast group (default 239.255.42.99)
    --port N           Motive data port (default 1511)
    --iface IP         Mac interface IP to join on (default: let macOS choose;
                       set to the Mac's Wi-Fi IP if it joins the wrong interface)

In the VM, keep connection_type: 0 (multicast) in the client config; the client's
data socket also receives the relayed unicast packets on port 1511.
Stop with Ctrl+C.
"""
import argparse
import socket
import struct
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--vm", required=True)
    ap.add_argument("--vm-port", type=int, default=1511)
    ap.add_argument("--mcast", default="239.255.42.99")
    ap.add_argument("--port", type=int, default=1511)
    ap.add_argument("--iface", default="0.0.0.0")
    a = ap.parse_args()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):  # lets other Mac apps listen on 1511 too
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    rx.bind(("", a.port))
    rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                  struct.pack("4s4s", socket.inet_aton(a.mcast), socket.inet_aton(a.iface)))
    rx.settimeout(1.0)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (a.vm, a.vm_port)
    print("relaying %s:%d -> %s:%d (Ctrl+C to stop)" % (a.mcast, a.port, a.vm, a.vm_port))

    total, window, t_last, sources = 0, 0, time.time(), set()
    try:
        while True:
            try:
                data, src = rx.recvfrom(65535)
            except socket.timeout:
                data = None
            if data is not None and src[0] != a.vm:  # never echo the VM's own packets
                try:
                    tx.sendto(data, dest)
                    total += 1
                    window += 1
                    sources.add(src[0])
                except OSError as e:
                    print("send to VM failed: %s" % e)
            now = time.time()
            if now - t_last >= 5.0:
                rate = window / (now - t_last)
                print("%.0f pkt/s, %d total, from %s" % (rate, total, ", ".join(sorted(sources)) or "-"))
                if window == 0:
                    print("  no multicast received: is Motive streaming in Multicast mode?")
                window, t_last = 0, now
    except KeyboardInterrupt:
        print("\nstopped after %d packets" % total)


if __name__ == "__main__":
    main()
