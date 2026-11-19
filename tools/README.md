# tools

Helpers for running the OptiTrack pipeline on an arm64 Linux VM (UTM on Apple Silicon),
where the C++ `mocap_optitrack_client` can't be built because OptiTrack's NatNet SDK
library (`libNatNet.so`) is x86_64 only.

| File | Runs on | What it does |
|---|---|---|
| `natnet_py_client.py` | Linux VM (ROS 2) | Drop-in replacement for `mocap_optitrack_client`. Parses NatNet 3.x/4.x UDP directly (no SDK) and publishes `mocap_optitrack_interfaces/RigidBodyArray` on `mocap_rigid_bodies`. Rigid bodies only; markers are not published. |
| `natnet_mcast_relay.py` | macOS host | Forwards Motive's multicast stream to the VM as unicast. Only needed when the Mac is on Wi-Fi (see below). Python standard library only. |

## Running the client

```bash
source /opt/ros/humble/setup.bash && source ~/swarm_ws/install/setup.bash
python3 tools/natnet_py_client.py --ros-args --params-file <your natnetclient.yaml>
```

It reads the same parameters as `mocap_optitrack_client/config/natnetclient.yaml`
(set `server_address` to the Motive PC's IP), plus optional ones:
`frame_id` (default `world`), `local_address` (interface to join multicast on, default `0.0.0.0`),
`natnet_version` (`[0, 0]` = ask the server).

Expected log: `Server NatNet version X.Y`, then `frames received: N` every 2 s.
Check with `ros2 topic hz /mocap_rigid_bodies`.

Parser self-test (no ROS or Motive needed; includes a real Motive 3 / NatNet 4.2 frame):

```bash
python3 tools/natnet_py_client.py --selftest
```

Tested live against Motive 3.4 (NatNet 4.2) at about 120 Hz.

## Mac on Wi-Fi: use the multicast relay

macOS can't truly bridge a VM over Wi-Fi. UTM's "bridged" mode then forwards unicast to the
VM but drops multicast, so Motive's multicast stream (`239.255.42.99:1511`) reaches the Mac
but never the VM. Symptom: the client logs the server version (command port works) but
`frames received: 0`.

Fix: on the Mac, run

```bash
python3 natnet_mcast_relay.py --vm <VM_IP>     # VM_IP from `ip -4 -br addr` in the VM
```

and keep `connection_type: 0` (multicast) in the client config. The relay prints the packet
rate every 5 s (about 120 pkt/s expected). Alternatives: connect the Mac by Ethernet
(multicast then works directly), or set Motive's Transmission Type to Unicast and
`connection_type: 1` (not yet verified with this client).
