# Gaussian cluster transfer experiment — real-world setup

This replaces the Dubins-MPC obstacle-avoidance experiment with a 5-robot
run of the trained PPO policy in
`/home/eason/Swarm/gaussian_cluster_transfer_controller` (see that
package's own `README.md` for the algorithm/policy details — this document
is only the physical/ROS setup procedure). It follows the same pattern as
the circle-to-ellipse experiment (`CIRCLE_TO_ELLIPSE_EXPERIMENT.md`);
skim that doc too if anything here is terse.

## 1. Flash the 5 robots

`SwarmBotCode/include/config.h` already defines `ROBOT_ID` 1..5 with a
distinct LED color each: **1=red, 2=orange, 3=yellow, 4=green, 5=cyan**.
Flash each physical robot with its `ROBOT_ID` (`ROBOT_ID=1 pio run -e swarmbot
--target upload`, etc.) and use the LED color to identify which unit is
"robot1..5" when you place them — this is the mapping the position/beta
table below assumes. (Same robots/flash step as the circle-to-ellipse
experiment — no need to reflash if you already did that one.)

## 2. Physical arena

The policy was trained on a fixed **2 m x 2 m** square with x, y both in
**[-1, 1] m** — there is no arena-scale parameter to tune here (unlike the
old MPC's `arena_half_width`), so the physical square must actually be 2 m
per side.

- Tape or mark a 2 m x 2 m square on the floor.
- In Motive, set the ground plane / world origin so **(0, 0) is the exact
  center of that square**.
- Heading convention: `theta = 0` points along `+x`, positive rotation is
  counterclockwise. Verify your Motive axis convention matches this — drive
  one robot forward by hand with `ros2 topic pub` a small `+linear.x` and
  confirm in RViz that it moves in the direction, and sense, you expect
  before trusting the real run.

## 3. Assign Motive rigid-body IDs

Edit `robot_ids` in `config/swarm_params.yaml` (`mocap_republisher` block) to
the real Optitrack rigid-body asset IDs, **in the same robot1..5 order** as
the LED colors from step 1:

```yaml
mocap_republisher:
  ros__parameters:
    robot_ids: [<id for robot1>, <id for robot2>, <id for robot3>, <id for robot4>, <id for robot5>]
```

## 4. Place the robots at their initial positions

All 5 robots start at the positions/headings and fixed baseline beta `b_i`
in `gaussian_cluster_transfer_controller/README.md`'s table (also in
`target_gaussian.json` -> `robot_setup`, full precision) — reproduce those
positions as closely as you can measure:

| Robot | Initial x (m) | Initial y (m) | Heading (rad) |
| --- | --- | --- | --- |
| 1 | -0.5368 | -0.5203 | 0.8374 |
| 2 | -0.6328 | -0.5087 | 0.8006 |
| 3 | -0.3869 | -0.4910 | 0.7968 |
| 4 | -0.4613 | -0.6542 | 0.8288 |
| 5 | -0.4640 | -0.3406 | 0.8754 |

The nominal initial cluster (light-blue ring), nominal target cluster
(gold ring), and the policy's actual 40 target reference points (orange
dots) are drawn live in RViz by `gaussian_visualizer` (see step 6) so you
can check alignment visually before starting.

## 5. Launch

```bash
ros2 launch swarm_controller swarm_full.launch.py experiment:=gaussian_cluster_transfer
# (omit experiment:= or set it to circle_to_ellipse for that experiment
#  instead, or to mpc for the original obstacle-avoidance MPC pipeline.)
```

Confirm the log shows `Subscribing to /robotN/pose` for all 5 robots and
ends with `Ready. Place all 5 robots at their prescribed initial poses,
then call the ~/start service …` — it will **not** start moving robots on
its own.

## 6. Check RViz, then trigger the start

In RViz you should see: 5 colored robot spheres/trails (red, orange, yellow,
green, cyan) and, from `gaussian_visualizer`, a light-blue ring (nominal
initial cluster) and a gold ring (nominal target cluster) plus 40 small
orange dots (the policy's actual target reference points — see that
package's README for why their sampled center is only *approximately*
(0.5, 0.5)). Once all 5 robots are confirmed at their marked
positions/headings:

```bash
ros2 service call /gaussian_cluster_transfer_controller/start std_srvs/srv/Trigger {}
```

This runs at most **400 decisions (80 s)** and stops all 5 robots
automatically at the end; you can also `Ctrl+C` the launch at any time to
publish an immediate stop.

## 7. Safety

- Command speed is capped at **0.05 m/s** for every robot, both by the
  policy's own action range and by this node's beta-stage clip.
- Unlike circle_to_ellipse_controller's package, this package's README
  makes no claim about whether its policy corrects disturbances mid-run —
  treat it as unvalidated for that and watch the run physically, ready to
  cut power/kill the launch if a robot heads somewhere unsafe.
- **Known modeling approximation**: the package's per-robot `beta` law
  (`README.md` section 2) is written for an idealized model where a robot's
  world-frame `vx`/`vy` can be scaled independently. A real differential-drive
  robot can't move sideways, so `gaussian_cluster_transfer_controller.py`
  projects the desired `(vx, vy)` onto each robot's own heading to get a
  realizable forward speed, and sends the law's `omega` straight through as
  the turn rate. This is the one unavoidable deviation from the trained
  simulator's dynamics — expect the real trajectories to track the
  simulation (`gaussian_cluster_transfer.mp4`) only approximately, and watch
  it closely on the first run.

## Dependencies

The policy runs as a **subprocess in its own virtualenv**, not imported into
the ROS node's interpreter — `selected_model.zip` requires `numpy>=2` to
unpickle, which conflicts with this machine's system ROS/OpenCV stack
(`cv_bridge` etc. need `numpy<2`) in one interpreter. Do **not**
`pip install` the RL requirements into the system/ROS Python.

This package's `requirements.txt` is byte-identical to
`circle_to_ellipse_controller`'s, so `swarm_params.yaml`'s
`venv_python` for this node **already defaults to that package's existing
`.venv`** (confirmed working in a hardware-free integration test) — no
extra setup needed if you've already created that venv for the
circle-to-ellipse experiment. To use this package's own venv instead,
create it once and set `venv_python: ''` in `config/swarm_params.yaml`
(`gaussian_cluster_transfer_controller` block):

```bash
cd /home/eason/Swarm/gaussian_cluster_transfer_controller
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

`gaussian_cluster_transfer_controller.py` (the ROS node) launches
`<venv>/bin/python controller.py` itself and talks to it over stdin/stdout
(one state matrix in, one `[[u, v]]` command out per line — the same
protocol the package's own README describes for its command-line check).
