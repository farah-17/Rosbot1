# PDE2823 Assessment 1 – Autonomous Navigation on the Raspbot

A Raspbot V1 (YDLIDAR TG30, ROS 2 Humble, Raspberry Pi) that **maps a room, localises on the saved map,
and drives by itself to a destination chosen from a terminal menu**, with live feedback
(accepted / driving / replanning / completed / failed).

> **Quick start for the marker:** jump to [3. Run the demo](#3-run-the-demo-5-commands).
> No robot at hand? See [8. Run without the robot (simulator)](#8-run-without-the-robot-simulator).

---

## 1. What the system does

| Assessment task | How it is done | Where |
|---|---|---|
| Explore and map | YDLIDAR + Cartographer (SLAM), driven by keyboard **only while mapping** | `raspbot_mapping` (course package) |
| Save the map | `.pgm` + `.yaml` (for planning) and `.pbstream` (for localisation), cleaned by hand | `maps/my_room_fine.*` |
| Localise on the saved map | Cartographer **pure localisation** on the `.pbstream`; `set_start` tells it where the robot stands | `navigate.launch.py`, `set_start.py` |
| Autonomous navigation | `navigator` = ROS 2 action server `navigate_to_pose`. A\* on the saved map, then a turn-then-drive controller on `/cmd_vel` | `navigator.py`, `astar.py`, `grid_map.py` |
| Live lidar awareness | Every scan is turned into obstacle cells **in memory only** (the saved map is never modified). Blocked path → **replan** (up to 5 times) | `live_layer.py` |
| Terminal destination selection | Numbered menu, invalid-input handling, live status line, cancel | `destination_menu.py`, `destinations.yaml` |
| Feedback | `ACCEPTED / REJECTED / COMPLETED / CANCELLED / FAILED` + reason | `destination_menu.py`, `/navigation_status` |
| No manual control | After the start pose is set, the robot is driven only by the navigator | – |


## 2. Repository layout

```
.
├── README.md
├── maps/
│   ├── my_room_fine.pgm / .yaml / .pbstream     <- submitted, cleaned map (used for navigation)
│   └── my_room_raw.*                            <- uncleaned export, for comparison
├── raspbot_navigation/                          <- ROS 2 package (copy into <ws>/src/)
│   ├── setup.py  package.xml
│   ├── config/
│   │   ├── navigation.yaml                      <- speeds, tolerances, live-obstacle settings
│   │   └── destinations.yaml                    <- the menu entries (name, x, y in map frame)
│   ├── launch/
│   │   ├── navigate.launch.py                   <- real robot (lidar + motors + Cartographer + navigator)
│   │   └── sim.launch.py                        <- simulator, no hardware needed
│   ├── raspbot_navigation/
│   │   ├── destination_menu.py                  <- terminal interface (run this to demo)
│   │   ├── navigator.py                         <- action server, planning, driving, replanning
│   │   ├── astar.py   grid_map.py   live_layer.py
│   │   ├── set_start.py                         <- sets the start pose of Cartographer
│   │   ├── nav_gui.py                           <- optional map window with live lidar dots
│   │   ├── check_heading.py                     <- helper: checks heading vs real motion
│   │   └── sim_robot.py  astar_demo.py
│   └── test/                                    <- pytest unit tests
└── raspbot_mapping/                             <- course package (lidar driver, motor driver, Cartographer config)
    └── config/motors.yaml                       <- motor power settings (see 6.2)
```

---

## 3. Run the demo (5 commands)

All commands run **inside the Docker container** unless stated. Prerequisites and one-time setup are in
sections 4 and 5.

**Safety:** clear 1 m of floor around the robot, stay next to it. Ctrl+C in the launch terminal stops everything.

1. **Place the robot** on the tape mark at the map origin (x = 0, y = 0), facing the arrow (the +x direction).
2. **Terminal 1 – Raspberry Pi → open container, start the robot stack**
   ```bash
   lidar                                   # shortcut that opens the container (see 4.2)
   cd /root/raspbot_lidar_ws && source install/setup.bash
   ros2 launch raspbot_navigation navigate.launch.py map_name:=my_room_fine
   ```
   Wait until you see `navigator ready` and `set_start ready: waiting for /initialpose`.
3. **Set the start pose** (tells Cartographer where the robot is on the saved map). Either click
   *Start* in the map window and click the origin, **or** in a second container terminal:
   ```bash
   ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
     "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
   ```
   Terminal 1 must print `set_start: ... Cartographer: Success`.
   Check: `ros2 run tf2_ros tf2_echo map base_link` should show about x≈0, y≈0.
4. **Terminal 2 – open the menu** (a new Pi terminal, type `lidar` again, then):
   ```bash
   cd /root/raspbot_lidar_ws && source install/setup.bash
   python3 src/raspbot_navigation/raspbot_navigation/destination_menu.py
   ```
5. **Type a number + Enter.** The robot drives by itself. `c` + Enter (or Ctrl+C) cancels.

### The menu

```
=== Raspbot destinations ===
  1) Home          ( -0.01, -0.01)
  2) Blue box      (  0.23,  0.44)
  3) Small box     (  0.32,  1.19)
  4) Last ras box  (  0.83,  0.59)
  5) The book      (  0.32,  0.74)
  S) Set start position      R) Reload destinations      0) Exit
```

Typical output while driving:

```
Goal ACCEPTED -> Blue box (0.23, 0.44)
  to go 0.52 m | 4 s | replans 0 | pose (0.03,0.05) | lidar front 0.84 m
  Replanning: obstacle on the path
Goal COMPLETED. Final error 6 cm.
```

---

## 4. Requirements and Docker

### 4.1 Hardware
* Yahboom Raspbot V1 on a Raspberry Pi 4/5 (64-bit OS with Docker installed)
* YDLIDAR TG30 on `/dev/ydlidar` (512000 baud), mounted 0.13 m behind the robot centre, rotated 180°
  (launch args `laser_x:=-0.13 laser_z:=0.12 laser_yaw:=3.141592` – already the defaults)
* Charged battery. **Weak batteries make the robot refuse to move on the floor** (see 9)

### 4.2 Docker container
The project runs in the course container `raspbot_lidar` (ROS 2 Humble, Cartographer, nav2 map-server and the
`raspbot_mapping` package). On the Pi the shortcut `lidar` opens a terminal in it. To see exactly what it does:

```bash
type lidar          # shows the alias / function that starts the container
docker ps           # the container should be listed once started
```

If you must recreate the container, use the same command as the `lidar` alias. A template (adapt image name
and paths to yours):

```bash
docker run -it --rm --name raspbot_lidar \
  --privileged --net=host \
  -v /dev:/dev \
  -v ~/raspbot_lidar_ws:/root/raspbot_lidar_ws \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  <course_image> bash
```

| Flag | Why |
|---|---|
| `--privileged`, `-v /dev:/dev` | access to the lidar (`/dev/ydlidar`) and the motor driver (I²C) |
| `--net=host` | ROS 2 discovery between several container terminals |
| `-v ~/raspbot_lidar_ws:...` | your workspace and maps survive container restarts |
| `DISPLAY`, `/tmp/.X11-unix` | only needed for the map window / RViz. On the Pi desktop run `xhost +local:` first |

**One container, one stack.** Duplicate stacks cause `TF_OLD_DATA` / "Ignored subdivision of a LaserScan"
errors and lag. Before starting, check:

```bash
docker ps                                  # only ONE raspbot_lidar container
ros2 node list | sort | uniq -c            # every node must appear exactly once
# if something is duplicated:
docker stop $(docker ps -q)                # on the Pi, then start again with: lidar
```

### 4.3 Install the project (one time)

On the Pi (outside the container), put the files in the workspace that the container mounts:

```bash
cd ~/raspbot_lidar_ws/src
git clone <your-repo-url> tmp_repo
cp -r tmp_repo/raspbot_navigation .
cp tmp_repo/maps/* ~/raspbot_lidar_ws/maps/
```

Then inside the container:

```bash
cd /root/raspbot_lidar_ws
colcon build --symlink-install --packages-select raspbot_navigation
source install/setup.bash
ros2 pkg executables raspbot_navigation        # should list navigator, nav_gui, set_start, ...
ls maps | grep my_room_fine                    # .pgm  .yaml  .pbstream must all exist
```

> Tip: to change a file, **copy it** (`cp`), do not paste into an open editor – this avoids corrupted files.

---

## 5. Pre-demo checklist (2 minutes)

| Check | Command (in container) | Expected |
|---|---|---|
| Lidar publishes | `ros2 topic hz /scan` | ≈ 10 Hz or more |
| Single `motor_driver` | `ros2 node list \| sort \| uniq -c` | each line `1` |
| Localisation running | `ros2 run tf2_ros tf2_echo map base_link` | a pose, no errors (after step 3) |
| Navigator up | `ros2 action list` | `/navigate_to_pose` |
| Motors work | `ros2 topic pub -r 20 -t 40 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"` | robot drives ≈ 40–60 cm |
| Heading is right | `python3 src/raspbot_navigation/raspbot_navigation/check_heading.py` | `RESULT: OK` |

---

## 6. Configuration

### 6.1 Destinations – `config/destinations.yaml`
Coordinates are in the **map frame, metres**: origin = start spot, +x = direction the robot faces at the start,
+y = to the robot's left. Read them off RViz/`tf2_echo` by placing the robot on the spot.

```yaml
destinations:
  - {name: Home,         x: -0.01, y: -0.01}
  - {name: Blue box,     x: 0.23,  y: 0.44}
  - {name: Small box,    x: 0.32,  y: 1.19}
  - {name: Last ras box, x: 0.83,  y: 0.59}
  - {name: The book,     x: 0.32,  y: 0.74}
```
Edit, then press `R` in the menu to reload. A goal within about 12 cm of free space is snapped to it;
a goal inside a wall or outside the map is **rejected** with a message.
A different file: `python3 destination_menu.py --ros-args -p destinations_file:=/path/to/file.yaml`

### 6.2 Motors – `raspbot_mapping/config/motors.yaml`
The robot has no wheel encoders (open loop). The driver converts speed to power:
`power = max(|cmd| / max_linear × max_pwm, min_pwm)`. On this robot the wheels only move the robot on the floor
at about **80 %** power, so:

```yaml
min_pwm: 80
```
Try it live first: `ros2 param set /motor_driver min_pwm 80`.

### 6.3 Navigation – `config/navigation.yaml` (section `navigator`)

| Parameter | Value | Meaning |
|---|---|---|
| `linear_speed` | 0.4 | m/s command (= `min_pwm/100 × 0.5`) |
| `angular_speed` | 0.8 | rad/s command (= `min_pwm/100 × 1.0`) |
| `goal_tolerance` | 0.15 | m: goal reached |
| `waypoint_tolerance` | 0.18 | m: path point reached |
| `obstacle_distance` | 0.35 | m: stop/replan if the front is closer than this |
| `live_obstacles` | true | use the live lidar layer |
| `live_ttl` | 3.0 | s an obstacle stays after its last lidar hit |
| `live_min_hits` | 2 | scans needed before a cell counts (noise filter) |
| `live_inflation` | 0.15 | m kept free around live obstacles |
| `max_replans` | 5 | after that the goal FAILS |

Restart the launch after editing. If a change has no effect: `colcon build --symlink-install` and `source install/setup.bash`.

---

## 7. How it works (for the demonstration)

1. **Mapping:** Cartographer builds a pose graph from lidar scans only (no odometry). The result was exported
   to `.pgm/.yaml` and cleaned (stray points, gaps) so it is suitable for planning, and saved as `.pbstream`.
2. **Localisation:** `navigate.launch.py` starts Cartographer in *pure localisation* with the `.pbstream`
   (saved trajectory frozen, a new trajectory matched against it). `set_start` begins that trajectory at the
   pose given on `/initialpose`. Cartographer then publishes `map → odom → base_link`.
3. **Planning:** `grid_map.py` loads the `.pgm`, inflates walls by the robot radius; `astar.py` finds the
   shortest path on the 8-connected grid; the path is thinned to waypoints.
4. **Driving:** for each waypoint the navigator turns toward it, then drives forward, 10 times a second,
   correcting heading with the Cartographer pose. If it stops publishing, the motor driver's `cmd_timeout`
   stops the robot.
5. **Live obstacles:** every scan point is projected into the map; cells hit repeatedly become temporary
   obstacles (kept in RAM, expire after `live_ttl`). If the path is blocked the navigator **replans** with them
   included. The saved map files are never written to.
6. **Interface:** `destination_menu.py` is an action client of `navigate_to_pose`. The action gives
   *accepted/rejected*, periodic feedback (distance remaining, replans) and a result
   (*succeeded/aborted/cancelled*); `/navigation_status` carries the human-readable failure reason.

### Feedback and error handling

| Situation | What the user sees |
|---|---|
| Invalid menu input (letters, out-of-range number, empty) | message and the menu again, no crash |
| Goal in a wall / outside the map | `Goal REJECTED: ...` |
| Navigator not running | `navigate_to_pose server not available` instead of hanging |
| Obstacle on the path | `Replanning ...`, then continues |
| No way around (5 replans) | `Goal FAILED. path blocked by an obstacle, no way around it after 5 replans` |
| Robot outside the map (lost localisation) | `FAILED: the robot is outside the map (localisation lost). Put it on its start spot and set the start position again` – motors stop |
| User cancels | `Goal CANCELLED`, robot stops |
| Repeated requests | a new goal can be chosen right after completion/failure; `S` re-sets the start |

---

## 8. Run without the robot (simulator)

```bash
ros2 launch raspbot_navigation sim.launch.py                     # pretend robot + lidar + navigator + map window
ros2 launch raspbot_navigation sim.launch.py map:=/root/raspbot_lidar_ws/maps/my_room_fine.yaml
python3 src/raspbot_navigation/raspbot_navigation/destination_menu.py     # in a second terminal
```

Unit tests (A\*, map, live layer):

```bash
cd /root/raspbot_lidar_ws/src/raspbot_navigation && python3 -m pytest -v test
```

---

## 9. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `TF_OLD_DATA`, "Ignored subdivision of a LaserScan", lag | duplicate stack running → `docker stop $(docker ps -q)`, restart with `lidar`, check `ros2 node list` |
| Robot does not move on the floor, but does when lifted | too little power → `min_pwm` 80 (6.2), charge the battery |
| Robot goes the wrong way | wrong start pose (set it again exactly on the mark, facing +x); run `check_heading.py` |
| Position jumps after fast turns | lower `angular_speed` or keep the room as when it was mapped (moved furniture breaks localisation) |
| `Goal REJECTED` | destination inside a wall – move it a little into free space in `destinations.yaml` |
| `couldn't connect to display ":0"` | map window needs a display: `xhost +local:` on the Pi desktop and `-e DISPLAY` in docker; the menu works without it |
| `no attribute 'main'` / `SyntaxError` after copying a file | the file was pasted into a half-open editor; `cp` the whole file again, then rebuild |
| `ros2: executable not found` | `source install/setup.bash` after `colcon build` |
| Lidar not found | `ls -l /dev/ydlidar`; container must run with `--privileged` and `-v /dev:/dev` |

---

## 10. Known limitations

* No wheel encoders: position comes from the lidar alone, so localisation degrades if the room changes a lot
  after mapping or if the robot turns very fast.
* Speeds are limited from below by the motor floor (`min_pwm`), so steering is coarse.
* The live obstacle layer only sees what the 2D lidar sees at its height.
* Destinations are fixed points in a YAML file, not drawn on the map.

---

## 11. Assessment mapping

| Rubric item | Evidence |
|---|---|
| Mapping (25 %) | `maps/my_room_fine.*` (cleaned) and `my_room_raw.*`; section 7.1 |
| Autonomous navigation (25 %) | `navigator.py`, `astar.py`, live replanning; section 3 |
| Terminal selection (10 %) | `destination_menu.py`; section 3 menu |
| Robustness (10 %) | section 7 error table |
| Code quality + README (10 %) | section 2 layout, commented modules, unit tests (section 8) |
| Demonstration (20 %) | section 3 (5 commands) and section 7 (how it works) |

*Author: Farah Emad · PDE2823 Robotics · ROS 2 Humble*
