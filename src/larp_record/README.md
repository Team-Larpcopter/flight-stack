# larp_record

Records every ROS 2 topic to a rosbag, with raw images replaced by their
compressed copies. It runs only when you start it; no launch file starts it.

```bash
ros2 run larp_record record                      # Ctrl-C stops and closes the bag
ros2 run larp_record record --name dronehacks-0.3
ros2 run larp_record record --duration 120       # stop after 2 minutes
ros2 run larp_record record --images raw         # also keep raw images (large)
ros2 run larp_record record --help               # all options
```

Start it in its own terminal, before or after the simulator or a flight launch
such as `ros2 launch flight_executive dronehacks.launch.py`. Topics are picked
up as they appear. Source the same overlays as the flight stack first, so
custom message types such as `flight_executive_msgs` can be recorded; topics
whose type cannot be loaded are skipped.

Bags go to `$LARP_BAG_DIR/<UTC start>[-name]/`, or `~/larp_bags/` when
`LARP_BAG_DIR` is unset. Inside the `flight-systems` Dev Container,
`export LARP_BAG_DIR=/workspaces/flight-systems/artifacts/bags` keeps bags in
the ignored `artifacts/` folder. The recorder refuses to start with less than
5 GiB free (`--min-free-gb`).

## What is recorded

Every topic, hidden topics included, except raw images:

| Recorded | Skipped |
| --- | --- |
| PX4 `/fmu/out/*` and `/fmu/in/*`, `/larper/*`, IMU, TF, `camera_info` | `/stereo/left/image`, `/stereo/right/image` |
| `*/compressed` (JPEG from the camera bridge) | `/camera/depth/image_raw`, `/camera/depth/image_visual` |
| `/camera/depth/image_raw/compressedDepth` (started by this script) | any other raw image topic |

A topic counts as an image when a name segment is `image` or starts with
`image_`. It is kept anyway when it ends in `/camera_info`, `/compressed` or
`/compressedDepth`. `--images none` also skips compressed images.

- Stereo images are JPEG at the camera bridge's quality (60), so they are
  lossy; record with `--images raw` if you need exact pixels, for example to
  replay into VIO.
- Metric depth is stored as quantised PNG: precise up close, coarser beyond
  about 10 m. `--depth-topic ""` turns this off.
- `/fmu/in/*` topics appear only while something publishes to PX4, such as the
  flight executive during a flight.

At the simulator's 30 Hz this is about 1 MB/s.

## Storage

`--storage auto` uses MCAP when `ros-humble-rosbag2-storage-mcap` is installed,
otherwise SQLite (`.db3`). Do not open a `.db3` while it is recording, for
example in Foxglove: the SQLite lock makes the recorder crash. Open it after
recording stops. The script repairs a bag that was not closed cleanly with
`ros2 bag reindex`.

`--split-seconds S` starts a new file every S seconds. `--zstd` compresses each
file when it closes; such bags are smaller but must be decompressed before
Foxglove can open them.

The recorder only subscribes; it never publishes to PX4. For the real drone,
see `jetson/flightbag/`.
