# Updated DroneHacks planning prior

The default 10 cm and 5 cm voxel priors contain the November W1/W2 layout:
W1 turns 45 degrees clockwise about its south attachment to W4; W2 runs
east-west near the north net, starting at the northwest corner. The eight-pane
mission runtime loads the 5 cm file. `full_stack.launch.py` defaults to 10 cm.
Both retain their existing names, so callers using the package defaults receive
the new layout after rebuilding the package/runtime bundle.

The transferred geometry is the **wall layout**, not a direct import of the
entire Scaniverse PLY. The PLY has an unregistered scan origin and does not
provide the observed-free cells consumed by `ObservedMap::loadCadPrior`.
The updated layout keeps the existing CAD panel dimensions and uses these
approximate schematic placements:

| Panel | Placement in course metres, southwest origin |
| --- | --- |
| W1 | Pivot `(1.1175, 3.556)`, length `1.613 m`, heading 45° from east |
| W2 | `(0.12, 5.70)` to `(1.733, 5.70)` |

Both panels retain CAD thickness `0.1016 m` and height `1.7272 m`. W2's center
is about 0.40 m inside the north net, corresponding to clearance for its
supports in the edited scan. The voxel prior remains a solid CAD wall proxy;
it does not add individually reconstructed support feet or other scan clutter.
The old panel volumes are cleared, except cells shared with W4 or the floor,
then the new panels are conservatively rasterized by voxel/solid intersection.
All cells outside those old/new panel volumes are retained byte-for-byte.
The existing tunnel, window, other obstacles and free-space assumptions remain.

## Coordinate registration and provenance

The layout is declared in
[`course_layout_2026_11.json`](../src/larp_navigation/config/course_layout_2026_11.json).
It records the edited PLY's checksum, the original prior checksums, and the
unchanged course-to-planner transform:

```text
map_x = course_north - 4.1624
map_y = 0.5334 - course_east
map_z = course_up - 0.0672
```

This is the existing training-launch registration. The map boundary, goal,
launch clearance, PX4 transform and alignment-verification settings are not
changed. Panel placement is inferred from the supplied schematic and scan,
not a surveyed hardware registration. Sensor updates can still revise the
prior; the hard net/geofence remains independent.

The JSON sidecars carry the new layout/source and map checksums, preserve the
original CAD provenance under `base_prior`, and record a checksum of all
unchanged rows. The updater refuses an unknown source map rather than clearing
space in an arbitrary map.

## Regenerate and verify

Use the supported ROS Humble Dev Container. From the `flight-stack` root:

```bash
python3 src/larp_navigation/tools/update_course_prior.py \
  --config-dir src/larp_navigation/config
python3 src/larp_navigation/tools/update_course_prior.py \
  --config-dir src/larp_navigation/config --check
```

The updater accepts the pinned original prior or the verified current layout;
running it again is idempotent. To regenerate from the historical baseline,
restore the original `.txt` and `.json` pair together in a separate checkout.

The package registers `course_layout_prior` for layout/frame/aperture/checksum
checks and `updated_course_route` for the actual SUPER A* and trajectory
frontend using the shipped 5 cm map under the 1.8 m ceiling. The explicit
route fixture used by `cad_corner_frontend` follows the new maze opening;
the old fixture starts at a location now occupied by W2.

Rebuild and test `larp_navigation` using
[`workspace.md`](workspace.md). For this nested checkout, use the parent
container and put colcon build/install/log output under the parent's
`ros2_ws`. Rebuild the deployment bundle before transferring to a Jetson;
editing the checkout does not replace files already baked into a runtime image.
No running system is restarted by this change.
