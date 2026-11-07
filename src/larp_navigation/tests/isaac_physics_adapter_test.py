"""Run with the isolated Isaac Python: actual PhysX integration, no PX4/arming."""
from isaacsim import SimulationApp
import traceback
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'third_party/flyx_sim/generator'))

app = SimulationApp({"headless": True, "disable_viewport_updates": True,
                     "limit_cpu_threads": 8})
exit_code = 1
try:
    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    from pegasus.simulator.logic.physics_adapter import PhysicsAdapter

    world = World(physics_dt=1.0/240, rendering_dt=1.0/30, backend="numpy")
    world.get_physics_context().set_gravity(0.0)
    callbacks = [0, 0]
    def first(dt): callbacks[0] += 1
    def second(dt): callbacks[1] += 1
    world.add_physics_callback('first', first)
    world.add_physics_callback('second', second)
    for name, x, q in [("rotated", 0, [2**-0.5, 0, 0, 2**-0.5]),
                       ("summed", 10, [1, 0, 0, 0]),
                       ("offset", 20, [1, 0, 0, 0])]:
        world.scene.add(DynamicCuboid(prim_path=f"/World/{name}", name=name,
            position=np.array([x, 0., 5.]), orientation=np.array(q),
            scale=np.ones(3), mass=2.))
    world.reset()
    from isaac_compat import rebind_physics_callbacks
    assert rebind_physics_callbacks(world) == ['first', 'second']
    callbacks[:] = [0, 0]
    physics = PhysicsAdapter()
    rotated, summed, offset = [physics.get_rigid_body('/World/'+n)
                              for n in ('rotated', 'summed', 'offset')]
    assert abs(physics.get_rigid_body_properties(rotated).mass - 2) < 1e-5
    initial = physics.get_rigid_body_pose(rotated).p.copy()
    for i in range(240):
        physics.apply_body_force(rotated, [2, 0, 0], [0, 0, 0], False)
        physics.apply_body_force(summed, [1, 0, 0], [0, 0, 0], False)
        physics.apply_body_force(summed, [1, 0, 0], [0, 0, 0], False)
        # Global force at a global lever arm, to validate force-position semantics.
        p = physics.get_rigid_body_pose(offset).p
        physics.apply_body_force(offset, [1, 0, 0], p + [0, 0.5, 0], True)
        world.step(render=False)
    vr = physics.get_rigid_body_linear_velocity(rotated)
    vs = physics.get_rigid_body_linear_velocity(summed)
    wo = physics.get_rigid_body_angular_velocity(offset)
    assert np.allclose(vr, [0, 1, 0], atol=.015), vr
    assert np.allclose(vs, [1, 0, 0], atol=.015), vs
    assert wo[2] < -1.0 and np.linalg.norm(wo[:2]) < .01, wo
    # State queries must advance on physics-only ticks, without render sync.
    assert physics.get_rigid_body_pose(rotated).p[1] - initial[1] > .49
    assert callbacks == [240, 240], callbacks
    print('PHYSICS_ADAPTER_PASS', {'local_force_velocity': vr.tolist(),
          'summed_force_velocity': vs.tolist(), 'offset_angular_velocity': wo.tolist()}, flush=True)
    exit_code = 0
except Exception:
    traceback.print_exc()
finally:
    app.close(exit_code=exit_code)
