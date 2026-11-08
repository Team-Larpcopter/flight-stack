"""The follower planner component must match larp_autonomy's planner component exactly,
except for the executable and the follower-only parameters."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext

HERE = Path(__file__).resolve().parent
ORIGINAL = HERE.parents[1] / 'larp_autonomy' / 'launch' / 'mission_runtime.launch.py'
FOLLOWER = HERE.parent / 'launch' / 'mission_runtime.launch.py'
FOLLOWER_ONLY = {'planner_backend', 'follower_horizon_m', 'follower_replan_period_s', 'follower_route_file'}


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Captured:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


@pytest.fixture
def mission(tmp_path):
    rig = tmp_path / 'rig.json'
    rig.write_text(yaml.safe_dump({'T_body_depth': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
                                   'depth_frame': 'native_oak_depth_optical'}))
    path = tmp_path / 'mission.yaml'
    path.write_text(yaml.safe_dump({
        'goal': [1., 0., 1., 0.], 'hard_boundary_lower': [-2., -2., 0.], 'hard_boundary_upper': [2., 2., 1.8],
        'hard_boundary_reference': 'unit test', 'map_origin_ned': [0., 0., 0.], 'initial_position_m': [0., 0., 0.],
        'takeoff_height': 1., 'map_heading_ned': 0., 'initial_yaw_rad': 0., 'rig_file': str(rig)}))
    return path


def planner_node(module, mission, monkeypatch, tmp_path, **extra):
    monkeypatch.setattr(module, 'Node', Captured)
    monkeypatch.setattr(module, 'share', lambda package: str(tmp_path / package))
    context = LaunchContext()
    context.launch_configurations.update(component='planner', mode='shadow', mission_file=str(mission),
                                         planner='super', **extra)
    nodes = [n for n in module.nodes(context) if isinstance(n, Captured)]
    assert len(nodes) == 1
    return nodes[0].kwargs


def test_planner_component_matches_original(mission, monkeypatch, tmp_path):
    original = planner_node(load(ORIGINAL, 'original_runtime'), mission, monkeypatch, tmp_path)
    follower = planner_node(load(FOLLOWER, 'follower_runtime'), mission, monkeypatch, tmp_path,
                            follower_horizon_m='1.5', follower_replan_period_s='0.4', follower_route_file='')
    assert (original['package'], original['executable']) == ('larp_navigation', 'depth_map_node')
    assert (follower['package'], follower['executable']) == ('larp_route_follower', 'route_follower_node')
    assert follower['remappings'] == original['remappings']
    (a,), (b,) = original['parameters'], follower['parameters']
    assert b['planner_backend'] == 'follower'
    assert {k: v for k, v in a.items() if k != 'planner_backend'} == \
           {k: v for k, v in b.items() if k not in FOLLOWER_ONLY}


def test_whole_stack_launch_starts_every_component_through_the_follower(mission, monkeypatch, tmp_path):
    module = load(HERE.parent / 'launch' / 'follower_system.launch.py', 'follower_system')
    captured = []
    monkeypatch.setattr(module, 'IncludeLaunchDescription',
                        lambda source, launch_arguments: captured.append((source, dict(launch_arguments))))
    monkeypatch.setattr(module, 'share', lambda package: str(tmp_path / package))
    monkeypatch.setattr(module, 'PythonLaunchDescriptionSource', lambda path: path)
    context = LaunchContext()
    context.launch_configurations.update(mission_file=str(mission), mode='shadow', components='',
                                         follower_horizon_m='1.5', follower_replan_period_s='0.4',
                                         follower_route_file='')
    module.components(context)
    assert [args['component'] for _, args in captured] == list(module.DEFAULT)
    assert all(source.endswith('larp_route_follower/launch/mission_runtime.launch.py')
               for source, _ in captured)
    assert all(args['mission_file'] == str(mission) and args['mode'] == 'shadow' for _, args in captured)
    context.launch_configurations['components'] = 'depth,nope'
    with pytest.raises(ValueError):
        module.components(context)
