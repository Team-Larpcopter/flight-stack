"""Launch graph checks only: no processes, sensors or PX4 connections."""
import importlib.util
from pathlib import Path
import pytest

launch = pytest.importorskip('launch')
if not hasattr(launch, 'LaunchContext'):
    pytest.skip('ROS launch is unavailable', allow_module_level=True)
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node


def load_module():
    path = Path(__file__).parents[1] / 'launch/full_stack.launch.py'
    spec = importlib.util.spec_from_file_location('full_stack_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(tmp_path, monkeypatch):
    module = load_module()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    file = tmp_path / 'input.yaml'
    file.write_text('{}')
    c = launch.LaunchContext()
    c.launch_configurations.update({
        'mode': 'shadow', 'alignment_verified': 'false', 'mission_id': '',
        **{k: str(file) for k in ('rig_file', 'navigation_config',
           'cad_prior_file', 'boundary_config', 'estimator_config', 'bridge_params')},
        **{k: '0.0' for k in ('goal_x', 'goal_y', 'goal_z', 'goal_yaw', 'initial_map_x',
           'initial_map_y', 'initial_map_z', 'initial_map_yaw', 'map_heading_ned',
           'map_origin_ned_x', 'map_origin_ned_y', 'map_origin_ned_z')},
        'takeoff_height': '0.75', 'hold_timeout': '5.0', 'start_localization': 'true',
        'agent_transport': 'udp4', 'agent_port': '8888', 'agent_device': '/dev/px4', 'agent_baud': '921600'})
    return module, c


def test_shadow_and_hardware_graph(tmp_path, monkeypatch):
    module, c = context(tmp_path, monkeypatch)
    actions = module.nodes(c)
    assert isinstance(actions[0], RegisterEventHandler)
    assert isinstance(actions[1], ExecuteProcess) and not isinstance(actions[1], Node)
    assert sum(isinstance(a, Node) for a in actions) == 4  # camera, VIO, goal, viz
    include = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    assert dict(include.launch_arguments)['mode'] == 'shadow'
    c.launch_configurations['mode'] = 'hardware'
    with pytest.raises(ValueError, match='alignment_verified'):
        module.nodes(c)
    c.launch_configurations.update(alignment_verified='true', mission_id='flight-1')
    actions = module.nodes(c)
    assert sum(isinstance(a, Node) for a in actions) == 5  # plus external vision
    include = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    assert dict(include.launch_arguments)['mode'] == 'hardware'
    c.launch_configurations['start_localization'] = 'false'
    assert sum(isinstance(a, Node) for a in module.nodes(c)) == 3


@pytest.mark.parametrize('key,value', [('mode', 'simulation'), ('agent_transport', 'typo'),
    ('agent_port', '0'), ('goal_x', 'nan'), ('rig_file', '/missing/rig')])
def test_invalid_inputs_rejected_before_process_creation(tmp_path, monkeypatch, key, value):
    module, c = context(tmp_path, monkeypatch)
    c.launch_configurations[key] = value
    with pytest.raises(ValueError):
        module.nodes(c)


def test_serial_command(tmp_path, monkeypatch):
    module, c = context(tmp_path, monkeypatch)
    c.launch_configurations['agent_transport'] = 'serial'
    assert len(module.nodes(c)) == 7
    c.launch_configurations['agent_device'] = 'not-a-device'
    with pytest.raises(ValueError):
        module.nodes(c)
