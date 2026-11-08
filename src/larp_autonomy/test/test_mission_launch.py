"""Construct launch descriptions without executing processes or opening devices."""
import importlib.util
from pathlib import Path
import pytest

launch = pytest.importorskip('launch')
if not hasattr(launch, 'LaunchContext'):
    pytest.skip('ROS launch is unavailable', allow_module_level=True)


def load_launch(name):
    path = Path(__file__).parents[1] / 'launch' / name
    spec = importlib.util.spec_from_file_location('tested_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(mode):
    c = launch.LaunchContext()
    c.launch_configurations.update({
        'mode': mode, 'route_backend': 'super_astar', 'alignment_verified': 'false', 'mission_id': '',
        'rig_file': '/unused/rig', 'navigation_config': '/unused/nav',
        'cad_prior_file': '/unused/cad',
        'boundary_config': '/unused/bounds', 'map_heading_ned': '0.0',
        'map_origin_ned_x': '0.0', 'map_origin_ned_y': '0.0', 'map_origin_ned_z': '0.0',
        'initial_map_x': '0.0', 'initial_map_y': '0.0', 'initial_map_z': '0.0',
        'initial_map_yaw': '0.0', 'takeoff_height': '0.75', 'hold_timeout': '5.0'})
    return c


def test_shadow_has_no_executive_and_active_modes_require_alignment():
    module = load_launch('planner_mission.launch.py')
    assert len(module.nodes(context('shadow'))) == 3
    for mode in ('simulation', 'hardware'):
        c = context(mode)
        with pytest.raises(ValueError):
            module.nodes(c)
        c.launch_configurations.update(alignment_verified='true', mission_id='test-flight')
        assert len(module.nodes(c)) == 4
    with pytest.raises(ValueError):
        module.nodes(context('typo'))


def test_full_profile_constructs_camera_vio_and_shadow_planner(tmp_path):
    module = load_launch('profile_pipeline.launch.py')
    c = context('shadow')
    data = tmp_path / 'input.yaml'
    data.write_text('{}')
    for key in ('rig_file', 'navigation_config', 'cad_prior_file', 'boundary_config'):
        c.launch_configurations[key] = str(data)
    c.launch_configurations.update(start_localization='true', estimator_config=str(data))
    assert len(module.nodes(c)) == 4
    c.launch_configurations['start_localization'] = 'false'
    assert len(module.nodes(c)) == 2
    c.launch_configurations['rig_file'] = str(tmp_path / 'missing')
    with pytest.raises(ValueError):
        module.nodes(c)


def test_removed_route_backend_is_rejected():
    module = load_launch('planner_mission.launch.py')
    c = context('shadow')
    c.launch_configurations['route_backend'] = 'unsupported'
    with pytest.raises(ValueError, match='super_astar'):
        module.nodes(c)
