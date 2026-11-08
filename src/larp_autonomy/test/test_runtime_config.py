from pathlib import Path
import yaml
import pytest
from larp_autonomy.runtime_config import load_config


def fixture(tmp_path):
    base=Path(__file__).parents[1]/'config/mission.yaml'
    cfg=yaml.safe_load(base.read_text())
    rig=tmp_path/'rig.json';rig.write_text('depth_frame: native_oak_depth_optical\nT_body_depth: [[1]]\n')
    bridge=tmp_path/'bridge.yaml';bridge.write_text('{}')
    cfg.update(rig_file=str(rig),bridge_params=str(bridge),alignment_verified=True)
    return cfg


def load(tmp_path,cfg,mode='hardware'):
    p=tmp_path/'mission.yaml';p.write_text(yaml.safe_dump(cfg));return load_config(p,mode)


def test_course_goal_and_ceiling(tmp_path):
    cfg=fixture(tmp_path);cfg['hard_boundary_upper'][2]=2.95
    actual=load(tmp_path,cfg)
    assert actual['hard_boundary_upper'][2]==1.8
    assert actual['goal']==[-1.90688,-.47244,1.,0.]
    assert actual['takeoff_height']==1.


@pytest.mark.parametrize('field,value', [('goal',[0.,0.,1.5,0.]),('takeoff_height',1.5),('alignment_verified',False)])
def test_invalid_mission_rejected(tmp_path,field,value):
    cfg=fixture(tmp_path);cfg[field]=value
    with pytest.raises(ValueError):load(tmp_path,cfg)


def test_shadow_does_not_require_hardware_alignment(tmp_path):
    cfg=fixture(tmp_path);cfg['alignment_verified']=False
    load(tmp_path,cfg,'shadow')


def test_each_runtime_component_constructs_without_starting_processes(tmp_path,monkeypatch):
    import importlib.util
    from launch import LaunchContext
    from larp_autonomy.runtime_config import COMPONENTS
    path=Path(__file__).parents[1]/'launch/mission_runtime.launch.py'
    spec=importlib.util.spec_from_file_location('mission_runtime',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'share',lambda pkg:str(Path(__file__).parents[2]/pkg))
    cfg=fixture(tmp_path);p=tmp_path/'mission.yaml';p.write_text(yaml.safe_dump(cfg))
    for mode in ('shadow','hardware'):
        for part in COMPONENTS:
            c=LaunchContext();c.launch_configurations.update(component=part,mode=mode,mission_file=str(p))
            actions=module.nodes(c)
            assert len(actions)>=2
