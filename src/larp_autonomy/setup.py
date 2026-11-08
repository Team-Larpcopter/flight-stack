from glob import glob
from setuptools import setup
setup(name='larp_autonomy', version='0.1.0', packages=['larp_autonomy'],
 data_files=[('share/ament_index/resource_index/packages',['resource/larp_autonomy']),
 ('share/larp_autonomy',['package.xml']),('share/larp_autonomy/launch',glob('launch/*.py')),
 ('share/larp_autonomy/config',glob('config/*'))], install_requires=['setuptools'],
 maintainer='Team Larpcopter',maintainer_email='team@larpcopter.local',
 description='Calibrated hardware inputs and certified planner-to-executive integration.',license='Apache-2.0',
 entry_points={'console_scripts':[
 'stereo_frontend=larp_autonomy.stereo_frontend:main',
 'capture_calibration=larp_autonomy.calibration:main',
 'bench_probe=larp_autonomy.bench_probe:main',
 'planner_visualization=larp_autonomy.planner_visualization:main',
 'planner_latency=larp_autonomy.planner_latency:main',
 'trajectory_bridge=larp_autonomy.trajectory_bridge:main']})
