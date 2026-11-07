from setuptools import setup
setup(name='larp_px4_bridge', version='0.1.0', packages=['larp_px4_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/larp_px4_bridge']),
                  ('share/larp_px4_bridge', ['package.xml', 'README.md']),
                  ('share/larp_px4_bridge/launch', ['launch/bridge.launch.py'])],
      install_requires=['setuptools', 'numpy'], tests_require=['pytest'],
      entry_points={'console_scripts': ['vins_px4_bridge = larp_px4_bridge.node:main']})
