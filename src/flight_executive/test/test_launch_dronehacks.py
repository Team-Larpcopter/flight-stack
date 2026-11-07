"""Keep the launch default in step with the course definition."""

import importlib.util
from pathlib import Path

from flight_trajectory.dronehacks_course import CRUISE_HEIGHT


def test_launch_cruise_height_matches_course():
    path = Path(__file__).parents[1] / 'launch' / 'dronehacks.launch.py'
    spec = importlib.util.spec_from_file_location('dronehacks_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert float(module.CRUISE_HEIGHT) == CRUISE_HEIGHT
