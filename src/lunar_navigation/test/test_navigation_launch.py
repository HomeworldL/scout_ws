"""Check the default map path and independent Nav2 entry without starting processes."""
from pathlib import Path
from unittest.mock import patch

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def test_default_map_is_in_workspace_even_from_another_directory(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    source = Path(__file__).resolve().parents[1] / 'launch/navigation.launch.py'
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
                                 launch_arguments={'rviz': 'false'}.items()),
    ]))
    with patch.object(Node, 'execute', return_value=[]):
        assert service.run() == 0
    expected = Path(__file__).resolve().parents[3] / 'maps/site/map.yaml'
    assert service.context.launch_configurations['map'] == str(expected)
