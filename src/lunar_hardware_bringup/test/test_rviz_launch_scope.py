"""Execute real launch conditions/includes without starting hardware or GUI processes."""
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


@pytest.mark.parametrize('entry', ['mapping', 'localization'])
@pytest.mark.parametrize('rviz', [None, 'true', 'false'])
def test_description_does_not_override_parent_rviz(entry, rviz, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    arguments = {}
    if rviz is not None:
        arguments['rviz'] = rviz
    source = Path(__file__).parents[1] / 'launch' / f'{entry}.launch.py'
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
                                 launch_arguments=arguments.items()),
    ]))
    executed = []

    def record_node(action, context):
        executed.append(tuple(perform_substitutions(
            context, normalize_to_list_of_substitutions(value))
            for value in (action.node_package, action.node_executable)))
        return []

    # LaunchService still expands the real nested description and evaluates conditions.
    with patch.object(Node, 'execute', record_node):
        assert service.run() == 0

    assert executed.count(('rviz2', 'rviz2')) == (0 if rviz == 'false' else 1)
    assert ('robot_state_publisher', 'robot_state_publisher') in executed
    assert ('fastlio2', 'lio_node') in executed
    assert not any(package == 'livox_ros_driver2' for package, _ in executed)
    backend = ('pgo', 'pgo_node') if entry == 'mapping' else ('localizer', 'localizer_node')
    assert backend in executed
    assert service.context.launch_configurations['rviz'] == (rviz or 'true')
    assert 'start_lidar' not in service.context.launch_configurations
    if entry == 'localization':
        expected_pcd = str(Path(__file__).resolve().parents[3] / 'maps/site/map.pcd')
        assert service.context.launch_configurations['pcd'] == expected_pcd
        assert service.context.launch_configurations['base_z'] == 'auto'
