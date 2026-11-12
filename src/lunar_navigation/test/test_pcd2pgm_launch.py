"""Check the third-party conversion/export wiring without starting ROS processes."""
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


@pytest.mark.parametrize('rviz', ['true', 'false'])
def test_upstream_converter_and_nav2_export(tmp_path, rviz):
    pcd = tmp_path / 'source.pcd'
    pcd.touch()  # Nodes are intercepted; only path validation executes.
    output = tmp_path / 'export/site/map'
    service = LaunchService()
    source = Path(__file__).resolve().parents[1] / 'launch/pcd2pgm.launch.py'
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
                                 launch_arguments={'pcd': str(pcd), 'output': str(output),
                                                   'rviz': rviz}.items()),
    ]))
    executed = []

    def record_node(action, context):
        executed.append(tuple(perform_substitutions(
            context, normalize_to_list_of_substitutions(value))
            for value in (action.node_package, action.node_executable)))
        return []

    with patch.object(Node, 'execute', record_node):
        assert service.run() == 0
    assert ('pcd2pgm', 'pcd2pgm_node') in executed
    assert ('nav2_map_server', 'map_saver_cli') in executed
    assert executed.count(('rviz2', 'rviz2')) == (rviz == 'true')
    assert not any(executable == 'pcd_to_grid' for _, executable in executed)
    assert output.parent.is_dir()
    assert service.context.launch_configurations['pcd'] == str(pcd)
    assert service.context.launch_configurations['output'] == str(output)


def test_missing_pcd_fails_before_starting_converter(tmp_path):
    source = Path(__file__).resolve().parents[1] / 'launch/pcd2pgm.launch.py'
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
                                 launch_arguments={'pcd': str(tmp_path/'missing.pcd'),
                                                   'output': str(tmp_path/'output/map')}.items()),
    ]))
    with patch.object(Node, 'execute') as start:
        assert service.run() != 0
        start.assert_not_called()
    assert not (tmp_path/'output').exists()
