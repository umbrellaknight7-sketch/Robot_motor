from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

from wheel_human_bringup.model import build_description, validate_description


BRINGUP = Path(get_package_share_directory('wheel_human_bringup'))
DESCRIPTION = Path(get_package_share_directory('wheel_human_description'))


@pytest.fixture
def contract():
    return (build_description(BRINGUP, DESCRIPTION),
            yaml.safe_load((BRINGUP / 'config/robot_mock.yaml').read_text()),
            yaml.safe_load((BRINGUP / 'config/controllers.yaml').read_text()))


def test_geometry_is_preserved():
    # Semantic snapshot of the 2026-09-24 dummy_bot.urdf, including wheels
    # and revolute wrists; checked against the sibling reference when updated.
    # Keep this check portable when only the colcon workspace is deployed.
    display = ET.fromstring(xacro.process_file(str(DESCRIPTION / 'urdf/wheel_human.urdf.xacro')).toxml())

    def structure(element):
        return (element.tag, element.attrib, (element.text or '').strip(),
                [structure(child) for child in element])

    snapshot = json.dumps([structure(child) for child in display], sort_keys=True).encode()
    assert hashlib.sha256(snapshot).hexdigest() == '1de8f4d270368b211af75c430be2c48cd13014165dcd9e0f64d2ef9716a56066'
    assert len(display.findall('link')) == 20
    assert len(display.findall('joint')) == 19
    assert len(display.findall("joint[@type='revolute']")) == 19
    links = {link.attrib['name'] for link in display.findall('link')}
    children = {joint.find('child').attrib['link'] for joint in display.findall('joint')}
    assert links - children == {'base_link'}
    rviz = yaml.safe_load((DESCRIPTION / 'rviz/display.rviz').read_text())
    assert rviz['Visualization Manager']['Global Options']['Fixed Frame'] == 'base_link'
    assert display.find('ros2_control') is None


def test_default_contract(contract):
    description, config, controllers = contract
    validate_description(description, config, controllers)
    root = ET.fromstring(description)
    assert len(root.findall('ros2_control/joint')) == 15
    assert root.find('ros2_control/hardware/plugin').text == 'erob_hardware/ErobSystem'


@pytest.mark.parametrize('change', ['duplicate', 'missing', 'fixed', 'limits', 'controller', 'recovery'])
def test_invalid_configuration_rejected(contract, change):
    description, config, controllers = deepcopy(contract)
    if change == 'duplicate':
        config['motors'][1]['joint'] = config['motors'][0]['joint']
    elif change == 'missing':
        config['motors'].pop()
    elif change == 'fixed':
        root = ET.fromstring(description)
        root.find("joint[@name='" + config['motors'][0]['joint'] + "']").set('type', 'fixed')
        description = ET.tostring(root)
    elif change == 'limits':
        config['motors'][0]['max_position'] = 9
    elif change == 'controller':
        controllers['position_controller']['ros__parameters']['joints'][0] = 'unknown'
    else:
        controllers['position_controller']['ros__parameters']['open_loop_control'] = True
    with pytest.raises(ValueError):
        validate_description(description, config, controllers)


def test_yaml_order_and_optional_effort(tmp_path, contract):
    _, config, controllers = contract
    config['motors'].reverse()
    config['profile']['effort_verified'] = False
    config['motors'][0]['effort_verified'] = True
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(config))
    description = build_description(BRINGUP, DESCRIPTION, config_file=str(path))
    validate_description(description, config, controllers)
    joints = ET.fromstring(description).findall('ros2_control/joint')
    assert [joint.attrib['name'] for joint in joints] == [motor['joint'] for motor in config['motors']]
    assert joints[0].find("state_interface[@name='effort']") is not None
    assert all(joint.find("state_interface[@name='effort']") is None for joint in joints[1:])


@pytest.mark.parametrize('kwargs', [
    {'backend': 'invalid'},
    {'backend': 'soem'},
    {'backend': 'soem', 'config_file': str(BRINGUP / 'config/robot_real.template.yaml')},
    {'backend': 'mock', 'config_file': str(BRINGUP / 'config/robot_real.template.yaml')},
    {'backend': 'soem', 'config_file': str(BRINGUP / 'config/robot_real.template.yaml'),
     'model_file': str(DESCRIPTION / 'urdf/wheel_human.urdf.xacro')},
])
def test_launch_guards(kwargs):
    with pytest.raises(ValueError):
        build_description(BRINGUP, DESCRIPTION, **kwargs)


def test_feedforward_contract():
    config = BRINGUP / 'config/robot_mock_feedforward.yaml'
    controllers = BRINGUP / 'config/controllers_feedforward.yaml'
    description = build_description(BRINGUP, DESCRIPTION, config_file=str(config),
                                    controllers_file=str(controllers))
    joints = ET.fromstring(description).findall('ros2_control/joint')
    assert len(joints) == 15
    assert all([c.attrib['name'] for c in j.findall('command_interface')] ==
               ['position', 'velocity_offset', 'torque_offset'] for j in joints)
    with pytest.raises((KeyError, ValueError)):
        build_description(BRINGUP, DESCRIPTION, config_file=str(config))


def test_fault_injection_boolean_is_canonical():
    description = build_description(BRINGUP, DESCRIPTION, enable_mock_fault_injection=True)
    flag = ET.fromstring(description).find("ros2_control/hardware/param[@name='enable_mock_fault_injection']")
    assert flag.text == 'true'


@pytest.mark.parametrize('wrists_enabled', [False, True])
def test_wrists_and_wheels_are_separate_from_ethercat(contract, wrists_enabled):
    _, config, controllers = contract
    description = build_description(BRINGUP, DESCRIPTION, wrists_enabled=wrists_enabled)
    root = ET.fromstring(description)
    assert len(root.findall('ros2_control/joint')) == 15
    assert len(root.findall("joint[@type='revolute']")) == 19
    for name in ('Left_Wrist_Joint', 'Right_Wrist_Joint', 'Left_Wheel_Joint', 'Right_Wheel_Joint'):
        assert root.find("joint[@name='" + name + "']").attrib['type'] == 'revolute'
        assert root.find("ros2_control/joint[@name='" + name + "']") is None
    root.find("joint[@name='Left_Wrist_Joint']/axis").set('xyz', '0 0 0')
    with pytest.raises(ValueError):
        validate_description(ET.tostring(root), config, controllers)


@pytest.mark.parametrize('name', [
    'Left_Wrist_Joint', 'Right_Wrist_Joint', 'Left_Wheel_Joint', 'Right_Wheel_Joint',
])
def test_non_body_motor_mapping_rejected(contract, name):
    description, config, controllers = deepcopy(contract)
    config['motors'][0]['joint'] = name
    with pytest.raises(ValueError, match='Motor mapping must cover'):
        validate_description(description, config, controllers)


def test_unrecognized_movable_joint_rejected(contract):
    description, config, controllers = contract
    root = ET.fromstring(description)
    extra = deepcopy(root.find("joint[@name='Left_Wheel_Joint']"))
    extra.set('name', 'Extra_Wheel_Joint')
    extra.find('child').set('link', 'Extra_Wheel_Link')
    root.append(extra)
    ET.SubElement(root, 'link', name='Extra_Wheel_Link')
    with pytest.raises(ValueError, match='Motor mapping must cover'):
        validate_description(ET.tostring(root), config, controllers)
