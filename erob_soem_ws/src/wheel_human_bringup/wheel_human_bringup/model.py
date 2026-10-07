"""Validate the robot-specific contract before starting any ROS nodes."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml


DUMMY_MARKER = 'SOFTWARE-ONLY PLACEHOLDER MODEL'


def validate_description(description, config, controllers):
    robot = ET.fromstring(description)
    joints = robot.findall('joint')
    names = [joint.attrib['name'] for joint in joints]
    if len(names) != len(set(names)):
        raise ValueError('Robot model contains duplicate joint names')
    movable = {joint.attrib['name']: joint for joint in joints
               if joint.attrib['type'] != 'fixed'}
    motors = config['motors']
    mapped = [motor['joint'] for motor in motors]
    if len(mapped) != 15 or config['bus']['expected_motors'] != 15:
        raise ValueError('This robot requires exactly 15 configured motors')
    wrists = {'Left_Wrist_Joint', 'Right_Wrist_Joint'}
    # SDK wrists and description-only wheels are outside EtherCAT control.
    # Explicit names keep any other unmapped movable joint a contract error.
    wheels = {'Left_Wheel_Joint', 'Right_Wheel_Joint'}
    for name in wrists & set(movable):
        joint = movable[name]
        limit, axis = joint.find('limit'), joint.find('axis')
        if joint.attrib['type'] != 'revolute' or limit is None or axis is None:
            raise ValueError('SDK wrist requires revolute type, axis and limits: ' + name)
        values = [float(limit.attrib[k]) for k in ('lower', 'upper', 'velocity', 'effort')]
        direction = [float(x) for x in axis.attrib['xyz'].split()]
        if (not all(math.isfinite(x) for x in values + direction) or len(direction) != 3
                or not math.isclose(sum(x*x for x in direction), 1., abs_tol=1e-6)
                or values[0] >= values[1] or min(values[2:]) <= 0):
            raise ValueError('Invalid SDK wrist axis/limits: ' + name)
    expected_body = set(movable) - wrists - wheels
    if len(set(mapped)) != len(mapped) or set(mapped) != expected_body:
        raise ValueError('Motor mapping must cover each body-controlled model joint exactly once')

    # The control block must preserve the motor YAML order and exported interfaces.
    systems = robot.findall('ros2_control')
    if len(systems) != 1 or systems[0].attrib.get('name') != 'ErobSystem':
        raise ValueError('Expected one ErobSystem control component')
    control_joints = systems[0].findall('joint')
    if [joint.attrib['name'] for joint in control_joints] != mapped:
        raise ValueError('Control joint order must match the motor YAML')
    feedforward = config['bus'].get('csp_feedforward', False)
    expected_commands = ['position', 'velocity_offset', 'torque_offset'] if feedforward else ['position']
    for motor, control in zip(motors, control_joints):
        name = motor['joint']
        profile = dict(config['profile'], **motor)
        expected_states = ['position', 'velocity']
        if profile['effort_verified']:
            expected_states.append('effort')
        if ([item.attrib['name'] for item in control.findall('command_interface')] != expected_commands
                or [item.attrib['name'] for item in control.findall('state_interface')] != expected_states):
            raise ValueError('Invalid control interfaces for ' + name)
        joint = movable[name]
        if joint.attrib['type'] != 'revolute':
            raise ValueError('Expected a revolute joint: ' + name)
        limit = joint.find('limit')
        if limit is None:
            raise ValueError('Missing model limits for ' + name)
        lower, upper, velocity, effort = [float(limit.attrib[key])
                                          for key in ('lower', 'upper', 'velocity', 'effort')]
        minimum, maximum, max_velocity, max_effort = [float(profile[key]) for key in
            ('min_position', 'max_position', 'max_velocity', 'max_effort')]
        if (not all(math.isfinite(value) for value in
                    (lower, upper, velocity, effort, minimum, maximum, max_velocity, max_effort))
                or not lower <= minimum < maximum <= upper
                or not 0 < max_velocity <= velocity or not 0 < max_effort <= effort):
            raise ValueError('Driver limits must fit within model limits: ' + name)

    # Match the controller contract to the selected position or feedforward profile.
    controller = 'csp_controller' if feedforward else 'position_controller'
    params = controllers[controller]['ros__parameters']
    controlled = params['joints']
    if len(controlled) != len(mapped) or set(controlled) != set(mapped):
        raise ValueError('Trajectory controller must cover each motor joint exactly once')
    if params['command_interfaces'] != expected_commands or params['state_interfaces'] != ['position', 'velocity']:
        raise ValueError('Trajectory controller requires position commands and position/velocity states')
    if (params.get('open_loop_control') is not False
            or params.get('set_last_command_interface_value_as_state_on_activation') is not False):
        raise ValueError('Trajectory controller must initialize from measured feedback')


def build_description(bringup_share, description_share, *, backend='mock',
                      config_file='', model_file='', enable_mock_fault_injection=False,
                      controllers_file='', wrists_enabled=False):
    bringup_share, description_share = Path(bringup_share), Path(description_share)
    if backend not in ('mock', 'soem'):
        raise ValueError('backend must be mock or soem')
    if backend == 'soem':
        if not config_file or not model_file:
            raise ValueError('Real hardware requires explicit config_file and calibrated model_file')
        if enable_mock_fault_injection:
            raise ValueError('Fault injection is available only with backend=mock')

    config_path = Path(config_file or bringup_share / 'config/robot_mock.yaml').resolve(strict=True)
    model_path = Path(model_file or description_share / 'urdf/wheel_human.urdf.xacro').resolve(strict=True)
    config = yaml.safe_load(config_path.read_text())
    if config['bus']['backend'] != backend:
        raise ValueError('Requested backend must match bus.backend in the configuration')
    controllers = yaml.safe_load(Path(controllers_file or bringup_share / 'config/controllers.yaml').read_text())

    # Compose the selected model, then validate the final expanded description.
    description = xacro.process_file(
        str(bringup_share / 'urdf/wheel_human_control.urdf.xacro'),
        mappings={'config_file': str(config_path), 'model_file': str(model_path),
                  'backend': backend, 'wrists_enabled': str(wrists_enabled).lower(),
                  'enable_mock_fault_injection': str(enable_mock_fault_injection).lower()},
    ).toxml()
    if backend == 'soem' and DUMMY_MARKER in description:
        raise ValueError('The software-only dummy model cannot be used with real hardware')
    validate_description(description, config, controllers)
    if wrists_enabled:
        root = ET.fromstring(description)
        for name in ('Left_Wrist_Joint', 'Right_Wrist_Joint'):
            joint = root.find("joint[@name='" + name + "']")
            if joint is None or joint.attrib['type'] != 'revolute':
                raise ValueError('Enabled SDK wrist missing from model: ' + name)
    return description
