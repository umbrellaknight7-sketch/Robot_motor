"""Load complete named poses against a selected URDF, independent of ROS I/O."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


class _UniqueLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str):
            raise ValueError('Pose mapping keys must be strings')
        if key in result:
            raise ValueError('Duplicate pose key: ' + key)
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def _model(description):
    robot = ET.fromstring(description)
    joints = {}
    seen = set()
    for joint in robot.findall('joint'):
        name = joint.get('name')
        if not name or name in seen:
            raise ValueError('Missing or duplicate model joint name')
        seen.add(name)
        if joint.get('type') == 'fixed':
            continue
        if joint.get('type') != 'revolute':
            raise ValueError('Radian reference poses require bounded revolute joints: ' + name)
        limit = joint.find('limit')
        if limit is None:
            raise ValueError('Missing position limits: ' + name)
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
            raise ValueError('Invalid position limits: ' + name)
        joints[name] = (lower, upper)
    return robot.get('name'), joints


def validate_positions(positions, description):
    """Return floats in URDF joint order after validating the complete mapping."""
    _, joints = _model(description)
    if not isinstance(positions, dict) or set(positions) != set(joints):
        raise ValueError('Pose must provide every movable joint exactly once; no unknown joints')
    result = {}

    # Return positions in model order, regardless of the YAML mapping order.
    for name, (lower, upper) in joints.items():
        value = positions[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError('Joint position must be numeric: ' + name)
        value = float(value)
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError('Non-finite or out-of-limit joint position: ' + name)
        result[name] = value
    return result


def load_pose(path, description):
    """Load a named radians pose; the model identifier must match the URDF name."""
    document = yaml.load(Path(path).read_text(), Loader=_UniqueLoader)
    if not isinstance(document, dict) or not isinstance(document.get('name'), str) or not document['name']:
        raise ValueError('Pose needs a nonempty name')
    model_name, _ = _model(description)
    if document.get('model') != model_name:
        raise ValueError('Reference pose model does not match the selected URDF')
    if document.get('units') != 'radians':
        raise ValueError('Reference pose units must be radians')
    return validate_positions(document.get('joint_positions'), description)


def zero_pose(description):
    """Explicit CAD zero, not the saved upright pose or a hardware home command."""
    _, joints = _model(description)
    return validate_positions(dict.fromkeys(joints, 0.0), description)


def display_description(description):
    """Drop root inertia only from the in-memory KDL display copy."""
    robot = ET.fromstring(description)
    children = {joint.find('child').get('link') for joint in robot.findall('joint')}
    for link in robot.findall('link'):
        if link.get('name') not in children:
            for inertia in link.findall('inertial'):
                link.remove(inertia)
    return ET.tostring(robot, encoding='unicode')
