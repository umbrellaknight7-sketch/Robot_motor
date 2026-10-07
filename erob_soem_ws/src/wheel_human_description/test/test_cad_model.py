"""Installed CAD contract and strict, explicit reference configurations."""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from wheel_human_description.poses import display_description, load_pose, zero_pose

PACKAGE = Path(__file__).resolve().parents[1]
XML = (PACKAGE / 'urdf/kinematics_fixed.urdf').read_text()
POSE = PACKAGE / 'config/poses/upright.yaml'


def test_cad_tree_and_tcp_contract():
    robot = ET.fromstring(XML)
    links = {link.get('name') for link in robot.findall('link')}
    joints = robot.findall('joint')
    assert len(links) == 22 and len(joints) == 21
    assert sum(j.get('type') == 'revolute' for j in joints) == 19
    assert links - {j.find('child').get('link') for j in joints} == {'base_link'}
    children = {}
    for j in joints:
        child = j.find('child').get('link')
        assert child not in children
        children[child] = j.find('parent').get('link')
        assert children[child] in links
    for child in children:
        seen = set()
        while child in children:
            assert child not in seen
            seen.add(child)
            child = children[child]
        assert child == 'base_link'
    for side in ('Left', 'Right'):
        joint = robot.find(f"joint[@name='{side}_Hand_TCP_Joint']")
        assert joint.get('type') == 'fixed'
        assert joint.find('origin').attrib == {'xyz': '0.05 0 0', 'rpy': '0 0 0'}
        assert joint.find('parent').get('link') == side + '_Hand_Link'
        assert joint.find('child').get('link') == side + '_Hand_TCP'
    for j in joints:
        if j.get('type') == 'revolute':
            assert j.find('limit').attrib == {'lower': '-3.14', 'upper': '3.14', 'effort': '0', 'velocity': '0'}


def test_import_hashes_and_package_meshes():
    provenance = json.loads((PACKAGE / 'config/kinematics_fixed.provenance.json').read_text())
    assert hashlib.sha256((PACKAGE / provenance['packaged_urdf']).read_bytes()).hexdigest() == provenance['packaged_urdf_sha256']
    for name, expected in provenance['meshes_sha256'].items():
        assert hashlib.sha256((PACKAGE / name).read_bytes()).hexdigest() == expected
    for mesh in ET.fromstring(XML).findall('.//mesh'):
        prefix = 'package://wheel_human_description/'
        assert mesh.get('filename').startswith(prefix)
        assert (PACKAGE / mesh.get('filename')[len(prefix):]).is_file()


def test_upright_and_zero_are_distinct():
    positions = load_pose(POSE, XML)
    assert len(positions) == 19
    for name, value in positions.items():
        assert value == (-math.pi / 2 if name in ('Left_Arm_Joint3', 'Right_Arm_Joint3') else 0.0)
    assert zero_pose(XML) == dict.fromkeys(positions, 0.0)


@pytest.mark.parametrize('bad', ['missing', 'unknown', 'fixed', 'nan', 'inf', 'limit', 'bool', 'string', 'units', 'model', 'name'])
def test_invalid_pose_rejected(tmp_path, bad):
    document = yaml.safe_load(POSE.read_text())
    positions = document['joint_positions']
    if bad == 'missing':
        positions.pop('Left_Wrist_Joint')
    elif bad in ('unknown', 'fixed'):
        positions['unknown' if bad == 'unknown' else 'Left_Hand_TCP_Joint'] = positions.pop('Left_Wrist_Joint')
    elif bad in ('nan', 'inf', 'limit', 'bool', 'string'):
        positions['Left_Wrist_Joint'] = {'nan': float('nan'), 'inf': float('inf'), 'limit': 3.15, 'bool': True, 'string': '0'}[bad]
    else:
        document[bad] = {'units': 'degrees', 'model': 'different_model', 'name': ''}[bad]
    path = tmp_path / 'pose.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_pose(path, XML)


def test_duplicate_joint_rejected_before_yaml_overwrite(tmp_path):
    path = tmp_path / 'pose.yaml'
    path.write_text(POSE.read_text() + '  Left_Arm_Joint3: 0.0\n')
    with pytest.raises(ValueError, match='Duplicate'):
        load_pose(path, XML)


def test_name_order_is_not_configuration_order(tmp_path):
    document = yaml.safe_load(POSE.read_text())
    document['joint_positions'] = dict(reversed(list(document['joint_positions'].items())))
    path = tmp_path / 'pose.yaml'
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    assert load_pose(path, XML) == load_pose(POSE, XML)


def test_display_only_removes_root_inertia():
    original = ET.fromstring(XML)
    expected = deepcopy(original)
    base = expected.find("link[@name='base_link']")
    assert base.find('inertial') is not None
    base.remove(base.find('inertial'))
    assert ET.tostring(expected) == ET.tostring(ET.fromstring(display_description(XML)))
    assert original.find("link[@name='base_link']/inertial") is not None
