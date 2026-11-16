#!/usr/bin/env python3
"""Render the unchanged GO2 model, then correct only the MID360 reference chain.

The upstream model collapses its policy IMU proxy into trunk. The manufacturer's
MID360 extrinsic is relative to the physical internal IMU, whose origin differs
from trunk. An independent, massless reference frame preserves the policy's
existing IMU sampling point while giving ray emission and TF the same mounting.
"""
import argparse
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import rospkg
import xacro
import yaml


def corrected_description(xml, reference):
    root = ET.fromstring(xml)
    joint = root.find("joint[@name='lidar_joint']")
    if joint is None or joint.find('parent').get('link') != reference['parent']:
        raise ValueError('Unexpected upstream lidar_joint/parent; mount correction stopped')
    if joint.find('child').get('link') != 'lidar_link':
        raise ValueError('Unexpected upstream MID360 frame')
    name = reference['frame']
    if root.find("link[@name='%s']" % name) is not None:
        raise ValueError('MID360 reference frame already exists')
    for key in ('xyz', 'rpy'):
        if len(reference[key]) != 3 or not all(math.isfinite(float(v)) for v in reference[key]):
            raise ValueError('Invalid mounting reference '+key)
    # No new inertia, collision or visual: fixed-joint reduction must retain
    # exactly the existing physical model. Keep lidar_joint's official origin.
    ET.SubElement(root, 'link', {'name': name})
    ref_joint = ET.SubElement(root, 'joint', {
        'name': 'go2_body_imu_reference_joint', 'type': 'fixed'})
    ET.SubElement(ref_joint, 'parent', {'link': reference['parent']})
    ET.SubElement(ref_joint, 'child', {'link': name})
    ET.SubElement(ref_joint, 'origin', {
        key: ' '.join(format(float(v), '.12g') for v in reference[key])
        for key in ('xyz', 'rpy')})
    joint.find('parent').set('link', name)
    return ET.tostring(root, encoding='unicode')


def render(mappings, mount_reference='official_imu'):
    packages = rospkg.RosPack()
    upstream = Path(packages.get_path('go2_description'))/'xacro/robot.xacro'
    xml = xacro.process_file(str(upstream), mappings=mappings).toxml()
    if mount_reference == 'legacy_trunk':
        return xml
    config = Path(packages.get_path('lrae_go2'))/'config/go2_body_imu_reference.yaml'
    with config.open() as handle:
        reference = yaml.safe_load(handle)
    return corrected_description(xml, reference)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mount-reference', choices=['official_imu', 'legacy_trunk'],
                        default='official_imu')
    parser.add_argument('mappings', nargs='*', help='Existing xacro name:=value arguments')
    args = parser.parse_args()
    mappings = {}
    for item in args.mappings:
        if ':=' not in item:
            parser.error('Expected xacro name:=value, got '+item)
        key, value = item.split(':=', 1)
        mappings[key] = value
    sys.stdout.write(render(mappings, args.mount_reference))


if __name__ == '__main__':
    main()
